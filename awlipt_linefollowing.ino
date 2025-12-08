#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_Accel.h>
#include "index.h"   // your webpage HTML

/********************* WIFI CONFIG ***************************/
const char* ssid     = "ESP32-LineBot";
const char* password = "12345678";
WebServer server(80);

/********************* ACCELEROMETER **************************/
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(12345);
float accelData[3] = {0,0,0};   // X, Y, Z

/*************** COLOR ENUM + STRUCT ***************/
enum FollowColor { RED, GREEN, BLUE };

struct DirectionResult {
  int index;
  int strength;
  int leftMotor;
  int rightMotor;
};

FollowColor targetColor = RED;

/**************** PIN SETUP ****************/
#define ENA_PIN 5     // Motor A PWM
#define IN1_PIN 2
#define IN2_PIN 3

#define ENB_PIN 6     // Motor B PWM
#define IN3_PIN 4
#define IN4_PIN 7

#define LED_PIN 17     // NeoPixel ring
#define LED_COUNT 16

// NOTE: On many ESP32 boards A7 is NOT defined. If you get a compile error, change SENSOR_PIN to a valid ADC pin (for example 34).
#define SENSOR_PIN A7

/*************** NEOPIXEL SETUP ***************/
Adafruit_NeoPixel ring(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

/*************** SAFE ESP32 PWM CHANNELS ***************/
const int PWM_CHANNEL_A = 4;
const int PWM_CHANNEL_B = 5;
const int PWM_FREQ = 5000;
const int PWM_RES = 8; // 8-bit resolution (0-255)

/********************* WEB HANDLERS ***************************/
void handleRoot() {
  // Replace placeholders in HTML with current accel values
  String html = index_html;
  html.replace("%ACCEL_X%", String(accelData[0], 2));
  html.replace("%ACCEL_Y%", String(accelData[1], 2));
  html.replace("%ACCEL_Z%", String(accelData[2], 2));
  server.send(200, "text/html", html);
}

void handleSetColor() {
  if (!server.hasArg("color")) {
    server.send(400, "text/plain", "missing color");
    return;
  }
  String c = server.arg("color");
  if      (c == "red")   targetColor = RED;
  else if (c == "green") targetColor = GREEN;
  else if (c == "blue")  targetColor = BLUE;
  server.send(200, "text/plain", "OK");
}

/********************* SETUP ************************/
void setup() {
  // Serial start
  Serial.begin(115200);

  // Short initial delay to allow USB host to stabilize power during flashing.
  // This helps avoid brownouts that close the USB serial during upload.
  delay(2000);

  // Configure motor pins as outputs and ensure motors are off while we initialize.
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);

  // Setup PWM channels for motor enable pins
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA_PIN, PWM_CHANNEL_A);
  ledcAttachPin(ENB_PIN, PWM_CHANNEL_B);
  // Start with 0 duty to avoid drawing current from motors during init
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);

  // Initialize accelerometer early (low-power)
  if (!accel.begin()) {
    Serial.println("Warning: Accel not detected.");
  } else {
    Serial.println("Accel initialized.");
  }

  // Initialize NeoPixel library but keep LEDs off/very dim to avoid large current spikes.
  ring.begin();
  ring.setBrightness(8);  // very low brightness at boot
  ring.clear();
  ring.show();

  // Another short pause before enabling WiFi/AP. Staggering these prevents a big simultaneous current draw.
  delay(1500);

  // Start WiFi Access Point (after we've settled other peripherals)
  WiFi.softAP(ssid, password);
  Serial.println("AP Ready. Connect to WiFi.");

  // Start webserver
  server.on("/", handleRoot);
  server.on("/setColor", handleSetColor);
  server.begin();

  // After everything is up and stable, increase NeoPixel brightness a bit for normal operation.
  // This is done after WiFi/AP to avoid a simultaneous spike.
  delay(500);
  ring.setBrightness(30); // set to your normal brightness (lower is safer)
  ring.show();
}

/********************* MOTOR CONTROL ************************/
void setMotorA(int speed, int direction) {
  // clamp speed to 0..255 (8-bit)
  int s = constrain(abs(speed), 0, (1 << PWM_RES) - 1);
  if (direction > 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else if (direction < 0) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  }
  ledcWrite(PWM_CHANNEL_A, s);
}

void setMotorB(int speed, int direction) {
  int s = constrain(abs(speed), 0, (1 << PWM_RES) - 1);
  if (direction > 0) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
  } else if (direction < 0) {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
  } else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
  }
  ledcWrite(PWM_CHANNEL_B, s);
}

void runMotors(int left, int right) {
  setMotorA(left, left > 0 ? 1 : (left < 0 ? -1 : 0));
  setMotorB(right, right > 0 ? 1 : (right < 0 ? -1 : 0));
}

/********************* DIRECTION SCAN ************************/
DirectionResult getDirectionForColor(FollowColor color) {
  int bestIndex = -1;
  int bestValue = -1;

  for (int i = 0; i < LED_COUNT; i++) {
    ring.clear();
    // NeoPixel color ordering (GRB/BRG can vary by hardware) - you're using NEO_GRB
    ring.setPixelColor(i, ring.Color(
      color == RED ? 255 : 0,
      color == GREEN ? 255 : 0,
      color == BLUE ? 255 : 0
    ));
    ring.show();
    delay(8); // short settle for sensor reading

    // Read reflectance (ADC). Be aware: analogRead pins differ per ESP32 board.
    int sensorValue = analogRead(SENSOR_PIN);

    if (sensorValue >= bestValue) {
      bestValue = sensorValue;
      bestIndex = i;
    }
  }

  // Protect against not finding any index (shouldn't normally happen)
  if (bestIndex < 0) bestIndex = 0;

  /**************** PROPORTIONAL STEERING ****************/
  int offset = bestIndex;

  int left = 170;
  int right = 180;

  if (offset == 0 || (offset <= 15 && offset >= 14)) {
    left = 0;
    right = 200;
  } else if (offset == 1 || offset == 2) {
    left = 170;
    right = 180;
  } else if (offset >= 3 && offset <= 7) {
    left = 200;
    right = 0;
  }

  DirectionResult res;
  res.index = bestIndex;
  res.strength = bestValue;
  res.leftMotor = left;
  res.rightMotor = right;
  return res;
}

/********************* MAIN LOOP ************************/
void loop() {
  server.handleClient();

  // read accelerometer
  sensors_event_t event;
  accel.getEvent(&event);
  accelData[0] = event.acceleration.x;
  accelData[1] = event.acceleration.y;
  accelData[2] = event.acceleration.z;

  DirectionResult dir = getDirectionForColor(targetColor);

  runMotors(dir.leftMotor, dir.rightMotor);

  delay(20);
}
