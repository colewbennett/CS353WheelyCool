//#include <Arduino.h>
//#include <WiFi.h>
//#include <WebServer.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_Accel.h>
#include "index.h"   // your webpage HTML

/********************* WIFI CONFIG ***************************/
// const char* ssid     = "ESP32-LineBot";
// const char* password = "12345678";
// WebServer server(80);

/********************* ACCELEROMETER **************************/
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(12345);
float accelData[3] = {0,0,0};   // X, Y, Z

/*************** COLOR ENUM ***************/
enum FollowColor { RED, GREEN, BLUE };

struct DirectionResult {
  int index;
  int strength;
  int leftMotor;
  int rightMotor;
};

FollowColor targetColor = RED;
 
/********************* WEB HANDLERS ***************************/
// void handleRoot() {
//   // Replace placeholders in HTML with current accel values
//   String html = index_html;
//   html.replace("%ACCEL_X%", String(accelData[0], 2));
//   html.replace("%ACCEL_Y%", String(accelData[1], 2));
//   html.replace("%ACCEL_Z%", String(accelData[2], 2));
//   server.send(200, "text/html", html);
// }

// void handleSetColor() {
//   if (!server.hasArg("color")) return;
//   String c = server.arg("color");
//   if      (c == "red")   targetColor = RED;
//   else if (c == "green") targetColor = GREEN;
//   else if (c == "blue")  targetColor = BLUE;
//   server.send(200, "text/plain", "OK");
// }

/**************** PIN SETUP ****************/
#define ENA_PIN 5     // Motor A PWM
#define IN1_PIN 2
#define IN2_PIN 3

#define ENB_PIN 6     // Motor B PWM
#define IN3_PIN 4
#define IN4_PIN 7

#define LED_PIN 17     // NeoPixel ring
#define LED_COUNT 16
#define SENSOR_PIN A7

/*************** NEOPIXEL SETUP ***************/
Adafruit_NeoPixel ring(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

/*************** SAFE ESP32 PWM CHANNELS ***************/
const int PWM_CHANNEL_A = 4;
const int PWM_CHANNEL_B = 5;
const int PWM_FREQ = 5000;
const int PWM_RES = 8;


/********************* SETUP ************************/
void setup() {
  Serial.begin(115200);

  ring.begin();
  ring.show();
  ring.setBrightness(30);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA_PIN, PWM_CHANNEL_A);
  ledcAttachPin(ENB_PIN, PWM_CHANNEL_B);

  // WiFi AP
  // WiFi.softAP(ssid, password);
  // Serial.println("AP Ready. Connect to WiFi.");

  // // Web server
  // server.on("/", handleRoot);
  // server.on("/setColor", handleSetColor);
  // server.begin();
}

/********************* MOTOR CONTROL ************************/
void setMotorA(int speed, int direction) {
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
  ledcWrite(PWM_CHANNEL_A, abs(speed));
}

void setMotorB(int speed, int direction) {
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
  ledcWrite(PWM_CHANNEL_B, abs(speed));
}

void runMotors(int left, int right) {
  setMotorA(abs(left), left > 0 ? 1 : (left < 0 ? -1 : 0));
  setMotorB(abs(right), right > 0 ? 1 : (right < 0 ? -1 : 0));
}

/********************* DIRECTION SCAN ************************/
DirectionResult getDirectionForColor(FollowColor color) {

  int bestIndex = -1;
  int bestValue = -1;

  for (int i = 0; i < LED_COUNT; i++) {
    ring.clear();
    ring.setPixelColor(i, ring.Color(
      color == RED ? 255 : 0,
      color == GREEN ? 255 : 0,
      color == BLUE ? 255 : 0
    ));
    ring.show();
    delay(8);

    // Read reflectance
    int sensorValue = analogRead(SENSOR_PIN);

    if (sensorValue >= bestValue) {
      bestValue = sensorValue;
      bestIndex = i;
    }
  }
  /**************** PROPORTIONAL STEERING ****************/
  // Center direction is LED index 0
  int offset = bestIndex;
  //Serial.println(bestIndex);

  int left = 170;
  int right = 180;

  if (offset == 0 || (offset <= 15 && offset >= 14)) {
    left = 0;
    right = 200;
  } else if (offset == 1 || offset == 2) {
    left = 170;
    right = 180;
  } else if (offset >= 3 && offset <=7 ) {
    left = 200;
    right = 0;
  }

  left = left;
  right = right;
  // Serial.print("Right: ");
  // Serial.print(right);
  // Serial.print(" Left: ");
  // Serial.println(left);
  return { bestIndex, bestValue, left, right };
}

/********************* MAIN LOOP ************************/
void loop() {
  //server.handleClient();

  //--- read accelerometer ---
  // sensors_event_t event;
  // accel.getEvent(&event);
  // accelData[0] = event.acceleration.x;
  // accelData[1] = event.acceleration.y;
  // accelData[2] = event.acceleration.z;

  DirectionResult dir = getDirectionForColor(targetColor);

  runMotors(dir.leftMotor, dir.rightMotor);

  delay(20);
}


