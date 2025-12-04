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

/********************* LED RING CONFIG ************************/
#define LED_PIN    17
#define LED_COUNT  16
#define SENSOR_PIN A0
Adafruit_NeoPixel ring(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

enum FollowColor { RED=0, GREEN=1, BLUE=2 };
volatile FollowColor targetColor = RED;

/********************* ACCELEROMETER **************************/
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(12345);
float accelData[3] = {0,0,0};   // X, Y, Z

/********************* MOTOR CONFIG ***************************/
const int MIC_PIN  = A4;
const int ENA_PIN  = 8;
const int IN1_PIN  = 5;
const int IN2_PIN  = 6;
const int ENB_PIN  = 9;
const int IN3_PIN  = 7;
const int IN4_PIN  = 10;

void stopMotors();

/********************* LINE FOLLOWING *************************/
struct DirectionResult {
  int bestIndex;
  int lightValue;
  int leftMotor;
  int rightMotor;
};
DirectionResult getDirectionForColor(FollowColor color);

/********************* MOTOR CONTROL **************************/
void runMotors(int left, int right) {
  digitalWrite(IN1_PIN, left >= 0);
  digitalWrite(IN2_PIN, left < 0);
  analogWrite(ENA_PIN, min(abs(left), 255));

  digitalWrite(IN3_PIN, right >= 0);
  digitalWrite(IN4_PIN, right < 0);
  analogWrite(ENB_PIN, min(abs(right), 255));
}
void stopMotors() {
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);
}

/********************* MICROPHONE STOP ************************/
bool micDetectsClap() {
  int total = 0;
  for (int i = 0; i < 40; i++)
    total += analogRead(MIC_PIN);
  int value = total / 40;
  return value > 550;
}

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
  if (!server.hasArg("color")) return;
  String c = server.arg("color");
  if      (c == "red")   targetColor = RED;
  else if (c == "green") targetColor = GREEN;
  else if (c == "blue")  targetColor = BLUE;
  server.send(200, "text/plain", "OK");
}

/********************* SETUP **********************************/
void setup() {
  Serial.begin(115200);

  // LED ring
  ring.begin();
  ring.show();

  // Motors
  pinMode(ENA_PIN, OUTPUT); pinMode(IN1_PIN, OUTPUT); pinMode(IN2_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT); pinMode(IN3_PIN, OUTPUT); pinMode(IN4_PIN, OUTPUT);
  stopMotors();

  // Accelerometer
  if(!accel.begin()) {
    Serial.println("Failed to initialize accelerometer!");
    while(1);
  }

  // WiFi AP
  WiFi.softAP(ssid, password);
  Serial.println("AP Ready. Connect to WiFi.");

  // Web server
  server.on("/", handleRoot);
  server.on("/setColor", handleSetColor);
  server.begin();
}

/********************* MAIN LOOP *******************************/
void loop() {
  server.handleClient();

  // --- read accelerometer ---
  sensors_event_t event;
  accel.getEvent(&event);
  accelData[0] = event.acceleration.x;
  accelData[1] = event.acceleration.y;
  accelData[2] = event.acceleration.z;

  // --- microphone STOP override ---
  if (micDetectsClap()) {
    stopMotors();
    delay(300);
    return;
  }

  // --- line-following ---
  DirectionResult dir = getDirectionForColor(targetColor);
  runMotors(dir.leftMotor, dir.rightMotor);

  delay(20);
}

/********************* LINE FOLLOW LOGIC ************************/
DirectionResult getDirectionForColor(FollowColor color) {
  int bestIndex = -1, bestValue = -1;

  for (int i = 0; i < LED_COUNT; i++) {
    for (int j = 0; j < LED_COUNT; j++)
      ring.setPixelColor(j, (j == i ? ring.Color(
        color == RED ? 255 : 0,
        color == GREEN ? 255 : 0,
        color == BLUE ? 255 : 0
      ) : 0));
    ring.show();
    delay(5);

    int value = analogRead(SENSOR_PIN);
    if (value > bestValue) { bestValue = value; bestIndex = i; }
  }

  int left = 200, right = 200;
  int offset = bestIndex - 0;
  if (offset < 0) offset += LED_COUNT;
  if (offset > 8) right -= (offset - 8) * 18;
  else left -= (8 - offset) * 18;
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  return {bestIndex, bestValue, left, right};
}
