/*
 * This Arduino Nano ESP32 code was developed by newbiely.com
 *
 * This Arduino Nano ESP32 code is made available for public use without any restriction
 *
 * For comprehensive instructions and wiring diagrams, please visit:
 * https://newbiely.com/tutorials/arduino-nano-esp32/arduino-nano-esp32-web-server
 */

#include "index.h"

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <DIYables_ESP32_WebServer.h>

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

// A small helper
void error(const __FlashStringHelper* err) {
  Serial.println(err);
  while (1);
}

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

void displayMagSensorDetails(void) {
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       "); Serial.println(sensor.name);
  Serial.print("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id); 
  Serial.print("Max Value:    "); Serial.print(sensor.max_value);   Serial.println(" uT");
  Serial.print("Min Value:    "); Serial.print(sensor.min_value);   Serial.println(" uT");
  Serial.print("Resolution:   "); Serial.print(sensor.resolution);  Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayAccelSensorDetails(void) {
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       "); Serial.println(sensor.name);
  Serial.print("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    "); Serial.print(sensor.max_value);   Serial.println(" m/s^2");
  Serial.print("Min Value:    "); Serial.print(sensor.min_value);   Serial.println(" m/s^2");
  Serial.print("Resolution:   "); Serial.print(sensor.resolution);  Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

// WiFi credentials
// const char WIFI_SSID[] = "YOUR_WIFI_SSID";
// const char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";
const char WIFI_SSID[] = "";
const char WIFI_PASSWORD[] = "";

// Create web server instance
DIYables_ESP32_WebServer server;

// data from accelerometer/magnetometer
float data[] = {0, 0, 0, 0, 0, 0};

// Page handlers
void handleHome(WiFiClient& client, const String& method, const String& request, const QueryParams& params, const String& jsonData) {
  String html = HTML_CONTENT;

  html.replace("%DATA0%", String(data[0], 1));
  html.replace("%DATA1%", String(data[1], 1));
  html.replace("%DATA2%", String(data[2], 1));
  html.replace("%DATA3%", String(data[3], 1));
  html.replace("%DATA4%", String(data[4], 1));
  html.replace("%DATA5%", String(data[5], 1));

  server.sendResponse(client, html.c_str());
}

void setup() {
  Serial.begin(9600);
  delay(1000);

  #ifndef ESP8266
    while (!Serial);  // required for Flora & Micro
  #endif
  delay(500);

  Serial.println();

  Serial.println("Magnetometer Test");
  Serial.println("");

  /* Enable auto-gain */
  mag.enableAutoRange(true);

  /* Initialise the sensor */
  if (!mag.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }

  /* Display some basic information on this sensor */
  displayMagSensorDetails();

  Serial.println("Accelerometer Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!accel.begin()) {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }

  /* Display some basic information on this sensor */
  displayAccelSensorDetails();
  
  Serial.println("Arduino Nano ESP32 Web Server");

  // Configure web server routes
  server.addRoute("/", handleHome);

  // Start web server with WiFi connection
  server.begin(WIFI_SSID, WIFI_PASSWORD);
}

void loop() {
  /* Get a new sensor event */
  sensors_event_t magEvent;
  sensors_event_t accEvent;
  mag.getEvent(&magEvent);
  accel.getEvent(&accEvent);

  // update data
  data[0] = accEvent.magnetic.x;
  data[1] = accEvent.magnetic.y;
  data[2] = accEvent.magnetic.z;
  data[3] = magEvent.acceleration.x;
  data[4] = magEvent.acceleration.y;
  data[5] = magEvent.acceleration.z;

  // Serial.print("Mag:");
  // Serial.print('\t');
  // Serial.print(magEvent.magnetic.x);
  // Serial.print('\t');
  // Serial.print(magEvent.magnetic.y);
  // Serial.print('\t');
  // Serial.println(magEvent.magnetic.z);

  // Serial.print("Acc:");
  // Serial.print('\t');
  // Serial.print(accEvent.acceleration.x);
  // Serial.print('\t');
  // Serial.print(accEvent.acceleration.y);
  // Serial.print('\t');
  // Serial.println(accEvent.acceleration.z);

  server.handleClient();
}

/**************************************************************************/
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/
void getUserInput(char buffer[], uint8_t maxSize) {
  memset(buffer, 0, maxSize);
  while (Serial.available() == 0) {
    delay(1);
  }

  uint8_t count = 0;

  do {
    count += Serial.readBytes(buffer + count, maxSize);
    delay(2);
  } while ((count < maxSize) && !(Serial.available() == 0));
}
