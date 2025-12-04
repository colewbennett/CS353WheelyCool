/*
  CS353 Final Project Demo
  DFPlayer Mini MP3 Player with HC-SR04 Ultrasonic Sensor Trigger
  
  Plays "0001.mp3" through speaker when an object comes within a set distance.
  The sound will only play once until the object moves away.
  
  Required Library:
    - DFRobotDFPlayerMini by DFRobot
*/

#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

// --- MP3 Module Pins ---
static const uint8_t PIN_MP3_RX = 10; 
static const uint8_t PIN_MP3_TX = 11; 
SoftwareSerial mySoftwareSerial(PIN_MP3_RX, PIN_MP3_TX);
DFRobotDFPlayerMini myDFPlayer;

// --- Ultrasonic Sensor Pins ---
const int trigPin = 7;
const int echoPin = 8;

// --- Settings ---
// The distance in cm at which the sound will trigger
const int thresholdDistance = 20; 

// --- State Variables ---
// This flag prevents the sound from playing over and over
boolean isObjectNear = false; 

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing DFPlayer and Ultrasonic Sensor...");

  // Setup MP3 Module
  mySoftwareSerial.begin(9600);
  if (!myDFPlayer.begin(mySoftwareSerial)) {
    Serial.println("Unable to begin DFPlayer:");
    Serial.println("1. Please recheck the connection!");
    Serial.println("2. Please insert the SD card!");
    while(true);
  }
  myDFPlayer.volume(20); // Set volume (0 to 30)
  Serial.println("DFPlayer Mini online.");

  // Setup Ultrasonic Sensor Pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  Serial.println("System Ready.");
  Serial.print("Sound will trigger when distance is less than ");
  Serial.print(thresholdDistance);
  Serial.println(" cm.");
}

void loop() {
  long duration;
  int distance;

  // --- 1. Get Distance from HC-SR04 ---
  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Set the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance in cm
  // Speed of sound wave = 343 m/s or 0.034 cm/µs
  distance = duration * 0.034 / 2;

  // Print the distance for debugging
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // --- 2. Check if the object is close and if we haven't played the sound yet ---
  if (distance < thresholdDistance && !isObjectNear) {
    // Object has just entered the range!
    Serial.println("Object detected! Playing sound...");
    myDFPlayer.play(1); // Play the first file (0001.mp3)
    
    // Set the flag to true to prevent re-triggering
    isObjectNear = true; 
  }
  
  // --- 3. Check if the object has moved away ---
  else if (distance >= thresholdDistance && isObjectNear) {
    // Object has moved out of range, so we can reset the flag
    Serial.println("Object moved away. Ready for next detection.");
    isObjectNear = false;
  }

  // --- 4. Check for status messages from the DFPlayer ---
  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read());
  }

  // A small delay to keep the serial monitor from flooding
  delay(10); 
}

// --- Helper Function for DFPlayer Details ---
void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}