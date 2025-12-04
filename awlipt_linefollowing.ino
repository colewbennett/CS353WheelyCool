#include <Adafruit_NeoPixel.h>

#define LED_PIN    17
#define LED_COUNT  16
#define SENSOR_PIN A0

Adafruit_NeoPixel ring(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

uint32_t scanColors[] = {
  ring.Color(255, 0, 0),    // Red
  ring.Color(0, 255, 0),    // Green
  ring.Color(0, 0, 255)     // Blue
};

const int numColors = 3;

int readings[numColors][LED_COUNT];

int pixelIndex = 0;
int colorIndex = 0;

int smooth(int oldVal, int newVal) {
  return (oldVal * 0.7) + (newVal * 0.3);
}

void setup() {
  Serial.begin(9600);
  ring.begin();
  ring.show();
  ring.setBrightness(80);

  for (int c = 0; c < numColors; c++)
    for (int i = 0; i < LED_COUNT; i++)
      readings[c][i] = 0;
}

void loop() {

  ring.clear();

  ring.setPixelColor(pixelIndex, scanColors[colorIndex]);
  ring.show();

  delay(5);

  int raw = analogRead(SENSOR_PIN);
  readings[colorIndex][pixelIndex] =
      smooth(readings[colorIndex][pixelIndex], raw);

  pixelIndex++;

  if (pixelIndex >= LED_COUNT) {
    pixelIndex = 0;
    colorIndex++;

    if (colorIndex >= numColors) {
      colorIndex = 0;

      analyzeScan();
    }
  }

  delay(20);
}

void analyzeScan() {

  int bestColor = -1;
  int bestValue = -1;

  int bestPixel = -1;

  for (int c = 0; c < numColors; c++) {
    for (int i = 0; i < LED_COUNT; i++) {

      if (readings[c][i] > bestValue) {
        bestValue = readings[c][i];
        bestColor = c;
        bestPixel = i;
      }
    }
  }

  const char* colorName =
      (bestColor == 0 ? "RED" :
       bestColor == 1 ? "GREEN" : "BLUE");

  Serial.print("Strongest Color: ");
  Serial.println(colorName);

  Serial.print("Direction Index: ");
  Serial.println(bestPixel);

  int center = LED_COUNT / 2;

  if (bestPixel < center - 1)
    Serial.println("Move LEFT");
  else if (bestPixel > center + 1)
    Serial.println("Move RIGHT");
  else
    Serial.println("Move STRAIGHT");

  Serial.println("----------------------------");
}
