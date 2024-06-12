#include <Adafruit_NeoPixel.h>

#define LED_PIN    6
#define LED_COUNT  1

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  // Turn on the LED
  strip.setPixelColor(0, 255, 255, 255); // White color, you can change these values for different colors
  strip.show();

  delay(10000); // Keep the LED on for 10 seconds

  // Turn off the LED
  strip.setPixelColor(0, 0, 0, 0); // Turn off the LED
  strip.show();

  delay(1000); // Wait for a second before repeating the loop
}
