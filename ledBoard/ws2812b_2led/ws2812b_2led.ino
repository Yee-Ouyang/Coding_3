#include <Adafruit_NeoPixel.h>

#define LED_PIN1    6
#define LED_PIN2    7
#define LED_COUNT  1

Adafruit_NeoPixel strip1(LED_COUNT, LED_PIN1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(LED_COUNT, LED_PIN2, NEO_GRB + NEO_KHZ800);

int ldrPin = A0;  // LDR传感器接口

void setup() {
  Serial.begin(9600);
  strip1.begin();
  strip1.show(); // Initialize all pixels to 'off'
  strip2.begin();
  strip2.show(); // Initialize all pixels to 'off'
  pinMode(ldrPin, INPUT); //光敏
}

void loop() {
  // Turn on the LED
  strip1.setPixelColor(0, 0, 255, 0); // White color, you can change these values for different colors
  strip1.show();
  
  strip2.setPixelColor(0, 0, 255, 0); // White color, you can change these values for different colors
  strip2.show();

  //delay(10000); // Keep the LED on for 10 seconds
  for (int i = 0; i < 10; i++) { // 10 times for 10 seconds
    delay(1000); // Wait for 1 second

  // Turn off the LED
  //strip1.setPixelColor(0, 0, 0, 0); // Turn off the LED
  //strip1.show();

  //delay(1000); // Wait for a second before repeating the loop

  //strip2.setPixelColor(0, 0, 0, 0); // Turn off the LED
  //strip2.show();

   // 读取LDR值
  int ldrValue = analogRead(ldrPin);
  Serial.print("LDR Value: ");
  Serial.println(ldrValue);
  }
}
