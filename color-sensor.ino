

#include <Wire.h>
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Turns RGB values (0-255) into a simple color name
const char* colorName(float r, float g, float b) {
  float mx = max(r, max(g, b));
  float mn = min(r, min(g, b));
  float spread = mx - mn;

  if (mx < 40) return "Black / Very dark";
  if (spread < 30) {
    if (mx > 200) return "White";
    return "Gray";
  }

  if (r >= g && r >= b) {            // red is dominant
    if (b > 0.4 * r && b > 0.6 * g) return "Pink";
    if (g > 0.7 * r) return "Yellow";
    if (g > 0.4 * r) return "Orange";
    return "Red";
  } else if (g >= r && g >= b) {     // green is dominant
    if (r > 0.75 * g) return "Yellow";
    if (b > 0.75 * g) return "Cyan";
    return "Green";
  } else {                           // blue is dominant
    if (r > 0.75 * b) return "Purple / Magenta";
    if (g > 0.75 * b) return "Cyan";
    return "Blue";
  }
}

void setup() {
  Serial.begin(9600);

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
}

void loop() {
  float red, green, blue;

  tcs.setInterrupt(false);  // turn on the sensor's onboard light
  delay(60);                // takes 50ms to read
  tcs.getRGB(&red, &green, &blue);
  tcs.setInterrupt(true);   // turn off the sensor's onboard light

  Serial.print("R: "); Serial.print((int)red);
  Serial.print("\tG: "); Serial.print((int)green);
  Serial.print("\tB: "); Serial.print((int)blue);
  Serial.print("\t-> ");
  Serial.println(colorName(red, green, blue));

  delay(500);  // slow down output so it's readable
}
