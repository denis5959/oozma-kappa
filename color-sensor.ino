#include <Wire.h>
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 color_sensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Calibra estos dos valores con tu sensor (ver pasos abajo)
const uint16_t BLACK_CLEAR_MAX = 300;   // clear por debajo de esto = negro
const uint16_t WHITE_CLEAR_MIN = 2000;  // clear por encima de esto = blanco

enum DominantChannel { RED_DOMINANT, GREEN_DOMINANT, BLUE_DOMINANT };

DominantChannel get_dominant_channel(float red_value, float green_value, float blue_value) {
  if (red_value >= green_value && red_value >= blue_value) return RED_DOMINANT;
  if (green_value >= red_value && green_value >= blue_value) return GREEN_DOMINANT;
  return BLUE_DOMINANT;
}

const char* classify_red(float red_value, float green_value, float blue_value) {
  if (blue_value > 0.4 * red_value && blue_value > 0.6 * green_value) return "Pink";
  if (green_value > 0.7 * red_value) return "Yellow";
  if (green_value > 0.4 * red_value) return "Orange";
  return "Red";
}

const char* classify_green(float red_value, float green_value, float blue_value) {
  if (red_value > 0.75 * green_value) return "Yellow";
  if (blue_value > 0.75 * green_value) return "Cyan";
  return "Green";
}

const char* classify_blue(float red_value, float green_value, float blue_value) {
  if (red_value > 0.75 * blue_value) return "Purple / Magenta";
  if (green_value > 0.75 * blue_value) return "Cyan";
  return "Blue";
}

// Turns RGB proportions (0-255) + brightness into a simple color name
const char* get_color_name(float red_value, float green_value, float blue_value, uint16_t clear_value) {
  float highest_channel = max(red_value, max(green_value, blue_value));
  float lowest_channel  = min(red_value, min(green_value, blue_value));
  float channel_spread  = highest_channel - lowest_channel;

  if (clear_value < BLACK_CLEAR_MAX) return "Black / Very dark";
  if (channel_spread < 40) {
    if (clear_value > WHITE_CLEAR_MIN) return "White";
    return "Gray";
  }

  switch (get_dominant_channel(red_value, green_value, blue_value)) {
    case RED_DOMINANT:   return classify_red(red_value, green_value, blue_value);
    case GREEN_DOMINANT: return classify_green(red_value, green_value, blue_value);
    case BLUE_DOMINANT:  return classify_blue(red_value, green_value, blue_value);
  }
  return "Unknown";
}

void read_color(float* red, float* green, float* blue, uint16_t* clear) {
  uint16_t raw_red, raw_green, raw_blue;

  color_sensor.setInterrupt(false);  // turn on the sensor's onboard light
  delay(60);                         // takes 50ms to read
  color_sensor.getRawData(&raw_red, &raw_green, &raw_blue, clear);
  color_sensor.setInterrupt(true);   // turn off the sensor's onboard light

  if (*clear == 0) {                 // avoid dividing by zero
    *red = 0; *green = 0; *blue = 0;
    return;
  }
  *red   = (float)raw_red   / *clear * 255.0;
  *green = (float)raw_green / *clear * 255.0;
  *blue  = (float)raw_blue  / *clear * 255.0;
}

void print_color(float red, float green, float blue, uint16_t clear) {
  Serial.print("R: "); Serial.print((int)red);
  Serial.print("\tG: "); Serial.print((int)green);
  Serial.print("\tB: "); Serial.print((int)blue);
  Serial.print("\tC: "); Serial.print(clear);
  Serial.print("\t-> ");
  Serial.println(get_color_name(red, green, blue, clear));
}

void init_sensor() {
  if (color_sensor.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
}

void setup() {
  Serial.begin(9600);
  init_sensor();
}

void loop() {
  float red, green, blue;
  uint16_t clear;

  read_color(&red, &green, &blue, &clear);
  print_color(red, green, blue, clear);

  delay(500);  // slow down output so it's readable
}
