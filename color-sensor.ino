#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_TCS34725.h"

// - configuración para la pantalla oled -
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);

// - sensor de color TCS34725 (direccion 0x29, no choca con la oled) -
Adafruit_TCS34725 color_sensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Calibra este valor con tu sensor
const uint16_t BLACK_CLEAR_MAX = 300;   // clear por debajo de esto = negro

enum DominantChannel { RED_DOMINANT, GREEN_DOMINANT, BLUE_DOMINANT };

DominantChannel get_dominant_channel(float red_value, float green_value, float blue_value) {
  if (red_value >= green_value && red_value >= blue_value) return RED_DOMINANT;
  if (green_value >= red_value && green_value >= blue_value) return GREEN_DOMINANT;
  return BLUE_DOMINANT;
}

const char* classify_red(float red_value, float green_value, float blue_value) {
  if (blue_value > 0.4 * red_value && blue_value > 0.6 * green_value) return "Pink";
  if (green_value > 0.7 * red_value) return "Yellow";
  if (green_value > 0.275 * red_value) return "Orange";  // rojo da ~0.25, naranja ~0.30, corte en medio
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
  if (channel_spread < 45) return "White";  // sin gris, tu blanco da ~32

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

void print_color(float red, float green, float blue, uint16_t clear, const char* color_name) {
  Serial.print(F("R: "));   Serial.print((int)red);
  Serial.print(F("\tG: ")); Serial.print((int)green);
  Serial.print(F("\tB: ")); Serial.print((int)blue);
  Serial.print(F("\tC: ")); Serial.print(clear);
  Serial.print(F("\t-> "));
  Serial.println(color_name);
}

// - funciones de la oled -

// barrita horizontal para cada canal (0-255 -> 0-ancho)
void dibujarBarra(int y, char etiqueta, float valor) {
  const int x_barra = 10;
  const int ancho_max = 80;

  display.setCursor(0, y);
  display.print(etiqueta);

  int ancho = (int)(constrain(valor, 0, 255) / 255.0 * ancho_max);
  display.drawRect(x_barra, y, ancho_max, 6, SSD1306_WHITE);
  display.fillRect(x_barra, y, ancho, 6, SSD1306_WHITE);

  display.setCursor(x_barra + ancho_max + 4, y);
  display.print((int)valor);
}

void mostrarOLED(float red, float green, float blue, uint16_t clear, const char* color_name) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // titulo
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("color :D   C:"));
  display.print(clear);

  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  // nombre del color: grande si cabe, chico si es largo
  // (tamaño 2 = 12 px por letra -> caben 10 letras en 128 px)
  if (strlen(color_name) <= 10) {
    display.setTextSize(2);
  } else {
    display.setTextSize(1);
  }
  display.setCursor(0, 15);
  display.println(color_name);

  // barras de R, G y B
  display.setTextSize(1);
  dibujarBarra(36, 'R', red);
  dibujarBarra(46, 'G', green);
  dibujarBarra(56, 'B', blue);

  display.display();
}

void mostrarErrorOLED(const __FlashStringHelper *mensaje) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 20);
  display.println(F("ERROR:"));

  display.setCursor(0, 35);
  display.println(mensaje);

  display.display();
}

void init_sensor() {
  if (color_sensor.begin()) {
    Serial.println(F("Found sensor"));
  } else {
    Serial.println(F("No TCS34725 found ... check your connections"));
    mostrarErrorOLED(F("No hay TCS34725"));
    while (1) {
      delay(100); // halt!
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // primero la oled, para poder mostrar errores del sensor en pantalla
  if (!display.begin(
        SSD1306_SWITCHCAPVCC,
        OLED_ADDRESS
      )) {

    Serial.println(F("No se encontro OLED"));

    while (true) {
      delay(100);
    }
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.println(F("Iniciando..."));
  display.display();

  init_sensor();

  delay(500);
}

void loop() {
  float red, green, blue;
  uint16_t clear;

  read_color(&red, &green, &blue, &clear);

  const char* color_name = get_color_name(red, green, blue, clear);

  print_color(red, green, blue, clear, color_name);
  mostrarOLED(red, green, blue, clear, color_name);

  delay(500);  // slow down output so it's readable
}
// falta checar el cyan y el rosa pero en la cancha real
