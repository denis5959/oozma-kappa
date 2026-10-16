#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_TCS34725.h"

// - Configuración para la pantalla OLED y multiplexor -
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C
#define TCA_ADDR 0x70

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);

// - Dos sensores de color TCS34725 en el multiplexor -
Adafruit_TCS34725 tcs1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 tcs2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Definición del tipo de dato movida arriba para evitar el error del preprocesador
enum DominantChannel { RED_DOMINANT, GREEN_DOMINANT, BLUE_DOMINANT };

// Calibra este valor con tu sensor
const uint16_t BLACK_CLEAR_MAX = 300;   // clear por debajo de esto = negro

// Umbral para separar rojo de naranja (calibrado con tus lecturas)
// Rojo da ~0.06-0.08, naranja da ~0.24-0.28
const float ORANGE_MIN_RATIO = 0.16;    // (G - B) / R

// --- NUEVO: Umbrales para separar verde de amarillo cuando domina el verde ---
// Tus lecturas de verde: R/G = 0.57 - 0.79, B/G = 0.51 - 0.60
// El amarillo real tiene mucho rojo Y poco azul.
const float YELLOW_RG_STRONG = 0.85;    // R/G mayor a esto = amarillo seguro
const float YELLOW_RG_WEAK   = 0.75;    // R/G entre WEAK y STRONG = amarillo solo si hay poco azul
const float YELLOW_BG_MAX    = 0.50;    // B/G menor a esto = poco azul (típico del amarillo)

// Función para seleccionar el canal del multiplexor PCA9548A
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

DominantChannel get_dominant_channel(float red_value, float green_value, float blue_value) {
  if (red_value >= green_value && red_value >= blue_value) return RED_DOMINANT;
  if (green_value >= red_value && green_value >= blue_value) return GREEN_DOMINANT;
  return BLUE_DOMINANT;
}

const char* classify_red(float red_value, float green_value, float blue_value) {
  if (blue_value > 0.4 * red_value && blue_value > 0.6 * green_value) return "Pink";
  if (green_value > 0.7 * red_value) return "Yellow";

  // Qué tanto sobresale el verde sobre el azul, relativo al rojo
  float orange_ratio = (green_value - blue_value) / red_value;
  if (orange_ratio > ORANGE_MIN_RATIO) return "Orange";

  return "Red";
}

// --- MODIFICADO: ahora usa rojo Y azul para decidir entre verde y amarillo ---
const char* classify_green(float red_value, float green_value, float blue_value) {
  float rg = red_value  / green_value;   // cuánto rojo hay respecto al verde
  float bg = blue_value / green_value;   // cuánto azul hay respecto al verde

  // Amarillo claro: casi tanto rojo como verde
  if (rg > YELLOW_RG_STRONG) return "Yellow";

  // Zona dudosa: solo es amarillo si además tiene poco azul
  if (rg > YELLOW_RG_WEAK && bg < YELLOW_BG_MAX) return "Yellow";

  if (blue_value > 0.75 * green_value) return "Cyan";
  return "Green";
}

const char* classify_blue(float red_value, float green_value, float blue_value) {
  if (red_value > 0.75 * blue_value) return "Purple / Magenta";
  if (green_value > 0.75 * blue_value) return "Cyan";
  return "Blue";
}

const char* get_color_name(float red_value, float green_value, float blue_value, uint16_t clear_value) {
  float highest_channel = max(red_value, max(green_value, blue_value));
  float lowest_channel  = min(red_value, min(green_value, blue_value));
  float channel_spread  = highest_channel - lowest_channel;

  if (clear_value < BLACK_CLEAR_MAX) return "Black / Dark";
  if (channel_spread < 45) return "White"; 

  switch (get_dominant_channel(red_value, green_value, blue_value)) {
    case RED_DOMINANT:   return classify_red(red_value, green_value, blue_value);
    case GREEN_DOMINANT: return classify_green(red_value, green_value, blue_value);
    case BLUE_DOMINANT:  return classify_blue(red_value, green_value, blue_value);
  }
  return "Unknown";
}

void read_color(Adafruit_TCS34725 &sensor, uint8_t canal, float* red, float* green, float* blue, uint16_t* clear) {
  tcaselect(canal);
  uint16_t raw_red, raw_green, raw_blue;

  sensor.setInterrupt(false);  // encender luz del sensor
  delay(60);                   // espera de lectura (50ms)
  sensor.getRawData(&raw_red, &raw_green, &raw_blue, clear);
  sensor.setInterrupt(true);   // apagar luz del sensor

  if (*clear == 0) {           
    *red = 0; *green = 0; *blue = 0;
    return;
  }
  *red   = (float)raw_red   / *clear * 255.0;
  *green = (float)raw_green / *clear * 255.0;
  *blue  = (float)raw_blue  / *clear * 255.0;
}

// --- NUEVO: imprime valores y proporciones por serie para calibrar ---
void imprimirDebug(const char* etiqueta, const char* nombre, float r, float g, float b, uint16_t c) {
  Serial.print(etiqueta);
  Serial.print(F(" -> "));
  Serial.print(nombre);
  Serial.print(F("  R:")); Serial.print((int)r);
  Serial.print(F(" G:"));  Serial.print((int)g);
  Serial.print(F(" B:"));  Serial.print((int)b);
  Serial.print(F(" C:"));  Serial.print(c);
  if (g > 0) {
    Serial.print(F("  R/G:")); Serial.print(r / g, 2);
    Serial.print(F(" B/G:"));  Serial.print(b / g, 2);
  }
  Serial.println();
}

// - Funciones de la OLED (Canal 4) -
void mostrarOLED(const char* name1, float r1, float g1, float b1, const char* name2, float r2, float g2, float b2) {
  tcaselect(4); // Seleccionar canal de la OLED
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Sensor 1 (Canal 2)
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("S1(C2): ")); display.println(name1);
  display.setCursor(0, 9);
  display.print(F("R:")); display.print((int)r1);
  display.print(F(" G:")); display.print((int)g1);
  display.print(F(" B:")); display.print((int)b1);

  display.drawLine(0, 29, 127, 29, SSD1306_WHITE);

  // Sensor 2 (Canal 3)
  display.setCursor(0, 34);
  display.print(F("S2(C3): ")); display.println(name2);
  display.setCursor(0, 43);
  display.print(F("R:")); display.print((int)r2);
  display.print(F(" G:")); display.print((int)g2);
  display.print(F(" B:")); display.print((int)b2);

  display.display();
}

void mostrarErrorOLED(const __FlashStringHelper *mensaje) {
  tcaselect(4);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.println(F("ERROR:"));
  display.setCursor(0, 35);
  display.println(mensaje);
  display.display();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Inicializar OLED en Canal 4
  tcaselect(4);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("No se encontro OLED en canal 4"));
    while (true) { delay(100); }
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.println(F("Iniciando sensores..."));
  display.display();

  // Inicializar Sensor 1 en Canal 2
  tcaselect(2);
  if (tcs1.begin()) {
    Serial.println(F("Sensor 1 encontrado en canal 2"));
  } else {
    Serial.println(F("No se encontro Sensor 1"));
    mostrarErrorOLED(F("Falta Sensor 1 (C2)"));
    while (1) { delay(100); }
  }

  // Inicializar Sensor 2 en Canal 3
  tcaselect(3);
  if (tcs2.begin()) {
    Serial.println(F("Sensor 2 encontrado en canal 3"));
  } else {
    Serial.println(F("No se encontro Sensor 2"));
    mostrarErrorOLED(F("Falta Sensor 2 (C3)"));
    while (1) { delay(100); }
  }

  delay(500);
}

void loop() {
  float r1, g1, b1, r2, g2, b2;
  uint16_t clear1, clear2;

  // Leer Sensor 1 (Canal 2)
  read_color(tcs1, 2, &r1, &g1, &b1, &clear1);
  const char* name1 = get_color_name(r1, g1, b1, clear1);

  // Leer Sensor 2 (Canal 3)
  read_color(tcs2, 3, &r2, &g2, &b2, &clear2);
  const char* name2 = get_color_name(r2, g2, b2, clear2);

  // Imprimir por puerto serie (con valores y proporciones para calibrar)
  imprimirDebug("S1", name1, r1, g1, b1, clear1);
  imprimirDebug("S2", name2, r2, g2, b2, clear2);

  // Mostrar ambos en la OLED (Canal 4)
  mostrarOLED(name1, r1, g1, b1, name2, r2, g2, b2);

  delay(500); 
}
