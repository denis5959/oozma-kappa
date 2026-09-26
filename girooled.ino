#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//configuración para la pantalla oled
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

//configuración para el giroscopio MPU-6050

const byte MPU = 0x68;//dirección del giroscopio

//calibración al giroscopio al frente del carrito
const float OFFSET_GX = -325.397;
const float OFFSET_GY = -102.176;
const float OFFSET_GZ = -68.553;


const float SENSIBILIDAD_GIRO = 131.0;

//lo agarré del otro código
const float ZONA_MUERTA_DPS = 0.20;

//ángulo acumulado
float anguloX = 0.0;
float anguloY = 0.0;
float anguloZ = 0.0;

//variables para pode rmedir el tiempo
unsigned long tiempoAnterior = 0;
unsigned long tiempoAnteriorOLED = 0;

// funciones del giroscopio

void escribirRegistro(byte registro, byte valor) {
  Wire.beginTransmission(MPU);
  Wire.write(registro);
  Wire.write(valor);
  Wire.endTransmission(true);
}

void configurarMPU6050() {
  // Despierta el sensor.
  escribirRegistro(0x6B, 0x00);

  // Configura giroscopio en ±250 °/s.
  escribirRegistro(0x1B, 0x00);

  delay(100);
}

bool leerGiroscopioRaw(int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU);
  Wire.write(0x43);

  byte error = Wire.endTransmission(false);

  if (error != 0) {
    return false;
  }

  byte recibidos = Wire.requestFrom(MPU, (byte)6, (byte)true);

  if (recibidos != 6) {
    return false;
  }

  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();

  return true;
}

float aplicarZonaMuerta(float velocidadDps) {
  if (velocidadDps > -ZONA_MUERTA_DPS &&
      velocidadDps < ZONA_MUERTA_DPS) {
    return 0.0;
  }

  return velocidadDps;
}

//funciones de la oled

void mostrarOLED(float gradosZ, float velocidadZ) {
  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("GIRO REAL DEL CARRITO"));

  display.drawLine(0, 11, 127, 11, SSD1306_WHITE);

  //el sensor lo llama z
  // yo lo rebrandee como giro xd
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.print(F("Giro: "));
  display.print(gradosZ, 1);

  display.setTextSize(1);
  display.setCursor(0, 47);
  display.print(F("grados (eje Z)"));

  display.setCursor(0, 56);
  display.print(F("Vel: "));
  display.print(velocidadZ, 2);
  display.print(F(" deg/s"));

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

void setup() {
  Serial.begin(115200);

  //inicia el bus i2c
  Wire.begin();

  // inicia  pantalla oled
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("No se encontro el OLED"));

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

  configurarMPU6050();

  Serial.println(F("MPU-6050 iniciado"));
  Serial.println(F("OLED iniciado"));
  Serial.println(F("Eje Z = giro real del carrito"));

  tiempoAnterior = micros();

  delay(800);
}


void loop() {
  int16_t gxRaw;
  int16_t gyRaw;
  int16_t gzRaw;

  if (!leerGiroscopioRaw(gxRaw, gyRaw, gzRaw)) {
    Serial.println(F("Error leyendo MPU-6050"));
    mostrarErrorOLED(F("MPU-6050"));

    delay(500);
    return;
  }

  //aplicar los offsets y convertir de raw a °/s.
  float gxDps = (gxRaw - OFFSET_GX) / SENSIBILIDAD_GIRO;
  float gyDps = (gyRaw - OFFSET_GY) / SENSIBILIDAD_GIRO;
  float gzDps = (gzRaw - OFFSET_GZ) / SENSIBILIDAD_GIRO;

  //incluir mi zona muerta
  gxDps = aplicarZonaMuerta(gxDps);
  gyDps = aplicarZonaMuerta(gyDps);
  gzDps = aplicarZonaMuerta(gzDps);

  //tiempo entre cada lectura
  unsigned long tiempoActual = micros();
  float dt = (tiempoActual - tiempoAnterior) / 1000000.0;
  tiempoAnterior = tiempoActual;

  //velocidades para acumular ángulos
  anguloX += gxDps * dt;
  anguloY += gyDps * dt;
  anguloZ += gzDps * dt;

  //mostrar en monitor serie también
  Serial.print(F("X: "));
  Serial.print(anguloX, 2);
  Serial.print(F("°\tY: "));
  Serial.print(anguloY, 2);
  Serial.print(F("°\tZ: "));
  Serial.print(anguloZ, 2);

  Serial.print(F("\t| Velocidad Z: "));
  Serial.print(gzDps, 2);
  Serial.println(F(" °/s"));

  //actualiza el OLED 10 veces cada segundo
  if (millis() - tiempoAnteriorOLED >= 100) {
    tiempoAnteriorOLED = millis();

    //el giro de mi carrito
    mostrarOLED(anguloZ, gzDps);
  }

  delay(20);
}
