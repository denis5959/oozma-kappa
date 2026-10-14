#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Configuración para la pantalla OLED
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

// Configuración para el giroscopio MPU-9250 / MPU-6050
const byte MPU = 0x68; // AD0 a GND

// Nuevos offsets de calibración
const float OFFSET_GX = 233.46;
const float OFFSET_GY = 106.81;
const float OFFSET_GZ = 209.57;

const float OFFSET_AX = 364.628;
const float OFFSET_AY = -90.550;
const float OFFSET_AZ = -19.849;

const float SENSIBILIDAD_GIRO = 131.0; // Rango ±250 °/s

// Zona muerta para reducir el drift
const float ZONA_MUERTA_DPS = 0.20;

// Ángulo acumulado
float angulo_x = 0.0;
float angulo_y = 0.0;
float angulo_z = 0.0;

// Variables para medir el tiempo
unsigned long tiempo_anterior = 0;
unsigned long tiempo_anterior_oled = 0;

// ==========================================
// - MOTORES 4x (DOBLE PUENTE H L298N) -
// ==========================================

// Puente H Delantero
const int IN1_DEL_DER = 53;
const int IN2_DEL_DER = 52;
const int ENA_DEL_DER = 4;   // PWM

const int IN3_DEL_IZQ = 51;
const int IN4_DEL_IZQ = 50;
const int ENB_DEL_IZQ = 5;   // PWM

// Puente H Trasero
const int IN1_TRA_DER = 22;
const int IN2_TRA_DER = 23;
const int ENA_TRA_DER = 2;   // PWM

const int IN3_TRA_IZQ = 24;
const int IN4_TRA_IZQ = 25;
const int ENB_TRA_IZQ = 3;   // PWM

// ==========================================
// - BANDERAS DE INVERSIÓN DE MOTORES -
// Cambia a 'true' si alguna rueda gira al revés
// ==========================================
const bool INV_DEL_DER = true;
const bool INV_DEL_IZQ = true;
const bool INV_TRA_DER = false;
const bool INV_TRA_IZQ = false;

// - PID -
float KP = 6.9;
float KI = 0.34;
float KD = 0.025;

// Límites del PWM
const int VELOCIDAD_MAXIMA = 130;
const int VELOCIDAD_MINIMA = 55;

// Sensibilidad y frenadoA
const float TOLERANCIA_GRADOS = 2.0;
const float ZONA_FRENADO_GRADOS = 12.0;

// - Rumbos de prueba -
const float RUMBOS[] = {
  0.0,
  90.0,
  180.0,
  270.0
};

const byte NUM_RUMBOS = 4;
byte indice_rumbo = 0;
float angulo_objetivo = 0.0;

// Cambia de rumbo cada 10 segundos
const unsigned long TIEMPO_RUMBO_MS = 3000;
unsigned long tiempo_inicio_rumbo = 0;

// - Memoria del PID -
float error_anterior_pid = 0.0;
float integral_error_pid = 0.0;
unsigned long tiempo_anterior_pid = 0;

// - Funciones del Giroscopio -
void escribirRegistro(byte registro, byte valor) {
  Wire.beginTransmission(MPU);
  Wire.write(registro);
  Wire.write(valor);
  Wire.endTransmission(true);
}

void configurarMPU6050() {
  escribirRegistro(0x6B, 0x00); // Despierta el sensor
  escribirRegistro(0x1B, 0x00); // Giroscopio en ±250 °/s
  delay(100);
}

bool leerGiroscopioRaw(int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU);
  Wire.write(0x43);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(MPU, (byte)6, (byte)true) != 6) {
    return false;
  }

  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();

  return true;
}

float aplicarZonaMuerta(float velocidad_dps) {
  if (velocidad_dps > -ZONA_MUERTA_DPS && velocidad_dps < ZONA_MUERTA_DPS) {
    return 0.0;
  }
  return velocidad_dps;
}

// - Ángulos -
float normalizar360(float angulo) {
  while (angulo >= 360.0) angulo -= 360.0;
  while (angulo < 0.0) angulo += 360.0;
  return angulo;
}

float calcularErrorAngular(float objetivo, float actual) {
  float error = normalizar360(objetivo) - normalizar360(actual);
  if (error > 180.0) error -= 360.0;
  if (error < -180.0) error += 360.0;
  return error;
}

// ==========================================
// - CONTROL INDIVIDUAL DE MOTORES CON INVERSIÓN -
// ==========================================

void enviarMotor(int in1, int in2, int ena, int velocidad, bool invertido) {
  if (invertido) {
    velocidad = -velocidad;
  }
  
  velocidad = constrain(velocidad, -255, 255);

  if (velocidad > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (velocidad < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  analogWrite(ena, abs(velocidad));
}

void setLadoIzquierdo(int velocidad) {
  enviarMotor(IN3_DEL_IZQ, IN4_DEL_IZQ, ENB_DEL_IZQ, velocidad, INV_DEL_IZQ);
  enviarMotor(IN3_TRA_IZQ, IN4_TRA_IZQ, ENB_TRA_IZQ, velocidad, INV_TRA_IZQ);
}

void setLadoDerecho(int velocidad) {
  enviarMotor(IN1_DEL_DER, IN2_DEL_DER, ENA_DEL_DER, velocidad, INV_DEL_DER);
  enviarMotor(IN1_TRA_DER, IN2_TRA_DER, ENA_TRA_DER, velocidad, INV_TRA_DER);
}

void detenerMotores() {
  setLadoIzquierdo(0);
  setLadoDerecho(0);
}

// - PID -
void reiniciarPID() {
  error_anterior_pid = 0.0;
  integral_error_pid = 0.0;
  tiempo_anterior_pid = millis();
}

void actualizarControlRumboPID() {
  float error = calcularErrorAngular(angulo_objetivo, angulo_z);

  unsigned long ahora = millis();
  float dt_pid = (ahora - tiempo_anterior_pid) / 1000.0;

  if (dt_pid <= 0.0) dt_pid = 0.001;
  tiempo_anterior_pid = ahora;

  if (abs(error) <= TOLERANCIA_GRADOS) {
    detenerMotores();
    integral_error_pid = 0.0;
    error_anterior_pid = error;
    return;
  }

  if (abs(error) < 25.0) {
    integral_error_pid += error * dt_pid;
  } else {
    integral_error_pid = 0.0;
  }

  integral_error_pid = constrain(integral_error_pid, -80.0, 80.0);

  float derivada = (error - error_anterior_pid) / dt_pid;
  error_anterior_pid = error;

  float salida = (KP * error) + (KI * integral_error_pid) + (KD * derivada);

  float proporcion = abs(error) / ZONA_FRENADO_GRADOS;
  proporcion = constrain(proporcion, 0.0, 1.0);

  float limite_pwm = VELOCIDAD_MINIMA + proporcion * (VELOCIDAD_MAXIMA - VELOCIDAD_MINIMA);

  salida = constrain(salida, -limite_pwm, limite_pwm);

  if (abs(salida) < VELOCIDAD_MINIMA) {
    salida = (salida >= 0) ? VELOCIDAD_MINIMA : -VELOCIDAD_MINIMA;
  }

  // Giro sobre su propio eje (lados opuestos)
  setLadoIzquierdo(-(int)salida);
  setLadoDerecho((int)salida);
}

void siguienteRumbo() {
  indice_rumbo++;
  if (indice_rumbo >= NUM_RUMBOS) {
    indice_rumbo = 0;
  }
  angulo_objetivo = RUMBOS[indice_rumbo];
  reiniciarPID();

  Serial.print(F("NUEVO OBJETIVO: "));
  Serial.print(angulo_objetivo, 1);
  Serial.println(F(" grados"));
}

// - OLED -
void mostrarOLED(float grados_z, float velocidad_z) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("giro carrito 4x :D"));

  display.drawLine(0, 11, 127, 11, SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(0, 18);
  display.print(F("R: "));
  display.print(normalizar360(grados_z), 1);

  display.setTextSize(1);
  display.setCursor(0, 40);
  display.print(F("Objetivo: "));
  display.print(angulo_objetivo, 0);
  display.println(F(" deg"));

  display.setCursor(0, 52);
  display.print(F("Z: "));
  display.print(velocidad_z, 2);
  display.print(F(" d/s"));

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
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("No se encontro OLED"));
    while (true) { delay(100); }
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.println(F("Iniciando con offsets..."));
  display.display();

  configurarMPU6050();

  // Configurar pines de los motores delanteros
  pinMode(IN1_DEL_DER, OUTPUT);
  pinMode(IN2_DEL_DER, OUTPUT);
  pinMode(ENA_DEL_DER, OUTPUT);
  pinMode(IN3_DEL_IZQ, OUTPUT);
  pinMode(IN4_DEL_IZQ, OUTPUT);
  pinMode(ENB_DEL_IZQ, OUTPUT);

  // Configurar pines de los motores traseros
  pinMode(IN1_TRA_DER, OUTPUT);
  pinMode(IN2_TRA_DER, OUTPUT);
  pinMode(ENA_TRA_DER, OUTPUT);
  pinMode(IN3_TRA_IZQ, OUTPUT);
  pinMode(IN4_TRA_IZQ, OUTPUT);
  pinMode(ENB_TRA_IZQ, OUTPUT);

  detenerMotores();

  angulo_z = 0.0;
  indice_rumbo = 0;
  angulo_objetivo = RUMBOS[indice_rumbo];

  tiempo_anterior = micros();
  tiempo_inicio_rumbo = millis();
  reiniciarPID();

  Serial.println(F("Robot listo con nuevos offsets de MPU!"));
  delay(800);
}

void loop() {
  int16_t gx_raw, gy_raw, gz_raw;

  if (!leerGiroscopioRaw(gx_raw, gy_raw, gz_raw)) {
    Serial.println(F("Error leyendo MPU"));
    mostrarErrorOLED(F("MPU-9250"));
    detenerMotores();
    delay(500);
    return;
  }

  float gx_dps = (gx_raw - OFFSET_GX) / SENSIBILIDAD_GIRO;
  float gy_dps = (gy_raw - OFFSET_GY) / SENSIBILIDAD_GIRO;
  float gz_dps = (gz_raw - OFFSET_GZ) / SENSIBILIDAD_GIRO;

  gx_dps = aplicarZonaMuerta(gx_dps);
  gy_dps = aplicarZonaMuerta(gy_dps);
  gz_dps = aplicarZonaMuerta(gz_dps);

  unsigned long tiempo_actual = micros();
  float dt = (tiempo_actual - tiempo_anterior) / 1000000.0;
  tiempo_anterior = tiempo_actual;

  angulo_x += gx_dps * dt;
  angulo_y += gy_dps * dt;
  angulo_z += gz_dps * dt;

  if (millis() - tiempo_inicio_rumbo >= TIEMPO_RUMBO_MS) {
    tiempo_inicio_rumbo = millis();
    siguienteRumbo();
  }

  actualizarControlRumboPID();

  float rumbo_actual = normalizar360(angulo_z);
  float error_actual = calcularErrorAngular(angulo_objetivo, angulo_z);

  // Impresión en monitor serie para calibración del PID
  Serial.print(F("Rumbo: "));
  Serial.print(rumbo_actual, 2);
  Serial.print(F(" | Objetivo: "));
  Serial.print(angulo_objetivo, 1);
  Serial.print(F(" | Error: "));
  Serial.print(error_actual, 2);
  Serial.print(F(" | Vel Z: "));
  Serial.println(gz_dps, 2);

  // Actualiza la OLED 10 veces por segundo
  if (millis() - tiempo_anterior_oled >= 100) {
    tiempo_anterior_oled = millis();
    mostrarOLED(angulo_z, gz_dps);
  }

  delay(20);
}
