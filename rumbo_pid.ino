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

const byte MPU = 0x68;//AD0 a GND

//calibracion nueva del giroscopio en base a este arreglo de robot
const float OFFSET_GX = -308.205;
const float OFFSET_GY = -92.243;
const float OFFSET_GZ = -57.700;

const float SENSIBILIDAD_GIRO = 131.0;//rango ±250 °/s

//lo agarré del otro código, ignora ruido pequeño y baja el drift
const float ZONA_MUERTA_DPS = 0.20;

//ángulo acumulado
float angulo_x = 0.0;
float angulo_y = 0.0;
float angulo_z = 0.0;

//variables para medir el tiempo
unsigned long tiempo_anterior = 0;
unsigned long tiempo_anterior_oled = 0;

// - motores l298n: direccion y pwm -

//direccion motor izquierdo
const int IN1_IZQ = 4;
const int IN2_IZQ = 5;

//direccion motor derecho
const int IN3_DER = 6;
const int IN4_DER = 7;

//pwm para la velocidad, ENA y ENB sin jumper(la capuchita de los puentes h)
const int ENA_IZQ = 9;
const int ENB_DER = 10;

// - pid -

float KP = 7;

float KI = 0.36;

float KD = 0.03;

//limites del pwm(por ahora)
const int VELOCIDAD_MAXIMA = 130;
const int VELOCIDAD_MINIMA = 55;

//sensibilidad
const float TOLERANCIA_GRADOS = 2.0;

//empieza a frenar dentro de los ultimos 12°
const float ZONA_FRENADO_GRADOS = 12.0;

// - rumbos de prueba - (referencia a mi equipo de hackmty letsgooo)

const float RUMBOS[] = { //alterna constantemente entre las direcciones
  0.0,
  90.0,
  180.0,
  270.0
};

const byte NUM_RUMBOS = 4; //elige los cuatro caminos asignados

byte indice_rumbo = 0;
float angulo_objetivo = 0.0;

//cambia de rumbo cada 10 segundos
const unsigned long TIEMPO_RUMBO_MS = 10000;
unsigned long tiempo_inicio_rumbo = 0;

// - memoria del pid -

float error_anterior_pid = 0.0;
float integral_error_pid = 0.0;
unsigned long tiempo_anterior_pid = 0;

// - funciones del giroscopio -

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
  if (velocidad_dps > -ZONA_MUERTA_DPS &&
      velocidad_dps < ZONA_MUERTA_DPS) {
    return 0.0;
  }

  return velocidad_dps;
}

// - angulos -

float normalizar360(float angulo) {
  while (angulo >= 360.0) {
    angulo -= 360.0;
  }

  while (angulo < 0.0) {
    angulo += 360.0;
  }

  return angulo;
}

float calcularErrorAngular(float objetivo, float actual) {
  float error = normalizar360(objetivo) - normalizar360(actual);

  if (error > 180.0) {
    error -= 360.0;
  }

  if (error < -180.0) {
    error += 360.0;
  }

  return error;
}

// motores l298n

void setMotorIzquierdo(int velocidad) {
  velocidad = constrain(velocidad, -255, 255);

  if (velocidad > 0) {
    digitalWrite(IN1_IZQ, HIGH);
    digitalWrite(IN2_IZQ, LOW);
  } 
  else if (velocidad < 0) {
    digitalWrite(IN1_IZQ, LOW);
    digitalWrite(IN2_IZQ, HIGH);
  } 
  else {
    digitalWrite(IN1_IZQ, LOW);
    digitalWrite(IN2_IZQ, LOW);
  }

  analogWrite(ENA_IZQ, abs(velocidad));
}

void setMotorDerecho(int velocidad) {
  velocidad = constrain(velocidad, -255, 255);

  if (velocidad > 0) {
    digitalWrite(IN3_DER, HIGH);
    digitalWrite(IN4_DER, LOW);
  } 
  else if (velocidad < 0) {
    digitalWrite(IN3_DER, LOW);
    digitalWrite(IN4_DER, HIGH);
  } 
  else {
    digitalWrite(IN3_DER, LOW);
    digitalWrite(IN4_DER, LOW);
  }

  analogWrite(ENB_DER, abs(velocidad));
}

void detenerMotores() {
  analogWrite(ENA_IZQ, 0);
  analogWrite(ENB_DER, 0);

  digitalWrite(IN1_IZQ, LOW);
  digitalWrite(IN2_IZQ, LOW);

  digitalWrite(IN3_DER, LOW);
  digitalWrite(IN4_DER, LOW);
}

//pid

void reiniciarPID() {
  error_anterior_pid = 0.0;
  integral_error_pid = 0.0;
  tiempo_anterior_pid = millis();
}

void actualizarControlRumboPID() {
  float error = calcularErrorAngular(
    angulo_objetivo,
    angulo_z
  );

  unsigned long ahora = millis();
  float dt_pid = (ahora - tiempo_anterior_pid) / 1000.0;

  if (dt_pid <= 0.0) {
    dt_pid = 0.001;
  }

  tiempo_anterior_pid = ahora;

  //si ya llegó lo suficientemente cerca, se detiene
  if (abs(error) <= TOLERANCIA_GRADOS) {
    detenerMotores();

    integral_error_pid = 0.0;
    error_anterior_pid = error;

    return;
  }

  //solo acumula integral si ya esta relativamente cerca, para evitar movimientos bruscos
  //esto baja el riesgo de integral wind-up
  if (abs(error) < 25.0) {
    integral_error_pid += error * dt_pid;
  } 
  else {
    integral_error_pid = 0.0;
  }

  integral_error_pid = constrain(
    integral_error_pid,
    -80.0,
    80.0
  );

  float derivada = (
    error - error_anterior_pid
  ) / dt_pid;

  error_anterior_pid = error;

  //salida del pid a los motores
  float salida = (KP * error)
               + (KI * integral_error_pid)
               + (KD * derivada);

  //reduce el pwm maximo conforme se va acercando
  //sin multiplicar la salida por otro factor despues
  float proporcion = abs(error) / ZONA_FRENADO_GRADOS;

  proporcion = constrain(
    proporcion,
    0.0,
    1.0
  );

  float limite_pwm = VELOCIDAD_MINIMA +
    proporcion *
    (VELOCIDAD_MAXIMA - VELOCIDAD_MINIMA);

  salida = constrain(
    salida,
    -limite_pwm,
    limite_pwm
  );

  //mientras no llegue, mantener fuerza minima para vencer la friccion y corregir ese valorcito
  if (abs(salida) < VELOCIDAD_MINIMA) {
    salida = (salida >= 0)
      ? VELOCIDAD_MINIMA
      : -VELOCIDAD_MINIMA;
  }

  //giro sobre su propio eje
  setMotorIzquierdo(-(int)salida);
  setMotorDerecho((int)salida);
}

// secuencia de rumbos

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

// - funciones de la oled -

void mostrarOLED(float grados_z, float velocidad_z) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("giro carrito :D"));

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

  configurarMPU6050();

  pinMode(IN1_IZQ, OUTPUT);
  pinMode(IN2_IZQ, OUTPUT);
  pinMode(ENA_IZQ, OUTPUT);

  pinMode(IN3_DER, OUTPUT);
  pinMode(IN4_DER, OUTPUT);
  pinMode(ENB_DER, OUTPUT);

  detenerMotores();

  //la direccion inicial se toma como 0°
  angulo_z = 0.0;

  indice_rumbo = 0;
  angulo_objetivo = RUMBOS[indice_rumbo];

  tiempo_anterior = micros();
  tiempo_inicio_rumbo = millis();

  reiniciarPID();

  Serial.println(F("Robot listo"));
  Serial.println(F("ENA D9 | ENB D10"));
  Serial.println(F("Prueba: 0 -> 30 -> 0 grados"));

  delay(800);
}

void loop() {
  int16_t gx_raw;
  int16_t gy_raw;
  int16_t gz_raw;

  if (!leerGiroscopioRaw(
        gx_raw,
        gy_raw,
        gz_raw
      )) {

    Serial.println(F("Error leyendo MPU-6050"));
    mostrarErrorOLED(F("MPU-6050"));
    detenerMotores();

    delay(500);
    return;
  }

  //aplicar los offsets y convertir de raw a °/s
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

  float error_actual = calcularErrorAngular(
    angulo_objetivo,
    angulo_z
  );

  Serial.print(F("Rumbo: "));
  Serial.print(rumbo_actual, 2);

  Serial.print(F(" | Objetivo: "));
  Serial.print(angulo_objetivo, 1);

  Serial.print(F(" | Error: "));
  Serial.print(error_actual, 2);

  Serial.print(F(" | Vel Z: "));
  Serial.println(gz_dps, 2);

  //actualiza el oled 10 veces por segundo
  if (millis() - tiempo_anterior_oled >= 100) {
    tiempo_anterior_oled = millis();
    mostrarOLED(angulo_z, gz_dps);
  }

  delay(20);
}
//gracias 
