#include <Wire.h>
//dirección i2c del MPU-6050, el ad0 conectado a gnd = 0x68.
const byte MPU = 0x68;

// los offsets de mi calibración
const float OFFSET_GX = -286.506;
const float OFFSET_GY = -109.500;
const float OFFSET_GZ = -83.863;

//131 unidades raw equivalen a un grado por segundo
const float SENSIBILIDAD_GIRO = 131.0;

// ponerle un treshold a las lecturas para que no se acumule el error tan brutalmente
const float ZONA_MUERTA_DPS = 0.30;

//almacenar el ángulo acumulado
float anguloX = 0.0;
float anguloY = 0.0;
float anguloZ = 0.0;

unsigned long tiempoAnterior = 0;

void escribirRegistro(byte registro, byte valor) {
  Wire.beginTransmission(MPU);
  Wire.write(registro);
  Wire.write(valor);
  Wire.endTransmission(true);
}

void configurarMPU6050() {
  //despierta el sensor
  escribirRegistro(0x6B, 0x00);

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

void setup() {
  Serial.begin(115200);
  Wire.begin(); //inicia comunicación i2c

  configurarMPU6050();

  Serial.println(F("MPU-6050 iniciado"));
  Serial.println(F("AD0 a GND: direccion I2C 0x68"));
  Serial.println(F("Angulo X\tAngulo Y\tAngulo Z"));
  Serial.println(F("Unidad: grados"));

  tiempoAnterior = micros();
}

void loop() {
  int16_t gxRaw;
  int16_t gyRaw;
  int16_t gzRaw;

  if (!leerGiroscopioRaw(gxRaw, gyRaw, gzRaw)) {
    Serial.println(F("Error leyendo el MPU-6050"));
    delay(500);
    return;
  }

  //resta error y convierte a grados entre segundos
  float gxDps = (gxRaw - OFFSET_GX) / SENSIBILIDAD_GIRO;
  float gyDps = (gyRaw - OFFSET_GY) / SENSIBILIDAD_GIRO;
  float gzDps = (gzRaw - OFFSET_GZ) / SENSIBILIDAD_GIRO;

  //elimina el ruido, para mantenerse fiel a la sensibilidad declarada antes.
  gxDps = aplicarZonaMuerta(gxDps);
  gyDps = aplicarZonaMuerta(gyDps);
  gzDps = aplicarZonaMuerta(gzDps);

  //el tiempo que pasó desde la medición anterior
  unsigned long tiempoActual = micros();
  float dt = (tiempoActual - tiempoAnterior) / 1000000.0;
  tiempoAnterior = tiempoActual;

  // ángulo = velocidad angular x tiempo
  anguloX += gxDps * dt;
  anguloY += gyDps * dt;
  anguloZ += gzDps * dt;

  Serial.print(F("X: "));
  Serial.print(anguloX, 2);
  Serial.print(F("°\tY: "));
  Serial.print(anguloY, 2);
  Serial.print(F("°\tZ: "));
  Serial.print(anguloZ, 2);

  Serial.print(F("\t| Velocidad: "));
  Serial.print(gxDps, 2);
  Serial.print(F(", "));
  Serial.print(gyDps, 2);
  Serial.print(F(", "));
  Serial.print(gzDps, 2);
  Serial.println(F(" °/s"));

  delay(20);
}
