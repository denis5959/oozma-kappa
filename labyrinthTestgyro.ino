#include <Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


const int TRIG_ADELANTE = A0;
const int ECHO_ADELANTE = A1;
const int TRIG_IZQ      = A2;
const int ECHO_IZQ      = A3;
const int TRIG_DER      = 13;   
const int ECHO_DER      = 12;   

const int MotorIA = 4;   
const int MotorIR = 5;   
const int MotorDA = 6;   
const int MotorDR = 7;   

const int ENA_IZQ = 3;
const int ENB_DER = 11;


const int PIN_SERVO = 8;
Servo servo;

const float DISTANCIA_PARED    = 5.0;   
const int   VELOCIDAD_AVANCE   = 120;   
const int   TIEMPO_AVANCE      = 150;   // ms que avanza antes de cerrar el servo (calibrar)
const int   TIEMPO_RETROCESO   = 600;   // ms que retrocede para salir del callejón (calibrar)

bool servoCerrado = false;

const byte MPU = 0x68;   // AD0 a GND

const float OFFSET_GZ = -57.700;

const float SENSIBILIDAD_GIRO = 131.0;   
const float ZONA_MUERTA_DPS   = 0.20;    // ignora ruido pequeño y baja el drift

float angulo_z = 0.0;        
float vel_z = 0.0;           
float rumbo_objetivo = 0.0;  
unsigned long tiempo_anterior_giro = 0;

float KP = 7;
float KI = 0.36;
float KD = 0.03;

const int   VELOCIDAD_MAXIMA    = 130;
const int   VELOCIDAD_MINIMA    = 55;
const float TOLERANCIA_GRADOS   = 2.0;
const float ZONA_FRENADO_GRADOS = 12.0;
const int   CICLOS_ESTABLE      = 5;    
const unsigned long TIMEOUT_GIRO_MS = 4000;  

float error_anterior_pid = 0.0;
float integral_error_pid = 0.0;
unsigned long tiempo_anterior_pid = 0;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledOK = false;
unsigned long tiempo_anterior_oled = 0;

void escribirRegistro(byte registro, byte valor) {
  Wire.beginTransmission(MPU);
  Wire.write(registro);
  Wire.write(valor);
  Wire.endTransmission(true);
}

void configurarMPU6050() {
  escribirRegistro(0x6B, 0x00);   // despierta el sensor
  escribirRegistro(0x1B, 0x00);   // giroscopio en ±250 °/s
  delay(100);
}

bool leerGiroscopioRaw(int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, (byte)6, (byte)true) != 6) return false;

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

bool actualizarGiro() {
  int16_t gx, gy, gz;
  if (!leerGiroscopioRaw(gx, gy, gz)) return false;

  vel_z = aplicarZonaMuerta((gz - OFFSET_GZ) / SENSIBILIDAD_GIRO);

  unsigned long ahora = micros();
  float dt = (ahora - tiempo_anterior_giro) / 1000000.0;
  tiempo_anterior_giro = ahora;

  angulo_z += vel_z * dt;
  return true;
}

// Como delay(), pero sigue leyendo el giroscopio para no perder el ángulo
void esperar(unsigned long ms) {
  unsigned long inicio = millis();
  while (millis() - inicio < ms) {
    actualizarGiro();
    delay(5);
  }
}

float normalizar360(float angulo) {
  while (angulo >= 360.0) angulo -= 360.0;
  while (angulo < 0.0)    angulo += 360.0;
  return angulo;
}

void setMotorIzquierdo(int velocidad) {
  velocidad = constrain(velocidad, -255, 255);
  if (velocidad > 0) {
    digitalWrite(MotorIA, HIGH); digitalWrite(MotorIR, LOW);
  } else if (velocidad < 0) {
    digitalWrite(MotorIA, LOW);  digitalWrite(MotorIR, HIGH);
  } else {
    digitalWrite(MotorIA, LOW);  digitalWrite(MotorIR, LOW);
  }
  analogWrite(ENA_IZQ, abs(velocidad));
}

void setMotorDerecho(int velocidad) {
  velocidad = constrain(velocidad, -255, 255);
  if (velocidad > 0) {
    digitalWrite(MotorDA, HIGH); digitalWrite(MotorDR, LOW);
  } else if (velocidad < 0) {
    digitalWrite(MotorDA, LOW);  digitalWrite(MotorDR, HIGH);
  } else {
    digitalWrite(MotorDA, LOW);  digitalWrite(MotorDR, LOW);
  }
  analogWrite(ENB_DER, abs(velocidad));
}

void adelante() {
  setMotorIzquierdo(VELOCIDAD_AVANCE);
  setMotorDerecho(VELOCIDAD_AVANCE);
}

void retroceder() {
  setMotorIzquierdo(-VELOCIDAD_AVANCE);
  setMotorDerecho(-VELOCIDAD_AVANCE);
}

void detener() {
  setMotorIzquierdo(0);
  setMotorDerecho(0);
}

void reiniciarPID() {
  error_anterior_pid = 0.0;
  integral_error_pid = 0.0;
  tiempo_anterior_pid = millis();
}

float pasoPID() {
  float error = rumbo_objetivo - angulo_z;

  unsigned long ahora = millis();
  float dt_pid = (ahora - tiempo_anterior_pid) / 1000.0;
  if (dt_pid <= 0.0) dt_pid = 0.001;
  tiempo_anterior_pid = ahora;

  if (fabs(error) <= TOLERANCIA_GRADOS) {
    detener();
    integral_error_pid = 0.0;
    error_anterior_pid = error;
    return error;
  }

  if (fabs(error) < 25.0) {
    integral_error_pid += error * dt_pid;
  } else {
    integral_error_pid = 0.0;
  }
  integral_error_pid = constrain(integral_error_pid, -80.0, 80.0);

  float derivada = (error - error_anterior_pid) / dt_pid;
  error_anterior_pid = error;

  float salida = (KP * error) + (KI * integral_error_pid) + (KD * derivada);

  float proporcion = constrain(fabs(error) / ZONA_FRENADO_GRADOS, 0.0, 1.0);
  float limite_pwm = VELOCIDAD_MINIMA + proporcion * (VELOCIDAD_MAXIMA - VELOCIDAD_MINIMA);
  salida = constrain(salida, -limite_pwm, limite_pwm);


  if (fabs(salida) < VELOCIDAD_MINIMA) {
    salida = (salida >= 0) ? VELOCIDAD_MINIMA : -VELOCIDAD_MINIMA;
  }


  setMotorIzquierdo(-(int)salida);
  setMotorDerecho((int)salida);

  return error;
}


void girarGrados(float grados) {
  rumbo_objetivo += grados;
  reiniciarPID();

  Serial.print(F("Girando a: "));
  Serial.println(normalizar360(rumbo_objetivo), 1);

  int vecesEnTolerancia = 0;
  unsigned long inicio = millis();

  while (vecesEnTolerancia < CICLOS_ESTABLE) {
    if (!actualizarGiro()) {
      Serial.println(F("Error leyendo MPU-6050 en giro"));
      break;
    }

    float error = pasoPID();

    if (fabs(error) <= TOLERANCIA_GRADOS) vecesEnTolerancia++;
    else vecesEnTolerancia = 0;

    if (millis() - inicio > TIMEOUT_GIRO_MS) {
      Serial.println(F("Timeout en el giro"));
      break;
    }

    delay(15);
  }

  detener();
}

void girarDerecha()  { girarGrados(-90); }
void girarIzquierda() { girarGrados(90); }
void mediaVuelta()   { girarGrados(-180); }


float medirDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // timeout de 12 ms (~2 m) para no frenar tanto el loop
  long duracion = pulseIn(echoPin, HIGH, 12000);
  if (duracion == 0) return 999;   // sin eco = camino libre

  return duracion * 0.0343 / 2;
}

void mostrarOLED(float dA, float dI, float dD) {
  if (!oledOK) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("laberinto :D"));
  display.drawLine(0, 11, 127, 11, SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(0, 16);
  display.print(F("R: "));
  display.print(normalizar360(angulo_z), 1);

  display.setTextSize(1);
  display.setCursor(0, 38);
  display.print(F("A:")); display.print(dA, 0);
  display.print(F(" I:")); display.print(dI, 0);
  display.print(F(" D:")); display.print(dD, 0);

  display.setCursor(0, 52);
  display.print(F("Servo: "));
  display.print(servoCerrado ? F("cerrado") : F("abierto"));

  display.display();
}

void mostrarErrorOLED(const __FlashStringHelper *mensaje) {
  if (!oledOK) return;
  display.clearDisplay();
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
  Wire.setClock(400000);   

  oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  if (!oledOK) {
    Serial.println(F("No se encontro OLED, sigo sin pantalla"));
  } else {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.println(F("Iniciando..."));
    display.println(F("No mover el robot"));
    display.display();
  }

  configurarMPU6050();

  pinMode(TRIG_ADELANTE, OUTPUT); pinMode(ECHO_ADELANTE, INPUT);
  pinMode(TRIG_IZQ, OUTPUT);      pinMode(ECHO_IZQ, INPUT);
  pinMode(TRIG_DER, OUTPUT);      pinMode(ECHO_DER, INPUT);

  pinMode(MotorIA, OUTPUT); pinMode(MotorIR, OUTPUT);
  pinMode(MotorDA, OUTPUT); pinMode(MotorDR, OUTPUT);
  pinMode(ENA_IZQ, OUTPUT); pinMode(ENB_DER, OUTPUT);

  servo.attach(PIN_SERVO);
  servo.write(0);  //opennn
  detener();

  delay(1000);

  angulo_z = 0.0;
  rumbo_objetivo = 0.0;
  tiempo_anterior_giro = micros();

  Serial.println(F("Good to gooooo"));
}

void loop() {
  if (!actualizarGiro()) {
    Serial.println(F("Error leyendo MPU-6050"));
    mostrarErrorOLED(F("MPU-6050"));
    detener();
    delay(500);
    return;
  }

  float dAdelante = medirDistancia(TRIG_ADELANTE, ECHO_ADELANTE);
  esperar(10);
  float dIzq = medirDistancia(TRIG_IZQ, ECHO_IZQ);
  esperar(10);
  float dDer = medirDistancia(TRIG_DER, ECHO_DER);
  actualizarGiro();

  bool paredAdelante = dAdelante < DISTANCIA_PARED;
  bool paredIzq      = dIzq      < DISTANCIA_PARED;
  bool paredDer      = dDer      < DISTANCIA_PARED;

  Serial.print(F("Adelante: ")); Serial.print(dAdelante); Serial.print(F(" cm"));
  Serial.print(F(" | Izq: "));   Serial.print(dIzq);      Serial.print(F(" cm"));
  Serial.print(F(" | Der: "));   Serial.print(dDer);      Serial.print(F(" cm"));
  Serial.print(F(" | Rumbo: ")); Serial.println(normalizar360(angulo_z), 1);


  if (millis() - tiempo_anterior_oled >= 200) {
    tiempo_anterior_oled = millis();
    mostrarOLED(dAdelante, dIzq, dDer);
  }

  byte paredes = (paredAdelante << 2) | (paredIzq << 1) | paredDer;
// Ob related to the sensor. Habrán 3 digitos en cada variable conformado de 1s y 0s
// El uno es para decir que hay pared, cero para los que no hay
//Adelante-Izq-Der
// Ejemplo: 0b110 = pared adelante e izquierda, derecha libre

  switch (paredes) {

    case 0b111:  
      Serial.println(F("Callejon sin salida"));
      detener();
      esperar(200);

      if (!servoCerrado) {
        adelante();
        esperar(TIEMPO_AVANCE);
        detener();
        servo.write(90);
        servoCerrado = true;
        Serial.println(F("Servo cerrado a 90"));
        esperar(500);
      }

      retroceder();
      esperar(TIEMPO_RETROCESO);
      detener();
      esperar(200);

      mediaVuelta();
      break;

    case 0b010: 
    case 0b110:   
      girarDerecha();
      break;

    case 0b100:   
    case 0b101:   
      girarIzquierda();
      break;

    default:      
      adelante();
      break;
  }

  delay(20)
}
