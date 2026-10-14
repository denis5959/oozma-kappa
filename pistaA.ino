#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_TCS34725.h"

// Direcciones I2C: OLED 0x3C, MPU-6050 0x68, TCS34725 0x29 (no chocan)

// ================== SENSORES ULTRASONICOS ==================
const int TRIG_ADELANTE = 45;
const int ECHO_ADELANTE = 44;
const int TRIG_IZQ      = 48;
const int ECHO_IZQ      = 49;
const int TRIG_DER      = 46;
const int ECHO_DER      = 47;

const float DISTANCIA_PARED = 5.0;   // cm para considerar que hay pared


// ================== SENSORES INFRARROJOS (OBSTACULOS) ==================
const int infraIZQ = 42;
const int infraDER = 43;
const int DETECTA  = LOW;   // cambiar a HIGH si el modulo IR manda HIGH al detectar

const int VELOCIDAD_CRUCERO    = 75;   // velocidad normal de avance
const int VELOCIDAD_OBSTACULOS = 150;   // velocidad al ver obstaculo (calibrar)
const unsigned long TIEMPO_OBSTACULOS = 1500;   // ms que sigue en modo obstaculo despues de dejar de verlo

enum EstadoVel { CRUCERO, OBSTACULO };
EstadoVel estadoVel = CRUCERO;
unsigned long ultimaDeteccion = 0;
bool irIzq = false;
bool irDer = false;


// ================== SENSOR DE COLOR TCS34725 ==================
Adafruit_TCS34725 color_sensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
bool colorOK = false;

const uint16_t BLACK_CLEAR_MAX = 300;          // clear por debajo de esto = negro (calibrar)
const unsigned long INTERVALO_COLOR = 200;     // ms entre lecturas de color (cada lectura tarda ~60 ms)
unsigned long tiempo_anterior_color = 0;

float colorR = 0, colorG = 0, colorB = 0;
uint16_t colorC = 0;
const char* colorActual = "---";


// ================== MOTORES (ORUGAS TIPO TANQUE) ==================
// puente h delantero
const int IN1_DEL_DER = 53;
const int IN2_DEL_DER = 52;
const int ENA_DEL_DER = 4;   // pwm

const int IN3_DEL_IZQ = 51;
const int IN4_DEL_IZQ = 50;
const int ENB_DEL_IZQ = 5;   // pwm

// puente h trasero
const int IN1_TRA_DER = 22;
const int IN2_TRA_DER = 23;
const int ENA_TRA_DER = 2;   // pwm

const int IN3_TRA_IZQ = 24;
const int IN4_TRA_IZQ = 25;
const int ENB_TRA_IZQ = 3;   // pwm

const bool INV_DEL_DER = false;
const bool INV_DEL_IZQ = false;
const bool INV_TRA_DER = false;
const bool INV_TRA_IZQ = false;

const unsigned long TIEMPO_RETROCESO   = 300;   // ms que retrocede en callejon (calibrar)
const unsigned long TIEMPO_AVANCE_GIRO = 400;   

// - correccion de rumbo al ir derecho -
const float KP_RECTO       = 3.0;
const int   CORRECCION_MAX = 40;


// ================== GIROSCOPIO ==================
const byte MPU = 0x68;   // AD0 a GND

float OFFSET_GZ = 209.57;                 // se recalcula al inicio si CALIBRAR_AL_INICIO = true
const bool CALIBRAR_AL_INICIO = true;     // el robot debe estar quieto al encender
const int  MUESTRAS_CALIBRACION = 500;

const float SENSIBILIDAD_GIRO = 131.0;    // ±250 °/s
const float ZONA_MUERTA_DPS   = 0.20;

float angulo_z = 0.0;
float vel_z = 0.0;
float rumbo_objetivo = 0.0;
unsigned long tiempo_anterior_giro = 0;


// ================== PID DE GIRO ==================
float KP = 6.9;
float KI = 0.34;
float KD = 0.025;

const int   VELOCIDAD_MAXIMA    = 130;
const int   VELOCIDAD_MINIMA    = 55;    // subir a 70-90 si las orugas se atoran al girar
const float TOLERANCIA_GRADOS   = 2.0;
const float ZONA_FRENADO_GRADOS = 12.0;
const int   CICLOS_ESTABLE      = 5;
const unsigned long TIMEOUT_GIRO_MS = 4000;

float error_anterior_pid = 0.0;
float integral_error_pid = 0.0;
unsigned long tiempo_anterior_pid = 0;


// ================== OLED ==================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledOK = false;
unsigned long tiempo_anterior_oled = 0;

const float DIST_MAX_BARRA = 30.0;

// para refrescar la oled al instante si algo cambia
const char* casoAnterior  = "";
const char* colorAnterior = "";
bool irIzqAnterior = false;
bool irDerAnterior = false;


// ================== FUNCIONES DEL GIROSCOPIO ==================

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

void calibrarGiroscopio() {
  long suma = 0;
  int validas = 0;
  for (int i = 0; i < MUESTRAS_CALIBRACION; i++) {
    int16_t gx, gy, gz;
    if (leerGiroscopioRaw(gx, gy, gz)) {
      suma += gz;
      validas++;
    }
    delay(2);
  }
  if (validas > 0) OFFSET_GZ = (float)suma / validas;

  Serial.print(F("OFFSET_GZ calibrado: "));
  Serial.println(OFFSET_GZ, 2);
}

float aplicarZonaMuerta(float velocidad_dps) {
  if (velocidad_dps > -ZONA_MUERTA_DPS && velocidad_dps < ZONA_MUERTA_DPS) return 0.0;
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

// como delay(), pero sigue leyendo el giroscopio
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

float calcularErrorAngular(float objetivo, float actual) {
  float error = normalizar360(objetivo) - normalizar360(actual);
  if (error > 180.0)  error -= 360.0;
  if (error < -180.0) error += 360.0;
  return error;
}


// ================== FUNCIONES INFRARROJOS ==================

// lee los IR y maneja el modo obstaculo (cambia la velocidad de avance)
void actualizarObstaculos() {
  irIzq = digitalRead(infraIZQ) == DETECTA;
  irDer = digitalRead(infraDER) == DETECTA;

  if (irIzq || irDer) {
    if (estadoVel == CRUCERO) {
      Serial.print(F("Obstaculo detectado -> velocidad "));
      Serial.println(VELOCIDAD_OBSTACULOS);
    }
    estadoVel = OBSTACULO;
    ultimaDeteccion = millis();   // mientras lo siga viendo, reinicia el tiempo
  } else if (estadoVel == OBSTACULO && millis() - ultimaDeteccion >= TIEMPO_OBSTACULOS) {
    estadoVel = CRUCERO;
    Serial.print(F("Obstaculo superado -> regresando a "));
    Serial.println(VELOCIDAD_CRUCERO);
  }
}

int velocidadAvance() {
  return (estadoVel == OBSTACULO) ? VELOCIDAD_OBSTACULOS : VELOCIDAD_CRUCERO;
}


// ================== FUNCIONES DEL SENSOR DE COLOR ==================

enum DominantChannel { RED_DOMINANT, GREEN_DOMINANT, BLUE_DOMINANT };

DominantChannel get_dominant_channel(float red_value, float green_value, float blue_value) {
  if (red_value >= green_value && red_value >= blue_value) return RED_DOMINANT;
  if (green_value >= red_value && green_value >= blue_value) return GREEN_DOMINANT;
  return BLUE_DOMINANT;
}

const char* classify_red(float red_value, float green_value, float blue_value) {
  if (blue_value > 0.4 * red_value && blue_value > 0.6 * green_value) return "Pink";
  if (green_value > 0.7 * red_value) return "Yellow";
  if (green_value > 0.275 * red_value) return "Orange";
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

const char* get_color_name(float red_value, float green_value, float blue_value, uint16_t clear_value) {
  float highest_channel = max(red_value, max(green_value, blue_value));
  float lowest_channel  = min(red_value, min(green_value, blue_value));
  float channel_spread  = highest_channel - lowest_channel;

  if (clear_value < BLACK_CLEAR_MAX) return "Black / Very dark";
  if (channel_spread < 45) return "White";

  switch (get_dominant_channel(red_value, green_value, blue_value)) {
    case RED_DOMINANT:   return classify_red(red_value, green_value, blue_value);
    case GREEN_DOMINANT: return classify_green(red_value, green_value, blue_value);
    case BLUE_DOMINANT:  return classify_blue(red_value, green_value, blue_value);
  }
  return "Unknown";
}

void read_color(float* red, float* green, float* blue, uint16_t* clear) {
  uint16_t raw_red, raw_green, raw_blue;

  color_sensor.setInterrupt(false);  // prende el led del sensor
  esperar(60);                       // la lectura tarda 50 ms (sigue leyendo el giroscopio)
  color_sensor.getRawData(&raw_red, &raw_green, &raw_blue, clear);
  color_sensor.setInterrupt(true);   // apaga el led

  if (*clear == 0) {
    *red = 0; *green = 0; *blue = 0;
    return;
  }
  *red   = (float)raw_red   / *clear * 255.0;
  *green = (float)raw_green / *clear * 255.0;
  *blue  = (float)raw_blue  / *clear * 255.0;
}

// lee el color solo cada INTERVALO_COLOR ms para no frenar el laberinto
void actualizarColor() {
  if (!colorOK) return;
  if (millis() - tiempo_anterior_color < INTERVALO_COLOR) return;
  tiempo_anterior_color = millis();

  read_color(&colorR, &colorG, &colorB, &colorC);
  colorActual = get_color_name(colorR, colorG, colorB, colorC);
}


// ================== FUNCIONES DE MOTORES ==================

void enviarMotor(int in1, int in2, int ena, int velocidad, bool invertido) {
  if (invertido) velocidad = -velocidad;
  velocidad = constrain(velocidad, -255, 255);

  if (velocidad > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (velocidad < 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);  digitalWrite(in2, LOW);
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

// un paso de avance manteniendo el rumbo, a velocidad crucero u obstaculo
void avanzarRecto() {
  float error = calcularErrorAngular(rumbo_objetivo, angulo_z);
  int correccion = constrain((int)(KP_RECTO * error), -CORRECCION_MAX, CORRECCION_MAX);
  int vel = velocidadAvance();

  setLadoIzquierdo(vel - correccion);
  setLadoDerecho(vel + correccion);
}

float medirDistancia(int trigPin, int echoPin);   // declarada abajo

// avanza derecho cierto tiempo, se para si ve pared adelante
void avanzarDurante(unsigned long ms) {
  unsigned long inicio = millis();
  while (millis() - inicio < ms) {
    actualizarGiro();
    actualizarObstaculos();
    if (medirDistancia(TRIG_ADELANTE, ECHO_ADELANTE) < DISTANCIA_PARED) break;
    avanzarRecto();
    delay(5);
  }
  detenerMotores();
}

void retroceder() {
  setLadoIzquierdo(-VELOCIDAD_CRUCERO);
  setLadoDerecho(-VELOCIDAD_CRUCERO);
}


// ================== PID DE GIRO ==================

void reiniciarPID() {
  error_anterior_pid = 0.0;
  integral_error_pid = 0.0;
  tiempo_anterior_pid = millis();
}

float pasoPID() {
  float error = calcularErrorAngular(rumbo_objetivo, angulo_z);

  unsigned long ahora = millis();
  float dt_pid = (ahora - tiempo_anterior_pid) / 1000.0;
  if (dt_pid <= 0.0) dt_pid = 0.001;
  tiempo_anterior_pid = ahora;

  if (fabs(error) <= TOLERANCIA_GRADOS) {
    detenerMotores();
    integral_error_pid = 0.0;
    error_anterior_pid = error;
    return error;
  }

  if (fabs(error) < 25.0) integral_error_pid += error * dt_pid;
  else                    integral_error_pid = 0.0;
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

  // giro sobre su propio eje: orugas en sentidos opuestos
  setLadoIzquierdo(-(int)salida);
  setLadoDerecho((int)salida);

  return error;
}

void girarGrados(float grados) {
  rumbo_objetivo = normalizar360(rumbo_objetivo + grados);
  reiniciarPID();

  Serial.print(F("Girando a: "));
  Serial.println(rumbo_objetivo, 1);

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

  detenerMotores();
}

void girarDerecha()   { girarGrados(-90); }
void girarIzquierda() { girarGrados(90); }
void mediaVuelta()    { girarGrados(-180); }


// ================== ULTRASONICOS ==================

float medirDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duracion = pulseIn(echoPin, HIGH, 12000);   // timeout ~2 m
  if (duracion == 0) return 999;                   // sin eco = camino libre

  return duracion * 0.0343 / 2;
}

const char* nombreCaso(byte paredes) {
  static const char* const casos[8] = {
    "0b000", "0b001", "0b010", "0b011",
    "0b100", "0b101", "0b110", "0b111"
  };
  return casos[paredes & 0b111];
}

const char* accionCaso(byte paredes) {
  switch (paredes) {
    case 0b111: return "CALLEJON";
    case 0b010:
    case 0b110: return "GIRO DER";
    case 0b100:
    case 0b101: return "GIRO IZQ";
    default:    return "ADELANTE";
  }
}


// ================== OLED ==================
// Distribucion de la pantalla:
//  y=0   caso ultrasonico + accion            R:rumbo
//  ----- linea
//  y=13  COLOR (grande si cabe)
//  y=31  obstaculos IR                        v:velocidad
//  y=40+ barras A / I / D                     mini mapa

void dibujarBarraDist(int y, char etiqueta, float distancia, bool hayPared) {
  const int x_barra = 10;
  const int ancho_max = 56;

  display.setCursor(0, y);
  display.print(etiqueta);

  int ancho = (int)(constrain(distancia, 0, DIST_MAX_BARRA) / DIST_MAX_BARRA * ancho_max);
  display.drawRect(x_barra, y, ancho_max, 6, SSD1306_WHITE);
  display.fillRect(x_barra, y, ancho, 6, SSD1306_WHITE);

  display.setCursor(x_barra + ancho_max + 4, y);
  if (distancia >= 999) display.print(F("--"));
  else                  display.print(distancia, 0);
  if (hayPared) display.print(F("!"));
}

// mini mapa: triangulo = robot, rayas = paredes
void dibujarMapa(byte paredes) {
  display.fillTriangle(113, 46, 107, 60, 119, 60, SSD1306_WHITE);

  if (paredes & 0b100) display.fillRect(100, 40, 27, 2, SSD1306_WHITE);   // adelante
  if (paredes & 0b010) display.fillRect(100, 40, 2, 24, SSD1306_WHITE);   // izquierda
  if (paredes & 0b001) display.fillRect(125, 40, 2, 24, SSD1306_WHITE);   // derecha
}

void mostrarOLED(float dA, float dI, float dD, byte paredes, const char* caso, const char* accion) {
  if (!oledOK) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // fila 1: caso ultrasonico, accion y rumbo
  display.setCursor(0, 0);
  display.print(caso);
  display.print(' ');
  display.print(accion);
  display.setCursor(96, 0);
  display.print(F("R:"));
  display.print(normalizar360(angulo_z), 0);

  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  // color
  display.setCursor(0, 13);
  if (!colorOK) {
    display.print(F("Color: sin sensor"));
  } else {
    display.setTextSize(strlen(colorActual) <= 10 ? 2 : 1);
    display.print(colorActual);
  }

  // obstaculos infrarrojos
  display.setTextSize(1);
  display.setCursor(0, 31);
  if (irIzq || irDer) {
    display.print(F("OBST:"));
    if (irIzq) display.print(F(" IZQ"));
    if (irDer) display.print(F(" DER"));
  } else if (estadoVel == OBSTACULO) {
    display.print(F("OBST: pasando"));
  } else {
    display.print(F("Sin obstaculos"));
  }
  display.setCursor(96, 31);
  display.print(F("v:"));
  display.print(velocidadAvance());

  // distancias ultrasonicas
  dibujarBarraDist(40, 'A', dA, paredes & 0b100);
  dibujarBarraDist(48, 'I', dI, paredes & 0b010);
  dibujarBarraDist(56, 'D', dD, paredes & 0b001);

  dibujarMapa(paredes);

  display.display();
}

void mostrarMensajeOLED(const __FlashStringHelper *linea1, const __FlashStringHelper *linea2) {
  if (!oledOK) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.println(linea1);
  display.setCursor(0, 35);
  display.println(linea2);
  display.display();
}


// ================== SETUP ==================

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  // motores primero y detenidos, por seguridad
  pinMode(IN1_DEL_DER, OUTPUT); pinMode(IN2_DEL_DER, OUTPUT); pinMode(ENA_DEL_DER, OUTPUT);
  pinMode(IN3_DEL_IZQ, OUTPUT); pinMode(IN4_DEL_IZQ, OUTPUT); pinMode(ENB_DEL_IZQ, OUTPUT);
  pinMode(IN1_TRA_DER, OUTPUT); pinMode(IN2_TRA_DER, OUTPUT); pinMode(ENA_TRA_DER, OUTPUT);
  pinMode(IN3_TRA_IZQ, OUTPUT); pinMode(IN4_TRA_IZQ, OUTPUT); pinMode(ENB_TRA_IZQ, OUTPUT);
  detenerMotores();

  pinMode(TRIG_ADELANTE, OUTPUT); pinMode(ECHO_ADELANTE, INPUT);
  pinMode(TRIG_IZQ, OUTPUT);      pinMode(ECHO_IZQ, INPUT);
  pinMode(TRIG_DER, OUTPUT);      pinMode(ECHO_DER, INPUT);

  pinMode(infraIZQ, INPUT);
  pinMode(infraDER, INPUT);

  oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  if (!oledOK) Serial.println(F("No se encontro OLED, sigo sin pantalla"));
  mostrarMensajeOLED(F("Iniciando..."), F("aleja las manos :)"));

  // sensor de color: si no esta, el robot sigue sin el
  colorOK = color_sensor.begin();
  if (colorOK) {
    Serial.println(F("Sensor de color encontrado"));
  } else {
    Serial.println(F("No se encontro TCS34725, sigo sin color"));
    mostrarMensajeOLED(F("AVISO:"), F("No hay TCS34725"));
    delay(1500);
  }

  configurarMPU6050();

  if (CALIBRAR_AL_INICIO) {
    mostrarMensajeOLED(F("Calibrando gyro..."), F("NO MOVER"));
    delay(500);
    calibrarGiroscopio();
  }

  angulo_z = 0.0;
  rumbo_objetivo = 0.0;
  reiniciarPID();

  delay(500);
  tiempo_anterior_giro = micros();

  Serial.println(F("Good to gooooo"));
}

void loop() {
  if (!actualizarGiro()) {
    Serial.println(F("Error leyendo MPU-6050"));
    detenerMotores();
    mostrarMensajeOLED(F("ERROR:"), F("MPU-6050"));
    delay(500);
    tiempo_anterior_giro = micros();
    return;
  }

  actualizarObstaculos();
  actualizarColor();

  float dAdelante = medirDistancia(TRIG_ADELANTE, ECHO_ADELANTE);
  esperar(10);
  float dIzq = medirDistancia(TRIG_IZQ, ECHO_IZQ);
  esperar(10);
  float dDer = medirDistancia(TRIG_DER, ECHO_DER);
  actualizarGiro();

  bool paredAdelante = dAdelante < DISTANCIA_PARED;
  bool paredIzq      = dIzq      < DISTANCIA_PARED;
  bool paredDer      = dDer      < DISTANCIA_PARED;

  byte paredes = (paredAdelante << 2) | (paredIzq << 1) | paredDer;
  // 3 digitos: 1 = hay pared, 0 = libre (Adelante-Izq-Der)
  // Ejemplo: 0b110 = pared adelante e izquierda, derecha libre

  const char* caso   = nombreCaso(paredes);
  const char* accion = accionCaso(paredes);

  Serial.print(F("A: "));      Serial.print(dAdelante);
  Serial.print(F(" | I: "));   Serial.print(dIzq);
  Serial.print(F(" | D: "));   Serial.print(dDer);
  Serial.print(F(" | Caso: ")); Serial.print(caso);
  Serial.print(F(" ("));       Serial.print(accion); Serial.print(F(")"));
  Serial.print(F(" | Rumbo: ")); Serial.print(normalizar360(angulo_z), 1);
  Serial.print(F(" | Color: ")); Serial.print(colorActual);
  Serial.print(F(" (R:"));     Serial.print((int)colorR);
  Serial.print(F(" G:"));      Serial.print((int)colorG);
  Serial.print(F(" B:"));      Serial.print((int)colorB);
  Serial.print(F(" C:"));      Serial.print(colorC);
  Serial.print(F(") | IR: "));  Serial.print(irIzq ? F("I") : F("-"));
  Serial.print(irDer ? F("D") : F("-"));
  Serial.print(F(" | Vel: "));  Serial.println(velocidadAvance());

  // refresca la oled al instante si cambio algo, si no cada 200 ms
  bool cambio = strcmp(caso, casoAnterior) != 0 ||
                strcmp(colorActual, colorAnterior) != 0 ||
                irIzq != irIzqAnterior || irDer != irDerAnterior;

  if (cambio || millis() - tiempo_anterior_oled >= 200) {
    tiempo_anterior_oled = millis();
    casoAnterior  = caso;
    colorAnterior = colorActual;
    irIzqAnterior = irIzq;
    irDerAnterior = irDer;
    mostrarOLED(dAdelante, dIzq, dDer, paredes, caso, accion);
  }

  switch (paredes) {

    case 0b111:   // callejon sin salida
      Serial.println(F("Callejon sin salida"));
      detenerMotores();
      esperar(200);

      retroceder();
      esperar(TIEMPO_RETROCESO);
      detenerMotores();
      esperar(200);

      mostrarOLED(dAdelante, dIzq, dDer, paredes, caso, "GIRANDO");
      mediaVuelta();
      esperar(100);

      casoAnterior = "";
      break;

    case 0b010:
    case 0b110:   // derecha libre
      detenerMotores();
      esperar(100);
      girarDerecha();
      esperar(100);
      avanzarDurante(TIEMPO_AVANCE_GIRO);
      casoAnterior = "";
      break;

    case 0b100:
    case 0b101:   // izquierda libre
      detenerMotores();
      esperar(100);
      girarIzquierda();
      esperar(100);
      avanzarDurante(TIEMPO_AVANCE_GIRO);
      casoAnterior = "";
      break;

    default:      // camino libre: avanza manteniendo el rumbo
      avanzarRecto();
      break;
  }

  esperar(20);
}
