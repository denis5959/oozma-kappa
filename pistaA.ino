#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_TCS34725.h"

// CAMBIO: los enum van hasta arriba porque el Arduino IDE pone los prototipos
// de las funciones antes que todo, y si el enum esta abajo no lo reconoce
enum DominantChannel { RED_DOMINANT, GREEN_DOMINANT, BLUE_DOMINANT };

// Direcciones I2C: OLED 0x3C, MPU-6050 0x68, TCS34725 0x29 (no chocan)

// ================== SENSORES ULTRASONICOS ==================
const int TRIG_ADELANTE = 45;
const int ECHO_ADELANTE = 44;
const int TRIG_IZQ      = 48;
const int ECHO_IZQ      = 49;
const int TRIG_DER      = 46;
const int ECHO_DER      = 47;

// umbrales separados para frente y lados.
const float DISTANCIA_PARED_FRENTE = 7.0;    // cm (calibrar)
const float DISTANCIA_PARED_LADO   = 12.0;   // cm (calibrar)


// ================== SENSORES INFRARROJOS (OBSTACULOS) ==================
const int infraIZQ = 42;
const int infraDER = 43;
const int DETECTA  = LOW;   // cambiar a HIGH si el modulo IR manda HIGH al detectar

const int VELOCIDAD_CRUCERO    = 25;   // velocidad normal de avance
const int VELOCIDAD_OBSTACULOS = 100;  // velocidad al ver obstaculo (calibrar)
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
const unsigned long INTERVALO_COLOR = 200;     // ms entre lecturas de color
unsigned long tiempo_anterior_color = 0;

float colorR = 0, colorG = 0, colorB = 0;
uint16_t colorC = 0;
const char* colorActual = "---";

const char* COLOR_META = "Red";


// ================== MOTORES (L298N, MISMO CODIGO QUE EL DE INFRARROJO) ==================

// puente h delantero
// motor delantero derecho
const int IN1_DEL_DER = 53;
const int IN2_DEL_DER = 52;
const int ENA_DEL_DER = 4;   // pwm

// motor delantero izquierdo
const int IN3_DEL_IZQ = 51;
const int IN4_DEL_IZQ = 50;
const int ENB_DEL_IZQ = 5;   // pwm

// puente h trasero
// motor trasero derecho
const int IN1_TRA_DER = 22;
const int IN2_TRA_DER = 23;
const int ENA_TRA_DER = 2;   // pwm

// motor trasero izquierdo
const int IN3_TRA_IZQ = 24;
const int IN4_TRA_IZQ = 25;
const int ENB_TRA_IZQ = 3;   // pwm

// motores en false porque estan bien polarizados
const bool INV_DEL_DER = false;
const bool INV_DEL_IZQ = false;
const bool INV_TRA_DER = false;
const bool INV_TRA_IZQ = false;

// CAMBIO LLANTAS: limites de pwm del codigo de infrarrojo.
// Abajo de 70 los motores zumban pero no giran, por eso se fuerza el minimo.
const int VELOCIDAD_MIN = 40;
const int VELOCIDAD_MAX = 255;

const unsigned long TIEMPO_RETROCESO   = 300;   // ms que retrocede en callejon (calibrar)
const unsigned long TIEMPO_AVANCE_GIRO = 400;   // ms que avanza despues de girar (calibrar)
const unsigned long TIEMPO_AVANCE_PRE_GIRO = 250;   // ms (calibrar: ~media casilla)

// - correccion de rumbo al ir derecho -
const float KP_RECTO       = 3.0;
const int   CORRECCION_MAX = 40;


// ================== GIROSCOPIO ==================
const byte MPU = 0x68;   // AD0 a GND

// CAMBIO 270: offset escalado al nuevo rango (antes 209.57 en ±250 °/s).
// Igual se recalibra al encender si CALIBRAR_AL_INICIO = true.
float OFFSET_GZ = 52.39;
const bool CALIBRAR_AL_INICIO = true;     // el robot debe estar quieto al encender
const int  MUESTRAS_CALIBRACION = 500;

// CAMBIO 270: en ±250 °/s el sensor se SATURABA al girar con pwm 130-255.
// El robot giraba mas rapido de 250 °/s, el gyro se quedaba "topado" en 250
// y contaba menos grados de los reales: por eso daba ~270 en vez de 90.
// Ahora en ±1000 °/s (sensibilidad 32.8 LSB por °/s).
const float SENSIBILIDAD_GIRO = 32.8;     // ±1000 °/s
const float ZONA_MUERTA_DPS   = 0.20;

float angulo_z = 0.0;
float vel_z = 0.0;
float rumbo_objetivo = 0.0;
unsigned long tiempo_anterior_giro = 0;
bool giroSaturado = false;   // CAMBIO 270: avisa si el gyro llega a su limite


// ================== PID DE GIRO ==================
float KP = 6.9;
float KI = 0.34;
float KD = 0.025;

// CAMBIO GIRO: girar sobre su eje necesita mucha mas fuerza que avanzar
// (las llantas se arrastran). Con 70-130 se quedaba zumbando sin moverse.
// Si sigue sin girar, sube los dos (max 255). Si gira muy brusco, bajalos.
const int   VELOCIDAD_MAXIMA    = 170;
const int   VELOCIDAD_MINIMA    = 80; 
const float TOLERANCIA_GRADOS   = 2.0;
const float ZONA_FRENADO_GRADOS = 12.0;
const int   CICLOS_ESTABLE      = 5;

// CAMBIO GIRO: ya no hay timeout. Si no avanza hacia el angulo en este tiempo,
// le sube la fuerza y sigue intentando hasta llegar.
const unsigned long TIEMPO_SIN_AVANCE_MS = 800;
const int   EXTRA_GIRO_PASO     = 25;     // pwm extra cada vez que se atora
int extraGiro = 0;

// si en cualquier momento queda chueco mas de esto, primero se endereza
const float ERROR_MANTENER_RUMBO = 10.0;

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
  escribirRegistro(0x1B, 0x10);   // CAMBIO 270: giroscopio en ±1000 °/s (antes 0x00 = ±250)
  delay(100);
}

bool leerGiroscopioRaw(int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, (byte)6, (byte)true) != 6) return false;

  // se lee byte por byte para asegurar el orden (alto, luego bajo)
  byte b[6];
  for (int i = 0; i < 6; i++) b[i] = Wire.read();
  gx = (int16_t)((b[0] << 8) | b[1]);
  gy = (int16_t)((b[2] << 8) | b[3]);
  gz = (int16_t)((b[4] << 8) | b[5]);
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

  if (gz > 32000 || gz < -32000) giroSaturado = true;

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

void actualizarObstaculos() {
  irIzq = digitalRead(infraIZQ) == DETECTA;
  irDer = digitalRead(infraDER) == DETECTA;

  if (irIzq || irDer) {
    if (estadoVel == CRUCERO) {
      Serial.print(F("Obstaculo detectado -> velocidad "));
      Serial.println(VELOCIDAD_OBSTACULOS);
    }
    estadoVel = OBSTACULO;
    ultimaDeteccion = millis();
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
  esperar(60);
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

void actualizarColor() {
  if (!colorOK) return;
  if (millis() - tiempo_anterior_color < INTERVALO_COLOR) return;
  tiempo_anterior_color = millis();

  read_color(&colorR, &colorG, &colorB, &colorC);
  colorActual = get_color_name(colorR, colorG, colorB, colorC);
}


// ================== FUNCIONES DE MOTORES (DEL CODIGO DE INFRARROJO) ==================

void setMotor(int pin_in1, int pin_in2, int pin_enable, int velocidad, bool invertir) {
  velocidad = constrain(velocidad, -VELOCIDAD_MAX, VELOCIDAD_MAX);

  // si no es 0, respeta el minimo para que el motor si alcance a girar
  if (velocidad > 0 && velocidad < VELOCIDAD_MIN)  velocidad = VELOCIDAD_MIN;
  if (velocidad < 0 && velocidad > -VELOCIDAD_MIN) velocidad = -VELOCIDAD_MIN;

  if (invertir) {
    velocidad = -velocidad;
  }

  if (velocidad > 0) {
    digitalWrite(pin_in1, HIGH);
    digitalWrite(pin_in2, LOW);
  }
  else if (velocidad < 0) {
    digitalWrite(pin_in1, LOW);
    digitalWrite(pin_in2, HIGH);
  }
  else {
    digitalWrite(pin_in1, LOW);
    digitalWrite(pin_in2, LOW);
  }

  analogWrite(pin_enable, abs(velocidad));
}

void setMotorDelanteroDerecho(int velocidad) {
  setMotor(IN1_DEL_DER, IN2_DEL_DER, ENA_DEL_DER, velocidad, INV_DEL_DER);
}

void setMotorDelanteroIzquierdo(int velocidad) {
  setMotor(IN3_DEL_IZQ, IN4_DEL_IZQ, ENB_DEL_IZQ, velocidad, INV_DEL_IZQ);
}

void setMotorTraseroDerecho(int velocidad) {
  setMotor(IN1_TRA_DER, IN2_TRA_DER, ENA_TRA_DER, velocidad, INV_TRA_DER);
}

void setMotorTraseroIzquierdo(int velocidad) {
  setMotor(IN3_TRA_IZQ, IN4_TRA_IZQ, ENB_TRA_IZQ, velocidad, INV_TRA_IZQ);
}

void avanzar(int velocidad) {
  setMotorDelanteroDerecho(velocidad);
  setMotorDelanteroIzquierdo(velocidad);
  setMotorTraseroDerecho(velocidad);
  setMotorTraseroIzquierdo(velocidad);
}

void detenerse() {
  setMotorDelanteroDerecho(0);
  setMotorDelanteroIzquierdo(0);
  setMotorTraseroDerecho(0);
  setMotorTraseroIzquierdo(0);
}

// CAMBIO MECANUM: los pines del puente trasero estan cruzados respecto al
// delantero. Con el emparejamiento normal, al "girar" las diagonales iban en
// sentidos opuestos y con llantas mecanum eso lo hace deslizarse de lado.
// Lados reales: {delantero IZQ + trasero DER} y {delantero DER + trasero IZQ}.
const bool LADOS_CRUZADOS = true;

// Si al girar da vueltas sin parar (gira hacia el lado contrario), cambia esto a true.
const bool INTERCAMBIAR_LADOS = false;

void setParA(int velocidad) {   // lado que antes llamabamos izquierdo
  setMotorDelanteroIzquierdo(velocidad);
  if (LADOS_CRUZADOS) setMotorTraseroDerecho(velocidad);
  else                setMotorTraseroIzquierdo(velocidad);
}

void setParB(int velocidad) {   // lado que antes llamabamos derecho
  setMotorDelanteroDerecho(velocidad);
  if (LADOS_CRUZADOS) setMotorTraseroIzquierdo(velocidad);
  else                setMotorTraseroDerecho(velocidad);
}

void setLadoIzquierdo(int velocidad) {
  if (INTERCAMBIAR_LADOS) setParB(velocidad);
  else                    setParA(velocidad);
}

void setLadoDerecho(int velocidad) {
  if (INTERCAMBIAR_LADOS) setParA(velocidad);
  else                    setParB(velocidad);
}

// mismo nombre que usa el resto del codigo
void detenerMotores() {
  detenerse();
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
    if (medirDistancia(TRIG_ADELANTE, ECHO_ADELANTE) < DISTANCIA_PARED_FRENTE) break;
    avanzarRecto();
    delay(5);
  }
  detenerMotores();
}

void retroceder() {
  avanzar(-VELOCIDAD_CRUCERO);
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

  // CAMBIO GIRO: extraGiro sube la fuerza si el robot se atora
  float minimo = min(255, VELOCIDAD_MINIMA + extraGiro);
  float maximo = min(255, VELOCIDAD_MAXIMA + extraGiro);

  float proporcion = constrain(fabs(error) / ZONA_FRENADO_GRADOS, 0.0, 1.0);
  float limite_pwm = minimo + proporcion * (maximo - minimo);
  salida = constrain(salida, -limite_pwm, limite_pwm);

  if (fabs(salida) < minimo) {
    salida = (salida >= 0) ? minimo : -minimo;
  }

  // giro sobre su propio eje: orugas en sentidos opuestos
  setLadoIzquierdo(-(int)salida);
  setLadoDerecho((int)salida);

  return error;
}

// CAMBIO GIRO: gira hasta llegar a rumbo_objetivo, sin rendirse.
// Si se atora, le va subiendo la fuerza.
void llegarARumbo() {
  reiniciarPID();
  extraGiro = 0;
  giroSaturado = false;

  Serial.print(F("Girando a: "));
  Serial.println(rumbo_objetivo, 1);

  int vecesEnTolerancia = 0;
  float mejorError = fabs(calcularErrorAngular(rumbo_objetivo, angulo_z));
  unsigned long tiempoUltimoAvance = millis();

  while (vecesEnTolerancia < CICLOS_ESTABLE) {
    if (!actualizarGiro()) {
      // no se rinde: para, espera y vuelve a intentar leer el giroscopio
      Serial.println(F("Error leyendo MPU-6050 en giro, reintentando"));
      detenerMotores();
      delay(100);
      tiempo_anterior_giro = micros();
      continue;
    }

    float error = pasoPID();

    if (fabs(error) <= TOLERANCIA_GRADOS) vecesEnTolerancia++;
    else vecesEnTolerancia = 0;

    // revisa si se esta acercando al angulo
    if (fabs(error) < mejorError - 1.0) {
      mejorError = fabs(error);
      tiempoUltimoAvance = millis();
    } else if (millis() - tiempoUltimoAvance > TIEMPO_SIN_AVANCE_MS) {
      // se atoro: mas fuerza
      if (VELOCIDAD_MINIMA + extraGiro < 255) {
        extraGiro += EXTRA_GIRO_PASO;
        Serial.print(F("Atorado, subiendo fuerza: +"));
        Serial.println(extraGiro);
      }
      mejorError = fabs(error);
      tiempoUltimoAvance = millis();
    }

    delay(15);
  }

  extraGiro = 0;
  detenerMotores();
  Serial.println(F("Giro completado"));
  if (giroSaturado) {
    Serial.println(F("OJO: el giroscopio se saturo en este giro -> baja VELOCIDAD_MAXIMA o usa ±2000 °/s"));
  }
}

void girarGrados(float grados) {
  rumbo_objetivo = normalizar360(rumbo_objetivo + grados);
  llegarARumbo();
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

  long duracion = pulseIn(echoPin, HIGH, 12000);
  if (duracion == 0) return 999;                   // sin eco = camino libre

  return duracion * 0.0343 / 2;
}

byte leerParedes(float &dA, float &dI, float &dD) {
  dA = medirDistancia(TRIG_ADELANTE, ECHO_ADELANTE);
  esperar(10);
  dI = medirDistancia(TRIG_IZQ, ECHO_IZQ);
  esperar(10);
  dD = medirDistancia(TRIG_DER, ECHO_DER);
  actualizarGiro();

  bool paredAdelante = dA < DISTANCIA_PARED_FRENTE;
  bool paredIzq      = dI < DISTANCIA_PARED_LADO;
  bool paredDer      = dD < DISTANCIA_PARED_LADO;

  return (paredAdelante << 2) | (paredIzq << 1) | paredDer;
}

bool esCasoDeManiobra(byte paredes) {
  switch (paredes) {
    case 0b010:
    case 0b100:
    case 0b101:
    case 0b110:
    case 0b111: return true;
    default:    return false;
  }
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
    case 0b100:
    case 0b110: return "GIRO DER";
    case 0b101: return "GIRO IZQ";
    default:    return "ADELANTE";
  }
}


// ================== OLED ==================

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

  display.setCursor(0, 0);
  display.print(caso);
  display.print(' ');
  display.print(accion);
  display.setCursor(96, 0);
  display.print(F("R:"));
  display.print(normalizar360(angulo_z), 0);

  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.setCursor(0, 13);
  if (!colorOK) {
    display.print(F("Color: sin sensor"));
  } else {
    display.setTextSize(strlen(colorActual) <= 10 ? 2 : 1);
    display.print(colorActual);
  }

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

void finLaberinto() {
  detenerMotores();
  Serial.println(F("Callejon con ROJO -> FIN"));

  if (oledOK) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(4);
    display.setCursor((SCREEN_WIDTH - 3 * 24) / 2, (SCREEN_HEIGHT - 32) / 2);
    display.print(F("FIN"));
    display.display();
  }

  while (true) {
    detenerMotores();
    delay(1000);
  }
}


// ================== SETUP ==================

void setup() {
  Serial.begin(115200);
  Serial.println(F("1) Arranco setup"));

  // CAMBIO: I2C a 100 kHz (como el de infrarrojo). A 400 kHz con cables largos
  // y 3 sensores el MPU puede fallar y el loop deja los motores apagados.
  Wire.begin();
  // Wire.setClock(400000);
  // si un sensor I2C no responde, no se queda trabado para siempre
  #if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(3000, true);
  #endif

  // motores primero y detenidos, por seguridad (igual que en el de infrarrojo)
  pinMode(IN1_DEL_DER, OUTPUT);
  pinMode(IN2_DEL_DER, OUTPUT);
  pinMode(ENA_DEL_DER, OUTPUT);

  pinMode(IN3_DEL_IZQ, OUTPUT);
  pinMode(IN4_DEL_IZQ, OUTPUT);
  pinMode(ENB_DEL_IZQ, OUTPUT);

  pinMode(IN1_TRA_DER, OUTPUT);
  pinMode(IN2_TRA_DER, OUTPUT);
  pinMode(ENA_TRA_DER, OUTPUT);

  pinMode(IN3_TRA_IZQ, OUTPUT);
  pinMode(IN4_TRA_IZQ, OUTPUT);
  pinMode(ENB_TRA_IZQ, OUTPUT);

  detenerse();

  // PRUEBA DE LLANTAS: antes de tocar cualquier sensor.
  // Si aqui NO se mueven, el problema es de alimentacion/cables, no del codigo.
  // Cuando ya funcione todo, puedes poner PROBAR_MOTORES en false.
  const bool PROBAR_MOTORES = false;   // CAMBIO: apagado, hacia un giro fijo de 1 s al encender
  if (PROBAR_MOTORES) {
    Serial.println(F("2) Prueba de llantas: adelante 1 s"));
    delay(2000);              // tiempo para alejar las manos
    avanzar(120);
    delay(1000);
    detenerse();
    delay(500);

    // prueba de giro sobre su eje con la potencia minima del PID
    Serial.print(F("   Prueba de giro con pwm "));
    Serial.println(VELOCIDAD_MINIMA);
    setLadoIzquierdo(VELOCIDAD_MINIMA);
    setLadoDerecho(-VELOCIDAD_MINIMA);
    delay(1000);
    detenerse();
    Serial.println(F("   Prueba de llantas terminada"));
  }

  pinMode(TRIG_ADELANTE, OUTPUT); pinMode(ECHO_ADELANTE, INPUT);
  pinMode(TRIG_IZQ, OUTPUT);      pinMode(ECHO_IZQ, INPUT);
  pinMode(TRIG_DER, OUTPUT);      pinMode(ECHO_DER, INPUT);

  pinMode(infraIZQ, INPUT);
  pinMode(infraDER, INPUT);

  Serial.println(F("3) Iniciando OLED"));
  oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  if (!oledOK) Serial.println(F("No se encontro OLED, sigo sin pantalla"));
  mostrarMensajeOLED(F("Iniciando..."), F("aleja las manos :)"));

  Serial.println(F("4) Iniciando sensor de color"));
  colorOK = color_sensor.begin();
  if (colorOK) {
    Serial.println(F("Sensor de color encontrado"));
  } else {
    Serial.println(F("No se encontro TCS34725, sigo sin color"));
    mostrarMensajeOLED(F("AVISO:"), F("No hay TCS34725"));
    delay(1500);
  }

  Serial.println(F("5) Iniciando MPU-6050"));
  configurarMPU6050();
  {
    int16_t gx, gy, gz;
    if (leerGiroscopioRaw(gx, gy, gz)) Serial.println(F("   MPU-6050 OK"));
    else Serial.println(F("   MPU-6050 NO RESPONDE -> el robot no va a avanzar"));
  }

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


// ================== MODO REACTIVO (SOLO ULTRASONICOS) ==================
// CAMBIO REACTIVO: ya no hay movimientos por tiempo (avanzar 250 ms, girar 90
// por gyro, retroceder 300 ms...). El robot solo hace:
//   - pared adelante -> gira sobre su eje HASTA que el sensor de adelante vea libre
//   - si no -> avanza derecho, alejandose de la pared si queda muy pegado de lado

const float DISTANCIA_FRENTE_LIBRE = 20.0;  // cm: al girar, para cuando adelante ve mas que esto (calibrar)
const float DISTANCIA_LADO_MIN     = 5.0;   // cm: si un lado queda mas cerca que esto, se aleja
const int   EMPUJE_LADO            = 25;    // pwm de diferencia para alejarse de la pared
const int   LECTURAS_LIBRE_SEGUIDAS = 2;    // lecturas libres seguidas para dejar de girar

// dir = +1 gira a la derecha, -1 a la izquierda
void girarHastaLibre(int dir) {
  Serial.println(dir > 0 ? F("Girando DER hasta ver libre") : F("Girando IZQ hasta ver libre"));

  int libres = 0;
  while (libres < LECTURAS_LIBRE_SEGUIDAS) {
    actualizarGiro();
    setLadoIzquierdo(dir * VELOCIDAD_MINIMA);
    setLadoDerecho(-dir * VELOCIDAD_MINIMA);

    float d = medirDistancia(TRIG_ADELANTE, ECHO_ADELANTE);
    if (d > DISTANCIA_FRENTE_LIBRE) libres++;
    else                            libres = 0;
    delay(10);
  }

  detenerMotores();
  actualizarGiro();
  rumbo_objetivo = angulo_z;   // "derecho" ahora es hacia donde quedo viendo
  esperar(50);
}

// avanza derecho; el gyro solo ayuda a no irse chueco, no decide nada
void avanzarReactivo(float dI, float dD) {
  int vel = velocidadAvance();

  if (dI < DISTANCIA_LADO_MIN) {          // muy pegado a la izquierda -> hacia la derecha
    setLadoIzquierdo(vel + EMPUJE_LADO);
    setLadoDerecho(vel - EMPUJE_LADO);
    rumbo_objetivo = angulo_z;
  } else if (dD < DISTANCIA_LADO_MIN) {   // muy pegado a la derecha -> hacia la izquierda
    setLadoIzquierdo(vel - EMPUJE_LADO);
    setLadoDerecho(vel + EMPUJE_LADO);
    rumbo_objetivo = angulo_z;
  } else {
    avanzarRecto();
  }
}


// ================== LOOP ==================

void loop() {
  actualizarGiro();          // si falla, no pasa nada: igual avanza
  actualizarObstaculos();
  actualizarColor();

  float dAdelante, dIzq, dDer;
  byte paredes = leerParedes(dAdelante, dIzq, dDer);

  const char* caso   = nombreCaso(paredes);
  const char* accion = accionCaso(paredes);

  Serial.print(F("A: "));      Serial.print(dAdelante);
  Serial.print(F(" | I: "));   Serial.print(dIzq);
  Serial.print(F(" | D: "));   Serial.print(dDer);
  Serial.print(F(" | Caso: ")); Serial.print(caso);
  Serial.print(F(" (")); Serial.print(accion); Serial.print(F(")"));
  Serial.print(F(" | Color: ")); Serial.print(colorActual);
  Serial.print(F(" | Vel: "));  Serial.println(velocidadAvance());

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

  // ===== PARED ADELANTE: girar hacia donde haya espacio =====
  if (paredes & 0b100) {
    detenerMotores();
    esperar(30);

    // confirma que de verdad hay pared (evita lecturas falsas)
    if (medirDistancia(TRIG_ADELANTE, ECHO_ADELANTE) >= DISTANCIA_PARED_FRENTE) return;

    bool izqLibre = !(paredes & 0b010);
    bool derLibre = !(paredes & 0b001);

    // callejon: revisa si es la meta roja
    if (!izqLibre && !derLibre && colorOK) {
      read_color(&colorR, &colorG, &colorB, &colorC);
      colorActual = get_color_name(colorR, colorG, colorB, colorC);
      tiempo_anterior_color = millis();
      if (strcmp(colorActual, COLOR_META) == 0) finLaberinto();   // no regresa
    }

    mostrarOLED(dAdelante, dIzq, dDer, paredes, caso, "GIRANDO");

    int dir;
    if (derLibre)      dir = +1;                     // mano derecha primero
    else if (izqLibre) dir = -1;
    else               dir = (dDer >= dIzq) ? +1 : -1;  // callejon: hacia el lado con mas espacio

    girarHastaLibre(dir);
    casoAnterior = "";
    return;
  }

  // ===== ADELANTE LIBRE: avanzar =====
  avanzarReactivo(dIzq, dDer);
  esperar(20);
}
