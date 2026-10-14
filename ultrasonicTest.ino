#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================== SENSORES ULTRASONICOS ==================
const int TRIG_ADELANTE = 45;
const int ECHO_ADELANTE = 44;
const int TRIG_IZQ      = 48;
const int ECHO_IZQ      = 49;
const int TRIG_DER      = 46;
const int ECHO_DER      = 47;

const float DISTANCIA_PARED = 5.0;   // cm para considerar que hay pared


// ================== OLED ==================
// en el Mega: SDA = pin 20, SCL = pin 21
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledOK = false;
unsigned long tiempo_anterior_oled = 0;

const float DIST_MAX_BARRA = 30.0;     // cm que llena la barra completa
const char* casoAnterior = "";         // para refrescar la oled al instante si cambia el caso


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

// el caso tal cual en binario (Adelante-Izq-Der)
const char* nombreCaso(byte paredes) {
  static const char* const casos[8] = {
    "0b000", "0b001", "0b010", "0b011",
    "0b100", "0b101", "0b110", "0b111"
  };
  return casos[paredes & 0b111];
}

// lo que haria el robot en ese caso
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

// barrita horizontal de distancia, el "!" indica que ese sensor ve pared
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

// mini mapa: triangulo = robot, rayas = paredes detectadas
void dibujarMapa(byte paredes) {
  display.fillTriangle(113, 42, 107, 56, 119, 56, SSD1306_WHITE);

  if (paredes & 0b100) display.fillRect(100, 36, 27, 2, SSD1306_WHITE);   // adelante
  if (paredes & 0b010) display.fillRect(100, 36, 2, 27, SSD1306_WHITE);   // izquierda
  if (paredes & 0b001) display.fillRect(125, 36, 2, 27, SSD1306_WHITE);   // derecha
}

void mostrarOLED(float dA, float dI, float dD, byte paredes, const char* caso, const char* accion) {
  if (!oledOK) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // titulo
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("laberinto :D"));

  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  // caso en grande y la accion chiquita al lado
  display.setTextSize(2);
  display.setCursor(0, 15);
  display.print(caso);

  display.setTextSize(1);
  display.setCursor(68, 19);
  display.print(accion);

  // barras de distancia
  dibujarBarraDist(36, 'A', dA, paredes & 0b100);
  dibujarBarraDist(46, 'I', dI, paredes & 0b010);
  dibujarBarraDist(56, 'D', dD, paredes & 0b001);

  dibujarMapa(paredes);

  display.display();
}


// ================== SETUP ==================

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  pinMode(TRIG_ADELANTE, OUTPUT); pinMode(ECHO_ADELANTE, INPUT);
  pinMode(TRIG_IZQ, OUTPUT);      pinMode(ECHO_IZQ, INPUT);
  pinMode(TRIG_DER, OUTPUT);      pinMode(ECHO_DER, INPUT);

  oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  if (!oledOK) {
    Serial.println(F("No se encontro OLED, sigo sin pantalla"));
  } else {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.println(F("Iniciando..."));
    display.display();
  }

  delay(1000);
  Serial.println(F("Good to gooooo"));
}


// ================== LOOP ==================

void loop() {
  float dAdelante = medirDistancia(TRIG_ADELANTE, ECHO_ADELANTE);
  delay(10);
  float dIzq = medirDistancia(TRIG_IZQ, ECHO_IZQ);
  delay(10);
  float dDer = medirDistancia(TRIG_DER, ECHO_DER);

  bool paredAdelante = dAdelante < DISTANCIA_PARED;
  bool paredIzq      = dIzq      < DISTANCIA_PARED;
  bool paredDer      = dDer      < DISTANCIA_PARED;

  byte paredes = (paredAdelante << 2) | (paredIzq << 1) | paredDer;
  // 3 digitos: 1 = hay pared, 0 = libre (Adelante-Izq-Der)
  // Ejemplo: 0b110 = pared adelante e izquierda, derecha libre

  const char* caso   = nombreCaso(paredes);
  const char* accion = accionCaso(paredes);

  Serial.print(F("Adelante: ")); Serial.print(dAdelante); Serial.print(F(" cm"));
  Serial.print(F(" | Izq: "));   Serial.print(dIzq);      Serial.print(F(" cm"));
  Serial.print(F(" | Der: "));   Serial.print(dDer);      Serial.print(F(" cm"));
  Serial.print(F(" | Caso: "));  Serial.print(caso);
  Serial.print(F(" ("));         Serial.print(accion);    Serial.println(F(")"));

  // si cambio el caso se actualiza al instante, si no cada 200 ms
  if (strcmp(caso, casoAnterior) != 0 || millis() - tiempo_anterior_oled >= 200) {
    tiempo_anterior_oled = millis();
    casoAnterior = caso;
    mostrarOLED(dAdelante, dIzq, dDer, paredes, caso, accion);
  }

  delay(20);
}
