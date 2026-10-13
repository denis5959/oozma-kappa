#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// - configuración para la pantalla oled -
// en el Mega: SDA = pin 20, SCL = pin 21
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

bool oledOK = false;   // si no hay oled, el robot sigue funcionando igual

// - sensores infrarrojos -
const int infraIZQ = 42;
const int infraDER = 43;

const int DETECTA = LOW; // cambiar a HIGH si el módulo IR manda HIGH al detectar

// - motores l298n: direccion y pwm -

//puente h delantero

//motor delantero derecho
const int IN1_DEL_DER = 53;
const int IN2_DEL_DER = 52;
const int ENA_DEL_DER = 4;   //pwm

//motor delantero izquierdo
const int IN3_DEL_IZQ = 51;
const int IN4_DEL_IZQ = 50;
const int ENB_DEL_IZQ = 5;   //pwm

//puente h trasero

//motor trasero derecho
const int IN1_TRA_DER = 22;
const int IN2_TRA_DER = 23;
const int ENA_TRA_DER = 2;   //pwm

//motor trasero izquierdo
const int IN3_TRA_IZQ = 24;
const int IN4_TRA_IZQ = 25;
const int ENB_TRA_IZQ = 3;   //pwm

// - motores en cero porque estan bien polarizados
const bool INV_DEL_DER = false;
const bool INV_DEL_IZQ = false;

const bool INV_TRA_DER = false;
const bool INV_TRA_IZQ = false;

// - velocidades -
const int VELOCIDAD_MIN        = 70;
const int VELOCIDAD_MAX        = 255;
const int VELOCIDAD_CRUCERO    = 75;
const int VELOCIDAD_OBSTACULOS = 100;   // mas lento que crucero (no bajar de VELOCIDAD_MIN)

// tiempo que sigue lento despues de dejar de ver el obstaculo
const unsigned long TIEMPO_OBSTACULOS = 1500;

enum Estado { CRUCERO, OBSTACULO };
Estado estado = CRUCERO;
unsigned long ultimaDeteccion = 0;

// para refrescar la oled solo cuando algo cambia (redibujar tarda 25 ms)
int ultimoEstadoMostrado = -1;
bool ultimaIzqMostrada = false;
bool ultimaDerMostrada = false;

void setMotor(int pin_in1, int pin_in2, int pin_enable, int velocidad, bool invertir) {
  velocidad = constrain(velocidad, -VELOCIDAD_MAX, VELOCIDAD_MAX);

  //si no es 0, respeta el minimo para que el motor si alcance a girar
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

bool detectaIzq() {
  return digitalRead(infraIZQ) == DETECTA;
}

bool detectaDer() {
  return digitalRead(infraDER) == DETECTA;
}

void mostrarMensajeOLED(const __FlashStringHelper *mensaje) {
  if (!oledOK) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.println(mensaje);
  display.display();
}

void mostrarEstadoOLED(Estado est, bool izq, bool der) {
  if (!oledOK) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // titulo
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("Oozma Kappa"));
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  // estado en grande (tamaño 2 -> caben 10 letras)
  display.setTextSize(2);
  display.setCursor(0, 15);
  if (est == OBSTACULO) {
    display.println(F("OBSTACULO!"));
  } else {
    display.println(F("Todo OK"));
  }

  // velocidad actual
  display.setTextSize(1);
  display.setCursor(0, 38);
  display.print(F("Velocidad: "));
  display.print(est == OBSTACULO ? VELOCIDAD_OBSTACULOS : VELOCIDAD_CRUCERO);

  // que sensor esta viendo algo
  display.setCursor(0, 52);
  display.print(F("IZQ: "));
  display.print(izq ? F("SI") : F("--"));
  display.setCursor(64, 52);
  display.print(F("DER: "));
  display.print(der ? F("SI") : F("--"));

  display.display();
}

// solo redibuja si cambio el estado o algun sensor
void actualizarOLED(bool izq, bool der) {
  if (estado != ultimoEstadoMostrado || izq != ultimaIzqMostrada || der != ultimaDerMostrada) {
    mostrarEstadoOLED(estado, izq, der);
    ultimoEstadoMostrado = estado;
    ultimaIzqMostrada = izq;
    ultimaDerMostrada = der;
  }
}

// - configuracion -

void setup() {
  //sensores
  pinMode(infraIZQ, INPUT);
  pinMode(infraDER, INPUT);

  //puente h delantero
  pinMode(IN1_DEL_DER, OUTPUT);
  pinMode(IN2_DEL_DER, OUTPUT);
  pinMode(ENA_DEL_DER, OUTPUT);

  pinMode(IN3_DEL_IZQ, OUTPUT);
  pinMode(IN4_DEL_IZQ, OUTPUT);
  pinMode(ENB_DEL_IZQ, OUTPUT);

  //puente h trasero
  pinMode(IN1_TRA_DER, OUTPUT);
  pinMode(IN2_TRA_DER, OUTPUT);
  pinMode(ENA_TRA_DER, OUTPUT);

  pinMode(IN3_TRA_IZQ, OUTPUT);
  pinMode(IN4_TRA_IZQ, OUTPUT);
  pinMode(ENB_TRA_IZQ, OUTPUT);

  //inicia con motores detenidos por cualquier cosa
  detenerse();

  Serial.begin(9600);

  //oled: si no la encuentra, avisa por serial pero el robot sigue
  Wire.begin();
  oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  if (!oledOK) {
    Serial.println(F("No se encontro OLED, sigo sin pantalla"));
  }

  mostrarMensajeOLED(F("Iniciando... aleja las manos :)"));
  Serial.println(F("dale, todo en orden :)"));

  //tiempo para alejar las manos cuando ponga la batería
  delay(2000);
}

// - programa principal -

void loop() {
  bool izq = detectaIzq();
  bool der = detectaDer();
  bool obstaculo = izq || der;

  switch (estado) {

    case CRUCERO:
      if (obstaculo) {
        ultimaDeteccion = millis();
        estado = OBSTACULO;
        Serial.print(F("Obstaculo detectado -> acelerando "));
        Serial.println(VELOCIDAD_OBSTACULOS);
        avanzar(VELOCIDAD_OBSTACULOS);
      } else {
        avanzar(VELOCIDAD_CRUCERO);
      }
      break;

    case OBSTACULO:
      if (obstaculo) {
        ultimaDeteccion = millis();   // sigue viendo el obstaculo, reinicia el tiempo
      }

      if (millis() - ultimaDeteccion >= TIEMPO_OBSTACULOS) {
        estado = CRUCERO;
        Serial.print(F("Obstaculo superado -> regresando a "));
        Serial.println(VELOCIDAD_CRUCERO);
        avanzar(VELOCIDAD_CRUCERO);
      } else {
        avanzar(VELOCIDAD_OBSTACULOS);
      }
      break;
  }

  actualizarOLED(izq, der);

  delay(10);
}
