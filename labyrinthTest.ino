#include <Servo.h>

const int TRIG_ADELANTE = A0;
const int ECHO_ADELANTE = A1;
const int TRIG_IZQ      = A2;
const int ECHO_IZQ      = A3;
const int TRIG_DER      = A4;
const int ECHO_DER      = A5;

const int MotorIA = 4;
const int MotorIR = 5;

const int MotorDA = 6;
const int MotorDR = 7;

const int TIEMPO_AVANCE    = 150;  // ms que avanza antes de cerrar el servo (calibrar)
const int TIEMPO_RETROCESO = 600;  // ms que retrocede para salir del callejón (calibrar)

const int PIN_SERVO = 8;
Servo servo;

// ---------------- AJUSTES ----------------
const int DISTANCIA_PARED = 5;    
// const int VELOCIDAD_MAX = 255; considerar velocidades luego
// const int VELOCIDAD_MIN = 70; considerar velocidades luego
const int TIEMPO_GIRO     = 400;   // ms para girar 90 grados (calibrar)

bool servoCerrado = false;

// ---------------- MOVIMIENTOS ----------------
void adelante() {
  digitalWrite(MotorIA, HIGH); digitalWrite(MotorIR, LOW);
  digitalWrite(MotorDA, HIGH); digitalWrite(MotorDR, LOW);
}

void detener() {
  digitalWrite(MotorIA, LOW); digitalWrite(MotorIR, LOW);
  digitalWrite(MotorDA, LOW); digitalWrite(MotorDR, LOW);
}

void girarDerecha() {
  digitalWrite(MotorIA, HIGH); digitalWrite(MotorIR, LOW);   // izquierdo adelante
  digitalWrite(MotorDA, LOW);  digitalWrite(MotorDR, HIGH);  // derecho atras
  delay(TIEMPO_GIRO);
  detener();
}

void girarIzquierda() {
  digitalWrite(MotorIA, LOW);  digitalWrite(MotorIR, HIGH);  // izquierdo atras
  digitalWrite(MotorDA, HIGH); digitalWrite(MotorDR, LOW);   // derecho adelante
  delay(TIEMPO_GIRO);
  detener();
}

void retroceder() {
  digitalWrite(MotorIA, LOW); digitalWrite(MotorIR, HIGH);
  digitalWrite(MotorDA, LOW); digitalWrite(MotorDR, HIGH);
}

float medirDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duracion = pulseIn(echoPin, HIGH, 30000);
  if (duracion == 0) return 999;   // sin eco = camino libre

  return duracion * 0.0343 / 2;    // arreglo para el ultrasónico
}

void setup() {

  pinMode(TRIG_ADELANTE, OUTPUT); pinMode(ECHO_ADELANTE, INPUT);
  pinMode(TRIG_IZQ, OUTPUT);      pinMode(ECHO_IZQ, INPUT);
  pinMode(TRIG_DER, OUTPUT);      pinMode(ECHO_DER, INPUT);

  pinMode(MotorIA, OUTPUT);
  pinMode(MotorIR, OUTPUT);
  pinMode(MotorDA, OUTPUT);
  pinMode(MotorDR, OUTPUT);

  servo.attach(PIN_SERVO);
  servo.write(0);   // posicion inicial (abierto)
  detener();
  delay(1000);

    Serial.begin(9600);

  Serial.println("Good to gooooo");
}

void loop() {
  
  // Medir con pausas cortas para que los ecos no se crucen
  float dAdelante = medirDistancia(TRIG_ADELANTE, ECHO_ADELANTE);
  delay(10);
  float dIzq = medirDistancia(TRIG_IZQ, ECHO_IZQ);
  delay(10);
  float dDer = medirDistancia(TRIG_DER, ECHO_DER);

  bool paredAdelante = dAdelante < DISTANCIA_PARED;
  bool paredIzq      = dIzq      < DISTANCIA_PARED;
  bool paredDer      = dDer      < DISTANCIA_PARED;

  Serial.print("Adelante: "); Serial.print(dAdelante); Serial.print(" cm");
  Serial.print(" | Izq: ");   Serial.print(dIzq);      Serial.print(" cm");
  Serial.print(" | Der: ");   Serial.print(dDer);      Serial.println(" cm");

      if (paredAdelante && paredIzq && paredDer) {
    Serial.println("Callejon sin salida");
    detener();
    delay(200);

    if (!servoCerrado) {
      // Avanzar un poco y cerrar el servo (solo la primera vez)
      adelante();
      delay(TIEMPO_AVANCE);
      detener();
      servo.write(90);
      servoCerrado = true;
      Serial.println("Servo cerrado a 90");
      delay(500);   // dar tiempo al servo de llegar
    }

    // Retroceder para salir del callejón
    retroceder();
    delay(TIEMPO_RETROCESO);
    detener();
    delay(200);

    // Dar media vuelta (180°) para regresar al camino
    girarDerecha();
    girarDerecha();
  }
  
  else if (paredIzq && !paredAdelante && !paredDer) {
    girarDerecha();
  }
  else if (paredAdelante && paredIzq && !paredDer) {
    // Esquina: pared adelante e izquierda -> girar a la derecha
    girarDerecha();
  }
  else if (paredAdelante && !paredIzq) {
    girarIzquierda();
  }
  else {
    adelante();
  }

  delay(50);
}
