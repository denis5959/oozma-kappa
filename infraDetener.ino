//motores de movimiento chassis
const int MotorIA = 4;
const int MotorIR = 5;
const int MotorDA = 6;
const int MotorDR = 7;

//infrarrojos
const int SensorIzq = 8;
const int SensorDer = 11;

void setup() {
  pinMode(MotorIA, OUTPUT);
  pinMode(MotorIR, OUTPUT);
  pinMode(MotorDA, OUTPUT);
  pinMode(MotorDR, OUTPUT);

  pinMode(SensorIzq, INPUT);
  pinMode(SensorDer, INPUT);
}

void loop() {

  int izquierda = digitalRead(SensorIzq);
  int derecha = digitalRead(SensorDer);

//la condicion es que ambos detecten
  if (izquierda == LOW && derecha == LOW) {

//se detiene
    digitalWrite(MotorIA, LOW);
    digitalWrite(MotorIR, LOW);
    digitalWrite(MotorDA, LOW);
    digitalWrite(MotorDR, LOW);

  } else {

//avanza
    digitalWrite(MotorIA, LOW);
    digitalWrite(MotorIR, HIGH);

    digitalWrite(MotorDA, LOW);
    digitalWrite(MotorDR, HIGH);
  }
}
