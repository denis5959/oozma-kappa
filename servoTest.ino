#include <Servo.h>

Servo miServo;

const int PIN_SERVO = 10;          
const unsigned long INTERVALO = 3000; // 3 segundos en milisegundos

int posicion = 0;                 // Posición actual (0 o 90)
unsigned long ultimoMovimiento = 0;

void setup() {
  miServo.attach(PIN_SERVO);
  miServo.write(posicion);        // Arranca en 0 grados
  Serial.begin(9600);
  Serial.println("Servo listo en 0 grados");
}

void loop() {
  unsigned long ahora = millis();

  if (ahora - ultimoMovimiento >= INTERVALO) {
    ultimoMovimiento = ahora;

    // Alterna entre 0 y 90 grados
    if (posicion == 0) {
      posicion = 90;
    } else {
      posicion = 0;
    }

    miServo.write(posicion);
    Serial.print("Servo movido a: ");
    Serial.print(posicion);
    Serial.println(" grados");
  }
}
