// - sensores infrarrojos -
const int infraIZQ = 8;
const int infraDER = 9;

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
const int VELOCIDAD_CRUCERO    = 100;
const int VELOCIDAD_OBSTACULOS = 180;

const unsigned long TIEMPO_OBSTACULOS = 1500;

bool subiendoObstaculo = false;
unsigned long ultimaDeteccion = 0;

// - funciones de motores -

//controla un motor individual
//velocidad positiva es avance, negativa es retroceso, 0 es detenido
void setMotor(int pin_in1, int pin_in2, int pin_enable, int velocidad, bool invertir) {
  velocidad = constrain(velocidad, -VELOCIDAD_MAX, VELOCIDAD_MAX);

  //si no es 0, respeta el minimo para que el motor si alcance a girar
  if (velocidad > 0 && velocidad < VELOCIDAD_MIN)  velocidad = VELOCIDAD_MIN;
  if (velocidad < 0 && velocidad > -VELOCIDAD_MIN) velocidad = -VELOCIDAD_MIN;

  //invierte la direccion
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

//avance recto, cuatro motores iguales
void avanzar(int velocidad) {
  setMotorDelanteroDerecho(velocidad);
  setMotorDelanteroIzquierdo(velocidad);
  setMotorTraseroDerecho(velocidad);
  setMotorTraseroIzquierdo(velocidad);
}

//detiene todo
void detenerse() {
  setMotorDelanteroDerecho(0);
  setMotorDelanteroIzquierdo(0);
  setMotorTraseroDerecho(0);
  setMotorTraseroIzquierdo(0);
}

// - sensores -

bool detectar() {
  int lecturaIzq = digitalRead(infraIZQ);
  int lecturaDer = digitalRead(infraDER);
  return (lecturaIzq == DETECTA) || (lecturaDer == DETECTA);
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
  Serial.println("dale, todo en orden :)");

  //tiempo para alejar las manos cuando ponga la batería
  delay(2000);
}

// - programa principal -

void loop() {
  bool obstaculo = detectar();

  if (obstaculo) {
    ultimaDeteccion = millis();
   enum Estado { CRUCERO, SUBIENDO };
   Estado estado = CRUCERO;

  if (subiendoObstaculo && (millis() - ultimaDeteccion >= TIEMPO_OBSTACULOS)) {
    subiendoObstaculo = false;
    Serial.print("Obstaculo superado -> regresando a ");
    Serial.println(VELOCIDAD_CRUCERO);
  }

  if (subiendoObstaculo) {
    avanzar(VELOCIDAD_OBSTACULOS);
  } else {
    avanzar(VELOCIDAD_CRUCERO);
  }

  delay(10);
}
