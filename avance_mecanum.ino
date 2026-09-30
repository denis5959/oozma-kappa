// - prueba de avance recto con chassis mecanum -
//avanza durante 5 segundos y despues se detiene

// - motores l298n: direccion y pwm -

//puente h delantero

//motor delantero derecho
const int IN1_DEL_DER = 4;
const int IN2_DEL_DER = 7;

//pwm para el motor delantero derecho
const int ENA_DEL_DER = 3;

//motor delantero izquierdo
const int IN3_DEL_IZQ = 8;
const int IN4_DEL_IZQ = 11;

//pwm para el motor delantero izquierdo
const int ENB_DEL_IZQ = 5;

//puente h trasero

//motor trasero derecho
const int IN1_TRA_DER = 12;
const int IN2_TRA_DER = 13;

//pwm para el motor trasero derecho
const int ENA_TRA_DER = 6;

//motor trasero izquierdo
const int IN3_TRA_IZQ = A0;
const int IN4_TRA_IZQ = A1;

//pwm para el motor trasero izquierdo
const int ENB_TRA_IZQ = 9;

// - inversion de motores -

//los motores de mecanum estan encontrados,
//conecté los cables de arriba a los Outs 1 y 3, y los de abajo a los outs 2 y 4
const bool INV_DEL_DER = false;
const bool INV_DEL_IZQ = true;

const bool INV_TRA_DER = true;
const bool INV_TRA_IZQ = false;

// - prueba de avance -

//pwm para evitar movimientos bruscos
const int VELOCIDAD_AVANCE = 120;

//tiempo de avance: 5 segundos
const unsigned long TIEMPO_AVANCE_MS = 5000;
bool prueba_terminada = false;

// - funciones de motores -

//controla un motor individual
//velocidad positiva es avance 
//velocidad negativa es retroceso 
void setMotor(
  int pin_in1,
  int pin_in2,
  int pin_enable,
  int velocidad,
  bool invertir
) {
  velocidad = constrain(
    velocidad,
    -255,
    255
  );

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

  analogWrite(
    pin_enable,
    abs(velocidad)
  );
}

// - funciones para cada motor - 
//manda ordenes a determinados motores
void setMotorDelanteroDerecho(int velocidad) {
  setMotor(
    IN1_DEL_DER,
    IN2_DEL_DER,
    ENA_DEL_DER,
    velocidad,
    INV_DEL_DER
  );
}

void setMotorDelanteroIzquierdo(int velocidad) {
  setMotor(
    IN3_DEL_IZQ,
    IN4_DEL_IZQ,
    ENB_DEL_IZQ,
    velocidad,
    INV_DEL_IZQ
  );
}

void setMotorTraseroDerecho(int velocidad) {
  setMotor(
    IN1_TRA_DER,
    IN2_TRA_DER,
    ENA_TRA_DER,
    velocidad,
    INV_TRA_DER
  );
}

void setMotorTraseroIzquierdo(int velocidad) {
  setMotor(
    IN3_TRA_IZQ,
    IN4_TRA_IZQ,
    ENB_TRA_IZQ,
    velocidad,
    INV_TRA_IZQ
  );
}

//avance recto, cuatro motores iguales
void avanzarRecto(int velocidad) {
  setMotorDelanteroDerecho(velocidad);
  setMotorDelanteroIzquierdo(velocidad);

  setMotorTraseroDerecho(velocidad);
  setMotorTraseroIzquierdo(velocidad);
}

//función para detener todo
void detenerMotores() {
  setMotorDelanteroDerecho(0);
  setMotorDelanteroIzquierdo(0);

  setMotorTraseroDerecho(0);
  setMotorTraseroIzquierdo(0);
}

// - configuracion -

void setup() {

  //configurar puente h delantero
  pinMode(IN1_DEL_DER, OUTPUT);
  pinMode(IN2_DEL_DER, OUTPUT);
  pinMode(ENA_DEL_DER, OUTPUT);

  pinMode(IN3_DEL_IZQ, OUTPUT);
  pinMode(IN4_DEL_IZQ, OUTPUT);
  pinMode(ENB_DEL_IZQ, OUTPUT);

  //configurar puente h trasero
  pinMode(IN1_TRA_DER, OUTPUT);
  pinMode(IN2_TRA_DER, OUTPUT);
  pinMode(ENA_TRA_DER, OUTPUT);

  pinMode(IN3_TRA_IZQ, OUTPUT);
  pinMode(IN4_TRA_IZQ, OUTPUT);
  pinMode(ENB_TRA_IZQ, OUTPUT);

  //inicia con motores detenidos por cualquier cosa
  detenerMotores();

  //tiempo para alejar las manos cuando ponga la batería
  delay(2000);
}

// - programa principal -

void loop() {
  //solo se hace una vez
  if (prueba_terminada) {
    return;
  }

  //los cuatro motores hacen avance 
  avanzarRecto(VELOCIDAD_AVANCE);

  //avanza por 5 segundos
  delay(TIEMPO_AVANCE_MS);

  //detiene todo despues de los 5 segundos
  detenerMotores();

  //evita que vuelva a arrancar
  prueba_terminada = true;

  //se acabó gg
}
