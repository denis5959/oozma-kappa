const int infraIZQ = 8;
const int infraDER = 9;

const int DETECTA = LOW; // cambiar a HIGH si el módulo IR manda HIGH al detectar

const int MotorIA = 4;
const int MotorIR = 5;   
const int MotorDA = 6;
const int MotorDR = 3;   

const int VELOCIDAD_MIN     = 70;
const int VELOCIDAD_MAX     = 255;
const int VELOCIDAD_CRUCERO = 100;
const int VELOCIDAD_OBSTACULOS   = 180;

const unsigned long TIEMPO_OBSTACULOS = 1500;

bool subiendoObstaculo = false;
unsigned long ultimaDeteccion = 0;

void setup() {
  pinMode(infraIZQ, INPUT);
  pinMode(infraDER, INPUT);

  pinMode(MotorIA, OUTPUT);
  pinMode(MotorIR, OUTPUT);
  pinMode(MotorDA, OUTPUT);
  pinMode(MotorDR, OUTPUT);

  detenerse();

  Serial.begin(9600);
  Serial.println("dale, todo en orden :)");
  delay(1000);  
}


void loop() {
  bool obstaculo = detectar();

  if (obstaculo) {
    ultimaDeteccion = millis();          
    if (!subiendoObstaculo) {
      subiendoObstaculo = true;
      Serial.print("Obstaculo detectado -> acelerando a ");
      Serial.println(VELOCIDAD_OBSTACULOS);
    }
  }

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

bool detectar() {
  int lecturaIzq = digitalRead(infraIZQ);
  int lecturaDer = digitalRead(infraDER);
  return (lecturaIzq == DETECTA) || (lecturaDer == DETECTA);
}


void avanzar(int velocidad) {
  velocidad = constrain(velocidad, VELOCIDAD_MIN, VELOCIDAD_MAX);

  digitalWrite(MotorIA, LOW);
  analogWrite(MotorIR, velocidad);

  digitalWrite(MotorDA, LOW);
  analogWrite(MotorDR, velocidad);
}


void detenerse() {
  analogWrite(MotorIR, 0);
  analogWrite(MotorDR, 0);
  digitalWrite(MotorIA, LOW);
  digitalWrite(MotorDA, LOW);
}
