const int trig = 10;
const int echo = 9;

const int MotorIA = 4;
const int MotorIR = 5;

const int MotorDA = 6;
const int MotorDR = 7;

const int DISTANCIA_PARAR = 15;    
const int DISTANCIA_FRENAR = 60;    

const int VELOCIDAD_MAX = 255;
const int VELOCIDAD_MIN = 70;


void setup() {

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  pinMode(MotorIA, OUTPUT);
  pinMode(MotorIR, OUTPUT);
  pinMode(MotorDA, OUTPUT);
  pinMode(MotorDR, OUTPUT);

  Serial.begin(9600);

  Serial.println("todo en orden pibe :D");
}


void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duracion = pulseIn(echoPin, HIGH, 30000);

  if (duracion == 0) {

    Serial.println("SIN ECO");
    detenerse();
    delay(100);
    return;
  }

  float distancia = duracion * 0.0343 / 2; //arreglo para el ultrasónico

  Serial.print("Distancia: ");
  Serial.print(distancia);
  Serial.println(" cm");


  if (distancia <= DISTANCIA_PARAR) {
    detenerse();
    Serial.println("no se puede avanzar, obstáculo en la vía wacho");

  } 
  
  else {
    int velocidad;
    if (distancia >= DISTANCIA_FRENAR) {

      velocidad = VELOCIDAD_MAX;

    } 
    
    else {
      velocidad = map(
        distancia,
        DISTANCIA_PARAR,
        DISTANCIA_FRENAR,
        VELOCIDAD_MIN,
        VELOCIDAD_MAX
      );
    }

    avanzar(velocidad);

    Serial.print("Velocidad con pwm: ");
    Serial.println(velocidad);
  }

  delay(100);
}



void avanzar(int velocidad) {
//todo en ceros

  digitalWrite(MotorIA, LOW);
  analogWrite(MotorIR, velocidad);


  analogWrite(MotorDA, 0);
  analogWrite(MotorDR, velocidad);
}




void detenerse() {

  analogWrite(MotorIR, 0);
  analogWrite(MotorDR, 0);

  digitalWrite(MotorIA, LOW);
  digitalWrite(MotorDA, LOW);
}
