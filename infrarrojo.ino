const int infraIZQ = 8;
const int infraDER = 9;

void setup() {
  pinMode(infraIZQ, INPUT);
  pinMode(infraDER, INPUT);
  Serial.begin(9600);
  Serial.println("dale, todo en orden :)");
}

void loop() {
  detectar();
  delay(2000); //entre cada lectura pasan dos segundos
}

void detectar() {
  int lecturaIzq = digitalRead(infraIZQ);
  int lecturaDer = digitalRead(infraDER);

  Serial.println("nueva lectura");
  Serial.print("IZQ: ");
  Serial.print(lecturaIzq);
  Serial.print(" | DER: ");
  Serial.println(lecturaDer);
}
