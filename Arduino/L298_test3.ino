#define PIN3 3
#define PIN4 4
#define PIN5 5
#define PIN6 6
#define PIN7 7

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN3, OUTPUT);
  pinMode(PIN4, OUTPUT);
  pinMode(PIN5, INPUT);
  pinMode(PIN6, INPUT);
  pinMode(PIN7, INPUT);
  digitalWrite(PIN3, HIGH);
  digitalWrite(PIN4, LOW);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  long duration;
  int i, j;
  
  duration = pulseIn(PIN7, HIGH);
  i = digitalRead(PIN5);
  j = digitalRead(PIN6);
  Serial.print(" PIN5  : ");
  Serial.print(i);
  Serial.print(" ");
  Serial.print(" PIN6 :");
  Serial.println(j);
  Serial.print(" PWM duration  :");
  Serial.println(duration);
}
