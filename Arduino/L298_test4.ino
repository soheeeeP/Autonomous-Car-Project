#include <Servo.h>
Servo servo;

void setup() {
  // put your setup code here, to run once:
  servo.attach(3);
}

void loop() {
  // put your main code here, to run repeatedly:
  servo.write(90);
}
