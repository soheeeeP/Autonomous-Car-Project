#define TRIG 13 // TRIG 핀 설정 (초음파 보내는 핀)
#define ECHO 12 // ECHO 핀 설정 (초음파 받는 핀)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // PC 모니터로 센서값을 확인하기위해서 시리얼 통신을 정의해줍니다.
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  long duration, distance;
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH); // 물체에 반사되어 돌아온 초음파의 시간을
  distance = duration * 17 / 1000;
  Serial.println(duration);
  Serial.print("\nDistance : ");
  Serial.print(distance);
  Serial.println(" Ca");
  delay(1000);
}
