#define ENA 3
#define IN1 2
#define IN2 4

#define ENB 5
#define IN3 6
#define IN4 7

int line_sensor_pin[5] = {8, 9, 10, 11, 12};
int line_sensor_pin_data[5] = {0, 0, 0, 0, 0};
int pwm_base = 70;
int pwm_offset = 0;
int pwm_steer = 0;

void setup() {
  // put your setup code here, to run once:
  /* IN#은 output으로 사용 */
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  /* initialization (변수 할당한 후 초기화 필수) */
  analogWrite(ENA, 0);  // 처음부터 값이 있으면 급발진함 -> 0으로 설정
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); // LOW = 0
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  for (int i = 0; i < 5; i++)
    pinMode(line_sensor_pin[i], INPUT);

  Serial.begin(115200);
}

void Motor_R_Control(int direction, int pwm){
  switch(direction){
    case 1: // 전진
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, pwm);
      break;
    case 0: // 정지
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 0);
      break;
    case -1:  // 후진
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, pwm);
      break;
  }

}


void Motor_R_Control1(int pwm){
  if (pwm > 0){ // 전진
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, pwm);
  }
  else {  // 후진
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, pwm);
  }
}

void Motor_L_Control(int direction, int pwm){
  switch(direction){
    case 1: // 전진
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, pwm);
      break;
    case 0: // 정지
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 0);
      break;
    case -1:  // 후진
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, pwm);
      break;
  }

}

void Motor_L_Control1(int pwm){
  if (pwm > 0){ // 전진
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, pwm);
  }
  else {  // 후진
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, pwm);
  }
}

int read_line_sensor(){
  int type = -100;
  int sum = 0;
  
  for (int i = 0; i < 5; i++){
    line_sensor_pin_data[i] = digitalRead(line_sensor_pin[i]);
    if (line_sensor_pin_data[i] == 1) sum++;
  }

  if (sum == 1){
    if (line_sensor_pin_data[0] == 1) type = -4;
    if (line_sensor_pin_data[1] == 1) type = -2;
    if (line_sensor_pin_data[2] == 1) type = 0;
    if (line_sensor_pin_data[3] == 1) type = 2;
    if (line_sensor_pin_data[4] == 1) type = 4;
  }
  if (sum == 2){
    if ((line_sensor_pin_data[0] == 1) && (line_sensor_pin_data[1] == 1)) type = -3;
    if ((line_sensor_pin_data[1] == 1) && (line_sensor_pin_data[2] == 1)) type = -1;
    if ((line_sensor_pin_data[2] == 1) && (line_sensor_pin_data[3] == 1)) type = 1;
    if ((line_sensor_pin_data[3] == 1) && (line_sensor_pin_data[4] == 1)) type = 3;
  }

  return type;
}

void loop() {
  // put your main code here, to run repeatedly:
  int line_type = -100;
  int pwm_r, pwm_l;
  pwm_r = pwm_l = 0;
  
  line_type = read_line_sensor();
  for (int i = 0; i < 5; i++){
    Serial.print(line_sensor_pin_data[i]);
    Serial.print(" ");
  }
  Serial.print(line_type);
  Serial.println(" ");

  
  if (line_type != -100)
    pwm_steer = line_type * 8;
    
  pwm_l = pwm_base + pwm_steer;
  pwm_r = pwm_base + pwm_offset - pwm_steer;
  
  Motor_R_Control1(pwm_r);
  Motor_L_Control1(pwm_l);
}
