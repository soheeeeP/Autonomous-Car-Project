#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <wiringPi.h>
#include <signal.h>
#include <softPwm.h>

//GPIO Motor Control
#define ENA 1	
#define IN1 4
#define IN2 5

#define ENB 0
#define IN3 2
#define IN4 3

#define MAX_PWM_DUTY 100

//Ultrasonic Sensor
#define TRIG 21	//output
#define ECHO 22	//input
#define OBSTACLE_DISTANCE 20

int flag_obstacle_data = 0;
float u_sensor_data = 0;
int pwm_r,pwm_l;

void sig_Handler(int sig);

void motor_control_r(int pwm){
	if(pwm>0){
		digitalWrite(IN1,HIGH);
		digitalWrite(IN2,LOW);
		softPwmWrite(ENA,pwm);
	}
	else if(pwm==0){
		digitalWrite(IN1,LOW);
		digitalWrite(IN2,LOW);
		softPwmWrite(ENA,0);		
	}
	else{
		digitalWrite(IN1,LOW);
		digitalWrite(IN2,HIGH);
		softPwmWrite(ENA,-pwm);
	}
}

void motor_control_l(int pwm){
	if(pwm>0){
		digitalWrite(IN3,LOW);
		digitalWrite(IN4,HIGH);
		softPwmWrite(ENB,pwm);
	}
	else if(pwm==0){
		digitalWrite(IN3,LOW);
		digitalWrite(IN4,LOW);
		softPwmWrite(ENB,0);		
	}
	else{
		digitalWrite(IN3,HIGH);
		digitalWrite(IN4,LOW);
		softPwmWrite(ENB,-pwm);
	}
}

int GPIO_control_setup(void){
	if(wiringPiSetup() == -1){
		printf("WiringPi Setup error!\n");
		return -1;
	}
	
	
	pinMode(ENA,OUTPUT);
	pinMode(IN1,OUTPUT);
	pinMode(IN2,OUTPUT);
	
	pinMode(ENB,OUTPUT);
	pinMode(IN3,OUTPUT);
	pinMode(IN4,OUTPUT);
	
	pinMode(TRIG,OUTPUT);
	pinMode(ECHO,INPUT);
	
	softPwmCreate(ENA,1,MAX_PWM_DUTY);
	softPwmCreate(ENB,1,MAX_PWM_DUTY);
	
	softPwmWrite(ENA,0);
	softPwmWrite(ENB,0);
	
	pwm_r=pwm_l=0;
	return 0;
}
float ultrasonic_sensor(){
	long start_time, end_time;
	long temp_time1, temp_time2;
	 
	int duration=-1;
	float distance=0;
	
	//input trigger
	digitalWrite(TRIG,LOW);
	delayMicroseconds(5);
	digitalWrite(TRIG,HIGH);
	delayMicroseconds(10);
	digitalWrite(TRIG,LOW);
	
	//print sonic burst
	delayMicroseconds(200);			//wait for burst signal(40kH*8= 8*25us=200)
	
	temp_time1 = micros();
	//wait until ECHO pin is HIGH
	while(digitalRead(ECHO) == LOW)
	{
		temp_time2 = micros();
		duration = temp_time2 - temp_time1;
		//printf("%d\n",duration);
		
		if(duration > 1000) return -1;
	}
	
	start_time=micros();
	//wait until ECHO pin is LOW
	while(digitalRead(ECHO) == HIGH)
	{
		temp_time2 = micros();
		duration = temp_time2 - start_time;
		if(duration > 2500) return -1;
		//printf("%d\n",duration);
	}
	end_time=micros();

	duration = end_time - start_time;
	distance = duration / 58 ;
	
	return distance;

		
}
void* ultrasonic_sensor_thread(void* num){

	while(1){
		u_sensor_data = ultrasonic_sensor();
		if(u_sensor_data <= OBSTACLE_DISTANCE) {
			printf("Obstacle detected\n\n");
			flag_obstacle_data = 1;
		}
			
	}
	
	//pthread_exit(NULL);
}

void* motor_control_thread(void* num){
	while(1){
		motor_control_l(pwm_l);
		motor_control_r(pwm_r);
	}
}
int main(){
	
	pthread_t pthread_A,pthread_B;
	int cnt=0;
	
	if (GPIO_control_setup()==-1){
		return -1;
	}
	//signal(SIGINT,sig_Handler);
	
	printf("Create Thread A \n");
	pthread_create(&pthread_A, NULL, ultrasonic_sensor_thread, NULL);
	
	printf("Create Thread B \n");
	pthread_create(&pthread_B, NULL, motor_control_thread, NULL);	
	
	//pthread_join(ultrasonic_sensor_thread,NULL);
	//pthread_join(motor_control_thread,NULL);
	
	while(1){
		printf("UltraSonic sensor : %6.3lf [cm]\n",u_sensor_data);
		printf("Thread test %3d\n ",cnt);
		cnt++;
		cnt = cnt%100;
		pwm_r = pwm_l = cnt;
		delay(100);
	}
	return 1;	
}

void sig_Handler(int sig){
	printf("\n\n\n\nStop Program and Motor!\n\n\n");
	//motor_control_l(0);
	//motor_control_r(0);
	exit(0);
}
