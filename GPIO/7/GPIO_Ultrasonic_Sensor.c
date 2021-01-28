#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <termio.h>
#include <softPwm.h>

#include <opencv2/opencv.hpp>
#include <iostream>  
using namespace cv;
using namespace std;

#define IMG_Width     640
#define IMG_Height    480

#define GPIO0 0	//Physical 11
#define GPIO3 3	//Physical 15

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

//Serial Communication
#define baud_rate 115200

void sig_Handler(int sig);

int getch(void){
	int ch;
	struct termios buf;
	struct termios save;
	
	tcgetattr(0,&save);
	buf=save;
	buf.c_lflag &= ~(ICANON|ECHO);
	buf.c_cc[VMIN]=1;
	buf.c_cc[VTIME]=0;
	tcsetattr(0,TCSAFLUSH,&buf);
	ch=getchar();
	tcsetattr(0,TCSAFLUSH,&save);
	return ch;
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
	
	return 0;
}

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

float ultrasonic_sensor(void){
	long start_time, end_time;
	long temp_time1, temp_time2;
	
	int duration;
	int ret_duration;
	
	float distance;
	
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
		
		if(duration > 1000) break;
	}
	
	start_time=micros();
	//wait until ECHO pin is LOW
	while(digitalRead(ECHO) == HIGH)
	{
		temp_time2 = micros();
		duration = temp_time2 - start_time;
		if(duration > 2000) break;
		//printf("%d\n",duration);
	}
	end_time=micros();
	duration = end_time - start_time;
	distance = duration / 58 ;	
	 	
	return distance;
	
}
int main(void){
	int fd;
	int pwm_r=0;
	int pwm_l=0;
	unsigned char test, receive_char;
	
	int img_width, img_height;
	img_width = 640;
	img_height = 480;

	Mat mat_image_org_color;
	
	if (GPIO_control_setup()==-1){
		return -1;
	}
	//signal(SIGINT,sig_Handler);

	while(1){		
		printf("%6.3lf [cm]\n",ultrasonic_sensor());
		delay(100);
		//test = getch();
		test='y';
		switch(test){
			case 'w':	//accelerate
				motor_control_r(pwm_r);
				motor_control_l(pwm_l);
				pwm_r++;
				pwm_l++;
				if(pwm_r>100) pwm_r=100;
				if(pwm_l>100) pwm_l=100;
				break;
			case 's':	//stop
				motor_control_r(0);
				motor_control_l(0);
				break;
			case 'x':
				motor_control_r(-pwm_r);
				motor_control_l(-pwm_l);
				pwm_r--;
				pwm_l--;
				if(pwm_r<-100) pwm_r=-100;
				if(pwm_l<-100) pwm_l=-100;
				break;
			case 'a':	//left
				motor_control_r(pwm_r);
				motor_control_l(pwm_l);
				pwm_r++;
				pwm_l--;
				if(pwm_r>100) pwm_r=100;
				if(pwm_l<-100) pwm_l=-100;
				break;
			case 'd':	//right
				motor_control_r(pwm_r);
				motor_control_l(pwm_l);
				pwm_r--;
				pwm_l++;
				if(pwm_r<-100) pwm_r=-100;
				if(pwm_l>100) pwm_l=100;
				break;
			case 'p':
				motor_control_r(0);
				motor_control_l(0);
				return 0;
				return 0;
				break;
		}
		
	}
	return 0;
}

void sig_Handler(int sig){
	printf("\n\n\n\nStop Program and Motor!\n\n\n");
	motor_control_l(0);
	motor_control_r(0);
	exit(0);
}
