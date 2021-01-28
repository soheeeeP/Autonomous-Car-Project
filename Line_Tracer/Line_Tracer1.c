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
#include <opencv2/opencv.hpp>

//image processing
using namespace cv;
using namespace std;

#define IMG_Width     640
#define IMG_Height    480

#define ASSIST_BASE_LINE 320
#define ASSIST_BASE_WIDTH 60

int guide_width1=50;
int guide_height1=20;
int guide_center = (IMG_Width/2);

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

Mat Canny_Edge_detection(Mat img){
	Mat mat_blur_img, mat_canny_img;
	blur(img, mat_blur_img, Size(3, 3));
	Canny(mat_blur_img, mat_canny_img, 70, 170, 3);
	
	return mat_canny_img;
}
Mat Draw_Guide_Line(Mat img){

		Mat result_img;
		img.copyTo(result_img);
		
		rectangle(result_img,Point(50,ASSIST_BASE_LINE-ASSIST_BASE_WIDTH), Point(IMG_Width-50,ASSIST_BASE_LINE+ASSIST_BASE_WIDTH),Scalar(0,255,0),1,LINE_AA);
	  
		line(result_img,Point(guide_center-guide_width1,ASSIST_BASE_LINE), Point(guide_center,ASSIST_BASE_LINE),Scalar(0,255,255),1,0);
		line(result_img,Point(guide_center,ASSIST_BASE_LINE), Point(guide_center+guide_width1,ASSIST_BASE_LINE),Scalar(0,255,255),1,0);
	  
		line(result_img,Point(guide_center-guide_width1,ASSIST_BASE_LINE-guide_height1), Point(guide_center-guide_width1,ASSIST_BASE_LINE+guide_height1),Scalar(0,255,255),1,0);
		line(result_img,Point(guide_center+guide_width1,ASSIST_BASE_LINE-guide_height1), Point(guide_center+guide_width1,ASSIST_BASE_LINE+guide_height1),Scalar(0,255,255),1,0);

		line(result_img,Point(IMG_Width/2, ASSIST_BASE_LINE-guide_height1*1.5),Point(IMG_Width/2,ASSIST_BASE_LINE+guide_height1*1.5),Scalar(255,255,255),2,0);				
        
        return result_img;
        
	 }

int main(void){
	int fd;
	int pwm_r=0;
	int pwm_l=0;
	unsigned char test, receive_char;
	
	int img_width=640;
	int img_height=480;
	//////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////  OpenCV  변수 선언 ////////////////////////////////////
	Mat mat_image_org_color;
	Mat mat_image_org_color_overlay;
	Mat mat_image_org_gray;
	Mat mat_image_gray_result;
	Mat mat_image_canny_edge;
	Mat mat_image_line_image = Mat(IMG_Height, IMG_Width, CV_8UC1, Scalar(0));
	Mat image;
	
	Scalar GREEN(0,255,0);
	Scalar RED(0,0,255);
	Scalar BLUE(255,0,0);
	Scalar YELLOW(0,255,255);
	//////////////////////////////////////////////////////////////////////////////////////
	
	mat_image_org_color = imread("/home/pi/SOHEE/AutoCar/C++/Line_Tracer/1/images/line_1_0.jpg"); 
	mat_image_org_color.copyTo(mat_image_org_color_overlay);
	
	mat_image_org_color_overlay = Draw_Guide_Line(mat_image_org_color);
	
	img_width = mat_image_org_color.size().width ;
    img_height = mat_image_org_color.size().height;
	if(mat_image_org_color.empty()){
		cerr << "빈 영상입니다.\n";
		return -1;	  
	}
	        
    namedWindow("Display window", CV_WINDOW_NORMAL);   
	resizeWindow("Display window", img_width,img_height);   
    moveWindow("Display window", 10, 10);	
	
	namedWindow("Gray Image window", CV_WINDOW_NORMAL);   
	resizeWindow("Gray Image window", img_width,img_height);   
    moveWindow("Gray Image window", 700, 10);
	
	namedWindow("Canny Edge Image window", CV_WINDOW_NORMAL);   
	resizeWindow("Canny Edge Image window", img_width,img_height);   
    moveWindow("Canny Edge Image window", 10, 500);	
    
	if (GPIO_control_setup()==-1){
		return -1;
	}
	//signal(SIGINT,sig_Handler);

	while(1){
		
		cv::cvtColor(mat_image_org_color, mat_image_org_gray, CV_RGB2GRAY);	// color to gray conversion
		//threshold(mat_image_org_gray, mat_image_binary, 200, 255, THRESH_BINARY);
		mat_image_canny_edge = Canny_Edge_detection(mat_image_org_gray);
		
		mat_image_org_color_overlay = Draw_Guide_Line(mat_image_org_color);
		
		vector<Vec4i> linesP;
		HoughLinesP(mat_image_canny_edge, linesP, 1, CV_PI/180, 50, 20, 50);
		printf("Line Number : %3d\n", linesP.size());
		for(int i = 0; i < linesP.size(); i++){
			  Vec4i L = linesP[i];
			  line(mat_image_line_image,Point(L[0], L[1]), Point(L[2], L[3]), Scalar(255), 3, LINE_AA);
			  line(mat_image_org_color_overlay, Point(L[0], L[1]), Point(L[2], L[3]), Scalar(0, 0, 255), 1, LINE_AA);
			  printf("L :[%3d,%3d] , [%3d,%3d] \n", L[0], L[1], L[2], L[3]);
		}
		printf("\n\n\n");
	  
		imshow("Display window", mat_image_org_color_overlay);  
		//imshow("Gray Image window", mat_image_org_gray);
		imshow("Gray Image window", mat_image_line_image);
		imshow("Canny Edge Image window", mat_image_canny_edge);  
		
		if(waitKey(10)>0)
			break;

		
	}
	return 0;
}

void sig_Handler(int sig){
	printf("\n\n\n\nStop Program and Motor!\n\n\n");
	motor_control_l(0);
	motor_control_r(0);
	exit(0);
}
