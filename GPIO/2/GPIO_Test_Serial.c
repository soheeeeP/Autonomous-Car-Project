#include <stdio.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringSerial.h>

#define GPIO0 0	// Physical 11
#define GPIO3 3	// Physical 15

#define baud_rate 115200

int main(void){
	int fd;
	unsigned char test, receive_char;
	
	if (wiringPiSetup() == -1){
		printf("WiringPi Setup error !\n");
		return -1;
	}
	
	if ((fd = serialOpen("/dev/ttyS0", baud_rate)) < 0){
		printf("UART open error ! \n");
		return -1;
	}
	
	pinMode(GPIO0, INPUT);
	pinMode(GPIO3, OUTPUT);
	
	/*
	printf("GPIO PIN3 : LOW \n");
	digitalWrite(GPIO3, LOW);
	delay(1000); */
	
	test = 'A';
	
	while (1){
		serialPutchar(fd, test);
		delay(100);
		
		if (serialDataAvail(fd)){
			receive_char = serialGetchar(fd);
			printf(" Received char : %d %c\n", receive_char, receive_char);
		}
	}
	
	return 0;
}
