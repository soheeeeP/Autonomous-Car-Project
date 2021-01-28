#include <stdio.h>
#include <wiringPi.h>

#define GPIO0 0	// Physical 11
#define GPIO3 3	// Physical 15

int main(void){
	if (wiringPiSetup() == -1){
		printf("WiringPi Setup error !\n");
		return -1;
	}
	
	pinMode(GPIO0, INPUT);
	pinMode(GPIO3, OUTPUT);
	
	printf("GPIO PIN3 : LOW \n");
	digitalWrite(GPIO3, LOW);
	delay(1000);
	
	while (1){
		printf("GPIO0 Pin Read : %1d \n", digitalRead(GPIO0));
		/*
		printf("GPIO PIN3 : LOW \n");
		digitalWrite(GPIO3, LOW);
		delay(1000);
		*/
	}
	
	return 0;
}
