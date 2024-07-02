#include <nuttx/config.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/serial/serial.h>
#include <nuttx/clock.h>

#include <sys/ioctl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>

#include "TMC2209.h"



uint8_t array[19][8] = {
        {5, 0, 128, 0, 0, 1, 192, 246},
        {5, 0, 144, 0, 1, 31, 16, 223},
        {5, 0, 236, 16, 1, 0, 83, 151},
        {5, 0, 240, 193, 13, 0, 36, 6},
        {5, 0, 194, 0, 0, 0, 0, 69},
        {5, 0, 145, 0, 0, 0, 20, 31},
        {5, 0, 147, 0, 0, 0, 0, 15},
        {5, 0, 162, 0, 0, 0, 0, 14},
        {5, 0, 148, 0, 0, 0, 0, 52},
        {5, 0, 192, 0, 0, 0, 0, 141},
        {5, 0, 194, 0, 0, 0, 0, 69},
        {5, 0, 144, 0, 1, 0, 0, 91},
        {5, 0, 236, 16, 1, 0, 80, 217},
        {5, 0, 240, 193, 9, 0, 36, 69},
        {5, 0, 240, 193, 1, 0, 36, 231}, //idx14
        {5, 0, 236, 22, 1, 0, 80, 140},//set micro
        {5, 0, 144, 0, 1, 31, 0, 231},//set current
        {5, 0, 236, 22, 1, 0, 83, 194},//enable
        {5, 0, 162, 0, 0, 0, 200, 119}//vel
    };

int fd;




void TMC2209_init(){
	for(int i =0; i<15; i++){
		for(int j = 0; j < 8; j++) {
			write(fd, &array[i][j], sizeof(array[i][j]));
		}
		usleep(1000000);
	}
	printf("Init Completed\n");
}

void TMC2209_set_microsteps(){
	uint8_t msg[8] = {5, 0, 236, 22, 1, 0, 80, 140};
	for(int j = 0; j < 8; j++) {
		write(fd, &msg[j], sizeof(msg[j]));
	}
	usleep(1000000);
	printf("Micro Steps Completed\n");
}

void TMC2209_set_current(){
	uint8_t msg[8] = {5, 0, 144, 0, 1, 31, 0, 231};
	for(int j = 0; j < 8; j++) {
		write(fd, &msg[j], sizeof(msg[j]));
	}
	usleep(1000000);
	printf("Set Current\n");
}

void TMC2209_enable(){
	uint8_t msg[8] = {5, 0, 236, 22, 1, 0, 83, 194};
	for(int j = 0; j < 8; j++) {
		write(fd, &msg[j], sizeof(msg[j]));
	}
	usleep(1000000);
	printf("Driver Enabled\n");
}

void TMC2209_velocity(){
	uint8_t msg[8] = {5, 0, 162, 0, 0, 0, 200, 119};
	for(int j = 0; j < 8; j++) {
		write(fd, &msg[j], sizeof(msg[j]));
	}
	usleep(1000000);
	printf("Velocity set. Go!\n");
}

void custom_init(){
	// Print the array
	// u_int8_t x = 5;
	// u_int8_t y = 192;
	char hex_str[3];
    for(int i = 0; i < 19; i++) {
        for(int j = 0; j < 8; j++) {
			
			// sprintf(hex_str, "%02X", array[i][j]);
            printf("%d ", array[i][j]);
			write(fd, &array[i][j], sizeof(array[i][j]));
			
        }
        printf("\n");
		usleep(100000);
    }
	// write(fd,&x,sizeof(x));
	// write(fd,&y,sizeof(y));
}



int main(int argc, FAR char *argv[])
{
	FAR char *devpath = CONFIG_TMC2209_DEVPATH;
	
	fd = open(devpath, O_RDWR);
	if(fd < 0){
		fprintf(stderr, "ERROR: write open failed: %d\n", errno);
	}


  	printf("Starter.\n");
	// TMC2209_t * driver;
	// bool ret = TMC2209_Init(driver);
	// if(ret){
	// 	printf("Finished True\n"); fflush(stdout);
	// }
	// else{
	// 	printf("Finished False\n"); fflush(stdout);
	// }

	custom_init();

	// TMC2209_init();
	// TMC2209_set_microsteps();
	// TMC2209_set_current();
	// TMC2209_enable();
	// TMC2209_velocity();

	printf("Start of Custom Driver\n");
	init(fd);
	setMicrostepsPerStepPowerOfTwo(2);
	setRunCurrent(100);
	// enableCoolStep();
	enable();
	uint32_t vel = 200;
	
	moveAtVelocity(vel);
	
  
	return 0;
}



