/****************************************************************************
 * apps/examples/pwm/pwm_main.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <inttypes.h>

#include <nuttx/timers/pwm.h>

#include "keybpwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_PWM_MULTICHAN
#  if CONFIG_PWM_NCHANNELS > 1
#    if CONFIG_EXAMPLES_PWM_CHANNEL1 == CONFIG_EXAMPLES_PWM_CHANNEL2
#      error "Channel numbers must be unique"
#    endif
#  endif
#  if CONFIG_PWM_NCHANNELS > 2
#    if CONFIG_EXAMPLES_PWM_CHANNEL1 == CONFIG_EXAMPLES_PWM_CHANNEL3 || \
				CONFIG_EXAMPLES_PWM_CHANNEL2 == CONFIG_EXAMPLES_PWM_CHANNEL3
#      error "Channel numbers must be unique"
#    endif
#  endif
#  if CONFIG_PWM_NCHANNELS > 3
#    if CONFIG_EXAMPLES_PWM_CHANNEL1 == CONFIG_EXAMPLES_PWM_CHANNEL4 || \
				CONFIG_EXAMPLES_PWM_CHANNEL2 == CONFIG_EXAMPLES_PWM_CHANNEL4 || \
				CONFIG_EXAMPLES_PWM_CHANNEL3 == CONFIG_EXAMPLES_PWM_CHANNEL4
#      error "Channel numbers must be unique"
#    endif
#  endif
#  if CONFIG_PWM_NCHANNELS > 4
#    if CONFIG_EXAMPLES_PWM_CHANNEL1 == CONFIG_EXAMPLES_PWM_CHANNEL5 || \
				CONFIG_EXAMPLES_PWM_CHANNEL2 == CONFIG_EXAMPLES_PWM_CHANNEL5 || \
				CONFIG_EXAMPLES_PWM_CHANNEL3 == CONFIG_EXAMPLES_PWM_CHANNEL5 || \
				CONFIG_EXAMPLES_PWM_CHANNEL4 == CONFIG_EXAMPLES_PWM_CHANNEL5
#      error "Channel numbers must be unique"
#    endif
#  endif
#  if CONFIG_PWM_NCHANNELS > 5
#    if CONFIG_EXAMPLES_PWM_CHANNEL1 == CONFIG_EXAMPLES_PWM_CHANNEL6 || \
				CONFIG_EXAMPLES_PWM_CHANNEL2 == CONFIG_EXAMPLES_PWM_CHANNEL6 || \
				CONFIG_EXAMPLES_PWM_CHANNEL3 == CONFIG_EXAMPLES_PWM_CHANNEL6 || \
				CONFIG_EXAMPLES_PWM_CHANNEL4 == CONFIG_EXAMPLES_PWM_CHANNEL6 || \
				CONFIG_EXAMPLES_PWM_CHANNEL5 == CONFIG_EXAMPLES_PWM_CHANNEL6
#      error "Channel numbers must be unique"
#    endif
#  endif
#  if CONFIG_PWM_NCHANNELS > 6
#    error "Too many PWM channels"
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

//remove before compile
// #define CONFIG_PWM_MULTICHAN 
// #define CONFIG_PWM_NCHANNELS 4

struct pwm_state_s
{
	bool      initialized;
	FAR char *devpath;
#ifdef CONFIG_PWM_MULTICHAN
	uint8_t   channels[CONFIG_PWM_NCHANNELS];
	uint8_t   duties[CONFIG_PWM_NCHANNELS];
#else
	uint8_t   duty;
#endif
	uint32_t  freq;
#ifdef CONFIG_PWM_PULSECOUNT
	uint32_t  count;
#endif
	int       duration;
};


struct stlitMSG {
	int on_off;
	int motor;
	int freq;
	int duty;
};

typedef struct {
	int timer_group;
	int channel; //this should be a fd
	int fd;
} Pin;


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
/****************************************************************************
 * Private Data
 ****************************************************************************/

// static struct pwm_state_s g_pwmstate1;
// static struct pwm_state_s g_pwmstate2;//timer group


static void pwm_devpath(FAR struct pwm_state_s *pwm, FAR const char *devpath)
{
	/* Get rid of any old device path */

	if (pwm->devpath)
		{
			free(pwm->devpath);
		}

	/* Then set-up the new device path by copying the string */

	pwm->devpath = strdup(devpath);
}

void decodeBmsg(char* binaryMsg, struct stlitMSG* msg) {
	char binMsgStr[33];
	char msg_start[4], motor[5], on_off[2], freq[8], duty[15], msg_stop[4];
	int idx = 0;
	// printf("Printing Binary MSG in decodebmsg:: ");
	//loop through binary message and convert to string
	for (ssize_t i = 0; i < 4; ++i) {
		for (int j = 7; j >= 0; --j) {
			int bit = (binaryMsg[i] >> j) & 1; //get most significant bit & 1 to convert to binary string
			// printf("%d", bit);
			idx = (8*i) + (7-j);
			binMsgStr[idx] = bit + '0';  // Add '0' to convert the integer to a character
		}
			// printf(" ");
	}
	printf("\n");
	binMsgStr[32] = '\0';  // Null-terminate the string
	// printf("Bin Msg Str: ");
	printf("%s\n", binMsgStr);
	

	// Extract the binary strings
	strncpy(msg_start, binMsgStr, 3); msg_start[3] = '\0';
	strncpy(motor, binMsgStr + 3, 4); motor[4] = '\0';
	strncpy(on_off, binMsgStr + 7, 1); on_off[1] = '\0';
	strncpy(freq, binMsgStr + 8, 7); freq[7] = '\0';
	strncpy(duty, binMsgStr + 15, 14); duty[14] = '\0';
	strncpy(msg_stop, binMsgStr + 29, 3); msg_stop[3] = '\0';

	// Convert frequency from binary to integer
    int freq_int = strtol(freq, NULL, 2);
	int duty_int = strtol(duty, NULL, 2);
	int on_off_int = strtol(on_off, NULL, 2);
	int motor_int = strtol(motor, NULL, 2);

	// printf("Decoded Message:\n");
    // printf("Start: %s\n", msg_start);
    // printf("Motor: %d (binary: %s)\n", motor_int, motor);
	// printf("On/Off: %d (binary: %s)\n", on_off_int, on_off);
    // printf("Frequency: %d (binary: %s)\n", freq_int, freq);
    // printf("Duty: %d (binary: %s)\n", duty_int, duty);
    // printf("Stop: %s\n", msg_stop);

	
	msg->duty = duty_int;
	msg->freq = freq_int;
	msg->on_off = on_off_int;
	msg->motor = motor_int; 

}

//assuming pwm is started
//only needs pwm stats and fd ONLY
void setPWM(FAR struct pwm_info_s* pwm, Pin* p, int duty, int freq){
	// pwm->duty = b16divi(uitoub16(duty), 100000);
	pwm->frequency = freq;

	
	int chIDX = p->channel-1;
	pwm->channels[chIDX].channel = p->channel;
	pwm->channels[chIDX].duty = b16divi(uitoub16(duty), 100000);


	int ret = ioctl(p->fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)pwm));
	if (ret < 0){
		printf("pwm_main: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n",
						errno);
		close(p->fd);
	}
	ret = ioctl(p->fd, PWMIOC_START, 0);
	if (ret < 0){
		printf("pwm_main: ioctl(PWMIOC_START) failed: %d\n", errno);
		close(p->fd);
	}
}

//how to stop a specific channel does it just stop the entire timer group?
void stopPWM(FAR struct pwm_info_s* pwm, int fd){
	
	int ret = ioctl(fd, PWMIOC_STOP, 0);
	if(ret < 0){
		printf("pwm_main: ioctl(PWMIOC_STOP) failed: %d\n", errno);
		close(fd);
	}
	printf("STOP\n");
}

void startPWM(FAR struct pwm_info_s* pwm, int fd){
	int ret = ioctl(fd, PWMIOC_START, 0);
	if (ret < 0){
		printf("pwm_main: ioctl(PWMIOC_START) failed: %d\n", errno);
		close(fd);
	}
	printf("START\n");
}

// void initPWM(){
// 	//PWM1
// 	if (!g_pwmstate1.initialized)
// 	{
// 		// g_pwmstate1.duty        = CONFIG_EXAMPLES_PWM_DUTYPCT;
// 		g_pwmstate1.freq        = CONFIG_EXAMPLES_PWM_FREQUENCY;
// 		g_pwmstate1.duration    = CONFIG_EXAMPLES_PWM_DURATION;
// 		g_pwmstate1.initialized = true;
// 	}
// 	if (!g_pwmstate1.devpath)
// 	{
// 		/* No.. use the default device */
// 		pwm_devpath(&g_pwmstate1, CONFIG_EXAMPLES_KYBPWM1_DEVPATH);
// 	}
// 	//-------------------------------------------------------------

// 	//PWM2
// 	if (!g_pwmstate2.initialized)
// 	{
// 		// g_pwmstate2.duty       = CONFIG_EXAMPLES_PWM_DUTYPCT;
// 		g_pwmstate2.freq        = CONFIG_EXAMPLES_PWM_FREQUENCY;
// 		g_pwmstate2.duration    = CONFIG_EXAMPLES_PWM_DURATION;
// 		g_pwmstate2.initialized = true;
// 	}
// 	if (!g_pwmstate2.devpath)
// 	{
// 		/* No.. use the default device */
// 		pwm_devpath(&g_pwmstate2, CONFIG_EXAMPLES_KYBPWM2_DEVPATH);
// 	}
// 	//-------------------------------------------------------------
// }

//TODO... FREE MEMORYYYY!!!!!! DO NOT FORGETTTTTT
Pin* createPin(int timer_group, int channel, FAR const char *devpath){
	Pin* p = (Pin*) malloc(sizeof(Pin));
	//create fd here
	p->timer_group = timer_group; //theres no timer group to dev path map
	p->channel = channel;
	//end goal is to just have a fd element that setPWM can write to
	//generate fd


	int fd = open(devpath, O_RDONLY);
	printf("PWM DEVPATH: %s\n", devpath);
	if (fd < 0)
	{
		printf("pwm_main: open %s failed: %d\n", devpath, errno);
		fflush(stdout);
	}
	p->fd = fd;

	return p;
}


Pin** initPins(){
    //should read Konfig file.. will add that later
    Pin** pins = malloc(8 * sizeof(Pin*)); // Allocate memory for 8 pointers to Pin
    pins[0] = createPin(13, 1, CONFIG_EXAMPLES_KYBPWM_TIM13_DEVPATH); //17 ///should be a file descriptor instead
	pins[1] = createPin(5, 4, CONFIG_EXAMPLES_KYBPWM_TIM5_DEVPATH);
	pins[2] = createPin(5, 1, CONFIG_EXAMPLES_KYBPWM_TIM5_DEVPATH);

    // Initialize other pins...
    return pins;
}

void deletePins(Pin** pins){
    for(int i = 0; i < 8; i++){
        deletePin(pins[i]);
    }
    free(pins);
}

int main(int argc, FAR char *argv[])
{
	struct pwm_info_s info;
	struct stlitMSG msg;
	int ser_fd;
	int ret;
	char buf[4];//4 bytes for streamlit message

	//initialize serial port
	ser_fd = open("/dev/ttyS2", O_RDONLY | O_NONBLOCK);
	if (ser_fd < 0) {
		perror("unable to open serial port");
		return 1;
	}

	Pin** pins = initPins();

	setPWM(&info, pins[0], 5000, 50);
	setPWM(&info, pins[1], 5000, 50);
	setPWM(&info, pins[2], 5000, 50);

	//read serial to control output
	while(1){
		ssize_t n = read(ser_fd, buf, sizeof(buf));
		if(n<0){
			if (errno = EAGAIN){
				//No data available, try again
				usleep(10000);
				continue;
			}
			else{
				perror("read error in while loop");
				break;
			}
		}
		else {
			decodeBmsg(buf, &msg);
			printf("On/Off: %d Motor: %d Duty: %d Freq: %d\n", msg.on_off, msg.motor, msg.duty, msg.freq);
			//make this a function that just applies the message... maybe this is okay

			if(msg.on_off){ //1 is on
				// startPWM(&info,fd); // I dont think i need this
				setPWM(&info, pins[msg.motor-1], msg.duty, msg.freq);
			}
			else{
				//do we need to set a specific channel here to 0? how to deal with servos vs other motors... cant just set it to 0.
				stopPWM(&info, pins[msg.motor-1]->fd);
			}
		}
	}


	deletePins(pins);
	fflush(stdout);
	return OK;
errout:
	fflush(stdout);
	return ERROR;
}