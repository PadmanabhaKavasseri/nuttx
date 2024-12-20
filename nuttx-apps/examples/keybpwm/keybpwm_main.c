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
#include <nuttx/ioexpander/gpio.h>

#include "keybpwm_msg.h"
#include "keybpwm.h"

#include "qrc_msg_management.h"


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
	int channel;
	int fd;
	int duty;
	int freq;
} Pin;

Pin** pins;
int GPIOS[5];
int currStep = -1;
const int max_steps = 6288;

//how to know which gpio to turn on?


int numPins = 0;
/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
/****************************************************************************
 * Private Data
 ****************************************************************************/


static void pwm_devpath(FAR struct pwm_state_s *pwm, FAR const char *devpath)
{
	/* Get rid of any old device path */
	if (pwm->devpath){
		free(pwm->devpath);
	}
	/* Then set-up the new device path by copying the string */
	pwm->devpath = strdup(devpath);
}

static void setPWM(int pin_idx, int duty, int freq){
	// pwm->duty = b16divi(uitoub16(duty), 100000); duty was like 5000
	// printf("\n SETPWM msg: Motor:%d Duty:%d Frequency:%d \n", pin_idx, duty, freq);
	struct pwm_info_s* pwm = malloc(sizeof(struct pwm_info_s));

	pwm->frequency = freq;

	Pin* currPin = pins[pin_idx];
	currPin->duty = duty;

	printf("PinIDX: %d: - Frequency: %d - Duty: %d\n", pin_idx, pwm->frequency, currPin->duty);
	for(int i=0; i<numPins; i++){
		if(pins[i]->timer_group ==  currPin->timer_group){
			pwm->channels[pins[i]->channel-1].channel = pins[i]->channel;
			pwm->channels[pins[i]->channel-1].duty = b16divi(uitoub16(pins[i]->duty), 10000);
			// pwm->channels[pins[i]->channel-1].duty = b16divi(uitoub16(50), 50);
			printf("Channel: %d Duty: %d \n",pwm->channels[pins[i]->channel-1].channel,pwm->channels[pins[i]->channel-1].duty);
		}
	}

	int ret = ioctl(currPin->fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)pwm));
	if (ret < 0){
		printf("pwm_main: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n",
						errno);
		close(currPin->fd);
	}
	ret = ioctl(currPin->fd, PWMIOC_START, 0);
	if (ret < 0){
		printf("pwm_main: ioctl(PWMIOC_START) failed: %d\n", errno);
		close(currPin->fd);
	}
	free(pwm);
}

static void setGPIO(int gpio_num, bool value){
	// printf("Setting GPIO# %d, value: %d\n",gpio_num,value);
	//gpio_num is 1-5
	enum gpio_pintype_e pintype;
	pintype = 3;

	int fd = GPIOS[gpio_num];
	int ret = ioctl(fd, GPIOC_SETPINTYPE, (unsigned long) pintype);
	ret = ioctl(fd, GPIOC_WRITE, (unsigned long)value);

}

static bool getLS(int path){
	
	bool invalue;
	int ret = ioctl(path, GPIOC_READ, (unsigned long)((uintptr_t)&invalue));
	// printf("  Input pin:     Value=%u\n",(unsigned int)invalue);
	return invalue;
}



static void bitBangGPIO(int gpio_num, int steps){
	int dur = 5;
	if(steps == 0){
		return;
	}
	for(int i=0; i<steps; i++){
		setGPIO(gpio_num, true);
		// usleep(dur*2000);
		up_udelay(dur*10);

		setGPIO(gpio_num, false);
		// usleep(dur*2000);
		up_udelay(dur*100);
	}
}

void setStepperMid(){
	setGPIO(0,1);
	bitBangGPIO(1,3144);
	currStep = 3144;
}

static void homeStepper(){
	//move stepper until it hits limit switch then stop
	//cout number of steps it takes to get to other side
	//leave that as global const variable
	//keep another global variable that keeps track of how many steps we have done
	int path, ret;

	path = open("/dev/gpio7", O_RDONLY);
	while(true){
		setGPIO(0,0);
		bitBangGPIO(1,1);
		if(!getLS(path)){
			printf("Hit Stop!\n");
			currStep = 0;
			setStepperMid();
			break;
		}
	}
}

static int safeStepCheck(int steps_given, int direction){
	int res = 0;
	if(direction == 1){
		//need to add steps
		res = currStep + steps_given;
		if(res>max_steps){
			int moveFwd = max_steps-currStep;
			currStep = max_steps;
			return moveFwd; //move to max steps from current position
		}
		else{
			currStep = currStep + steps_given;
			return steps_given;
		}
	}
	else{
		//need to subtract steps
		res = currStep - steps_given;
		if(res < 0){
			int moveBack = currStep;
			currStep = 0;
			return moveBack; //so we move all they way back to 0
		}
		else{
			currStep = currStep - steps_given;
			return steps_given;
		}
	}
}

static void parseStepper(struct motor_msg_s *msg){
	// int num_steps = msg->data.stepper.angle; // / (1.8/8);
	int num_steps = safeStepCheck((int)msg->data.stepper.angle,msg->data.stepper.direction);
	printf("Num steps to move: %d\n", num_steps);
	setGPIO(msg->data.stepper.motor-3,msg->data.stepper.direction);
	bitBangGPIO(msg->data.stepper.motor-2,num_steps);
	printf("Currsteps: %d\n",currStep);
}


//how to stop a specific channel does it just stop the entire timer group?
static void stopPWM(FAR struct pwm_info_s* pwm, int fd){
	
	int ret = ioctl(fd, PWMIOC_STOP, 0);
	if(ret < 0){
		printf("pwm_main: ioctl(PWMIOC_STOP) failed: %d\n", errno);
		close(fd);
	}
	printf("STOP\n");
}

static void startPWM(FAR struct pwm_info_s* pwm, int fd){
	int ret = ioctl(fd, PWMIOC_START, 0);
	if (ret < 0){
		printf("pwm_main: ioctl(PWMIOC_START) failed: %d\n", errno);
		close(fd);
	}
	printf("START\n");
}

static void initGPIOS(){
	int fd=0;
	fd = open(CONFIG_EXAMPLES_KYBPWM_GPIO1_DEVPATH, O_RDWR);
	GPIOS[0] = fd;
	fd = open(CONFIG_EXAMPLES_KYBPWM_GPIO2_DEVPATH, O_RDWR);
	GPIOS[1] = fd;
	fd = open(CONFIG_EXAMPLES_KYBPWM_GPIO3_DEVPATH, O_RDWR);
	GPIOS[2] = fd;
	fd = open(CONFIG_EXAMPLES_KYBPWM_GPIO4_DEVPATH, O_RDWR);
	GPIOS[3] = fd;
	// fd = open(CONFIG_EXAMPLES_KYBPWM_GPIO5_DEVPATH, O_RDWR);
	// GPIOS[4] = fd;
}

static Pin* createPin(int timer_group, int channel, FAR const char *devpath){
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
	p->duty = 5000;

	numPins++;

	return p;
}

static Pin** initPins(){
    //should read Konfig file.. will add that later
    Pin** pins = malloc(8 * sizeof(Pin*)); // Allocate memory for 8 pointers to Pin
    pins[0] = createPin(13, 1, CONFIG_EXAMPLES_KYBPWM_TIM13_DEVPATH); //17 ///should be a file descriptor instead
	pins[1] = createPin(5, 4, CONFIG_EXAMPLES_KYBPWM_TIM5_DEVPATH);
	pins[2] = createPin(5, 1, CONFIG_EXAMPLES_KYBPWM_TIM5_DEVPATH);
	pins[3] = createPin(4, 4, CONFIG_EXAMPLES_KYBPWM_TIM4_DEVPATH);
	pins[4] = createPin(4, 3, CONFIG_EXAMPLES_KYBPWM_TIM4_DEVPATH);
	pins[5] = createPin(4, 2, CONFIG_EXAMPLES_KYBPWM_TIM4_DEVPATH);
	pins[6] = createPin(3, 4, CONFIG_EXAMPLES_KYBPWM_TIM3_DEVPATH);
	pins[7] = createPin(3, 3, CONFIG_EXAMPLES_KYBPWM_TIM3_DEVPATH);
    return pins;
}

// void deletePins(Pin** pins){
//     for(int i = 0; i < numPins; i++){
//         deletePin(pins[i]);
//     }
//     free(pins);
// }

/*qrc message callback*/
static void keybpwm_msg_parse(struct qrc_pipe_s *pipe, struct motor_msg_s *msg);
static void keybpwm_qrc_msg_cb(struct qrc_pipe_s *pipe,void * data, size_t len, bool response);


static void keybpwm_qrc_msg_cb(struct qrc_pipe_s *pipe,void * data, size_t len, bool response)
{
  struct motor_msg_s *msg;

  if (pipe == NULL || data ==NULL)
	{
		return;
	}
	printf("KEYBPWM DEBUG:: len:%d",len);
  if (len == sizeof(struct motor_msg_s))
    {
      msg = (struct motor_msg_s *)data;
      keybpwm_msg_parse(pipe, msg);
    }
   else
   {
	  printf("ERROR: keybpwm_qrc_msg_cb msg size mismatch \n");
   }
}

/* you can modify this function to fit your requirement */
static void keybpwm_msg_parse(struct qrc_pipe_s *pipe, struct motor_msg_s *msg)
{
	struct motor_msg_s reply_msg;

	switch (msg->type)
    {
		case MOTOR_BLDC_LA:
		{
			printf("\n Got MOTOR_BLDC_LA msg: Motor:%d On/Off:%d Duty:%f Frequency:%f \n", msg->data.bldc_la.motor, msg->data.bldc_la.on_off, msg->data.bldc_la.duty, msg->data.bldc_la.freq);
			if(msg->data.bldc_la.on_off){
				setPWM(msg->data.bldc_la.motor,msg->data.bldc_la.duty*100,msg->data.bldc_la.freq);
			}
			break;
		} 
		case MOTOR_STEPPER:
		{
			printf("\n Got STEPPER msg: Motor:%d Sleep:%d Direction:%d Angle:%f \n", msg->data.stepper.motor, msg->data.stepper.sleep, msg->data.stepper.direction, msg->data.stepper.angle);
			
			parseStepper(msg);

			// if(msg->data.stepper.on_off){

			// 	// setPWM(msg->data.stepper.motor,msg->data.stepper.duty*100,msg->data.stepper.freq);
			// 	// bitBangGPIO(msg->data.stepper.motor-2,msg->data.stepper.num_steps);
			// 	// setGPIO(msg->data.stepper.motor-3,msg->data.stepper.direction);
			// 	// setGPIO(0,0);
			// }
			break;
		}
		
		default:
			printf("keybpwm_msg_parse: unknow message type %d\n", msg->type);
	}
}



int main(int argc, FAR char *argv[])
{
	struct pwm_info_s info;
	struct stlitMSG msg;
	int ser_fd;
	int ret;
	char buf[1024];

	pins = initPins();
	initGPIOS();

	setPWM(0, 850, 50);
	setPWM(1, 500, 50);
	setPWM(2, 500, 50);
	//setPWM(3, 500, 50);
	//setPWM(4, 500, 50);
	//setPWM(5, 500, 50);
	//setPWM(6, 500, 50);
	//setPWM(7, 500, 50);
	
	//setGPIO(1,0);

	
	//setPWM(1, 500, 50);

	/* init qrc */
  	char pipe_name[] = PWM_PIPE;
	struct qrc_pipe_s *pipe;
	if (false == init_qrc_management())
    {
      syslog(LOG_ERR, "main: qrc init failed\n");
      return -1;
    }

	pipe =  qrc_get_pipe(pipe_name);
	if (pipe == NULL)
    {
      syslog(LOG_ERR,"qrc get pipe error\n");
      return -1;
    }

	/* register msg callback */
	if (!qrc_register_message_cb(pipe, keybpwm_qrc_msg_cb))
    {
      syslog(LOG_ERR,"qrc register robot control cb error\n");
      return -1;
    }

	syslog(LOG_INFO, "main: keybpwm startup completed\n");

	printf("This command was able to run\n");
	printf("Starting Homing\n");
	homeStepper(); 
	// currStep = 3144;
	printf("Done Homing\n");
	
	//setPWM(1, 500, 50);
	
	qrc_pipe_threads_join();
	
	return 0;
}
