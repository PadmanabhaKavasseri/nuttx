#include <nuttx/config.h>
#include <nuttx/ioexpander/gpio.h>

#include <sys/ioctl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>

#ifndef CONFIG_GPIO_BITBANG_GPIO1_DEVPATH
#  define CONFIG_GPIO_BITBANG_GPIO1_DEVPATH "/dev/gpio3"
#endif

#ifndef CONFIG_GPIO_BITBANG_GPIO2_DEVPATH
#  define CONFIG_GPIO_BITBANG_GPIO2_DEVPATH "/dev/gpio4"
#endif

#ifndef CONFIG_GPIO_BITBANG_GPIO3_DEVPATH
#  define CONFIG_GPIO_BITBANG_GPIO3_DEVPATH "/dev/gpio5"
#endif

#ifndef CONFIG_GPIO_BITBANG_GPIO4_DEVPATH
#  define CONFIG_GPIO_BITBANG_GPIO4_DEVPATH "/dev/gpio6"
#endif

#ifndef CONFIG_GPIO_BITBANG_GPIO5_DEVPATH
#  define CONFIG_GPIO_BITBANG_GPIO5_DEVPATH "/dev/gpio7"
#endif

int GPIOSs[5];

static void set(int gpio_num, bool value){
	printf("Setting GPIO# %d, value: %d\n",gpio_num,value);
	//gpio_num is 1-5
	enum gpio_pintype_e pintype;
	pintype = 3;

	int fd = GPIOSs[gpio_num];
	int ret = ioctl(fd, GPIOC_SETPINTYPE, (unsigned long) pintype);
	ret = ioctl(fd, GPIOC_WRITE, (unsigned long)value);

}

static void bbGPIO(int gpio_num, int steps){
	for(int i=0; i<steps; i++){
		// set(gpio_num, true);
		printf("sleep on\n"); fflush(stdout);
		// usleep(1000);
		// struct timespec to_sleep = { 1, 0 }; // Sleep for 1 second
		// while ((nanosleep(&to_sleep, &to_sleep) == -1) && (errno == EINTR));
		up_udelay(500);

		// printf("sleep off\n"); fflush(stdout);
		// set(gpio_num, false);
		printf("sleep off\n"); fflush(stdout);
		// usleep(1000);
		// struct timespec to_sleep = { 1, 0 }; // Sleep for 1 second
		// while ((nanosleep(&to_sleep, &to_sleep) == -1) && (errno == EINTR));
		up_udelay(500);

		// printf("sleep off\n"); fflush(stdout);
	}
}


static void initialize(void){
	int fd=0;
	fd = open(CONFIG_GPIO_BITBANG_GPIO1_DEVPATH, O_RDWR);
	GPIOSs[0] = fd;
	fd = open(CONFIG_GPIO_BITBANG_GPIO2_DEVPATH, O_RDWR);
	GPIOSs[1] = fd;
	fd = open(CONFIG_GPIO_BITBANG_GPIO3_DEVPATH, O_RDWR);
	GPIOSs[2] = fd;
	fd = open(CONFIG_GPIO_BITBANG_GPIO4_DEVPATH, O_RDWR);
	GPIOSs[3] = fd;
	fd = open(CONFIG_GPIO_BITBANG_GPIO5_DEVPATH, O_RDWR);
	GPIOSs[4] = fd;
}

int main(int argc, FAR char *argv[])
{
  printf("Starting GPIO Bitbang.\n");
//   initialize();
  bbGPIO(0,5);
  
  return 0;
}
