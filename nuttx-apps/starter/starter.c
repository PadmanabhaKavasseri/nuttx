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


int main(int argc, FAR char *argv[])
{
  printf("Starter.\n");

  for(int i=0; i<5; i++){
    printf("sleep on\n"); fflush(stdout);
		usleep(500*1000);
		// struct timespec to_sleep = { 1, 0 }; // Sleep for 1 second
		// while ((nanosleep(&to_sleep, &to_sleep) == -1) && (errno == EINTR));
		// up_udelay(500);
		// set(gpio_num, false);
		printf("sleep off\n"); fflush(stdout);
		usleep(500*1000);
		// struct timespec to_sleep = { 1, 0 }; // Sleep for 1 second
		// while ((nanosleep(&to_sleep, &to_sleep) == -1) && (errno == EINTR));
		// up_udelay(500);
  }
  
  return 0;
}
