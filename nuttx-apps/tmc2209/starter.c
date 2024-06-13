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

#include "TMC2209.h"

#define BUFCOUNT(head, tail, size) (((head) >= (tail)) ? ((head) - (tail)) : ((size) - (tail) + (head)))


int fd;

TMC_uart_write_datagram_t *tmc_uart_read (trinamic_motor_t driver, TMC_uart_read_datagram_t *datagram){
    TMC_uart_write_datagram_t * res = {0};
	volatile uint32_t dly = 50;
	// write the read_datagram_t
	printf("uart_read start\n"); fflush(stdout);
	ssize_t n = write(fd, datagram->data, sizeof(TMC_uart_read_datagram_t));
	if (n < 0){
		printf("write failed: %d\n", errno);
		fflush(stdout);
	}
	printf("uart_read write complete\n"); fflush(stdout);

	while(--dly);
	up_udelay(2000000);
	printf("Delay Completed\n");
	// count data being written out
	// add some delay
	// reset read buffer? what if the data comes in at that time?
	// wait for response for 3ms. Why do we have to wait for tx_buffer_count to be larger than 8?

	//when we know the data is ready
	// FAR char *buf = (FAR char *)malloc(CONFIG_TMC2209_BUFSIZE);
	char c;
	for(int i=0; i<4; i++){
		printf("in for\n");
		n = read(fd,&c,1);
		res->data[i] = c;
		printf("Rcvd: %c",c);
	}
	// for(int j=0; j<4; j++){
	// 	printf("Data: %s",res->data[j]);
	// }

	printf("\nuart_read read complete\n"); fflush(stdout);


    return res;

	// datagram 
}

void tmc_uart_write (trinamic_motor_t driver, TMC_uart_write_datagram_t *datagram){
    printf("uart_write start\n"); fflush(stdout);
	ssize_t n = write(fd, datagram->data, sizeof(TMC_uart_write_datagram_t));
	//wait while the datagram is delivered
	printf("uart_write write complete\n"); fflush(stdout);
}

int main(int argc, FAR char *argv[])
{
	FAR char *devpath = CONFIG_TMC2209_DEVPATH;
	
	fd = open(devpath, O_RDWR);
	if(fd < 0){
		fprintf(stderr, "ERROR: write open failed: %d\n", errno);
	}


  	printf("Starter.\n");
	TMC2209_t * driver;
	TMC2209_Init(driver);
	printf("Finished Initttt\n"); fflush(stdout);
  
	return 0;
}
