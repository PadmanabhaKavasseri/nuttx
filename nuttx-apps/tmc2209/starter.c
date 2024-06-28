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

#define BUFCOUNT(head, tail, size) (((head) >= (tail)) ? ((head) - (tail)) : ((size) - (tail) + (head)))

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
        {5, 0, 240, 193, 1, 0, 36, 231},
        {5, 0, 236, 22, 1, 0, 80, 140},
        {5, 0, 144, 0, 1, 31, 0, 231},
        {5, 0, 236, 22, 1, 0, 83, 194},
        {5, 0, 162, 0, 0, 0, 200, 119}
    };

int fd;
int get_tx_buffer_count(){
	//create file pointer
	FAR struct file *filep;
	int ret = fs_getfilep(fd, &filep);
	if(ret < 0){
		printf("Failed to get file structure: %d\n", ret);
	}
	//todo: handle error with ret
	FAR struct inode *inode = filep->f_inode;
	FAR uart_dev_t   *dev   = inode->i_private;

	//determine the number of bytes wiating in the TX buffer
	int count = 0;
	irqstate_t flags = enter_critical_section();
	if (dev->xmit.tail <= dev->xmit.head)
	{
		count = dev->xmit.head - dev->xmit.tail;
	}
	else
	{
		count = dev->xmit.size - (dev->xmit.tail - dev->xmit.head);
	}

	leave_critical_section(flags);
	return count;
}

void reset_read_buffer(){
	//create file pointer
	FAR struct file *filep;
	int ret = fs_getfilep(fd, &filep);
	if(ret < 0){
		printf("Failed to get file structure: %d\n", ret);
	}
	//todo: handle error with ret
	FAR struct inode *inode = filep->f_inode;
	FAR uart_dev_t   *dev   = inode->i_private;

	irqstate_t flags = enter_critical_section();
	//todo: is recv the rx?
	dev->recv.tail = dev->recv.head;
	leave_critical_section(flags);
}

int get_rx_buffer_count(){
	//FIONREAD
	//create file pointer
	FAR struct file *filep;
	int ret = fs_getfilep(fd, &filep);
	if(ret < 0){
		printf("Failed to get file structure: %d\n", ret);
	}
	//todo: handle error with ret
	FAR struct inode *inode = filep->f_inode;
	FAR uart_dev_t   *dev   = inode->i_private;

	int count;
	irqstate_t flags = enter_critical_section();

	/* Determine the number of bytes available in the RX buffer */
	if (dev->recv.tail <= dev->recv.head)
	{
		count = dev->recv.head - dev->recv.tail;
	}
	else
	{
		count = dev->recv.size - (dev->recv.tail - dev->recv.head);
	}

	leave_critical_section(flags);
	return count;
}

TMC_uart_write_datagram_t *tmc_uart_read (trinamic_motor_t driver, TMC_uart_read_datagram_t *datagram){
    TMC_uart_write_datagram_t * res = {0};
	volatile uint32_t dly = 50, ms = g_system_ticks;
	
	// write the read_datagram_t
	printf("uart_read start\n"); fflush(stdout);
	ssize_t n = write(fd, datagram->data, sizeof(TMC_uart_read_datagram_t));
	if (n < 0){
		printf("write failed: %d\n", errno);
		fflush(stdout);
	}
	printf("uart_read write complete\n"); fflush(stdout);

	while(get_tx_buffer_count());
	while(--dly);
	reset_read_buffer();

	//Wait for response with 3ms timeout
	while(get_rx_buffer_count() < 8){
		if(g_system_ticks - ms >= 3){
			printf("Break\n");
			break;
		}
		printf("Waiting for rx buffer to be > 8\n");
	}

	if(get_rx_buffer_count() >= 8){
		printf("In if\n");
		// char c;
		// for(int i=0; i<8; i++){
		// 	printf("in for\n");
		// 	// n = read(fd,&c,1);
		// 	// res->data[i] = c;
		// }
	}
	// else{
	// 	res->msg.addr.value = 0xFF;
	// }

	dly = 5000;
	while(--dly);

    return res;
}

void tmc_uart_write (trinamic_motor_t driver, TMC_uart_write_datagram_t *datagram){
    printf("uart_write start\n"); fflush(stdout);
	uint8_t byte;
	ssize_t n = write(fd, datagram->data, sizeof(TMC_uart_write_datagram_t));
	// for(int i = 0; i<sizeof(TMC_uart_write_datagram_t); i++){
	// 	byte = (datagram->data[i] >> (i * 8)) & 0xFF;
	// 	printf("%u", byte);
	// }

	// for(int i =0; i<8; i++){
	// 	prinf("%u",)
	// }

	printf("\n");
	//wait while the datagram is delivered
	while(get_tx_buffer_count());
	printf("uart_write write complete\n"); fflush(stdout);
}



void custom_init(){
	// Print the array
	// u_int8_t x = 5;
	// u_int8_t y = 192;
	char hex_str[3];
    for(int i = 0; i < 19; i++) {
        for(int j = 0; j < 8; j++) {
			
			// sprintf(hex_str, "%02X", array[i][j]);
            printf("%02X ", array[i][j]);
			write(fd, &array[i][j], sizeof(array[i][j]));
			
        }
        printf("\n");
		usleep(1000000);
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
	
	
  
	return 0;
}



