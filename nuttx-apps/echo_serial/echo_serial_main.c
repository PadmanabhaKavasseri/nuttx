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
  FAR char *devpath = CONFIG_ECHO_SERIALRX_DEVPATH;
  FAR char *buf = (FAR char *)malloc(CONFIG_ECHO_SERIALRX_BUFSIZE);
  int wfd = open(devpath, O_RDWR);
  if (wfd < 0){
    fprintf(stderr, "ERROR: write open failed: %d\n", errno);
  }
  // if (rfd < 0){
  //   fprintf(stderr, "ERROR: read open failed: %d\n", errno);
  // }

  // Write the string from argv[1] to the device
  size_t len = strlen(argv[1]);
  printf("Word: %s\n",argv[1]);
  ssize_t n = write(wfd, argv[1], len);
  if (n < 0)
  {
    printf("write failed: %d\n", errno);
    fflush(stdout);
  }
  printf("Write Completed\n");
  up_udelay(1000000);
  printf("Sleep Done\n");
  
  ioctl(wfd, FIONREAD, (unsigned long)&len);
  printf("LEN: %d\n",len);

  ssize_t nn = read(wfd, buf, wfd);
  up_udelay(10000);
  printf("Sleep Done\n");
  for (int i = 0; i < (int)nn; i++){
    printf("%c", buf[i]);
  }

  fflush(stdout);
  printf("\nDone Done\n");
  return 0;
}
