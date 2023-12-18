/****************************************************************************
 * apps/examples/hello/hello_main.c
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


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * hello_main
 ****************************************************************************/
//pb9 is gpio2
int main(int argc, FAR char *argv[])
{
  int gpio_fd, ser_fd, ret;
  enum gpio_pintype_e pintype;
  bool outvalue = false;
  char buf[1];



  printf("Hello, World!!\n");

  //open the pin driver
  gpio_fd = open("/dev/gpio2", O_RDWR);
  //set pin type
  pintype = 3; //fprintf(stderr, "\t 3: GPIO_OUTPUT_PIN\n");
  ret = ioctl(gpio_fd, GPIOC_SETPINTYPE, (unsigned long) pintype);
  //open the serial port to read from
  ser_fd = open("/dev/ttyS2", O_RDONLY | O_NONBLOCK);
  if (ser_fd < 0) {
    perror("unable to open serial port");
    return 1;
  }


  while(1){
    //read 1 byte from serial_fd
    ssize_t n = read(ser_fd, buf, 1);

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
      if(buf[0]=='h'){
        //set gpio high
        outvalue = true;
        ret = ioctl(gpio_fd, GPIOC_WRITE, (unsigned long)outvalue);
        printf("Set GPIO PB9 to HIGH\n");
      }
      else if(buf[0]=='l'){
        //set gpio low
        outvalue = false;
        ret = ioctl(gpio_fd, GPIOC_WRITE, (unsigned long)outvalue);
        printf("Set GPIO PB9 to LOW\n");
      }
    }
  }
  close(ser_fd);
  
  return 0;
}
