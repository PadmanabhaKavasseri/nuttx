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
	int fd, ret;
	bool invalue;

	fd = open("/dev/gpio0", O_RDONLY);
	printf("Starting Sandbox\n");

	
	// printf("%c", buf[i]);

	while(true){
		ret = ioctl(fd, GPIOC_READ, (unsigned long)((uintptr_t)&invalue));
		printf("  Input pin:     Value=%u\n",(unsigned int)invalue);
		up_udelay(1000*1000);
	}

	/*
	read gpio... move a step
	if gpio value goes low
	stop the motor and declare position as 0.
	
	
	*/
	



  
  return 0;
}
