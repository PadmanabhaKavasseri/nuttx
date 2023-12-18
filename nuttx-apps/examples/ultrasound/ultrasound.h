/****************************************************************************
 * apps/examples/capture/cap.h
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

#ifndef __APPS_EXAMPLES_ULTRASOUND_H
#define __APPS_EXAMPLES_ULTRASOUND_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/
#define RS485_MSG_LEN (16) /* 8 bytes message len & 8 for extern */
#define SINGLE_FRAME_LEN (6)	/* No CRC frame*/
#define RS485_RECEIVE_TIME_OUT (2000000) /* 接受等待函数2s 钟 */

#define BROADCAST_ADDR  (0xFF)
#define CONTROLLER_ADDR (0x01)
#define R_SINGLE_REG	(0x03)
#define W_SINGLE_REG	(0x06)

#define ADDR_REG        (0x0200)
#define DIST_REG        (0x0100)
#define RAWDIST_REG     (0x0101)
#define TEMP_REG        (0x0102)

struct ulteasound_example_s
{
  FAR char *devpath;     /* Path to the capture device */
  uint8_t addr;
  uint8_t dir;      /*Dir of r/w*/
  uint16_t reg;
  union
  {
    uint16_t cmd_data;
    uint16_t reg_len;
  };
  float dist;      /* Collect this number of samples */
  float tmp;       /* Delay this number of seconds between samples */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern struct ulteasound_example_s g_ulteasound;

#endif /* __APPS_EXAMPLES_ULTRASOUND_H */
