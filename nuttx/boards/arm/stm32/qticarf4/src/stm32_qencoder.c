/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/sensors/qencoder.h>
#include <arch/board/board.h>

#include "chip.h"
#include "stm32_qencoder.h"
#include "qcomcarf4.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32f4_qencoder_initialize(void)
{
  int ret = 0;
  char devpath[9];

  /* Initialize tim2 quadrature encoder interface. */
  strlcpy(devpath, "/dev/qeA",9);
  ret = stm32_qeinitialize(devpath, 2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_qeinitialize() failed: %d\n", ret);
    }
  /*  Init tim3 as qeB  */
  strlcpy(devpath, "/dev/qeB",9);
  ret = stm32_qeinitialize(devpath, 3);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_qeinitialize() failed: %d\n", ret);
    }
  return ret;
}
