/****************************************************************************
 * boards/arm/stm32/qcomcarf4/src/stm32_reset.c
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
 * Changes from Qualcomm Innovation Center are provided under the following license:
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include "stm32_dfumode.h"

#ifdef CONFIG_BOARDCTL_RESET

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_reset(int mode)
{
  if (mode == 0)
    {
      /* Normal reset */

      up_systemreset();
    }
  else
    {
      /* DFU reset */

      stm32_dfumode();
    }
}

#endif /* CONFIG_BOARDCTL_RESET */
