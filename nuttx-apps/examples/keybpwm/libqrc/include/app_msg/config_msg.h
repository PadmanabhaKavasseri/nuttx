/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#ifndef __CONFIG_MSG_H
#define __CONFIG_MSG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CONFIG_PIPE "config"


struct config_tbm_s
{
  int top_motor;
  int bottom_motor;
  int la;
  int stepper;
}__attribute__((aligned(4)));


enum config_msg_type_e
{
  TBM,
  CONFIG_MSG_TYPE_MAX,
};

enum mcb_task_id_e
{
  ID_MOTION_MANAGEMENT = 0x00,
  ID_CHARGER_MANAGEMENT,
  ID_RC_MANAGEMENT,
  ID_AVOID_MANAGEMENT,
  ID_TIME_SYNC,
  ID_IMU,
  ID_MISC,
  ID_MOTION_ODOM,
  ID_ROBOT_CONTROLLER,
  ID_CLIENT_CONTROLLER,
  ID_CHARGER_CONTROLLER,
  ID_REMOTE_CONTROLLER,
  ID_EMERGENCY,
  ID_MAX_TASK,
};

struct config_apply_s
{
  int status;	  /* 0 is success, not 0 is failed */
  enum mcb_task_id_e error_type;	/* which task caused error */
}__attribute__((aligned(4)));

struct config_msg_s
{
  enum config_msg_type_e type;
  union
  {
    struct config_tbm_s tbm;
  } data;
} __attribute__((aligned(4)));

#ifdef __cplusplus
}
#endif

#endif  // __ROBOT_CONFIG_MSG_H
