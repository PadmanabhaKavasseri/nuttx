/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

// #include "keybpwm.h"
#include "config_msg.h"
#include "qrc_msg_management.h"

// #include "motion_odom.h"
// #include "robot_controller.h"
// #include "motion_management.h"
// #include "time_sync.h"
// #include "imu.h"
// #include "rc_management.h"
// #include "remote_controller.h"
// #include "emergency_avoidance.h"
// #include "avoidance.h"
// #include "charger_management.h"
// #include "charger_controller.h"



/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_TIMEOUT  (6) /* second */

#define PI (3.14159f)

#define DEFAULT_TOP_MOTOR_PWM 1
#define DEFAULT_BOTTOM_MOTOR_PWM 1
#define DEFAULT_LA_MOTOR_PWM 1
#define DEFAULT_STEPPER_MOTOR_PWM 1



struct config_parameters_s
{
  pthread_mutex_t config_mutex;
  pthread_mutex_t set_params_mutex;
  pthread_cond_t config_cond;
  struct qrc_pipe_s *pipe;
  bool notify_initialized;
} __attribute__((aligned(4)));

struct mcb_config_parameters_s
{
	enum config_msg_type_e type;
  void * parameter;
  size_t length;
};

static struct config_parameters_s g_config_parameter;

struct config_tbm_s config_tbm =
{
  .top_motor = DEFAULT_TOP_MOTOR_PWM,
  .bottom_motor = DEFAULT_BOTTOM_MOTOR_PWM,
  .la = DEFAULT_LA_MOTOR_PWM,
  .stepper = DEFAULT_STEPPER_MOTOR_PWM,
};

static struct mcb_config_parameters_s  g_parameters_list[] =
{
  {TBM,                 &config_tbm,        sizeof(struct config_tbm_s)},
};

static void config_parameter_msg_parse(struct qrc_pipe_s *pipe, struct config_msg_s *config_msg)
{
  enum config_msg_type_e type;
  void * parameters;
  size_t length;
  struct config_msg_s config_msg_reply = {0};
  enum mcb_task_id_e task_error_id;
  int apply_status;
  enum qrc_write_status_e result;

  int status;

  if (pipe == NULL || config_msg ==NULL)
    {
      return;
    }

  status = pthread_mutex_lock(&g_config_parameter.set_params_mutex);
  if (status != 0)
    {
      syslog(LOG_ERR, "config_notify_completed:"
            "ERROR pthread_mutex_lock failed, status=%d\n", status);
      ASSERT(false);
    }

  type = config_msg->type;
  if ( type >= 0 && type < CONFIG_MSG_TYPE_MAX)
    {
      /* save parameters */
      parameters =  g_parameters_list[type].parameter;
      length =  g_parameters_list[type].length;
      memcpy(parameters, &config_msg->data, length);
      syslog(LOG_ERR, "config_parameter_msg_parse get type=%d \n",type);
      config_msg_reply.type = type;

      result = qrc_write(pipe, (uint8_t *)&config_msg_reply, sizeof(struct config_msg_s), false);
      if (result != SUCCESS)
      {
        syslog(LOG_ERR, "config_parameter_msg_parse msg send failed %d\n", result);
      }
    }
  else
    {
      syslog(LOG_ERR, "config_parameter_msg_parse type is invalid %d\\n", type);
    }

  status = pthread_mutex_unlock(&g_config_parameter.set_params_mutex);
  if (status != 0)
    {
      syslog(LOG_ERR, "config_notify_completed:"
              "ERROR pthread_mutex_unlock failed, status=%d\n", status);
      ASSERT(false);
    }
}

static void config_parameter_qrc_msg_cb(struct qrc_pipe_s *pipe, void *data, size_t len, bool response)
{
  struct config_msg_s *config_msg;

  if (pipe == NULL || data == NULL)
    {
      return;
    }
  if (len == sizeof(struct config_msg_s))
    {
      config_msg = (struct config_msg_s *)data;
      config_parameter_msg_parse(pipe, config_msg);
    }
  else
    {
      syslog(LOG_ERR,"config_parameter_qrc_msg_cb: message size mismatch len = %d, actual = %d\n",len,sizeof(struct config_msg_s));
    }
}

int config_parameter_init(void)
{
  char pipe_name[] = CONFIG_PIPE;
  int status;

  /* init cond & mutex */

  status = pthread_mutex_init(&g_config_parameter.config_mutex, NULL);
  if (status != 0)
    {
      syslog(LOG_ERR,"confg_parameter: ERROR pthread_mutex_init failed, status=%d\n",
              status);
      ASSERT(false);
    }

  status = pthread_mutex_init(&g_config_parameter.set_params_mutex, NULL);
  if (status != 0)
    {
      syslog(LOG_ERR,"confg_parameter: ERROR pthread_mutex_init failed, status=%d\n",
              status);
      ASSERT(false);
    }

  status = pthread_cond_init(&g_config_parameter.config_cond, NULL);
  if (status != 0)
    {
      syslog(LOG_ERR,"confg_parameter: ERROR pthread_cond_init failed, status=%d\n",
              status);
      ASSERT(false);
    }

  /* get pipe */
  g_config_parameter.pipe =  qrc_get_pipe(pipe_name);

  if (g_config_parameter.pipe == NULL)
    {
      syslog(LOG_ERR,"config_parameter: get qrc pipe error\n");
      return -1;
    }

  syslog(LOG_DEBUG, "configure register qrc pipe done \n");

  if (!qrc_register_message_cb(g_config_parameter.pipe, config_parameter_qrc_msg_cb))
    {
      syslog(LOG_ERR,"qrc register config parameter cb error\n");
      return -1;
    }

  syslog(LOG_INFO,"confg_parameter init done \n");

  return 0;
}
