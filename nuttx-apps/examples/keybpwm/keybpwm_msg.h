
#ifndef __PWM_QRCMSG_H
#define __PWM_QRCMSG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define PWM_PIPE "keybpwm"



enum pwm_msg_type_e
{
  GET_HELLO = 0,
  PRINT_HELLO,
  SET_VALUE,
  SET_MOTOR,
};

struct pwm_msg_s
{
  int type; /* Use int replace enum for 4-bytes-align */
  int motor; 
  int on_off;
  double duty;
  double freq;
  
}__attribute__((aligned(4)));

#ifdef __cplusplus
}
#endif

#endif


