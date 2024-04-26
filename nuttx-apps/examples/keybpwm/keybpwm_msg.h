
#ifndef __PWM_QRCMSG_H
#define __PWM_QRCMSG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define PWM_PIPE "keybpwm"

enum motor_msg_type_e
{
  MOTOR_BLDC_LA = 0,
  MOTOR_STEPPER,
};

struct motor_bldc_la
{
  // int type; /* Use int replace enum for 4-bytes-align */
  int motor; 
  int on_off;
  double duty;
  double freq;
  
}__attribute__((aligned(4)));


struct motor_stepper
{
  // int type; /* Use int replace enum for 4-bytes-align */
  int motor;
  int on_off;
  int lock;
  double duty;
  double freq;
  int direction;
  int num_steps;
  
}__attribute__((aligned(4)));


struct motor_msg_s
{
  int type;
  union
  {
    struct motor_bldc_la bldc_la;
    struct motor_stepper stepper;
  } data;
} __attribute__((aligned(4)));
// step dir enable

#ifdef __cplusplus
}
#endif

#endif


