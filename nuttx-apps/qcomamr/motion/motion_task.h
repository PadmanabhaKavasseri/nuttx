#ifndef __APP_QTIAMR_MOTION_TASK_H
#define __APP_QTIAMR_MOTION_TASK_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include "canopen.h"
#include "motor_driver.h"


#ifdef CONFIG_APP_QCOMAMR

/* motion task running frequency */
#define CONTROL_FREQUENCY    (50)


#define PI_MATH                   (3.1416f)

#define MAX_SPEED                 0.7         //(5 * QTIAMR1_WHEEL_PERIMETER)
#define MAX_ANGULAR_VELOCITY      2
#define SMOOTH_STEP               (0.03)      // Max speed : Max accelerate = 1:1.5

struct motor_control_s
{
    int rpm_goal;
    int rpm_actual;
};

struct amr_motion_s
{
    struct motor_control_s motor_left;
    struct motor_control_s motor_right;

    float smooth_speed;
	uint8_t control_mode;
	bool velocity_zero;
	bool target_reached;
	bool pos_cmd_running; //true: 当前pos cmd 正在执行。
	bool pos_controller_available;	//true:当前控制器处于可配置给定计数状态。false: 控制器处于busy 状态，不可配置.
	uint8_t target_control_mode;
	bool control_mode_update;

	bool quick_stop_enable;
	bool motor_stop_once;
};

void smooth_speed_control(float vx, float step);
void get_motor_actual_speed(int *left, int *right);
void get_motor_goal_speed(int *left, int *right);

void inverse_kinematics(float vx, float vz, int *rpm_l, int *rpm_r);
void inverse_kinematics_pos(float vx, float vz, int sub_mode, int *count_l, int *count_r);
void get_motor_driver_velocity(float *vx, float *vz);
void rpm_transfer_to_odom(amr_motor_data_t *data);
int motion_task(int argc, char *argv[]);
amr_motor_data_t* get_motor_odom(void);
void quick_stop_status_set(bool enable);
bool motor_stop_status_get();
void position_mode_init_parameters(uint8_t mode);
void speed_mode_init_parameters(uint8_t mode);
void change_mode_update_set(bool update);
bool position_mode_controller_available(void);

#endif
#endif
