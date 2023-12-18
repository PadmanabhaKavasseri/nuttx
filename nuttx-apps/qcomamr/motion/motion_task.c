#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <inttypes.h>
#include <syslog.h>

#include <nuttx/arch.h>

#include "motion_task.h"
#include "main.h"
#include "ros_com.h"
#include "ultrasound.h"

extern struct amr_ros_com_s g_amr_ros;

volatile struct amr_motion_s g_amr_motion;
static amr_motor_data_t amr_motor_data;

void smooth_speed_control(float vx, float step)
{
    if (vx > g_amr_motion.smooth_speed)
    {
        g_amr_motion.smooth_speed += step;
    }
        
    else if (vx < g_amr_motion.smooth_speed)
    {
        g_amr_motion.smooth_speed -= step;
    }
        
    else
    {
        g_amr_motion.smooth_speed = vx;
    }

    if ((abs(vx) <0.1) && g_amr_motion.smooth_speed < (2.5 * step) && g_amr_motion.smooth_speed > (-2.5 * step))
    {
        g_amr_motion.smooth_speed = vx;
    }
}

void get_motor_actual_speed(int *left, int *right)
{
    *left = g_amr_motion.motor_left.rpm_actual;
    *right = g_amr_motion.motor_right.rpm_actual;
}

void get_motor_goal_speed(int *left, int *right)
{
    *left = g_amr_motion.motor_left.rpm_goal;
    *right = g_amr_motion.motor_right.rpm_goal;
}

amr_motor_data_t* get_motor_odom(void)
{
	return &amr_motor_data;
}

//设置更新mode 标志
void change_mode_tag_set(bool update)
{
	g_amr_motion.control_mode_update = update;
}

void motion_change_mode(uint8_t mode)
{
	g_amr_motion.control_mode = mode;
}

void quick_stop_status_set(bool enable)
{
	g_amr_motion.quick_stop_enable = enable;
}

bool motor_stop_status_get()
{
	return g_amr_motion.motor_stop_once;
}


//切换为速度模式下要初始化的参数.
void speed_mode_init_parameters(uint8_t mode)
{
	if (mode != CAR_VELOCITY_MODE)
		return;

	g_amr_motion.target_control_mode = mode;
	g_amr_motion.velocity_zero = true; //模式初始化时的速度是0.

}

void position_mode_init_parameters(uint8_t mode)
{
	if (mode != CAR_POSITION_MODE)
		return;
	g_amr_motion.target_control_mode = mode;
	g_amr_motion.pos_cmd_running = false;
	g_amr_motion.pos_controller_available = true;
	g_amr_motion.target_reached = true; //无用的
}

bool position_mode_controller_available(void)
{
	return g_amr_motion.pos_controller_available;
}

//speed mode control function
static void speed_mode_control(void)
{
	float vx_rc = 0;
	float vz_rc = 0;
	uint32_t v_time = 0;
	float vx_ros = 0;
	float vz_ros = 0;
	int ret = OK;

	struct motor_control_s *motor_left;
    struct motor_control_s *motor_right;

	motor_left = &g_amr_motion.motor_left;
    motor_right = &g_amr_motion.motor_right;

	if (g_amr_motion.control_mode != CAR_VELOCITY_MODE)
		return;

	//速度模式有两种模式，RC,速度
	if(OK == get_rc_goal_speed(&vx_rc, &vz_rc, &v_time)) {
		inverse_kinematics(vx_rc, vz_rc, &motor_left->rpm_goal, &motor_right->rpm_goal);
		motor_speed_sync(motor_left->rpm_goal, motor_right->rpm_goal, v_time);
	}
	else if(OK == get_ros_goal_speed(&vx_ros, &vz_ros)) {
		inverse_kinematics(vx_ros, vz_ros, &motor_left->rpm_goal, &motor_right->rpm_goal);
		motor_speed_sync(motor_left->rpm_goal, motor_right->rpm_goal, v_time);
	}

	//速度模式下获取odom
	g_amr_motion.velocity_zero = motor_velocity_zero();
	ret = motor_speed_read(&amr_motor_data);
	if (ret < 0) {
		syslog(LOG_WARNING, "speed read failed\n");
	} else {
		rpm_transfer_to_odom(&amr_motor_data);
	}
}
//odom mode control function
static void position_mode_control(void)
{
	int sub_mode = 0;
	int present_sub_mode = sub_mode;
	float x_pos = 0;
	float z_pos = 0;
	int left_goal = 0;
	int right_goal = 0;

	if (g_amr_motion.control_mode != CAR_POSITION_MODE)
		return;

	if(OK == get_ros_goal_position(&x_pos, &z_pos, &sub_mode) &&
		g_amr_motion.pos_controller_available == true)
	{
		//sub模式切换的时候需要更新量程
		if (present_sub_mode != sub_mode) {
			motor_control_odom_mode_speed_range_set(sub_mode);
			present_sub_mode = sub_mode;
			syslog(LOG_WARNING, "jiong change position speed mode new_mode =%d\n",sub_mode);
		}
		//发送控制指令
		inverse_kinematics_pos(x_pos, z_pos, sub_mode, &left_goal, &right_goal);
		motor_position_set(left_goal, right_goal);
		g_amr_motion.pos_cmd_running = true;
		syslog(LOG_DEBUG, "jiong position mode x_pos =%f,z_pos =%f,left_goal = %d,right_goal = %d\n\n",x_pos,z_pos,left_goal,right_goal);
	}

	//位置模式下，获取odom

	motor_position_read(&amr_motor_data);
	count_transfer_to_odom(&amr_motor_data);
	//syslog(LOG_DEBUG, "jiong position mode running \n\n");
	//位置模式下，需要通过读取电机的运行状态确认是否到当前goal。
	if (g_amr_motion.pos_cmd_running == true) {
		g_amr_motion.pos_controller_available = motor_target_reached();
		if (g_amr_motion.pos_controller_available == true) { //如果位置到达，则可以接收下一个命令。
			g_amr_motion.pos_cmd_running =false;
			g_amr_ros.notify_position_reached=true;
		}

	}
}

int motion_task(int argc, char *argv[])
{
	struct timespec tp_start, tp_end;
	long delta = 0;

    uint32_t v_time = 0;
	g_amr_motion.motor_stop_once = false;

    int ret = OK;

    int i = 0;
	struct timespec tp = {60, 0};
	sigset_t set;
	struct siginfo value;

	sigemptyset(&set);
	sigaddset(&set, SIGUSR1);

	syslog(LOG_INFO,"motion task wait signal \n");
	sigtimedwait(&set, &value, &tp);
	syslog(LOG_INFO,"motion task receive signal %d code %d err %d \n",\
		value.si_signo,value.si_code, value.si_errno);

	motion_change_mode(CAR_VELOCITY_MODE); //初始化电机为速度模式
	speed_mode_init_parameters(CAR_VELOCITY_MODE); //初始化速度模式参数.
	g_amr_motion.target_control_mode = CAR_VELOCITY_MODE;
	g_amr_motion.control_mode_update = false;
	g_amr_motion.quick_stop_enable = true;
	g_amr_motion.pos_controller_available = true;
	syslog(LOG_DEBUG, "mode update:%d\n", g_amr_motion.control_mode_update);

    motor_driver_init();
    syslog(LOG_INFO,"motion task start running! \n");

    while (1)
    {
        usleep(1000000/CONTROL_FREQUENCY);
		clock_gettime(CLOCK_MONOTONIC, &tp_start);

		//两种模式的切换！！
		if (g_amr_motion.control_mode_update == true)
		{
			ret = motor_control_mode_switch(g_amr_motion.target_control_mode);
			if (ret == OK)
			{
				change_mode_tag_set(false); //模式切换的tag 设置为false.
				motion_change_mode(g_amr_motion.target_control_mode);
				g_amr_ros.notify_mode_switch = true;
				amr_motor_data.mode_switch = true;
				syslog(LOG_INFO,"motion task changed motion mode as %d\n\n",g_amr_motion.control_mode);
			}
			continue;
		}

		//两种模式确定odometry的参数.
		if(g_amr_motion.control_mode == CAR_VELOCITY_MODE) {
			speed_mode_control();
		}
		else if (g_amr_motion.control_mode == CAR_POSITION_MODE) {
			position_mode_control();
		}

        if (i++%20 == 0) {
            zlac8015d_driver_error_check();
		}
		clock_gettime(CLOCK_MONOTONIC, &tp_end);
		delta = tp_end.tv_nsec - tp_start.tv_nsec;

		//急停，
		if (motor_need_stop() && g_amr_motion.quick_stop_enable)
		{
			if (g_amr_motion.motor_stop_once == false)
			{
				motor_quick_stop();
			}
			g_amr_motion.motor_stop_once = true;
		}
		else if (g_amr_motion.motor_stop_once == true)
		{
			motor_resume_enable(g_amr_motion.control_mode);
			g_amr_motion.motor_stop_once = false;
		}
		
    }

    return 0;
}
