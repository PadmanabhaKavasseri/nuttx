#ifndef __APP_ACKERMAN_OLED_H
#define __APP_ACKERMAN_OLED_H
#ifdef CONFIG_APP_ACKERMAN

#include <car_main.h>



#define CAR_OLED_PRIORITY	(200)
#define CAR_OLED_STACKSIZE  (4*2048)

#define DEV_OLED_DC		"/dev/gpio6"
#define DEV_OLED_SCL	"/dev/gpio7"
#define DEV_OLED_SDA	"/dev/gpio8"
#define DEV_OLED_RST	"/dev/gpio9"

#define OLED_DATA_SIZE		(128)
#define OLED_DATA		(1)  /* write data */
#define OLED_CMD		(0)  /* write command */


#define CONTROL_RC		(2)
#define CONTROL_UART		(1)
#define CONTROL_ROS		(0)

struct oled_show_data_s {
	uint32_t car_mode;
	float voltage;
	bool  self_test_err_flag;
	float servo_bias;
	short imu_gyro_z;
	float right_v_goal;
	float right_v_actual;
	float left_v_goal;
	float left_v_actual;
	uint32_t servo_pwm;
	uint8_t control_mode;
	bool hw_en_flag;
	bool sw_stop_flag;
};

struct car_oled_s {
	bool	initialized;
	int rs_fd;
	int rst_fd;
	int sclk_fd;
	int sdin_fd;
	struct oled_show_data_s data;
};

int oled_task(int argc, char *argv[]);
void oled_deinit(void);

#endif
#endif