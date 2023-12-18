#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <nuttx/timers/pwm.h>
#include <nuttx/ioexpander/gpio.h>

#include "oled.h"
#include "oledfont.h"
#include "car_imu.h"
#include "motion_task.h"

static uint8_t  oled_gram[OLED_DATA_SIZE][8];

static struct car_oled_s g_car_oled;



static void oled_init(void);
static void oled_wr_byte(uint8_t dat, uint8_t cmd);
static void oled_display_on(void);
static void oled_display_off(void);
static void oled_refresh_gram(void);
static void oled_device_init(void);
static void oled_clear(void);
static void oled_drawpoint(uint8_t x, uint8_t y, uint8_t t);
static void oled_showchar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode);
static void oled_shownumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
static void oled_showstring(uint8_t x, uint8_t y, const int8_t *p);

static void oled_show(struct oled_show_data_s data);

static void oled_rs_clr(void) {
	if (g_car_oled.initialized = true) {
		ioctl(g_car_oled.rs_fd, GPIOC_WRITE, (unsigned long)0);
	}
}

static void oled_rs_set(void) {
	if (g_car_oled.initialized = true) {
		ioctl(g_car_oled.rs_fd, GPIOC_WRITE, (unsigned long)1);
	}
}

static void oled_rst_clr(void) {
	if (g_car_oled.initialized = true) {
		ioctl(g_car_oled.rst_fd, GPIOC_WRITE, (unsigned long)0);
	}
}

static void oled_rst_set(void) {
	if (g_car_oled.initialized = true) {
		ioctl(g_car_oled.rst_fd, GPIOC_WRITE, (unsigned long)1);
	}
}

static void oled_sclk_clr(void) {
	if (g_car_oled.initialized = true) {
		ioctl(g_car_oled.sclk_fd, GPIOC_WRITE, (unsigned long)0);
	}
}

static void oled_sclk_set(void) {
	if (g_car_oled.initialized = true) {
		ioctl(g_car_oled.sclk_fd, GPIOC_WRITE, (unsigned long)1);
	}
}

static void oled_sdin_clr(void) {
	if (g_car_oled.initialized = true) {
		ioctl(g_car_oled.sdin_fd, GPIOC_WRITE, (unsigned long)0);
	}
}

static void oled_sdin_set(void) {
	if (g_car_oled.initialized = true) {
		ioctl(g_car_oled.sdin_fd, GPIOC_WRITE, (unsigned long)1);
	}
}

/************************ oled control *****************************/

/***********
* Function: Refresh the OLED screen
* Input   : none
* Output  : none
************/
static void oled_refresh_gram(void)
{
	uint8_t i, n;
	for (i = 0; i < 8; i++)
	{
		/* Set page address (0~7) */
		oled_wr_byte (0xb0 + i, OLED_CMD);
		/* Set the display location - column low address */
		oled_wr_byte (0x00, OLED_CMD);
		/* Set the display location - column height address */
		oled_wr_byte (0x10, OLED_CMD);
		for (n = 0; n < 128; n++)
			oled_wr_byte(oled_gram[n][i], OLED_DATA);
	}
}


/**************************************************************************
Function: Refresh the OLED screen
Input   : Dat: data/command to write, CMD: data/command flag 0, represents the command;
1, represents data
Output  : none
**************************************************************************/
static void oled_wr_byte(uint8_t dat, uint8_t cmd)
{
	uint8_t i;
	if (cmd)
		oled_rs_set();
	else
		oled_rs_clr();
	for (i = 0; i < 8; i++)
	{
		oled_sclk_clr();
		if (dat & 0x80)
			oled_sdin_set();
		else
			oled_sdin_clr();
		oled_sclk_set();
		dat <<= 1;
	}
	oled_rs_set();
}


/**************************************************************************
Function: Turn on the OLED display
Input   : none
Output  : none
**************************************************************************/
static void oled_display_on(void)
{
	/* SET DCDC command */
	oled_wr_byte(0X8D, OLED_CMD);
	/* DCDC ON */
	oled_wr_byte(0X14, OLED_CMD);
	/* DISPLAY ON */
	oled_wr_byte(0XAF, OLED_CMD);
}

/**************************************************************************
* Function: Turn off the OLED display
* Input   : none
* Output  : none
**************************************************************************/
static void OLED_Display_Off(void)
{
	oled_wr_byte(0X8D, OLED_CMD);
	oled_wr_byte(0X10, OLED_CMD);
	oled_wr_byte(0XAE, OLED_CMD);
}

/**************************************************************************
Function: Screen clear function, clear the screen, 
the entire screen is black, and did not light up the same
Input   : none
Output  : none
**************************************************************************/
static void oled_clear(void)
{
	uint8_t i, n;
	for (i = 0; i < 8; i++)
		for (n = 0; n < 128; n++)
			oled_gram[n][i] = 0X00;
	oled_refresh_gram();
}

/**************************************************************************
Function: Draw point
Input   : x,y: starting coordinate;T :1, fill,0, empty
Output  : none
**************************************************************************/
static void oled_drawpoint(uint8_t x, uint8_t y, uint8_t t)
{
	uint8_t pos, bx, temp = 0;
	if (x > 127 || y > 63)return; //beyond range.
	pos = 7 - y / 8;
	bx = y % 8;
	temp = 1 << (7 - bx);
	if (t)
		oled_gram[x][pos] |= temp;
	else
		oled_gram[x][pos] &= ~temp;
}

/**************************************************************************
Function: Displays a character, including partial characters, at the specified position
Input   : x,y: starting coordinate;Len: The number of digits;
Size: font size;Mode :0, anti-white display,1, normal display
Output  : none
**************************************************************************/
static void oled_showchar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode)
{
	uint8_t temp, t, t1;
	uint8_t y0 = y;
	chr = chr - ' '; //Get the offset value.
	for (t = 0; t < size; t++)
	{
		if (size == 12)
			temp = oled_asc2_1206[chr][t];
		else
			temp = oled_asc2_1608[chr][t];
		for (t1 = 0; t1 < 8; t1++)
		{
			if (temp & 0x80)
				oled_drawpoint(x, y, mode);
			else 
				oled_drawpoint(x, y, !mode);
			temp <<= 1;
			y++;
			if ((y - y0) == size)
			{
				y = y0;
				x++;
				break;
			}
		}
	}
}

/**************************************************************************
Function: Find m to the NTH power
Input   : m: base number, n: power number
Output  : none
**************************************************************************/
static uint32_t oled_pow(uint8_t m, uint8_t n)
{
	uint32_t result = 1;
	while (n--)
		result *= m;
	return result;
}

/**************************************************************************
Function: Displays 2 numbers
Input   : x,y: starting coordinate;Len: The number of digits;
Size: font size;Mode: mode, 0, fill mode, 1, overlay mode;Num: value (0 ~ 4294967295);
Output  : none
**************************************************************************/
static void oled_shownumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
	uint8_t t, temp;
	uint8_t enshow = 0;
	for (t = 0; t < len; t++) {
		temp = (num / oled_pow(10, len - t - 1)) % 10;
		if (enshow == 0 && t < (len - 1)) {
			if (temp == 0) {
				oled_showchar(x + (size / 2)*t, y, ' ', size, 1);
				continue;
			}
			else
				enshow = 1;

		}
		oled_showchar(x + (size / 2)*t, y, temp + '0', size, 1);
	}
}

/**************************************************************************
Function: Display string
Input   : x,y: starting coordinate;*p: starting address of the string
Output  : none
**************************************************************************/
static void oled_showstring(uint8_t x, uint8_t y, const int8_t *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58
	while (*p != '\0') {
		if (x > MAX_CHAR_POSX) {
			x = 0;
			y += 16;
		}
		if (y > MAX_CHAR_POSY) {
			y = x = 0;
			oled_clear();
		}
		oled_showchar(x, y, *p, 12, 1);
		x += 8;
		p++;
	}
}

static void oled_init(void)
{
	int fd;
	fd = open(DEV_OLED_DC, O_RDWR);
	if (fd < 0){
		printf("oled_init: open %s failed: %d\n",
									DEV_OLED_DC, errno);
		return fd;
	}
	g_car_oled.rs_fd = fd;

	fd = open(DEV_OLED_SCL, O_RDWR);
	if (fd < 0){
		printf("oled_init: open %s failed: %d\n",
									DEV_OLED_SCL, errno);
		return fd;
	}
	g_car_oled.sclk_fd = fd;

	fd = open(DEV_OLED_SDA, O_RDWR);
	if (fd < 0){
		printf("oled_init: open %s failed: %d\n",
									DEV_OLED_SDA, errno);
		return fd;
	}
	g_car_oled.sdin_fd = fd;

	fd = open(DEV_OLED_RST, O_RDWR);
	if (fd < 0){
		printf("oled_init: open %s failed: %d\n",
									DEV_OLED_RST, errno);
		return fd;
	}
	g_car_oled.rst_fd = fd;
	g_car_oled.initialized = true;

	/*  oled device init */
	oled_rst_clr();
	usleep(100000);
	oled_rst_set();

	oled_wr_byte(0xAE, OLED_CMD); //Close display;
	oled_wr_byte(0xD5, OLED_CMD); //The frequency frequency factor, the frequency of the shock;
	oled_wr_byte(80, OLED_CMD);  //[3:0], the frequency dividing factor;[7:4], oscillation frequency;
	oled_wr_byte(0xA8, OLED_CMD); //Set the number of driver paths;
	oled_wr_byte(0X3F, OLED_CMD); //Default 0x3f(1/64);
	oled_wr_byte(0xD3, OLED_CMD); //Setting display deviation;
	oled_wr_byte(0X00, OLED_CMD); //Default is 0;

	oled_wr_byte(0x40, OLED_CMD); //Sets the number of rows to display starting line [5:0];

	oled_wr_byte(0x8D, OLED_CMD); //Charge pump setup;
	oled_wr_byte(0x14, OLED_CMD); //Bit2, on/off //bit2;
	oled_wr_byte(0x20, OLED_CMD); //Set up the memory address mode;
	oled_wr_byte(0x02, OLED_CMD); //[1:0],00, column address mode;01, line address mode;10. Page address mode;The default 10;
	oled_wr_byte(0xA1, OLED_CMD); //Segment redefine setting,bit0:0,0- >;0;1, 0 - & gt;127;;
	oled_wr_byte(0xC0, OLED_CMD); //Set the COM scan direction;Bit3:0, normal mode;1, Re-define schema COM[n-1]- >;COM0;N: Number of driving paths;
	oled_wr_byte(0xDA, OLED_CMD); //Set the COM hardware pin configuration;
	oled_wr_byte(0x12, OLED_CMD); //[5:4]configuration //[5:4];

	oled_wr_byte(0x81, OLED_CMD); //Contrast Settings;
	oled_wr_byte(0xEF, OLED_CMD); //1~ 255; Default 0x7f (brightness Settings, the bigger the brighter);
	oled_wr_byte(0xD9, OLED_CMD); //Set the pre-charging cycle
	oled_wr_byte(0xf1, OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
	oled_wr_byte(0xDB, OLED_CMD); //Setting vcomh voltage multiplier
	oled_wr_byte(0x30, OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	oled_wr_byte(0xA4, OLED_CMD); //Global display; Bit0:1, open; 0, close; (white screen/black screen);
	oled_wr_byte(0xA6, OLED_CMD); //Settings display mode; Bit0:1, anti-phase display; 0, normal;
	oled_wr_byte(0xAF, OLED_CMD); //Open display 
	oled_clear();
}

/***************** oled show function *******************/

/**************************************************************************
Function: The OLED display displays tasks
Input   : none
Output  : none
**************************************************************************/
static void oled_show(struct oled_show_data_s data)
{
	int voltage;
	oled_showstring(0, 0, "TYPE:");

	//Display robot type, show "X" when mismatch
	if (data.self_test_err_flag == 0)
		oled_shownumber(40, 0, data.car_mode, 1, 12);
	else
		oled_showstring(40, 0, "X");

	oled_showstring(55, 0, "BIAS"); //Servo calibration value
	if ( data.servo_bias < 0) {
		oled_showstring(90, 0, "-");
		oled_shownumber(100, 0, abs((int)data.servo_bias), 3, 12);
	}
	else {
		oled_showstring(90, 0, "+");
		oled_shownumber(100, 0, (int)data.servo_bias, 3, 12);
	}

	oled_showstring(00, 10, "GYRO_Z");
	if ( data.imu_gyro_z < 0) {
		oled_showstring(65, 10, "-"),
		oled_shownumber(75, 10, abs((int)data.imu_gyro_z), 5, 12);
	}
	else {
		oled_showstring(65, 10, "+");
		oled_shownumber(75, 10, (int)data.imu_gyro_z, 5, 12);
	}

	//The 3rd line show the target speed and current speed of motor A
	oled_showstring(0, 20, "R:");
	if ( data.right_v_goal < 0) {
		oled_showstring(15, 20, "-");
		oled_shownumber(20, 20, abs((int)(data.right_v_goal * 1000)), 5, 12);
	}
	else {
		oled_showstring(15, 20, "+");
		oled_shownumber(20, 20, (int)(data.right_v_goal * 1000), 5, 12);
	}

	if ( data.right_v_actual < 0) {
		oled_showstring(60, 20, "-");
		oled_shownumber(75, 20, abs((int)(data.right_v_actual * 1000)), 5, 12);
	}
	else {
		oled_showstring(60, 20, "+");
		oled_shownumber(75, 20, (int)(data.right_v_actual * 1000), 5, 12);
	}

	//The 4th line show the target speed and current speed of motor B
	oled_showstring(0, 30, "L:");
	if ( data.left_v_goal < 0) {
		oled_showstring(15, 30, "-");
		oled_shownumber(20, 30, abs((int)(data.left_v_goal * 1000)), 5, 12);
	}
	else {
		oled_showstring(15, 30, "+");
		oled_shownumber(20, 30, (int)(data.left_v_goal * 1000), 5, 12);
	}

	if ( data.left_v_actual < 0) {
		oled_showstring(60, 30, "-");
		oled_shownumber(75, 30, abs((int)(data.left_v_actual * 1000)), 5, 12);
	}
	else {
		oled_showstring(60, 30, "+");
		oled_shownumber(75, 30, (int)(data.left_v_actual * 1000), 5, 12);
	}

	//The 5th line show the current servo PWM control value
	oled_showstring(00, 40, "SERVO:");
	if ( data.servo_pwm < 0) {
		oled_showstring(60, 40, "-");
		oled_shownumber(80, 40,abs((int)(data.servo_pwm)), 4, 12);
	}
	else {
		oled_showstring(60, 40, "+");
		oled_shownumber(80, 40, (int)(data.servo_pwm), 4, 12);
	}

	//The 6th line show the current control mode & ON/OFF status & batt info
	if (data.control_mode == CONTROL_RC)
		oled_showstring(0, 50, "RC  ");
	else if (data.control_mode == CONTROL_UART)
		oled_showstring(0, 50, "USART");
	else
		oled_showstring(0, 50, "ROS  ");

	//Displays whether controls are allowed in the current car
	if (data.hw_en_flag == 1 && data.sw_stop_flag == 0)
		oled_showstring(45, 50, "O N");
	else
		oled_showstring(45, 50, "OFF");

	/* Displays the current battery voltage */
	voltage = (int)(data.voltage * 100);
	oled_showstring(88, 50, ".");
	oled_showstring(110, 50, "V");
	oled_shownumber(75, 50, (int)(voltage / 100), 2, 12);
	oled_shownumber(98, 50, (int)(voltage % 100), 2, 12);
	if (voltage % 100 < 10)
		oled_shownumber(92, 50, 0, 2, 12);

	/* Refresh the screen */
	oled_refresh_gram();
}

void oled_deinit(void)
{
	close(g_car_oled.rs_fd);
	close(g_car_oled.rst_fd);
	close(g_car_oled.sclk_fd);
	close(g_car_oled.sdin_fd);

}

/**************************************************************************
Function: Read the battery voltage, send data to APP, OLED display task
Input   : none
Output  : none
**************************************************************************/
int oled_task(int argc, char *argv[])
{
	struct oled_show_data_s data;

	oled_init();
	while (true) {
		usleep(1000000); /* 1HZ */

		/* get data */
		data.car_mode = 1; //ackerman 
		data.self_test_err_flag = false;
		get_power_voltage(&data.voltage);
		get_servo_zero_bias(&data.servo_bias);
		get_servo_pwm(&data.servo_pwm);
		get_motor_goal_speed(&data.right_v_goal, &data.left_v_goal);
		get_motor_actual_speed(&data.right_v_actual, &data.left_v_actual);
		get_imu_gyro_z(&data.imu_gyro_z);

		data.control_mode =0;  //need to do 
		data.sw_stop_flag = 0; //need to do
		data.hw_en_flag = 1;

		/* show data */
		oled_show(data);
	}

	oled_deinit();
	return 0;
}
