#include "ultrasound.h"
#include "task_config.h"
#include "motor_driver.h"

#define ULTRASOUND_DEV  "/dev/ttyS3"

static struct rs485_data_s g_rs485_bus;

const int ULRTUSOUND_NUM_ARRAY[CAR_TYPE_NUM] = {5, 7};

#define ULRTUSOUND_NUM(x)        ULRTUSOUND_NUM_ARRAY[x]

	
static void bottom_avoidance_handle(void* this)
{
    us_data_s* us_data = ((us_data_s*)(this));
	
    if(us_data->dist >= us_data->alert_limit[parameter_data.car_type])
    {
        //避障实体函数
		us_data->status = true;
        syslog(LOG_DEBUG, "bottom_%d Waring: Hanging ahead! height = %.2fmm\n",us_data->addr, us_data->dist);
    }
	else
	{
		us_data->status = false;
	}

}

static void side_avoidance_handle(void* this)
{
    us_data_s* us_data = ((us_data_s*)(this));
	
    if(us_data->dist <= us_data->alert_limit[parameter_data.car_type])
    {
        //避障实体函数
		us_data->status = true;
        syslog(LOG_DEBUG, "side_%d Waring: Obstacle ahead! dist = %.2fmm\n",us_data->addr, us_data->dist);
    }
	else
	{
		us_data->status = false;
	}

}

static us_data_s us_arr[ULRTUSOUND_NUM_MAX] = {
    {{0x01},{0.0},{0.0},{100,100},false,{*bottom_avoidance_handle}},
    {{0x02},{0.0},{0.0},{100,100},false,{*bottom_avoidance_handle}},
    {{0x03},{0.0},{0.0},{150,150},false,{*side_avoidance_handle}},
    {{0x04},{0.0},{0.0},{150,150},false,{*side_avoidance_handle}},
    {{0x05},{0.0},{0.0},{150,150},false,{*side_avoidance_handle}},
    {{0x06},{0.0},{0.0},{150,150},false,{*side_avoidance_handle}},
    {{0x07},{0.0},{0.0},{150,150},false,{*side_avoidance_handle}},
};

bool motor_need_stop()
{
	for(int i=0; i < ULRTUSOUND_NUM(parameter_data.car_type); i++)
	{
		if (us_arr[i].status == true)
		{
			return true;
		}
	}

	return false;
}

static uint16_t crc16_modbus(const uint8_t *data, uint8_t data_len){
	
	uint16_t ucrc = 0xffff;//CRC寄存器
	uint8_t num =0;


	for(uint8_t num=0; num<data_len; num++){
		ucrc = (*data++)^ucrc;//把数据与16位的CRC寄存器的低8位相异或，结果存放于CRC寄存器。
		for(uint8_t x=0;x<8;x++){	//循环8次
			if(ucrc&0x0001){	//判断最低位为：“1”
				ucrc = ucrc>>1;	//先右移
				ucrc = ucrc^0xA001;	//再与0xA001异或
			}else{	//判断最低位为：“0”
				ucrc = ucrc>>1;	//右移
			}
		}
	}
	return ucrc;//返回CRC校验值
}

/* ultrasound_cmd_frame_coding */
/*  输入：mc_buff，通常是6个字节； 输出：buff 大小端调整后的数据帧 */
/*  */
static void us_cmd_frame_coding(char* buff, uint8_t buff_len, struct us_cmd_buffer_s *us_buff) {
	
	int16_t data;
	uint8_t current = 0;
	uint8_t i;
	if (buff ==NULL || us_buff == NULL)
		return;
	
	memcpy(buff, us_buff, 2); //addr &id
	current = 2;

	data = ((us_buff->us_register & 0xff) << 0x8) + ((us_buff->us_register & 0xff00) >> 0x8);
	memcpy(&buff[current], &data, 2);

	current = current + 2;
	data = ((us_buff->us_reg_len & 0xff) << 0x8) + ((us_buff->us_reg_len & 0xff00) >> 0x8);
	memcpy(&buff[current], &data, 2);
}

static int rs485_send_msg(char *buff, int len, int fd){
    uint16_t crc16;
    uint32_t current = len;

    /* calculate crc */
	crc16 = crc16_modbus(buff, len);

	memcpy(&buff[current], &crc16, 2);
    crc16 = 0;
	current = current + 2;
	/* send fame to MC */
	write(fd, buff, current);
	//printf("DEBUG send cmd: ");
    // for(int i=0;i<current;i++)
    // {
    //     printf("%x ",buff[i]);
    // }
    //printf("\n");
}

/* 485传输接口，实现数据帧的读写功能。 */
/* function 功能：发送数据帧，输出返回帧(未处理数据)  */
static int mc_frame_sync(void *frame, uint8_t frame_size, char *frame_return, uint8_t return_size, int fd)
{
	int read_size,count;
	uint32_t current,time;

	/* 处理发送帧 */
	us_cmd_frame_coding(frame_return, frame_size, frame);
	current = frame_size;
}

static int rs485_revice(uint8_t *rec_buff, int fd){

    int current= 0;
	int count =0, crc16 =0, rec_crc16=0;
    int read_size;
    //printf("Waiting recive rs485 data ...\n");
	while ( current < RS485_RECEIVE_TIME_OUT )
	{
        //非阻塞读取
		read_size = read(fd, &rec_buff[count], 1);
		if (read_size > 0)
		{
            current =0;
            count = read_size + count; //已经接收到的字符数
            if(count > 2)
            {
                rec_crc16 = (rec_buff[count - 1] << 8 ) + (rec_buff[ count - 2 ]);//大小端处理
                crc16 = crc16_modbus(rec_buff, count - 2); //计算接收到字符-2的crc
                if(rec_crc16 == crc16)
                {
                    return count;
                }
            }
		}
		usleep(RS485_RECEIVE_TIME_OUT/20);
		current = current + RS485_RECEIVE_TIME_OUT/20;
	}
    //printf("ultrasound Recive time out!\n");
    return ERROR;
}

static int ultrasound_init(void){

	int ret;
    int fd;
	/* open imu fd */
	fd = open(ULTRASOUND_DEV, O_RDWR|O_NONBLOCK);
	if (fd < 0){
		syslog(LOG_INFO,"ultrasound_init: open %s failed: %d\n",
									ULTRASOUND_DEV, errno);
		return fd;
	}
    g_rs485_bus.rs485_fd = fd;
	syslog(LOG_INFO,"ultrasound_init: opened %s\n",ULTRASOUND_DEV);

	return OK;
}

static int ultrasound_sync_dist_data(us_data_s *us_data, int fd)
{
    uint8_t rec_buff[RS485_MSG_LEN];
    uint8_t send_buff[SINGLE_FRAME_LEN];

    //float rec_dist = 0.0;
    memset(rec_buff, 0, RS485_MSG_LEN);
    memset(send_buff, 0, SINGLE_FRAME_LEN);
    
    struct us_cmd_buffer_s us_buff;
    int ret;
	us_buff.us_addr = us_data->addr;
	us_buff.us_cmd_id = R_SINGLE_REG;
	us_buff.us_register = RAWDIST_REG;
	us_buff.us_reg_len = 0x01;

    us_cmd_frame_coding(send_buff, SINGLE_FRAME_LEN, &us_buff);
    rs485_send_msg(send_buff, SINGLE_FRAME_LEN, fd); //发送数据，默认添加CRC校验
    
    rs485_revice(rec_buff, fd);
    us_data->dist = ((rec_buff[3]<<8) +rec_buff[4]);

    //us_data->dist = rec_dist;
    if (us_data->dist > 0)
    {
    	us_data->avoidance_handle(us_data);
    }
    return ret;
}

float get_ultrasound_dist(uint8_t addr)
{
    for(int i=0; i < ULRTUSOUND_NUM(parameter_data.car_type); i++)
    {
        if(us_arr[i].addr == addr)
        {
            return us_arr[i].dist;
        }
    }
}

int ultrasound_task(int argc, char *argv[]){
	int ret;
	//int attached = 1;
    int fd;

	struct timespec tp = {60, 0};
	sigset_t set;
	struct siginfo value;

	sigemptyset(&set);
	sigaddset(&set, SIGUSR1);

	syslog(LOG_INFO,"ultrasound task wait signal \n");
	sigtimedwait(&set, &value, &tp);
	syslog(LOG_INFO,"ultrasound task receive signal %d code %d err %d \n",\
		value.si_signo,value.si_code, value.si_errno);
	
	/* init rc device */
	ultrasound_init();
	syslog(LOG_INFO, "car type %d ultrasound num %d\n", parameter_data.car_type, ULRTUSOUND_NUM(parameter_data.car_type));
    fd = g_rs485_bus.rs485_fd;
	/* attach rc */
	//ultrasound_check_attach(fd);

	/* loop and receive rc data */
	while(1){
        for(int i=0; i < ULRTUSOUND_NUM(parameter_data.car_type); i++)
        {
            ultrasound_sync_dist_data(&us_arr[i], fd);
            //printf("dist = %.2f mm \n", us_arr[i].dist);
            usleep(100);
        }
	}

    return 0;
}
