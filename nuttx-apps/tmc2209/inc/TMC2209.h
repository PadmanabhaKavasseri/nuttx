#ifndef TMC2209_H
#define TMC2209_H

#include <stdint.h>

#include <nuttx/config.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/serial/serial.h>
#include <nuttx/clock.h>

#include <sys/ioctl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>


extern int fd_;

extern uint8_t serial_address_;















// Serial Settings
const static uint8_t BYTE_MAX_VALUE = 0xFF;
const static uint8_t BITS_PER_BYTE = 8;

const static uint32_t ECHO_DELAY_INC_MICROSECONDS = 1;
const static uint32_t ECHO_DELAY_MAX_MICROSECONDS = 4000;

const static uint32_t REPLY_DELAY_INC_MICROSECONDS = 1;
const static uint32_t REPLY_DELAY_MAX_MICROSECONDS = 10000;

const static uint8_t STEPPER_DRIVER_FEATURE_OFF = 0;
const static uint8_t STEPPER_DRIVER_FEATURE_ON = 1;

// Datagrams
const static uint8_t WRITE_READ_REPLY_DATAGRAM_SIZE = 8;
const static uint8_t DATA_SIZE = 4;

typedef union 
{
    struct
    {
        uint64_t sync : 4;
        uint64_t reserved : 4;
        uint64_t serial_address : 8;
        uint64_t register_address : 7;
        uint64_t rw : 1;
        uint64_t data : 32;
        uint64_t crc : 8;
    };
    uint64_t bytes;
} WriteReadReplyDatagram;

const static uint8_t SYNC = 0b101;
const static uint8_t RW_READ = 0;
const static uint8_t RW_WRITE = 1;
const static uint8_t READ_REPLY_SERIAL_ADDRESS = 0b11111111;

const static uint8_t READ_REQUEST_DATAGRAM_SIZE = 4;
typedef union 
{
    struct
    {
        uint32_t sync : 4;
        uint32_t reserved : 4;
        uint32_t serial_address : 8;
        uint32_t register_address : 7;
        uint32_t rw : 1;
        uint32_t crc : 8;
    };
    uint32_t bytes;
} ReadRequestDatagram;

// General Configuration Registers
const static uint8_t ADDRESS_GCONF = 0x00;
typedef union 
{
    struct
    {
        uint32_t i_scale_analog : 1;
        uint32_t internal_rsense : 1;
        uint32_t enable_spread_cycle : 1;
        uint32_t shaft : 1;
        uint32_t index_otpw : 1;
        uint32_t index_step : 1;
        uint32_t pdn_disable : 1;
        uint32_t mstep_reg_select : 1;
        uint32_t multistep_filt : 1;
        uint32_t test_mode : 1;
        uint32_t reserved : 22;
    };
    uint32_t bytes;
} GlobalConfig;

extern GlobalConfig global_config_;

const static uint8_t ADDRESS_GSTAT = 0x01;
typedef union 
{
    struct
    {
        uint32_t reset : 1;
        uint32_t drv_err : 1;
        uint32_t uv_cp : 1;
        uint32_t reserved : 29;
    };
    uint32_t bytes;
} GlobalStatus;

const static uint8_t ADDRESS_IFCNT = 0x02;

const static uint8_t ADDRESS_REPLYDELAY = 0x03;
union ReplyDelay
{
struct
{
    uint32_t reserved_0 : 8;
    uint32_t replydelay : 4;
    uint32_t reserved_1 : 20;
};
uint32_t bytes;
};

const static uint8_t ADDRESS_IOIN = 0x06;
union Input
{
struct
{
    uint32_t enn : 1;
    uint32_t reserved_0 : 1;
    uint32_t ms1 : 1;
    uint32_t ms2 : 1;
    uint32_t diag : 1;
    uint32_t reserved_1 : 1;
    uint32_t pdn_serial : 1;
    uint32_t step : 1;
    uint32_t spread_en : 1;
    uint32_t dir : 1;
    uint32_t reserved_2 : 14;
    uint32_t version : 8;
};
uint32_t bytes;
};
const static uint8_t VERSION = 0x21;













// Velocity Dependent Driver Feature Control Register Set
typedef union
{
    struct
    {
        uint32_t ihold : 5;
        uint32_t reserved_0 : 3;
        uint32_t irun : 5;
        uint32_t reserved_1 : 3;
        uint32_t iholddelay : 4;
        uint32_t reserved_2 : 12;
    } bits;
    uint32_t bytes;
} DriverCurrent;

extern DriverCurrent driver_current_;


const static uint8_t PERCENT_MIN = 0;
const static uint8_t PERCENT_MAX = 100;
const static uint8_t CURRENT_SETTING_MIN = 0;
const static uint8_t CURRENT_SETTING_MAX = 31;
const static uint8_t HOLD_DELAY_MIN = 0;
const static uint8_t HOLD_DELAY_MAX = 15;
const static uint8_t IHOLD_DEFAULT = 16;
const static uint8_t IRUN_DEFAULT = 31;
const static uint8_t IHOLDDELAY_DEFAULT = 1;

const static uint8_t ADDRESS_TPOWERDOWN = 0x11;
const static uint8_t TPOWERDOWN_DEFAULT = 20;

const static uint8_t ADDRESS_TSTEP = 0x12;

const static uint8_t ADDRESS_TPWMTHRS = 0x13;
const static uint32_t TPWMTHRS_DEFAULT = 0;

const static uint8_t ADDRESS_VACTUAL = 0x22;
const static int32_t VACTUAL_DEFAULT = 0;
const static int32_t VACTUAL_STEP_DIR_INTERFACE = 0;




void moveAtVelocity(int32_t microsteps_per_period);
void pkwrite(uint8_t register_address, uint32_t data);
uint32_t reverseData(uint32_t data);
uint8_t calculateCrc(WriteReadReplyDatagram* datagram, uint8_t datagram_size);
void sendDatagramUnidirectional(WriteReadReplyDatagram* datagram, uint8_t datagram_size);
void serialWrite(uint8_t c);
void init(int fd);


#endif //TMC2209_H