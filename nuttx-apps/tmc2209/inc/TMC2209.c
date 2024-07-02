#include "TMC2209.h"


PwmConfig pwm_config_;
CoolConfig cool_config_;
bool cool_step_enabled_;
ChopperConfig chopper_config_;
DriverCurrent driver_current_;
GlobalConfig global_config_;

int fd_ = 0;
uint8_t toff_;
int16_t hardware_enable_pin_ = 0;


void printDG(WriteReadReplyDatagram* dg){
    printf("Bytes: %d ",dg->bytes);
    printf("Sync: %d ",dg->sync);
    printf("Serial Address: %d ",dg->serial_address);
    printf("register_address: %d ",dg->register_address);
    printf("RW: %d ",dg->rw);
    printf("data: %d ",dg->data);
    printf("crc: %d \n",dg->crc);
}


uint8_t serial_address_ = 0;
// DriverCurrent driver_current_;

void serialWrite(uint8_t c){
    // write(fd_,&c,sizeof(c));
    // printf("%d\n");
}

void sendDatagramUnidirectional(WriteReadReplyDatagram* datagram, uint8_t datagram_size)
{
    uint8_t byte;

    // printf("SendUnidgg\n");
    // printDG(datagram);

    for (uint8_t i=0; i<datagram_size; ++i)
    {
        byte = (datagram->bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
        printf("%d ",byte);
        serialWrite(byte);
    }
    printf("\n");
}

uint8_t calculateCrc(WriteReadReplyDatagram* datagram, uint8_t datagram_size)
{
    uint8_t crc = 0;
    uint8_t byte;
    for (uint8_t i=0; i<(datagram_size - 1); ++i){
        
        if (datagram_size == sizeof(WriteReadReplyDatagram)) {
            byte = (((WriteReadReplyDatagram*)datagram)->bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
        }
        for (uint8_t j=0; j<BITS_PER_BYTE; ++j){
            if ((crc >> 7) ^ (byte & 0x01)){
                crc = (crc << 1) ^ 0x07;
            }
            else{
                crc = crc << 1;
            }
            byte = byte >> 1;
        }
    }
    return crc;
}

uint32_t reverseData(uint32_t data)
{
    uint32_t reversed_data = 0;
    uint8_t right_shift;
    uint8_t left_shift;
    for (uint8_t i=0; i<DATA_SIZE; ++i){
        right_shift = (DATA_SIZE - i - 1) * BITS_PER_BYTE;
        left_shift = i * BITS_PER_BYTE;
        reversed_data |= ((data >> right_shift) & BYTE_MAX_VALUE) << left_shift;
    }
    return reversed_data;
}



void pkwrite(uint8_t register_address, uint32_t data){
    // printf("data: %d\n", data);
    WriteReadReplyDatagram write_datagram;
    write_datagram.bytes = 0;
    write_datagram.sync = SYNC;
    write_datagram.serial_address = serial_address_;
    write_datagram.register_address = register_address;
    write_datagram.rw = RW_WRITE;
    write_datagram.data = reverseData(data);
    write_datagram.crc = calculateCrc(&write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);

    // printDG(&write_datagram);
    sendDatagramUnidirectional(&write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);
}

void moveAtVelocity(int32_t microsteps_per_period){
    printf("mv@Vel:%d\n",microsteps_per_period);
    pkwrite(ADDRESS_VACTUAL, microsteps_per_period);
}

void writeStoredGlobalConfig()
{
  pkwrite(ADDRESS_GCONF, global_config_.bytes);
}

//initialize functions
void setOperationModeToSerial()
{
  global_config_.bytes = 0;
  global_config_.i_scale_analog = 0;
  global_config_.pdn_disable = 1;
  global_config_.mstep_reg_select = 1;
  global_config_.multistep_filt = 1;

  writeStoredGlobalConfig();
}

void setRegistersToDefaults()
{
  driver_current_.bytes = 0;
  driver_current_.bits.ihold = IHOLD_DEFAULT;
  driver_current_.bits.irun = IRUN_DEFAULT;
  driver_current_.bits.iholddelay = IHOLDDELAY_DEFAULT;
  pkwrite(ADDRESS_IHOLD_IRUN, driver_current_.bytes);

  chopper_config_.bytes = CHOPPER_CONFIG_DEFAULT;
  chopper_config_.tbl = TBL_DEFAULT;
  chopper_config_.hend = HEND_DEFAULT;
  chopper_config_.hstart = HSTART_DEFAULT;
  chopper_config_.toff = TOFF_DEFAULT;
  pkwrite(ADDRESS_CHOPCONF, chopper_config_.bytes);

  pwm_config_.bytes = PWM_CONFIG_DEFAULT;
  pkwrite(ADDRESS_PWMCONF, pwm_config_.bytes);

  cool_config_.bytes = COOLCONF_DEFAULT;
  pkwrite(ADDRESS_COOLCONF, cool_config_.bytes);

  pkwrite(ADDRESS_TPOWERDOWN, TPOWERDOWN_DEFAULT);
  pkwrite(ADDRESS_TPWMTHRS, TPWMTHRS_DEFAULT);
  pkwrite(ADDRESS_VACTUAL, VACTUAL_DEFAULT);
  pkwrite(ADDRESS_TCOOLTHRS, TCOOLTHRS_DEFAULT);
  pkwrite(ADDRESS_SGTHRS, SGTHRS_DEFAULT);
  pkwrite(ADDRESS_COOLCONF, COOLCONF_DEFAULT);
}

void minimizeMotorCurrent()
{
  driver_current_.bits.irun = CURRENT_SETTING_MIN;
  driver_current_.bits.ihold = CURRENT_SETTING_MIN;
  writeStoredDriverCurrent();
}

void disable()
{
  if (hardware_enable_pin_ >= 0)
  {
    //digitialWrite(hardware_enable_pin_, HIGH);
  }
  chopper_config_.toff = TOFF_DISABLE;
  writeStoredChopperConfig();
}

void disableAutomaticCurrentScaling()
{
  pwm_config_.pwm_autoscale = STEPPER_DRIVER_FEATURE_OFF;
  writeStoredPwmConfig();
}

void disableAutomaticGradientAdaptation()
{
  pwm_config_.pwm_autograd = STEPPER_DRIVER_FEATURE_OFF;
  writeStoredPwmConfig();
}

void writeStoredDriverCurrent()
{
  pkwrite(ADDRESS_IHOLD_IRUN, driver_current_.bytes);

  if (driver_current_.bits.irun >= SEIMIN_UPPER_CURRENT_LIMIT)
  {
    cool_config_.seimin = SEIMIN_UPPER_SETTING;
  }
  else
  {
    cool_config_.seimin = SEIMIN_LOWER_SETTING;
  }
  if (cool_step_enabled_)
  {
    pkwrite(ADDRESS_COOLCONF, cool_config_.bytes);
  }
}

void writeStoredChopperConfig()
{
  pkwrite(ADDRESS_CHOPCONF, chopper_config_.bytes);
}

// uint32_t readChopperConfigBytes()
// {
//   return read(ADDRESS_CHOPCONF);
// }

void writeStoredPwmConfig()
{
  pkwrite(ADDRESS_PWMCONF, pwm_config_.bytes);
}

// uint32_t readPwmConfigBytes()
// {
//   return read(ADDRESS_PWMCONF);
// }

uint32_t constrain_(uint32_t value, uint32_t low, uint32_t high)
{
  return ((value)<(low)?(low):((value)>(high)?(high):(value)));
}

int32_t map(int32_t value, int32_t fromLow, int32_t fromHigh, int32_t toLow, int32_t toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

uint8_t percentToCurrentSetting(uint8_t percent)
{
  uint8_t constrained_percent = constrain_(percent,
    PERCENT_MIN,
    PERCENT_MAX);
  uint8_t current_setting = map(constrained_percent,
    PERCENT_MIN,
    PERCENT_MAX,
    CURRENT_SETTING_MIN,
    CURRENT_SETTING_MAX);
  return current_setting;
}

void setMicrostepsPerStepPowerOfTwo(uint8_t exponent)
{
  switch (exponent)
  {
    case 0:
    {
      chopper_config_.mres = MRES_001;
      break;
    }
    case 1:
    {
      chopper_config_.mres = MRES_002;
      break;
    }
    case 2:
    {
      chopper_config_.mres = MRES_004;
      break;
    }
    case 3:
    {
      chopper_config_.mres = MRES_008;
      break;
    }
    case 4:
    {
      chopper_config_.mres = MRES_016;
      break;
    }
    case 5:
    {
      chopper_config_.mres = MRES_032;
      break;
    }
    case 6:
    {
      chopper_config_.mres = MRES_064;
      break;
    }
    case 7:
    {
      chopper_config_.mres = MRES_128;
      break;
    }
    case 8:
    default:
    {
      chopper_config_.mres = MRES_256;
      break;
    }
  }
  writeStoredChopperConfig();
}

void setRunCurrent(uint8_t percent)
{
  uint8_t run_current = percentToCurrentSetting(percent);
  driver_current_.bits.irun = run_current;
  writeStoredDriverCurrent();
}

void enableCoolStep(uint8_t lower_threshold,
    uint8_t upper_threshold)
{
  lower_threshold = constrain_(lower_threshold, SEMIN_MIN, SEMIN_MAX);
  cool_config_.semin = lower_threshold;
  upper_threshold = constrain_(upper_threshold, SEMAX_MIN, SEMAX_MAX);
  cool_config_.semax = upper_threshold;
  pkwrite(ADDRESS_COOLCONF, cool_config_.bytes);
  cool_step_enabled_ = true;
}

void enable()
{
  if (hardware_enable_pin_ >= 0)
  {
    // digitalWrite(hardware_enable_pin_, LOW);
  }
  chopper_config_.toff = toff_;
  writeStoredChopperConfig();
}

void init(int fd){
    
    fd_ = fd;
    serial_address_ = 0;
    hardware_enable_pin_ = 0;
    toff_ = TOFF_DEFAULT;

    
    

    setOperationModeToSerial();
    setRegistersToDefaults();

    minimizeMotorCurrent();
    disable();
    disableAutomaticCurrentScaling();
    disableAutomaticGradientAdaptation();
}