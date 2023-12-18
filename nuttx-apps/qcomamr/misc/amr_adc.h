#ifndef __APP_QCOMAMR_ADC_H
#define __APP_QCOMAMR_ADC_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <nuttx/fs/fs.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>


/* channel 1: SERVO slides; channel 2: power voltage  */

#define ADC_POWER_CHANNEL		(10)

#define ADC_MAX_RANGE			(4096)
#define ADC_MAX_RANGE_HALF			(2048)

#define DEV_VOLTAGE   "/dev/adc_power"

#define ADC_MAX_GROUPSIZE		(1) /* Actual 1 adc controller have 16 channels */

struct car_adc_s
{
  bool	initialized;
  FAR char *adc_devpath;
  struct adc_msg_s samples[ADC_MAX_GROUPSIZE];   /* adc data */
};

int get_power_voltage(float * voltage);

int amr_adc_init(void);
void amr_adc_deinit(void);

#endif
