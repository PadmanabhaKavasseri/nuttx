#ifndef __APP_QTIAMR_MAIN_H
#define __APP_QTIAMR_MAIN_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdio.h>
#include <stdint.h>

#ifdef CONFIG_APP_QCOMAMR

#include "rc.h"
#include "motion_task.h"

#define AMP_LIMIT(_val_, _min_, _max_)  \
	((_val_) < (_min_) ?  (_min_) : \
	((_val_) > (_max_) ? (_max_) : (_val_)))

#endif
#endif
