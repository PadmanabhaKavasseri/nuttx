
include $(APPDIR)/Make.defs

CAR_DIR := $(APPDIR)/qcomamr
PROGNAME  = qcomamr
PRIORITY  = SCHED_PRIORITY_DEFAULT
STACKSIZE = $(CONFIG_DEFAULT_TASK_STACKSIZE)
MODULE    = $(CONFIG_APP_QCOMAMR)

DEPPATH += --dep-path motion
DEPPATH += --dep-path ultrasound
DEPPATH += --dep-path imu
DEPPATH += --dep-path roscom
DEPPATH += --dep-path rc
DEPPATH += --dep-path auto_charging
DEPPATH += --dep-path misc

C_INCLUDES +=  \
    -I$(CAR_DIR) \
    -I$(CAR_DIR)/motion \
    -I$(CAR_DIR)/ultrasound \
    -I$(CAR_DIR)/imu \
    -I$(CAR_DIR)/roscom \
    -I$(CAR_DIR)/rc \
	-I$(CAR_DIR)/auto_charging \
	-I$(CAR_DIR)/misc \

MAINSRC = amr_main.c
CSRCS += $(CAR_DIR)/motion/kinematics.c
CSRCS += $(CAR_DIR)/motion/motion_task.c
CSRCS += $(CAR_DIR)/motion/motor_driver.c
CSRCS += $(CAR_DIR)/motion/canopen.c

CSRCS += $(CAR_DIR)/ultrasound/ultrasound.c

CSRCS += $(CAR_DIR)/roscom/ros_com.c

CSRCS += $(CAR_DIR)/imu/imu.c

CSRCS += $(CAR_DIR)/rc/rc.c
CSRCS += $(CAR_DIR)/auto_charging/auto_charging.c
CSRCS += $(CAR_DIR)/misc/amr_adc.c

CFLAGS += $(C_INCLUDES)

CFLAGS += -Wl,-u_printf_float

include $(APPDIR)/Application.mk
