include $(APPDIR)/Make.defs

# Hello, World! built-in application info

PROGNAME  = TMC2209
PRIORITY  = 100
STACKSIZE = $(CONFIG_DEFAULT_TASK_STACKSIZE)
MODULE    = $(CONFIG_TMC2209)

# DEPPATH += --dep-path app

C_INCLUDES +=  \
	-I$(APPDIR)/tmc2209/inc \


MAINSRC = starter.c

CSRCS += $(APPDIR)/tmc2209/inc/TMC2209.c
CFLAGS += $(C_INCLUDES)
CFLAGS += -Werror=implicit-function-declaration

include $(APPDIR)/Application.mk
