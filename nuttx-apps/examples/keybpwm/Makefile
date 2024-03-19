############################################################################
# apps/examples/pwm/Makefile
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(APPDIR)/Make.defs
QTIARM_DIR := $(APPDIR)/examples/keybpwm

# PWM built-in application info

PROGNAME = keybpwm
PRIORITY = SCHED_PRIORITY_DEFAULT
STACKSIZE = $(CONFIG_DEFAULT_TASK_STACKSIZE)
MODULE = $(CONFIG_EXAMPLES_KEYBPWM)

DEPPATH += --dep-path app

C_INCLUDES +=  \
	-I$(QTIARM_DIR)/libqrc/include/app_msg \
    -I$(QTIARM_DIR)/libqrc/include/module \
    -I$(QTIARM_DIR)/libqrc/protocol/qrc \
	-I$(QTIARM_DIR)/libqrc/protocol/qrc/* \
	-I$(QTIARM_DIR)/libqrc/protocol/qrc/tinyframe \
	-I$(QTIARM_DIR)/libqrc/protocol/tinyframe \


MAINSRC = keybpwm_main.c

CSRCS += $(QTIARM_DIR)/libqrc/protocol/qrc/qrc.c
CSRCS += $(QTIARM_DIR)/libqrc/protocol/qrc/qrc_threadpool.c
CSRCS += $(QTIARM_DIR)/libqrc/protocol/tinyframe/TinyFrame.c
CSRCS += $(QTIARM_DIR)/libqrc/protocol/qrc_msg_management.c

# add define
CFLAGS += -DQRC_MCB
CFLAGS += -DDEBUG_MCB

CFLAGS += $(C_INCLUDES)

CFLAGS += -Wl,-u_printf_float,-o0

include $(APPDIR)/Application.mk
