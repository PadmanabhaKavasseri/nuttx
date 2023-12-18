# Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear


#!/bin/bash

make distclean 
 ./tools/configure.sh -l qcomf427:nsh
make 

scp nuttx.bin  root@169.254.76.47:/data/uploader

#if 0
#make menuconfig

#make savedefconfig && /bin/cp defconfig boards/arm/stm32/qcomcarf4/configs/ackerman/defconfig -rf
#fi


