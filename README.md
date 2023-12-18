# nuttx_qcomamr

## 1  build nuttx bootloader
```
$ cd nuttx
$ ./tools/configure.sh -l qtiamrboot:bootloader
$ make
$ will generate nuttx.hex in current dictionary
```

## 2  flash nuttx bootloader
```
$ mkdir a folder named "flash_bootloader" in you Ubuntu PC
$ copy cmsis-dap.cfg stm32f4x.cfg from \\happy\IOE\edward\Nuttx\bootloader\0515 into flash_bootloader folder
$ copy nuttx.hex from your compiling dictionary into flash_bootloader folder
$ sudo apt install openocd
$ sudo openocd -f cmsis-dap.cfg -f stm32f4x.cfg  -c 'init'  -c 'program nuttx.hex' -c 'shutdown'
```

## 3  build nuttx OS image
```
$ cd nuttx
$ ./tools/configure.sh -l qcomf427:nsh
$ make
$ will generate nuttx.bin in current dictionary
```

## 4  flash nuttx OS image
```
$ git clone https://github.qualcomm.com/China-IoT-Robotics-Team/nuttx_bootloader_flash.git in your Ubuntu PC
$ cd nuttx_bootloader_flash
$ copy nuttx.bin from your compiling dictionary into nuttx_bootloader_flash folder
$ chmod 777 *
$ ./flash.sh
```

## 5 Note
```
$ Whether you compile nuttx bootloader or nuttx OS image, will generate different nuttx.bin and buttx.hex
$ But when we flash nuttx bootloader, we only use nuttx.hex
$ When we flash nuttx OS image, we only use nuttx.bin
$ The HEX file contains the address information, whereas the BIN file format contains only the data itself
```
