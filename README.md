Buiding Nuttx

## Build Nuttx OS image

```bash
cd /home/nuttx_qcomamr/nuttx/
make distclean 
OR 
make savedefconfig && cp defconfig boards/arm/stm32/qcomf427/configs/nsh/defconfig
./tools/configure.sh -l qcomf427:nsh
make
```

## Flash Nuttx OS image

Push nuttx.bin to bootloader flash

```bash
cp nuttx.bin ../../nuttx_bootloader_flash/.
cd ../../nuttx_bootloader_flash/    
./flash.sh
```

 Flash using mcbflash command

```bash
cp nuttx.bin /data/misc/mcb
mcbflash --flash
```

### Nuttx Configuration 🤪 remember to save 💹

```bash
cd /home/nuttx_qcomamr/nuttx/
make menuconfig
make savedefconfig && cp defconfig boards/arm/stm32/qcomf427/configs/nsh/defconfig
```
