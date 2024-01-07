make savedefconfig && cp defconfig boards/arm/stm32/qcomf427/configs/nsh/defconfig
printf "\n Saved Changes! \n"

./tools/configure.sh -l qcomf427:nsh
printf "\n Configuration Complete! \n"

make
printf "\n Make Complete! \n"

printf "\n Pushing Nuttx.bin \n"
cp nuttx.bin /data/misc/mcb

printf "\n Flashing Nuttx.bin to MCB\n"
mcbflash --flash
