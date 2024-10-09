#!/usr/bin/bash
eval "$(cat $(dirname "${BASH_SOURCE[0]}")/header.sh)"

export CROSS_COMPILE=$WORKSPACE_DIR/spe_toolchain/gcc-arm-none-eabi-7-2018-q2-update/bin/arm-none-eabi-
export TOP=$TOP_DIR/orin_r5code/

cd $TOP_DIR/orin_r5code/rt-aux-cpu-demo-fsp/
make bin_t23x

cd $TOP_DIR
cp $TOP_DIR/orin_r5code/rt-aux-cpu-demo-fsp/out/t23x/spe.bin $L4T_PATH/Linux_for_Tegra/bootloader/spe_t234.bin

cd $L4T_PATH/Linux_for_Tegra
sudo ./flash.sh -k A_spe-fw jetson-agx-orin-devkit mmcblk0p1

cd $TOP_DIR
