#!/usr/bin/bash
eval "$(cat $(dirname "${BASH_SOURCE[0]}")/header.sh)"

# There is no rootfs/usr/lib/modules directory until apply_binaries.sh is run so I don't copy nvgpu.ko
# sudo cp drivers/gpu/nvgpu/nvgpu.ko ../../rootfs/usr/lib/modules/5.10.104-tegra/kernel/drivers/gpu/nvgpu/nvgpu.ko
cp $PUBLIC_SOURCES_PATH/Linux_for_Tegra/source/public/kernel_src/kernel_out/arch/arm64/boot/dts/nvidia/* \
   $L4T_PATH/Linux_for_Tegra/kernel/dtb/
cp $PUBLIC_SOURCES_PATH/Linux_for_Tegra/source/public/kernel_src/kernel_out/arch/arm64/boot/Image \
   $L4T_PATH/Linux_for_Tegra/kernel/Image
cd $L4T_PATH/Linux_for_Tegra/
sudo ./apply_binaries.sh

cp $TOP_DIR/orin_r5code/rt-aux-cpu-demo-fsp/out/t23x/spe.bin $L4T_PATH/Linux_for_Tegra/bootloader/spe_t234.bin

cd $TOP_DIR
