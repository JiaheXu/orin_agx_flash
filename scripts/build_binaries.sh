#!/usr/bin/bash
eval "$(cat $(dirname "${BASH_SOURCE[0]}")/header.sh)"

export CROSS_COMPILE_AARCH64_PATH=$KERNEL_TOOLCHAIN_PATH
export CROSS_COMPILE_AARCH64=$KERNEL_TOOLCHAIN_PATH/bin/aarch64-buildroot-linux-gnu-

echo "Building kernel..."

cd $PUBLIC_SOURCES_PATH/Linux_for_Tegra/source/public/kernel_src
echo $KERNEL_TOOLCHAIN_PATH
ls $KERNEL_TOOLCHAIN_PATH
mkdir -p "$(pwd)/kernel_out"
./nvbuild.sh -o "$(pwd)/kernel_out"

echo "Building SPE..."

$SCRIPTS_DIR/build_spe.sh

cd $TOP_DIR
