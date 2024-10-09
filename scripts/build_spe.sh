#!/usr/bin/bash
eval "$(cat $(dirname "${BASH_SOURCE[0]}")/header.sh)"

export CROSS_COMPILE=$WORKSPACE_DIR/spe_toolchain/gcc-arm-none-eabi-7-2018-q2-update/bin/arm-none-eabi-
export TOP=$TOP_DIR/orin_r5code/

cd $TOP_DIR/orin_r5code/rt-aux-cpu-demo-fsp/
make bin_t23x

cd $TOP_DIR
