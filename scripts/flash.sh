#!/usr/bin/bash
eval "$(cat $(dirname "${BASH_SOURCE[0]}")/header.sh)"

cd $L4T_PATH/Linux_for_Tegra

sudo ./flash.sh jetson-agx-orin-devkit mmcblk0p1

cd $TOP_DIR
