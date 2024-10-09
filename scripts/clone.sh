#!/usr/bin/bash
eval "$(cat $(dirname "${BASH_SOURCE[0]}")/header.sh)"

# TODO define download and extract functions

function download() {
    url=$1
    output=$2

    
    if [ ! -e "$output" ]; then
	echo "download url: $url output: $output"
        wget $url -O $output
    fi
}

function extract() {
    opts=$1
    input=$2
    output=$3
    
    echo "extract opts: $opts input: $input output: $output"

    if [ ! -e "$output" ]; then
	mkdir -p $output
	tar -$opts $input -C $output
    else
	return 1
    fi
    return 0
}

echo "workspace $WORKSPACE_DIR"
mkdir -p $WORKSPACE_DIR
mkdir -p $RAW_DOWNLOADS

echo "Downloading..."

download $KERNEL_TOOLCHAIN_URL $RAW_DOWNLOADS/$KERNEL_TOOLCHAIN_FILENAME
download $SPE_TOOLCHAIN_URL $RAW_DOWNLOADS/$SPE_TOOLCHAIN_FILENAME
download $L4T_URL $RAW_DOWNLOADS/$L4T_FILENAME
download $ROOTFS_URL $RAW_DOWNLOADS/$ROOTFS_FILENAME
download $PUBLIC_SOURCES_URL $RAW_DOWNLOADS/$PUBLIC_SOURCES_FILENAME

echo "Extracting..."

extract xzf $RAW_DOWNLOADS/$KERNEL_TOOLCHAIN_FILENAME $KERNEL_TOOLCHAIN_PATH
extract xjf $RAW_DOWNLOADS/$SPE_TOOLCHAIN_FILENAME $SPE_TOOLCHAIN_PATH
extract xjf $RAW_DOWNLOADS/$L4T_FILENAME $L4T_PATH
if [ "$?" == "0" ]; then
    sudo tar -xjpf $RAW_DOWNLOADS/$ROOTFS_FILENAME -C $ROOTFS_PATH
fi
extract xjf $RAW_DOWNLOADS/$PUBLIC_SOURCES_FILENAME $PUBLIC_SOURCES_PATH
extract xjf $PUBLIC_SOURCES_PATH/Linux_for_Tegra/source/public/kernel_src.tbz2 $PUBLIC_SOURCES_PATH/Linux_for_Tegra/source/public/kernel_src

cd $TOP_DIR
