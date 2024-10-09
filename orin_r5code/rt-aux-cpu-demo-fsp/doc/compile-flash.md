Compiling and Flashing {#rt-compiling}
======================

@tableofcontents

This Document describes the steps to build and flash this project

# TOOLCHAIN PREREQUISITE#

You must download the external Toolchain. NVIDIA does not distribute this toolchain.

gcc-arm-none-eabi-7-2018-q2-update:

https://developer.arm.com/-/media/Files/downloads/gnu-rm/7-2018q2/gcc-arm-none-eabi-7-2018-q2-update-linux.tar.bz2?revision=bc2c96c0-14b5-4bb4-9f18-bceb4050fee7?product=GNU%20Arm%20Embedded%20Toolchain,64-bit,,Linux,7-2018-q2-update

# BUILD

Set the appropriate paths for the below variables:
```
export TOP=<path to root directory where rt-aux-cpu-demo-fsp, fsp and freertos source directories reside>
export CROSS_COMPILE=<path to installed cross compiler>/gcc-arm-none-eabi-7-2018-q2-update/bin/arm-none-eabi-

cd ${TOP}/rt-aux-cpu-demo-fsp
```

## To build firmware for a specific platform
Build firmware for t19x:
```
make bin_t19x
```
Build firmware for t23x:
```
make bin_t23x
```
@note Build speed is significantly faster by using the `-j <jobs>` option with `make`.
For example, `make -j16 bin_t19x`.

## To build doxygen documents

Make sure you have doxygen installed on your computer. For Ubuntu systems:
```
sudo apt-get install doxygen
```
To generate the documents:
```
make docs
```
The output can be found at `out/docs`.

## To build everything

```
make
```
or
```
make all
```
The above commands build all the targets i.e. spe firmware binaries for all the
SOCs and Doxygen documents.

## Build artifacts

The built SPE firmware binary can be found at `out/<soc>/spe.bin`.
The doxygen build generates `out/docs/index.html` file. Open it
using a browser to navigate the documentation.

## To clean the build artifacts
Clean everything (documentation and firmware):
```
	make clean
```
To clean t19x only firmware objects and build artifacts:
```
	make clean_t19x
```
To clean t23x only firmware objects and build artifacts:
```
	make clean_t23x
```
To clean only doxygen generated files:
```
	make clean_docs
```

# FLASH

1. Back up the original copies of `spe_t194.bin` and `spe_t234.bin` located in the
following directory:
```
    Linux_for_Tegra/bootloader/
```
2. Copy the generated `out/<soc>/spe.bin` to the following locating, depending on your target.
   - For the T194 SoC, copy it to:
```
         Linux_for_Tegra/bootloader/spe_t194.bin
```
   - For the T234 SoC, copy it to:
```
         Linux_for_Tegra/bootloader/spe_t234.bin
```
   The `Linux_for_Tegra` directory is part of the extracted L4T build that
   you used to flash the Jetson device.
3. Use the command below to just flash only the spe-fw partition:
```
       sudo ./flash.sh -k spe-fw <T194 SoC based Jetson platforms> mmcblk0p1
       sudo ./flash.sh -k A_spe-fw <T234 SoC based Jetson platforms> mmcblk0p1
```
