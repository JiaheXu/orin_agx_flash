# Orin AGX Flashing

## Select Config

For a 32GB Orin AGX, you can leave the settings unchanged. For a 64GB Orin AGX, change the `version=35.1.0` line in `setup.config` to `version=35.3.1`.

## Get Dependencies

```
sudo apt install flex bison qemu-user-static
```

Get the submodules:

```
git submodule update --init --recursive
```

## Flash

```
./scripts/full_process.sh
```

## Install Libraries

Use the SDK manager to install the nvidia libraries on the Orin AGX. On the screen for "Step 01", for "Target Operating System" choose "JetPack 5.0.2 (rev. 2)" for a 32GB Orin AGX or "JetPack 5.1.1 (rev. 1)" for a 64GB Orin AGX. You can uncheck "Host Machine" under "Hardware Configuration". On the screen for "Step 02", make sure to uncheck "Jetson Linux" under "Target Components" so that it doesn't reflash the Orin.

## Zed X
Get the ZED code:
```
git clone git@github.com:stereolabs/zedx-driver.git
cd zedx-driver
git checkout L4T35.3_Stereolabs
```

To enable the aon_echo device, edit the file `src/hardware/nvidia/soc/t23x/kernel-dts/tegra234-soc/tegra234-aon.dtsi` and on line 66 change `disabled` to `okay`.

Next, build everything:

```
./build.sh MAX96712
```

This puts the built files into an `output/` directory. Copy this over the orin:

```
scp -r output/ <orin>
```

Go into the `output/` directory on the orin and run the following commands:

```
sudo cp /boot/tegra234-p3701-0000-p3737-0000.dtb backup/
sudo cp /boot/dtb/kernel_tegra234-p3701-0000-p3737-0000.dtb backup/

sudo cp tegra234-p3701-0000-p3737-0000.dtb /boot/dtb/kernel_tegra234-p3701-0000-p3737-0000.dtb
sudo cp tegra234-p3701-0000-p3737-0000.dtb /boot/tegra234-p3701-0000-p3737-0000.dtb

sudo cp zedx_ar0234.isp /var/nvidia/nvcam/settings/camera_overrides.isp
sudo chmod 664 /var/nvidia/nvcam/settings/camera_overrides.isp
sudo chown root:root /var/nvidia/nvcam/settings/camera_overrides.isp
```

Create the file `/etc/rc.local` and make it executable by doing `sudo chmod a+x /etc/rc.local`.

Add the following lines to it so that the zed kernel modules are loaded.

```
#!/bin/bash                                                                                                                                                                                                                                                                       

insmod <the path to where you scped the output/ directory>/max96712.ko
insmod <the path to where you scped the output/ directory>/sl_zedx.ko
```

Reboot so all of the above changes take effect.

Next, install the ZED SDK and everything should work.