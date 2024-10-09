I2C application (app/i2c-app.c)
===============================

@tableofcontents

I2C application demonstrates how to access/manipulate Always On (AON) I2C from
the SPE/AON processor. There are three I2C instances (2, 8 and 10) in AON
domain, availability of the instance will be based on the Jetson platform being
used.

@note It is required to compile device tree and flash board to have them
updated after making kernel device tree changes as described in following
sections.


## Hardware Configuration

The sample app uses I2C bus 8 and BMI160 sensor module, similar to
https://hackspark.fr/en/electronics/1341-6dof-bosch-6-axis-acceleration-gyro-gravity-sensor-gy-bmi160.html
for the demo purposes.

The I2C app accesses the 40 pin header J30 and expects I2C external module at the pin map
shown below:

- Pin 1  - 3.3V
- Pin 3  - SDA
- Pin 5  - SCL
- Pin 34 - SAO
- Pin 39 - GND

The demo app reads BMI160 sensor ID. If the app successfully retrieves the
correct ID, it prints the following message:

    I2C test successful

@note BMI160 is configured to have the I2C address 0x68 by connecting its SAO pin to
Pin 34.

## Software Configuration

# Jetson AGX Orin
1. Make sure I2C bus 8 is disabled in linux kernel device tree and no other
processors using it. You must run source_sync.sh to get the dtsi file. After running
source_sync.sh, the file is located at Linux_for_Tegra/sources/hardware/nvidia/soc/t23x/kernel-dts/tegra234-soc/tegra234-soc-cvm.dtsi.
Disabling device node from the kernel makes that I2C controller and all the client devices
attached to it unaccessible from the kernel.
```
    i2c@c250000 {
            status = "disabled";
    };
```
2. Compile the device tree and copy the dtb file to Linux_for_Tegra/dtb/ directory.
3. Disable firewall setting in bootloader/tegra234-firewall-config-base.dtsi
so that SPE can access I2C registers.
```
        reg@2130 { /* CLK_RST_CONTROLLER_AON_SCR_I2C8_0 */
            exclusion-info = <3>;
            value = <0x30001610>;
        };
```

4. In `soc/t23x/target_specific.mk`, set `ENABLE_I2C_APP := 1` and rebuild the
application. Copy the output to `${L4T}/bootloader/spe_t234.bin`

5. Reflash all the partitions, including new device tree and SPE binaries
generated from the above steps by referencing the instructions in the **Building the Kernel**
section of the Linux Developer Guide.

The demo app reads BMI160 sensor ID. If the app successfully retrieves the
correct ID, it prints the following message:

    I2C test successful


# Jetson AGX
1. The sample app uses I2C bus 8 and on board audio codec chip for demo purposes.
Make sure access to codec chip and I2C bus 8 from other software stacks like
kernel/bootloader etc... is disabled. For kernel, related dts is
tegra194-audio-p2822-0000.dtsi and below is the sample snippet to disable:
```
    i2c@c250000 {
          status = "disabled";
          rt5658: rt5659.7-001a@1a {
                  compatible = "realtek,rt5658";
                  reg = <0x1a>;

                  realtek,jd-src = <RT5659_JD_NULL>;
                  realtek,dmic1-data-pin = <RT5659_DMIC1_NULL>;
                  realtek,dmic2-data-pin = <RT5659_DMIC2_DATA_IN2P>;

                  gpios = <&tegra_main_gpio TEGRA194_MAIN_GPIO(S, 5) 0>;

                  status = "disabled";
          };
    };
```

2. In `soc/t19x/target_specific.mk`, set `ENABLE_I2C_APP := 1` and rebuild the
application. Copy the output to `${L4T}/bootloader/spe_t194.bin`

3. Reflash all the paritions including new device tree and SPE binaries
generated from the above steps.

The demo app reads device ID. Test is successful if app retrieves correct ID and
should print "I2C test successful".
