AODMIC Application (app/aodmic-app.c)
==============================
@tableofcontents

The AODMIC application demonstrates how to operate the Digital Microphone (DMIC)
interface from the SPE/AON processor. DMIC5 is located in the Always-On (AON)
domain and is correspondingly referred to as AODMIC.  This demo additionally
shows how AODMIC can be used as a system wake-up source.

The app runs continuously and captures data from AODMIC using GPCDMA. It
periodically prints the R5 CPU tick count and the zero crossing count (on both
capture channels). These counts can be used to determine whether the AODMIC is
running at the expected rate. The app also computes and prints the capture
volume over fixed periods (computed as the mean square of the samples). Should
the computed volume exceed a predefined threshold, it triggers a system wake
event.

The `sample_rate` parameter of the aodmic config structure can be set to any of
these values:

* `TEGRA_AODMIC_RATE_8KHZ`
* `TEGRA_AODMIC_RATE_16KHZ` (default in `app/aodmic-app.c`)
* `TEGRA_AODMIC_RATE_44KHZ` (44.1 kHz)
* `TEGRA_AODMIC_RATE_48KHZ`

**NOTE**: The AODMIC uses a fixed 64x oversampling rate.  The programmer must
ensure that the corresponding clock speed falls within the supported range of
the microphone.  For example, the 8 kHz sampling rate corresponds to a 512 kHz
clock speed.  If the microphone supports 1 MHz - 3.25 MHz, then 16 kHz is the
lowest clock speed that can be used.

The `channel_config` parameter of the aodmic config structure can be set to any of
these values:

* `TEGRA_AODMIC_CHANNEL_STEREO` (default in `app/aodmic-app.c`)
* `TEGRA_AODMIC_CHANNEL_MONO_LEFT`
* `TEGRA_AODMIC_CHANNEL_MONO_RIGHT`

The `num_periods` parameter of the aodmic config structure can set to any value in
the range of 2 to `AODMIC_MAX_NUM_PERIODS` (default of 4).  The typical value
for this parameter is 2 (i.e. double buffered I/O).  The value of
`AODMIC_MAX_NUM_PERIODS` can be increased if necessary.  It is located in the file
`fsp/source/include/aodmic/tegra-aodmic.h`.
# Prerequisites

Stereo PDM mics should be connected to the AODMIC pins on the respective
Jetson platform. An example PDM mic part is
https://www.mouser.in/datasheet/2/389/mp34dt01-m-955068.pdf. Details of the
AODMIC pins are mentioned in the platform specific sections further down.

# Jetson AGX Xavier

## Hardware Configuration

AODMIC signals are available at the 40-pin header J30 with these pin mappings:
- Pin 16 - DMIC5_DAT (CAN1_STB)
- Pin 32 - DMIC5_CLK (CAN1_EN)

Power and ground will also be needed for the PDM board.  A typical setup
might use J30 Pin 1 (3.3V) and Pin 6 (GND).

## Software Configuration

1. Pinmux must be set for the J30 pins by following the instructions in
[*AGX Xavier Developer Guide*](https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%2520Linux%2520Driver%2520Package%2520Development%2520Guide%2Fadaptation_and_bringup_xavier.html%23wwpID0E0RN0HA).
For the purpose of performing a quick test, the pinmux files can be manually
updated by editing
`${L4T}/bootloader/t186ref/BCT/tegra19x-mb1-pinmux-p2888-0000-a04-p2822-0000-b01.cfg`:

    --- a/tegra19x-mb1-pinmux-p2888-0000-a04-p2822-0000-b01.cfg
    +++ b/tegra19x-mb1-pinmux-p2888-0000-a04-p2822-0000-b01.cfg
    @@ -67,10 +67,6 @@ pinmux.0x0c2f16c0 = 0x00000001; # CONFIG AA6
    pinmux.0x0c303030 = 0x00000000; # GPIO can0_wake_paa6
    pinmux.0x0c2f16e0 = 0x00000001; # CONFIG AA7
    pinmux.0x0c303038 = 0x00000000; # GPIO can0_err_paa7
    -pinmux.0x0c2f1800 = 0x00000001; # CONFIG BB0
    -pinmux.0x0c303040 = 0x00000000; # GPIO can1_stb_pbb0
    -pinmux.0x0c2f1820 = 0x00000001; # CONFIG BB1
    -pinmux.0x0c303048 = 0x00000000; # GPIO can1_en_pbb1
    pinmux.0x0c2f1840 = 0x00000001; # CONFIG BB2
    pinmux.0x0c303050 = 0x00000000; # GPIO can1_wake_pbb2
    pinmux.0x02212ac0 = 0x00000001; # CONFIG P6
    @@ -276,6 +272,8 @@ pinmux.0x0c301000 = 0x00000500; # shutdown_n: shutdown, tristate-disable, input-
    pinmux.0x0c301040 = 0x00001440; # pwr_i2c_scl_pee5: i2c5, tristate-disable, input-enable, io_high_voltage-disable, lpdr-disable
    pinmux.0x0c301048 = 0x00001440; # pwr_i2c_sda_pee6: i2c5, tristate-disable, input-enable, io_high_voltage-disable, lpdr-disable
    pinmux.0x0c301028 = 0x00001458; # batt_oc_pee3: soc, pull-up, tristate-enable, input-enable, lpdr-disable
    +pinmux.0x0c303040 = 0x0000c452; # can1_stb_pbb0: dmic5, tristate-enable, input-enable
    +pinmux.0x0c303048 = 0x0000c402; # can1_en_pbb1: dmic5, tristate-disable, input-disable
    pinmux.0x0c302048 = 0x00001400; # spi2_sck_pcc0: spi2, tristate-disable, input-disable, io_high_voltage-disable, lpdr-disable
    pinmux.0x0c302050 = 0x00001450; # spi2_miso_pcc1: spi2, tristate-enable, input-enable, io_high_voltage-disable, lpdr-disable
    pinmux.0x0c302028 = 0x00001400; # spi2_mosi_pcc2: spi2, tristate-disable, input-disable, io_high_voltage-disable, lpdr-disable
    @@ -360,8 +358,6 @@ pinmux.0x0c303020 = 0x0000c000; # can0_stb_paa4: rsvd0, tristate-disable, input-
    pinmux.0x0c303028 = 0x0000c000; # can0_en_paa5: rsvd0, tristate-disable, input-disable
    pinmux.0x0c303030 = 0x0000c058; # can0_wake_paa6: rsvd0, pull-up, tristate-enable, input-enable
    pinmux.0x0c303038 = 0x0000c048; # can0_err_paa7: rsvd0, pull-up, tristate-disable, input-enable
    -pinmux.0x0c303040 = 0x0000c054; # can1_stb_pbb0: rsvd0, pull-down, tristate-enable, input-enable
    -pinmux.0x0c303048 = 0x0000c054; # can1_en_pbb1: rsvd0, pull-down, tristate-enable, input-enable
    pinmux.0x0c303050 = 0x0000c058; # can1_wake_pbb2: rsvd0, pull-up, tristate-enable, input-enable
    pinmux.0x02430078 = 0x00000000; # soc_gpio05_pp5: rsvd0, tristate-disable, input-disable, lpdr-disable
    pinmux.0x02430080 = 0x00000058; # soc_gpio06_pp6: rsvd0, pull-up, tristate-enable, input-enable, lpdr-disable

2. In `soc/t19x/target_specific.mk`, set `ENABLE_AODMIC_APP := 1` and rebuild the
application. Copy the output to `${L4T}/bootloader/spe_t194.bin`.

3. Reflash all partitions to ensure that the pinmux settings are
updated on the board:
```
sudo ./flash.sh jetson-agx-xavier-devkit mmcblk0p1
```
        Once the proper pinmux settings have been programmed, it is possible to
        update only the SPE firmware with this command:
```
sudo ./flash.sh -k spe-fw jetson-agx-xavier-devkit mmcblk0p1
```
4. If the system is placed in suspended state, BPMP will disable
the dmic5 clock at the subsequent system wake, upon which AODMIC capture
will fail. You can patch this in either of two ways:
    - On the running target execute prior to the first suspend:
```
# sudo su -c 'echo 1 > /sys/kernel/debug/bpmp/debug/clk/dmic5/state'
```
*or*
    - Patch the platform BPMP DT file (e.g. `tegra194-a02-bpmp-p2888-a04.dtb`)
  following these steps:
        - Reconstruct the device tree source file from the DTB:

                $ dtc -I dtb -O dts <BPMP DT file> -o <temp_file.dts>

        - Patch the third and fourth arguments to the 'dmic5' entry in the
        generated `temp_file.dts` file (third argument = <sample_rate*64>):

            clocks {
                lateinit {
                    dmic5 = <0x86 0x5b 1024000 0x80000>;
                };
            };

        - Recreate the DTB file from the patched DTS:

                $ dtc -I dts -O dtb <temp_file.dts> -o <BPMP DT file>

        - Flash the `bpmp-fw-dtb` partition, or perform a full flash.

5. The command below may be executed on the running target in order to trigger
system suspend:
```
# sudo systemctl suspend
```

6. After the system is placed in the suspended state,
making a loud sound near the microphone will wake it up. The threshold can
be adjusted via the `SPE_CCPLEX_WAKE_THRESHOLD` definition in `aodmic-app.c`.

@note To demonstrate system wake, the corresponding wake event must be enabled
by CCPLEX/BPMP. In this case the wake event is wake83. When the system goes to
suspend state, the BPMP UART reports enabled masks of the wake events in the
log. Verify that in this log bit 83 is set, both in the wake mask and in the
Tier2 routing mask. For example:

    WAKE_MASK[95:64] = 0x1ff200
    TIER2[95:64] = 0xff200

# Jetson AGX Orin

## Hardware Configuration

AODMIC signals are available at the 40-pin header J30 with these pin mappings:
- Pin 16 - DMIC5_DAT (CAN1_EN)
- Pin 32 - DMIC5_CLK (CAN1_STB)

Power and ground will also be needed for the PDM board.  A typical setup
might use J30 Pin 1 (3.3V) and Pin 6 (GND).

## Software Configuration

1. Pinmuxing updates are recommended via generating new dtsi files using
the pinmux spreadsheet.  For the purpose of performing a quick test, the pinmux files can be manually
updated by editing `${L4T}/bootloader/t186ref/BCT/tegra234-mb1-bct-pinmux-p3701-0000-a04.dtsi`:

            --- a/tegra234-mb1-bct-pinmux-p3701-0000-a04.dtsi
            +++ b/tegra234-mb1-bct-pinmux-p3701-0000-a04.dtsi
            @@ -142,6 +142,22 @@
                                            nvidia,enable-input = <TEGRA_PIN_DISABLE>;
                                    };

            +                       can1_stb_pbb0 {
            +                               nvidia,pins = "can1_stb_pbb0";
            +                               nvidia,function = "dmic5";
            +                               nvidia,pull = <TEGRA_PIN_PULL_NONE>;
            +                               nvidia,tristate = <TEGRA_PIN_DISABLE>;
            +                               nvidia,enable-input = <TEGRA_PIN_DISABLE>;
            +                       };
            +
            +                       can1_en_pbb1 {
            +                               nvidia,pins = "can1_en_pbb1";
            +                               nvidia,function = "dmic5";
            +                               nvidia,pull = <TEGRA_PIN_PULL_NONE>;
            +                               nvidia,tristate = <TEGRA_PIN_ENABLE>;
            +                               nvidia,enable-input = <TEGRA_PIN_ENABLE>;
            +                       };
            +
                                    soc_gpio50_pbb2 {
                                            nvidia,pins = "soc_gpio50_pbb2";
                                            nvidia,function = "tsc";
            @@ -989,22 +1005,6 @@
                                            nvidia,enable-input = <TEGRA_PIN_ENABLE>;
                                    };

            -                       can1_stb_pbb0 {
            -                               nvidia,pins = "can1_stb_pbb0";
            -                               nvidia,function = "rsvd0";
            -                               nvidia,pull = <TEGRA_PIN_PULL_UP>;
            -                               nvidia,tristate = <TEGRA_PIN_ENABLE>;
            -                               nvidia,enable-input = <TEGRA_PIN_ENABLE>;
            -                       };
            -
            -                       can1_en_pbb1 {
            -                               nvidia,pins = "can1_en_pbb1";
            -                               nvidia,function = "rsvd0";
            -                               nvidia,pull = <TEGRA_PIN_PULL_DOWN>;
            -                               nvidia,tristate = <TEGRA_PIN_ENABLE>;
            -                               nvidia,enable-input = <TEGRA_PIN_ENABLE>;
            -                       };
            -
                                    can1_err_pbb3 {
                                            nvidia,pins = "can1_err_pbb3";
                                            nvidia,function = "rsvd0";


2. Update gpio configuration as below in `${L4T}/bootloader/tegra234-mb1-bct-gpio-p3701-0000-a04.dtsi`

            --- a/tegra234-mb1-bct-gpio-p3701-0000-a04.dtsi
            +++ b/tegra234-mb1-bct-gpio-p3701-0000-a04.dtsi
            @@ -117,8 +117,6 @@
                            TEGRA234_AON_GPIO(AA, 1)
                            TEGRA234_AON_GPIO(AA, 2)
                            TEGRA234_AON_GPIO(AA, 3)
            -               TEGRA234_AON_GPIO(BB, 0)
            -               TEGRA234_AON_GPIO(BB, 1)
                            TEGRA234_AON_GPIO(BB, 3)
                            >;

3. Modify firewall setting in `${L4T}/bootloader/tegra234-mb2-bct-scr-p3701-0000-override.dts` to allow SPE to write to the AODMIC clock register.

            +++ b/bootloader/tegra234-mb2-bct-scr-p3701-0000-override.dts
            @@ -24,6 +24,11 @@
                           value = <0x18000606>;
                       };

            +          reg@2126 { /* CLK_RST_CONTROLLER_AON_SCR_DMIC5_0 */
            +              exclusion-info = <3>;
            +              value = <0x30001400>;
            +          };
            +
                       reg@5114 { /* CBB_CENTRAL_CBB_FIREWALL_PWM5_BLF, READ_CTL */

4. In `soc/t23x/target_specific.mk`, set `ENABLE_AODMIC_APP := 1` and rebuild the
application. Copy the output to `${L4T}/bootloader/spe_t234.bin`.

5. Reflash all partitions to ensure that the pinmux and firewall settings are updated on the board.

6. Volume and zero-crossings are printed in the terminal.

@note SC7 wakeup by aodmic is not yet available for this platform.  It will be available
in a future release.
