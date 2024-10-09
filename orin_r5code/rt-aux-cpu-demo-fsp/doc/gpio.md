GPIO Application (app/gpio-app.c)
================================

@tableofcontents

GPIO application demonstrates how to access/manipulate Always On (AON) GPIOs from
the SPE/AON processor.

Pinmux changes are necessary in order for this demo to function correctly.
These changes relate to the MB1 bootloader.  The recommended method for changing
pinmux settings is to download the device-specific pinmux spreadsheet and
generate an updated set of pinmux configuration files.  In the steps below some
updates to various configuration files will be shown.  While the preferred method
is to leverage the pinmux spreadsheet, manually modifying the files will also
work for the purpose of quickly demonstrating GPIO functionality.

# AGX Xavier

## Hardware Configuration
The following pins from the 40 pin header (J30) should be connected together:
   * Pin 16 (PBB0, output)
   * Pin 32 (PBB1, input)

## Software Configuration
In order to access a AON GPIO from the Cortex-R5 SPE/AON for AGX Xavier, the
GPIO interrupt map and pinmux settings need to be updated as described in below
steps.

1. In `soc/t19x/target_specific.mk`, set `ENABLE_GPIO_APP := 1` and rebuild the
application. Copy the output to `${L4T}/bootloader/spe_t194.bin`.

2. Update gpio interrupt mapping as below in
`${L4T}/bootloader/t186ref/BCT/tegra194-mb1-bct-gpioint-p2888-0000-p2822-0000.cfg`:

         --- a/tegra194-mb1-bct-gpioint-p2888-0000-p2822-0000.cfg
         +++ b/tegra194-mb1-bct-gpioint-p2888-0000-p2822-0000.cfg
         @@ -17,7 +17,7 @@ gpio-intmap.port.AA.pin.5 = 4; # GPIO AA5 to INT4
         gpio-intmap.port.AA.pin.6 = 4; # GPIO AA6 to INT4
         gpio-intmap.port.AA.pin.7 = 4; # GPIO AA7 to INT4
         gpio-intmap.port.BB.pin.0 = 4; # GPIO BB0 to INT4
         -gpio-intmap.port.BB.pin.1 = 4; # GPIO BB1 to INT4
         +gpio-intmap.port.BB.pin.1 = 2; # GPIO BB1 to INT2
         gpio-intmap.port.BB.pin.2 = 4; # GPIO BB2 to INT4
         gpio-intmap.port.BB.pin.3 = 4; # GPIO BB3 to INT4
         gpio-intmap.port.CC.pin.0 = 4; # GPIO CC0 to INT4

3. Update pinmux configuration as below in the file
`${L4T}/bootloader/t186ref/BCT/tegra19x-mb1-pinmux-p2888-0000-a04-p2822-0000-b01.cfg`:

         --- a/tegra19x-mb1-pinmux-p2888-0000-a04-p2822-0000-b01.cfg
         +++ b/tegra19x-mb1-pinmux-p2888-0000-a04-p2822-0000-b01.cfg
         @@ -67,8 +67,6 @@ pinmux.0x0c2f16c0 = 0x00000001; # CONFIG AA6
         pinmux.0x0c303030 = 0x00000000; # GPIO can0_wake_paa6
         pinmux.0x0c2f16e0 = 0x00000001; # CONFIG AA7
         pinmux.0x0c303038 = 0x00000000; # GPIO can0_err_paa7
         -pinmux.0x0c2f1800 = 0x00000001; # CONFIG BB0
         -pinmux.0x0c303040 = 0x00000000; # GPIO can1_stb_pbb0
         pinmux.0x0c2f1820 = 0x00000001; # CONFIG BB1
         pinmux.0x0c303048 = 0x00000000; # GPIO can1_en_pbb1
         pinmux.0x0c2f1840 = 0x00000001; # CONFIG BB2
         @@ -150,6 +148,10 @@ pinmux.0x0c2f16a0 = 0x00000003; # CONFIG AA5
         pinmux.0x0c2f16ac = 0x00000000; # CONTROL AA5
         pinmux.0x0c2f16b0 = 0x00000000; # OUTPUT AA5
         pinmux.0x0c303028 = 0x00000000; # GPIO can0_en_paa5
         +pinmux.0x0c2f1800 = 0x00000003; # CONFIG BB0
         +pinmux.0x0c2f180c = 0x00000000; # CONTROL BB0
         +pinmux.0x0c2f1810 = 0x00000000; # OUTPUT BB0
         +pinmux.0x0c303040 = 0x00000000; # GPIO can1_stb_pbb0
         pinmux.0x02212c80 = 0x00000003; # CONFIG Q4
         pinmux.0x02212c8c = 0x00000000; # CONTROL Q4
         pinmux.0x02212c90 = 0x00000000; # OUTPUT Q4
         @@ -360,7 +362,7 @@ pinmux.0x0c303020 = 0x0000c000; # can0_stb_paa4: rsvd0, tristate-disable, input-
         pinmux.0x0c303028 = 0x0000c000; # can0_en_paa5: rsvd0, tristate-disable, input-disable
         pinmux.0x0c303030 = 0x0000c058; # can0_wake_paa6: rsvd0, pull-up, tristate-enable, input-enable
         pinmux.0x0c303038 = 0x0000c048; # can0_err_paa7: rsvd0, pull-up, tristate-disable, input-enable
         -pinmux.0x0c303040 = 0x0000c054; # can1_stb_pbb0: rsvd0, pull-down, tristate-enable, input-enable
         +pinmux.0x0c303040 = 0x0000c000; # can1_stb_pbb0: rsvd0, tristate-disable, input-disable
         pinmux.0x0c303048 = 0x0000c054; # can1_en_pbb1: rsvd0, pull-down, tristate-enable, input-enable
         pinmux.0x0c303050 = 0x0000c058; # can1_wake_pbb2: rsvd0, pull-up, tristate-enable, input-enable
         pinmux.0x02430078 = 0x00000000; # soc_gpio05_pp5: rsvd0, tristate-disable, input-disable, lpdr-disable

4. Reflash all partitions to ensure that the gpio interrupt mappings
   and pinmux settings are updated on the board.

5. The demo should produce the following output on the terminal:

         gpio_app_task - Setting GPIO_APP_OUT to 1 - IRQ should trigger
         can_gpio_irq_handler - gpio irq triggered - setting GPIO_APP_OUT to 0

# AGX Orin

## Hardware Configuration
The following pins from the 40 pin header (J30) should be connected together:
   * Pin 16 (PBB1, output)
   * Pin 32 (PBB0, input)

## Software Configuration
In order to access a AON GPIO from the Cortex-R5 SPE/AON for AGX Orin, the
GPIO interrupt map and pinmux settings need to be updated as described in below
steps.

1. In `soc/t23x/target_specific.mk`, set `ENABLE_GPIO_APP := 1` and rebuild the
application. Copy the output to `${L4T}/bootloader/spe_t234.bin`.

2. Update gpio interrupt mapping as below in `${L4T}/bootloader/t186ref/BCT/tegra234-mb1-bct-gpioint-p3701-0000.dts`:

         --- a/t186ref/BCT/tegra234-mb1-bct-gpioint-p3701-0000.dts
         +++ b/t186ref/BCT/tegra234-mb1-bct-gpioint-p3701-0000.dts
         @@ -234,7 +234,7 @@
                     pin-7-int-line = <4>; // GPIO AA7 to INT0
                  };
                  port@BB {
         -            pin-0-int-line = <4>; // GPIO BB0 to INT0
         +            pin-0-int-line = <2>; // GPIO BB0 to INT2
                     pin-1-int-line = <4>; // GPIO BB1 to INT0
                     pin-2-int-line = <4>; // GPIO BB2 to INT0
                     pin-3-int-line = <4>; // GPIO BB3 to INT0

3. Update gpio configuration as below in `${L4T}/bootloader/tegra234-mb1-bct-gpio-p3701-0000-a04.dtsi`:

         --- a/tegra234-mb1-bct-gpio-p3701-0000-a04.dtsi
         +++ b/tegra234-mb1-bct-gpio-p3701-0000-a04.dtsi
         @@ -118,12 +118,12 @@
                                       TEGRA234_AON_GPIO(AA, 2)
                                       TEGRA234_AON_GPIO(AA, 3)
                                       TEGRA234_AON_GPIO(BB, 0)
         -                              TEGRA234_AON_GPIO(BB, 1)
                                       TEGRA234_AON_GPIO(BB, 3)
                                       >;
                                 gpio-output-low = <
                                       TEGRA234_AON_GPIO(CC, 2)
                                       TEGRA234_AON_GPIO(CC, 3)
         +                              TEGRA234_AON_GPIO(BB, 1)
                                       >;
                                 gpio-output-high = <
                                       >;

4. Update pinmux configuration as below in the file `${L4T}/bootloader/t186ref/BCT/tegra234-mb1-bct-pinmux-p3701-0000-a04.dtsi`:

         --- a/t186ref/BCT/tegra234-mb1-bct-pinmux-p3701-0000-a04.dtsi
         +++ b/t186ref/BCT/tegra234-mb1-bct-pinmux-p3701-0000-a04.dtsi
         @@ -1000,9 +1000,9 @@
                                 can1_en_pbb1 {
                                       nvidia,pins = "can1_en_pbb1";
                                       nvidia,function = "rsvd0";
         -                               nvidia,pull = <TEGRA_PIN_PULL_DOWN>;
         -                               nvidia,tristate = <TEGRA_PIN_ENABLE>;
         -                               nvidia,enable-input = <TEGRA_PIN_ENABLE>;
         +                               nvidia,pull = <TEGRA_PIN_PULL_NONE>;
         +                               nvidia,tristate = <TEGRA_PIN_DISABLE>;
         +                               nvidia,enable-input = <TEGRA_PIN_DISABLE>;
                                 };
5. Reflash all partitions to ensure that the gpio interrupt mappings
   and pinmux settings are updated on the board.

6. The demo should produce the following output on the terminal:

         gpio_app_task - Setting GPIO_APP_OUT to 1 - IRQ should trigger
         can_gpio_irq_handler - gpio irq triggered - setting GPIO_APP_OUT to 0

