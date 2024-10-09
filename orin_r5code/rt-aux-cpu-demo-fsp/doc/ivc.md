IVC
===

Inter VM Communication provides interfaces to communicate between chip processors through memory channel.
The only channel supported in the current distribution is the echo channel.
The echo channel demonstrates:
- How to establish the IVC channel.
- How IVC communication works in the kernel.
- How IVC communication works in the Freertos code.

# AGX Xavier
1. The current IVC channel are described in the device tree repository. You must run source_sync.sh to get the dtsi file.
After running source_sync.sh, the file is located at Linux_for_Tegra/sources/hardware/nvidia/soc/t19x/kernel-dts/tegra194-soc/tegra194-aon.dtsi

2. To add/remove or enable/disable the channels in linux kernel, you must update the
device tree entries in the above dtsi file. The following code shows how to enable the IVC echo channel:
```
                };
        };

+       aon_echo {
+               compatible = "nvidia,tegra186-aon-ivc-echo";
+               mboxes = <&aon 0>;
+               status = "okay";
+       };
+
        aondbg {
                compatible = "nvidia,tegra186-aondbg";
                mboxes = <&aon 1>;
```

3. Compile the device tree and copy the dtb file to Linux_for_Tegra/dtb/ directory.
4. Rebuild the device tree and reflash the board by referencing the instructions in the **Building the Kernel** section of the Linux Developer Guide.


# AGX Orin
1. The current IVC channel are described in the device tree repository. You must run source_sync.sh to get the dtsi file.
After running source_sync.sh, the file is located at Linux_for_Tegra/sources/hardware/nvidia/soc/t23x/kernel-dts/tegra234-soc/tegra234-aon.dtsi

2. To add/remove or enable/disable the channels in linux kernel, you must update the
device tree entries in the above dtsi file. The following code shows how to enable the IVC echo channel:
```
        aon_echo {
                compatible = "nvidia,tegra186-aon-ivc-echo";
                mboxes = <&aon 1>;
-               status = "disabled";
+               status = "okay";
        };

```

3. Compile the device tree and copy the dtb file to Linux_for_Tegra/dtb/ directory.
4. Rebuild the device tree and reflash the board by referencing the instructions in the **Building the Kernel** section of the Linux Developer Guide.

# How to test
From the Linux kernel side, the `/sys/devices/platform/aon_echo/data_channel`
node is used to communicate with AON after enabling echo channel as mentioned above.
Messages sent to AON through this channel are echoed back to CCPLEX.
```
ubuntu@jetson:~$ sudo su -c 'echo tegra > /sys/devices/platform/aon_echo/data_channel'
ubuntu@jetson:~$ cat /sys/devices/platform/aon_echo/data_channel
tegra
ubuntu@jetson:~$
```

# Sources
The echo channel ivc code in the current firmware resides at:

    rt-aux-cpu-demo-fsp/app/ivc-echo-task.c

And the channel descriptions are at:

    rt-aux-cpu-demo-fsp/platform/ivc-channel-ids.c
