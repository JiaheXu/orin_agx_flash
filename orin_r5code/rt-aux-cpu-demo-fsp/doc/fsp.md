FSP (Firmware Support Package)
==================================

@tableofcontents

The FSP layer provides common set of on chip and bus peripheral drivers such as
but not limited to gpio, i2c, spi, hsp, ast, aodmic for the various auxiliary
processors present in the Jetson. This guide pertains to SPE or AON auxiliary
processor. The FSP also provides operating system abstraction
layer (OSA) to support different type and flavors of the operating systems,
here we are only concerned about FreeRTOS. Since FSP houses common set of
drivers across multiple of processors, it implements CPU abstraction layer (CPL)
which abstracts away CPU primitives such as but not limited to cache management,
barriers, memory managements. More details will be followed in later sections.

# OSA Layer #
The OSA provides the abstraction to underlying Real Time Operating System (RTOS)
(FreeRTOS in this case). This helps in porting the firmware drivers/applications
into different RTOS with little to no changes at the firmware drivers/application
level. The following files that implement the FreeRTOS OSA can be found in the
fsp/source/include/osa/freertosv10/osa directory:
- Semaphore
  - follow `osa-semaphore.h` for API details.
- Mutex
  - follow `osa-mutex.h` for API details.
- Event Group
  - follow `osa-event-group.h` for API details.
- Queue
  - follow `osa-queue.h` for API details.
- Threads and real time scheduling
  - follow `osa-task.h` for API details.
- Timer
  - follow `osa-timer.h` for API details.

# CPL Layer #
The FSP provides an abstraction to CPU configuration via CPL library.
This library includes:
- Cache configuration and control for enable/disable/flush/invalidate
the caches
  - follow `fsp/source/include/cpu/arm/common/cpu/cache.h` for API details.
- Register access
  - follow `fsp/source/include/cpu/arm/armv7/cortex-r5/reg-access/reg-access.h`
  for API details.
- CPU synchronization barriers
  - follow `fsp/source/include/cpu/arm//common/cpu/barriers.h` for API details.
- Vector Interrupt Control (VIC) for configuring, enabling and disabling
interrupt and its handlers
  - follow `fsp/source/include/cpu/arm/common/cpu/arm-vic.h` for API details.
- Chip-ID to get SoC version and SKUs
  - follow `fsp/source/include/chipid/chip-id.h` for API details.

# Drivers #
This layer implements common set up peripherals and on chip hardware module
drivers. The drivers use OSA and CPL layers to implement CPU or OS specific
primitives. For brevity this guide will not detail various drivers' APIs as it
can be accessed from their respective header files. For example, API details to
use GPIO driver are found at `fsp/source/include/gpio/tegra-gpio.h`.
Additionally, some drivers use port layer implemented at
`fsp/source/soc/<soc number>/port/aon` to abstract away processor specific
implementation. Each driver or peripheral has associated ID structure
implemented at `fsp/source/soc/<soc number>/ids/aon` which details
peripheral instance related information like base address, irq.
