/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
#include <timeserver.h>

struct TimeServerInformation
{
	unsigned int   ticks_persecond;
	unsigned int   phaseoffset_ticks;
	unsigned char synced;
};

extern struct TimeServerInformation timeserverdata;

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <printf-isr.h>

#include <tke/sections-tke.h>       // Immune from CT_ASSERT protection
#include <tke/tke-tegra-priv.h>     // for tegra_tke_id, tegra_tke_conf, tke...
#include <tke/tke-tegra-regs.h>     // for TKE_TIMER_TMRCR_0, TKE_TIMER_TMRSR_0
#include <tke/tke-tegra.h>          // for FSP__TKE__TKE_TEGRA_H, tegra_tke_...

#include <processor/tke-tegra-hw.h>
//#include <gpio-client.h>
#include "timer-app.h"
#include <reg-access/reg-access.h>
#include <misc/bitops.h>
#include <address_map_new.h>
#include <gpio/tegra-gpio.h>




//#include <gpio-provider.h>

// gpio-aon.h has GPIO_APP* defines
#include "gpio-aon.h"


#define TIMER2_PTV 					((uint32_t)(((double) timeserverdata.ticks_persecond) / 120.0))
#define IMU_ROLL_THRESH				(((double) timeserverdata.ticks_persecond) * 0.040)
#define TKE_TIMER_TMRCR_0_PTV		0x1fffffffU
#define TKE_TIMER_TMRCR_0			0x0
//#define TKE_TIMER_TMRCR_0_PER		1<<30
//#define TKE_TIMER_TMRCR_0_EN		1<<31
// #define PPS_ON_TIME					((uint32_t) (((double) timeserverdata.ticks_persecond / 10.0)+0.5))  	//100ms
// #define PPS_OFF_TIME				((uint32_t) (9.0*((double) timeserverdata.ticks_persecond / 10.0)+0.5))	//900ms
#define TKE_TIMER_TMRATR_0		0xC
#define TKE_TIMER_TMRATR_0_ATR		0x3fffffffU



static void timer2_callback(void *data)
{
  static uint64_t rising;
  static int32_t old_phase;
  static uint32_t count;

  //Thermal pin 120 hz. 50% duty cycle
  if(count%2 == 0)
    tegra_gpio_set_value(GPIO_APP_OUT, 1);
  else
    tegra_gpio_set_value(GPIO_APP_OUT, 0);

  //printf_isr("count %lu\r\n", count);
		
  if(count == 0){
    // Count 0 = top of second
    // Change to 1 if synced
    if(timeserverdata.synced>0)
      {
	//tegra_gpio_set_value(GPIO_APP_OUT, 1);
	tegra_gpio_set_value(GPIO_APP_IN, 1);
	printf_isr("TRIGGERING %lu\r\n", TIMER2_PTV);
      } else {
      printf_isr("No sync signal received from main computer. not outputting pulses\r\n");
    }

    // Get approximate time we asserted pin high
    rising = tegra_tke_get_tsc64();
			
    count++;
  }
  else if(count >= 119){
    // Change to 0
    //tegra_gpio_set_value(GPIO_APP_OUT, 0);
    tegra_gpio_set_value(GPIO_APP_IN, 0);

    // Find the nearest second we're in for the TSC, rising set just after rising edge
    rising = rising - (rising % timeserverdata.ticks_persecond);
    // We've removed our alignment to the start of TSC second
    // Add a full second for the start of next TSC second
    rising += timeserverdata.ticks_persecond;
    // Add old offset and we're at the start of the old RTC second
    rising += old_phase;
    // rising += (timeserverdata.ticks_persecond - timeserverdata.phaseoffset_ticks);

    uint32_t offset_inv = timeserverdata.ticks_persecond - timeserverdata.phaseoffset_ticks;
    int32_t phase_delta = (int32_t)offset_inv - old_phase;
    uint32_t phase_mag = abs(phase_delta);
    if(phase_delta > (int32_t)IMU_ROLL_THRESH){
      phase_delta = IMU_ROLL_THRESH;
      rising+=IMU_ROLL_THRESH;
    }
    else if(phase_mag > (int32_t)IMU_ROLL_THRESH) {
      phase_delta = 0L - (int32_t)IMU_ROLL_THRESH;
      rising-=IMU_ROLL_THRESH;
    }
    else if(phase_delta > 0L){
      rising+=phase_delta;
    }
    else if(phase_delta < 0L){
      rising-=phase_mag;
    }
    else{
      // Do nothing, safe to use these values
    }

    // Get current time
    uint64_t current = tegra_tke_get_tsc64();
    uint32_t rising32 = ((uint32_t)rising & TKE_TIMER_TMRATR_0_ATR);
    // Stupid ATR uses 30 bits [29:0] and treats this as signed :(
    uint32_t current_lo = (uint32_t) current;
    int32_t rising_signed = (int32_t)(rising32 << 2);
    int32_t current_lo_signed = (int32_t)(current_lo <<2);
    if(rising_signed <= current_lo_signed){
      // Wrap around, do nothing
    }
    else {
      // Set PTV to updated ticks_persecond
      tegra_tke_timer_writel_non_static(&tegra_tke_id_timer2,
			     ((TIMER2_PTV - 1) & TKE_TIMER_TMRCR_0_PTV) | TKE_TIMER_TMRCR_0_EN | TKE_TIMER_TMRCR_0_PER,
			     TKE_TIMER_TMRCR_0);

      //iowrite32_offset(((TIMER2_PTV - 1) & TKE_TIMER_TMRCR_0_PTV) | TKE_TIMER_TMRCR_0_EN | TKE_TIMER_TMRCR_0_PER,
      //		       tegra_tke_id_timer2.conf.base_addr,
      //		       TKE_TIMER_TMRCR_0);
      
      // Write ATR to keep phase alignment after updating PTV
      tegra_tke_timer_writel_non_static(&tegra_tke_id_timer2, rising32, TKE_TIMER_TMRATR_0);
      // Save old phase only when written
      old_phase += phase_delta;
    }	
			
			
    count = 0;
  }
  else if(count > 11){
    // Not at the top of the second
    // Change to 0
    //tegra_gpio_set_value(GPIO_APP_OUT, 0);
    tegra_gpio_set_value(GPIO_APP_IN, 0);
    count++;
  }
  else{
    count++;
  }
	

}

void timer_app_init(void)
{
  for(int i = 0; i < 100; i ++)
    printf_isr("TIMER APP INIT\n");

	int val = tegra_gpio_direction_out(GPIO_APP_OUT, 0);
	if (val) {
		return;
	}
	val = tegra_gpio_direction_out(GPIO_APP_IN, 0);
	if (val) {
		return;
	}
	//Set PTV to max value when using ATR, from Nvidia forums
	//todo - why must periodic be set to true for atr target to function
	tegra_tke_set_up_timer(&tegra_tke_id_timer2, TEGRA_TKE_CLK_SRC_TSC_BIT0,
			       true, TIMER2_PTV, timer2_callback, 0);
	uint64_t temp_tsc = tegra_tke_get_tsc64();
	temp_tsc = temp_tsc - (temp_tsc % timeserverdata.ticks_persecond);
	// uint32_t tsc_lo = (uint32_t) temp_tsc;
	// uint32_t tsc_align = temp_tsc % timeserverdata.ticks_persecond;
	// (tsc_lo - tsc_align) = start of tsc second, add phaseoffset_ticks, plus one second since that time sync for this "second frame" may have past
	// uint32_t offset_ticks = timeserverdata.phaseoffset_ticks + get_AON_offset();		
	//get_AON_offset() here causes this not to flash lol
	uint32_t offset_inv = timeserverdata.ticks_persecond - timeserverdata.phaseoffset_ticks;
	uint32_t tsc_tmr_atr = (uint32_t)temp_tsc + offset_inv + timeserverdata.ticks_persecond;
	tsc_tmr_atr = tsc_tmr_atr & TKE_TIMER_TMRATR_0_ATR;
	// if(tsc_tmr_atr < TIMER2_PTV){
	// 	// we wrapped around, this will cause an immmediate interrupt
	// 	// Let first loop of callback hamdle sync
	// } else {
	// 	// Not going to wrap around, should be fine
		tegra_tke_timer_writel_non_static(&tegra_tke_id_timer2, (tsc_tmr_atr & TKE_TIMER_TMRATR_0_ATR), TKE_TIMER_TMRATR_0);
	// }	
	
}
//*/


/*
#include <timeserver.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <printf-isr.h>

#include <tke/sections-tke.h>       // Immune from CT_ASSERT protection
#include <tke/tke-tegra-priv.h>     // for tegra_tke_id, tegra_tke_conf, tke...
#include <tke/tke-tegra-regs.h>     // for TKE_TIMER_TMRCR_0, TKE_TIMER_TMRSR_0
#include <tke/tke-tegra.h>          // for FSP__TKE__TKE_TEGRA_H, tegra_tke_...

#include <processor/tke-tegra-hw.h>
//#include <gpio-client.h>
#include "gpio-app.h"
#include <reg-access/reg-access.h>
#include <misc/bitops.h>
#include <address_map_new.h>
#include <gpio/tegra-gpio.h>




//#include <gpio-provider.h>

// gpio-aon.h has GPIO_APP* defines
#include "gpio-aon.h"


#define TIMER2_PTV 					((uint32_t)(((double) timeserverdata.ticks_persecond) / 120.0))
#define IMU_ROLL_THRESH				(((double) timeserverdata.ticks_persecond) * 0.040)
#define TKE_TIMER_TMRCR_0_PTV		0x1fffffffU
#define TKE_TIMER_TMRCR_0			0x0
//#define TKE_TIMER_TMRCR_0_PER		1<<30
//#define TKE_TIMER_TMRCR_0_EN		1<<31
// #define PPS_ON_TIME					((uint32_t) (((double) timeserverdata.ticks_persecond / 10.0)+0.5))  	//100ms
// #define PPS_OFF_TIME				((uint32_t) (9.0*((double) timeserverdata.ticks_persecond / 10.0)+0.5))	//900ms
#define TKE_TIMER_TMRATR_0		0xC
#define TKE_TIMER_TMRATR_0_ATR		0x3fffffffU



static void timer2_callback(void *data)
{
  //static int count = 0;
  //count++;
  //if(count % 120 == 0 && timeserverdata.synced)
  //  printf_isr("time server data: %d %d\r\n",
  //	       timeserverdata.ticks_persecond,
  //	       timeserverdata.phaseoffset_ticks);
  
  static uint64_t rising;
  static int32_t old_phase;
  static uint32_t count;

  //Thermal pin 120 hz. 50% duty cycle
  if(count%2 == 0)
    tegra_gpio_set_value(GPIO_APP_OUT, 1);
  else
    tegra_gpio_set_value(GPIO_APP_OUT, 0);

  //printf_isr("count %lu\r\n", count);
		
  if(count == 0){
    // Count 0 = top of second
    // Change to 1 if synced
    if(timeserverdata.synced>0)
      {
	//tegra_gpio_set_value(GPIO_APP_OUT, 1);
	tegra_gpio_set_value(GPIO_APP_IN, 1);
	printf_isr("TRIGGERING %lu\r\n", TIMER2_PTV);
      } else {
      printf_isr("No sync signal received from main computer. not outputting pulses\r\n");
    }

    // Get approximate time we asserted pin high
    rising = tegra_tke_get_tsc64();
			
    count++;
  }
  else if(count >= 119){
    // Change to 0
    //tegra_gpio_set_value(GPIO_APP_OUT, 0);
    tegra_gpio_set_value(GPIO_APP_IN, 0);

    // Find the nearest second we're in for the TSC, rising set just after rising edge
    rising = rising - (rising % timeserverdata.ticks_persecond);
    // We've removed our alignment to the start of TSC second
    // Add a full second for the start of next TSC second
    rising += timeserverdata.ticks_persecond;
    // Add old offset and we're at the start of the old RTC second
    rising += old_phase;
    // rising += (timeserverdata.ticks_persecond - timeserverdata.phaseoffset_ticks);

    uint32_t offset_inv = timeserverdata.ticks_persecond - timeserverdata.phaseoffset_ticks;
    int32_t phase_delta = (int32_t)offset_inv - old_phase;
    uint32_t phase_mag = abs(phase_delta);
    if(phase_delta > (int32_t)IMU_ROLL_THRESH){
      phase_delta = IMU_ROLL_THRESH;
      rising+=IMU_ROLL_THRESH;
    }
    else if(phase_mag > (int32_t)IMU_ROLL_THRESH) {
      phase_delta = 0L - (int32_t)IMU_ROLL_THRESH;
      rising-=IMU_ROLL_THRESH;
    }
    else if(phase_delta > 0L){
      rising+=phase_delta;
    }
    else if(phase_delta < 0L){
      rising-=phase_mag;
    }
    else{
      // Do nothing, safe to use these values
    }

    // Get current time
    uint64_t current = tegra_tke_get_tsc64();
    uint32_t rising32 = ((uint32_t)rising & TKE_TIMER_TMRATR_0_ATR);
    // Stupid ATR uses 30 bits [29:0] and treats this as signed :(
    uint32_t current_lo = (uint32_t) current;
    int32_t rising_signed = (int32_t)(rising32 << 2);
    int32_t current_lo_signed = (int32_t)(current_lo <<2);
    if(rising_signed <= current_lo_signed){
      // Wrap around, do nothing
    }
    else {
      // Set PTV to updated ticks_persecond
      tegra_tke_timer_writel_non_static(&tegra_tke_id_timer2,
			     ((TIMER2_PTV - 1) & TKE_TIMER_TMRCR_0_PTV) | TKE_TIMER_TMRCR_0_EN | TKE_TIMER_TMRCR_0_PER,
			     TKE_TIMER_TMRCR_0);

      //iowrite32_offset(((TIMER2_PTV - 1) & TKE_TIMER_TMRCR_0_PTV) | TKE_TIMER_TMRCR_0_EN | TKE_TIMER_TMRCR_0_PER,
      //		       tegra_tke_id_timer2.conf.base_addr,
      //		       TKE_TIMER_TMRCR_0);
      
      // Write ATR to keep phase alignment after updating PTV
      tegra_tke_timer_writel_non_static(&tegra_tke_id_timer2, rising32, TKE_TIMER_TMRATR_0);
      // Save old phase only when written
      old_phase += phase_delta;
    }	
			
			
    count = 0;
  }
  else if(count > 11){
    if(count == 15)
      printf_isr("setting low\r\n");
    // Not at the top of the second
    // Change to 0
    //tegra_gpio_set_value(GPIO_APP_OUT, 0);
    tegra_gpio_set_value(GPIO_APP_IN, 0);
    count++;
  }
  else{
    count++;
  }
}

void gpio_app_init(void)
{
  for(int i = 0; i < 100; i ++)
    printf_isr("TIMER APP INIT\r\n");

	int val = tegra_gpio_direction_out(GPIO_APP_OUT, 0);
	printf_isr("GPIO_APP_OUT val: %d\r\n", val);
	if (val) {
		return;
	}
	val = tegra_gpio_direction_out(GPIO_APP_IN, 0);
	printf_isr("GPIO_APP_IN val: %d\r\n", val);
	if (val) {
		return;
	}
	//Set PTV to max value when using ATR, from Nvidia forums
	//todo - why must periodic be set to true for atr target to function
	tegra_tke_set_up_timer(&tegra_tke_id_timer2, TEGRA_TKE_CLK_SRC_TSC_BIT0,
			       true, TIMER2_PTV, timer2_callback, 0);
	uint64_t temp_tsc = tegra_tke_get_tsc64();
	temp_tsc = temp_tsc - (temp_tsc % timeserverdata.ticks_persecond);
	// uint32_t tsc_lo = (uint32_t) temp_tsc;
	// uint32_t tsc_align = temp_tsc % timeserverdata.ticks_persecond;
	// (tsc_lo - tsc_align) = start of tsc second, add phaseoffset_ticks, plus one second since that time sync for this "second frame" may have past
	// uint32_t offset_ticks = timeserverdata.phaseoffset_ticks + get_AON_offset();		
	//get_AON_offset() here causes this not to flash lol
	uint32_t offset_inv = timeserverdata.ticks_persecond - timeserverdata.phaseoffset_ticks;
	uint32_t tsc_tmr_atr = (uint32_t)temp_tsc + offset_inv + timeserverdata.ticks_persecond;
	tsc_tmr_atr = tsc_tmr_atr & TKE_TIMER_TMRATR_0_ATR;
	// if(tsc_tmr_atr < TIMER2_PTV){
	// 	// we wrapped around, this will cause an immmediate interrupt
	// 	// Let first loop of callback hamdle sync
	// } else {
	// 	// Not going to wrap around, should be fine
	tegra_tke_timer_writel_non_static(&tegra_tke_id_timer2, (tsc_tmr_atr & TKE_TIMER_TMRATR_0_ATR), TKE_TIMER_TMRATR_0);
	// }	
	
}
//*/

/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <timeserver.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <printf-isr.h>
#include "gpio-app.h"


#include <tke/sections-tke.h>       // Immune from CT_ASSERT protection
#include <tke/tke-tegra-priv.h>     // for tegra_tke_id, tegra_tke_conf, tke...
#include <tke/tke-tegra-regs.h>     // for TKE_TIMER_TMRCR_0, TKE_TIMER_TMRSR_0
#include <tke/tke-tegra.h>          // for FSP__TKE__TKE_TEGRA_H, tegra_tke_...

#include <processor/tke-tegra-hw.h>
#include <reg-access/reg-access.h>
#include <misc/bitops.h>
#include <address_map_new.h>
#include <gpio/tegra-gpio.h>

// gpio-aon.h has GPIO_APP* defines
#include "gpio-aon.h"

#define TIMER2_PTV 1000000
#define TKE_TIMER_TMRCR_0_PTV		0x1fffffffU
#define TKE_TIMER_TMRCR_0		0x0

#define CODE_OFFSET_TICKS   125


//Let's run the timer at 120 Hz for now

static void timer2_callback(void *data)
{
	uint64_t tsc_aligned = (tegra_tke_get_tsc64() + timeserverdata.phaseoffset_ticks) % timeserverdata.ticks_persecond;
	//Past here all numbers should be small wrapping around every second.
	uint32_t increment = timeserverdata.ticks_persecond/120;//120Hz for now
	uint32_t count = tsc_aligned/increment;
	//This will have a little bit of delay for now but should be ok and is easiest for now.
	if(timeserverdata.synced>0)
	{
		//PPS PIN
		if(count <= 11) {
		  if(count == 0)
		    //printf_isr("ticks_persecond: %u, phaseoffset_ticks: %u\r\n", timeserverdata.ticks_persecond, timeserverdata.phaseoffset_ticks);
			tegra_gpio_set_value(GPIO_APP_IN, 1);
			//Let's not turn of the timer. Once it is synced once it should be good for a long time. 
			//--timeserverdata.synced;
        } else if(count > 11) {
			tegra_gpio_set_value(GPIO_APP_IN, 0);
        }
		//Thermal pin 120 hz. 50% duty cycle
		if(count%2 == 0)
			tegra_gpio_set_value(GPIO_APP_OUT, 1);
		else
			tegra_gpio_set_value(GPIO_APP_OUT, 0);
	}
	//else
	//{
	//	 if(count == 0) 
	//	 	printf_isr("No sync signal received from main computer. not outputting pulses\r\n");
	//}
	//Increment in "increments." Subract any misalignment for the next timer deadline.
	uint32_t newcountergoal = increment - ( tsc_aligned - increment * count);
	// Don't setup a completely new timer. Just change the target count. Not sure why we need -1 but that was in the original setup code.
	uint32_t tmrcr = ((newcountergoal - 1) & TKE_TIMER_TMRCR_0_PTV) | TKE_TIMER_TMRCR_0_EN | TKE_TIMER_TMRCR_0_PER;
	tegra_tke_timer_writel_non_static(&tegra_tke_id_timer2, tmrcr, TKE_TIMER_TMRCR_0);
}

void gpio_app_init(void)
{
	

	int val = tegra_gpio_direction_out(GPIO_APP_OUT, 0);
	if (val) {
		return;
	}
	val = tegra_gpio_direction_out(GPIO_APP_IN, 0);
	if (val) {
		return;
	}

	tegra_tke_set_up_timer(&tegra_tke_id_timer2, TEGRA_TKE_CLK_SRC_TSC_BIT0,
			       true, TIMER2_PTV, timer2_callback, 0);

	

}
