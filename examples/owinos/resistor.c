/*
 * Copyright (c) 2016, Zolertia - http://www.zolertia.com
 * Copyright (c) 2020, Oppila Microsystems - http://www.oppila.in
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup owinos-examples
 * @{
 *
 * \defgroup owinos-ldr-sensor-test LDR sensor test
 *
 * Demonstrates the operation of the analog LDR
 * @{
 *
 * \file
 *  LDR sensor example using the ADC sensors wrapper
 *
 * \author
 *         Antonio Lignan <alinan@zolertia.com>
 */
/*---------------------------------------------------------------------------*/
#include <stdio.h>
#include "contiki.h"
#include "dev/owinos-adc.h"
/*---------------------------------------------------------------------------*/
#define ADC_PIN    	     4 
#define SENSOR_READ_INTERVAL (CLOCK_SECOND / 4)
/*---------------------------------------------------------------------------*/
PROCESS(resistor_process, "resistor test");
AUTOSTART_PROCESSES(&resistor_process);
/*---------------------------------------------------------------------------*/
static struct etimer et;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(resistor_process, ev, data)
{
  PROCESS_BEGIN();

  uint16_t value;
 

  /* pin number should be used here Ex: if PA5 then use 5. in omote the onboard LDR sensor is connected to PA5 */
  adc_sensors.configure(RESISTOR_SENSOR,4 );

  /* poll the sensor periodically */

  while(1) {
    etimer_set(&et, SENSOR_READ_INTERVAL);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    value = adc_sensors.value(RESISTOR_SENSOR);

    if(value!= ADC_WRAPPER_ERROR) {
     printf("light Value = %u\t\n", value);
    // pwm_enable(16000,value,0,1,0);
      //  pwm_start(1,0,3,4);
     if(value<=750)
    leds_on(LEDS_GREEN);
   else
	 leds_off(LEDS_GREEN);
    }
    
   
    }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
	 * @}
 */

