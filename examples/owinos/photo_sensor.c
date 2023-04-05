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
#include "cpu.h"
#include "dev/leds.h"
#include "dev/watchdog.h"
#include "dev/sys-ctrl.h"
#include "pwm.h"
#include "lpm.h"
#include "dev/ioc.h"
#include <stdint.h>
#include "sys/clock.h"
/*--------------------------------------------------------------------------*/
#define DEBUG  1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/*---------------------------------------------------------------------------*/
typedef struct {
  uint8_t timer;
  uint8_t ab;
  uint8_t port;
  uint8_t pin;
  uint8_t duty;
  uint8_t off_state;
  uint32_t freq;
} pwm_config_t;


#define ADC_PIN    	      5          
#define MAX_PWM               1
#define SENSOR_READ_INTERVAL (CLOCK_SECOND / 4)
static const pwm_config_t pwm_num[MAX_PWM] = {
  {
    .timer = PWM_TIMER_1,
    .ab = PWM_TIMER_A,
    .port = GPIO_A_NUM,
    .pin = 4,
    .duty = 15,
    .freq = 100,
    .off_state = PWM_OFF_WHEN_STOP,
  }
};
static uint8_t pwm_en[MAX_PWM];
#if DEBUG
static const char *
gpt_name(uint8_t timer)
{
  switch(timer) {
  case PWM_TIMER_0:
    return "PWM TIMER 0";
  case PWM_TIMER_1:
    return "PWM TIMER 1";
  case PWM_TIMER_2:
    return "PWM TIMER 2";
  case PWM_TIMER_3:
    return "PWM TIMER 3";
  default:
    return "Unknown";
  }
}
#endif

/*---------------------------------------------------------------------------*/
PROCESS(PHOTO_sensor_process, "PHOTO test");
AUTOSTART_PROCESSES(&PHOTO_sensor_process);
/*---------------------------------------------------------------------------*/
static struct etimer et;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(PHOTO_sensor_process, ev, data)
{
	PROCESS_BEGIN();
	uint8_t j;
	memset(pwm_en, 0, MAX_PWM);
	/* pin number should be used here Ex: if PA5 then use 5. in omote the onboard LDR sensor is connected to PA5 */
	adc_sensors.configure(ONBOARD_LDR_SENSOR,5);
	/* poll the sensor periodically */

	while(1) {
		etimer_set(&et, SENSOR_READ_INTERVAL);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

		j = adc_sensors.value(ONBOARD_LDR_SENSOR);

                                       
			printf("PHOTO Value = %u\t\n",j);                                                                                       		         	if(pwm_enable(160000,j,0,pwm_num[0].timer,pwm_num[0].ab)==PWM_SUCCESS)
				{
					pwm_en[0]=1;
				
				}
				if((pwm_en[0]) && (pwm_start(pwm_num[0].timer, pwm_num[0].ab, pwm_num[0].port, pwm_num[0].pin) != PWM_SUCCESS))
					PRINTF(" successfully  started \n");
				
		PROCESS_END();
	
}
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */

