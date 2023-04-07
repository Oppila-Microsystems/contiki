/*
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
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or prowinos products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup owinos-adc-sensors
 * \file
 * Generic driver for the Owinos ADC wrapper for analogue sensors
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "sys/clock.h"
#include "dev/ioc.h"
#include "dev/gpio.h"
#include "dev/adc.h"
#include "owinos-adc.h"
#include "owinos-sensors.h"
#include <stdio.h>
#include <stdint.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
typedef struct {
	int type;
	uint8_t pin_mask;
	uint8_t vdd3;
} adc_info_t;

typedef struct {
	uint8_t sensors_num;
	uint8_t sensors_ports;
	adc_info_t sensor[ADC_SENSORS_MAX];
} adc_wrapper_t;

static adc_wrapper_t sensors;
/*---------------------------------------------------------------------------*/
static uint8_t decimation_rate;
static uint8_t enabled_channels;
/*---------------------------------------------------------------------------*/
static int
fix_decimation_rate(uint8_t rate)
{
	switch(rate) {
		case SOC_ADC_ADCCON_DIV_64:
		case SOC_ADC_ADCCON_DIV_128:
		case SOC_ADC_ADCCON_DIV_256:
		case SOC_ADC_ADCCON_DIV_512:
			decimation_rate = rate;
			break;
		default:
			return OWINOS_SENSORS_ERROR;
	}
	return decimation_rate;
}
/*---------------------------------------------------------------------------*/
static int
channel_pin(int type)
{
	if((OWINOS_SENSORS_ADC1) && (type == OWINOS_SENSORS_ADC1)) { 
		return SOC_ADC_ADCCON_CH_AIN0 + ADC_SENSORS_ADC1_PIN;
	}
	if((OWINOS_SENSORS_ADC2) && (type == OWINOS_SENSORS_ADC2)) {
		return SOC_ADC_ADCCON_CH_AIN0 + ADC_SENSORS_ADC2_PIN;
	}
	if((OWINOS_SENSORS_ADC3) && (type == OWINOS_SENSORS_ADC3)) {
		return SOC_ADC_ADCCON_CH_AIN0 + ADC_SENSORS_ADC3_PIN;
	}
	if((OWINOS_SENSORS_ADC4) && (type == OWINOS_SENSORS_ADC4)) {
		return SOC_ADC_ADCCON_CH_AIN0 + ADC_SENSORS_ADC4_PIN;
	}
	if((OWINOS_SENSORS_ADC5) && (type == OWINOS_SENSORS_ADC5)) {
		return SOC_ADC_ADCCON_CH_AIN0 + ADC_SENSORS_ADC5_PIN;
	}
	return OWINOS_SENSORS_ERROR;
}
/*---------------------------------------------------------------------------*/
static int
pin_configure(int type, int value)
{
	switch(type) {
		case SENSORS_HW_INIT:

			GPIO_SOFTWARE_CONTROL(GPIO_A_BASE, value);
			GPIO_SET_INPUT(GPIO_A_BASE, value);

			if(value & OWINOS_SENSORS_ADC1) {
				ioc_set_over(GPIO_A_NUM, ADC_SENSORS_ADC1_PIN, IOC_OVERRIDE_ANA);
			}
			if(value & OWINOS_SENSORS_ADC2) {
				ioc_set_over(GPIO_A_NUM, ADC_SENSORS_ADC2_PIN, IOC_OVERRIDE_ANA);
			}
			if(value & OWINOS_SENSORS_ADC3) {
				ioc_set_over(GPIO_A_NUM, ADC_SENSORS_ADC3_PIN, IOC_OVERRIDE_ANA);
			}
			if(value & OWINOS_SENSORS_ADC4) {
				ioc_set_over(GPIO_A_NUM, ADC_SENSORS_ADC4_PIN, IOC_OVERRIDE_ANA);
			}
			if(value & OWINOS_SENSORS_ADC5) {
				ioc_set_over(GPIO_A_NUM, ADC_SENSORS_ADC5_PIN, IOC_OVERRIDE_ANA);
			}
			adc_init();
			fix_decimation_rate(SOC_ADC_ADCCON_DIV_512);
			enabled_channels |= value;
			PRINTF("ADC: enabled channels 0x%02X\n", enabled_channels);
			break;

		case OWINOS_SENSORS_CONFIGURE_TYPE_DECIMATION_RATE:
			return fix_decimation_rate((uint8_t)value);

		default:
			return OWINOS_SENSORS_ERROR;
	}
	return 0;
}
/*---------------------------------------------------------------------------*/

static uint16_t
convert_to_value(uint8_t index)
{
	uint32_t value;
	int channel;

	channel = channel_pin(sensors.sensor[index].pin_mask);
	value = adc_get(channel, ADC_SENSORS_REFERENCE, decimation_rate);
	if((ADC_SENSORS_REFERENCE != SOC_ADC_ADCCON_REF_EXT_DIFF) && (value < 0)) {
		value = 0;
	}
	if(value == OWINOS_SENSORS_ERROR) {
		PRINTF("ADC sensors: failed retrieving data\n");
		return ADC_WRAPPER_ERROR;
	}
	if(!sensors.sensor[index].vdd3) {
		value *= ADC_WRAPPER_EXTERNAL_VREF;
		value /= ADC_WRAPPER_EXTERNAL_VREF_CROSSVAL;
	}
	switch(sensors.sensor[index].type) {
		case ONBOARD_LDR_SENSOR:
			/* Light dependant resistor (LDR) resistance value*/
			//value=(10230 - (value * 10));
		        value =((value/3)*0.0935);
			//value =1;
                         return (uint16_t)value;
		case ADC_PINOUT_ONE:
			/* TODO: change the caluclations according to the sensor */
			value = (10230 - (value * 10)) / value;
			return (uint16_t)value;

		case RESISTOR_SENSOR:
			/* TODO: change the caluclations according to the sensor */
		//	value = (value/20)*0.6;
			value=(value/1000);
			return (uint16_t)value;

			/* VDD+5 sensors */ 
		case SOUND_SENSOR:
			/* TODO: change the caluclations according to the sensor */
			value = (((value/100)*2)/9);
			return (uint16_t)value;

		case ONBOARD_TOUCH_SENSOR:
			/* TODO: change the caluclations according to the sensor */
			value = ((value/100)*2);
                        return (uint16_t)value;
		default:
			return ADC_WRAPPER_ERROR;
	}

	return ADC_WRAPPER_ERROR;
}
/*---------------------------------------------------------------------------*/
static int
value(int type)
{
	uint8_t index=0,i;
	uint16_t sensor_value;
	for(i = 0; i <= sensors.sensors_num; i++) {
		if(sensors.sensor[i].type == type) {
			index=i+1;
		}
	}
	if(!index) {
		PRINTF("ADC sensors: sensor not registered\n");
		return ADC_WRAPPER_SUCCESS;
	}

	/* Restore index value after the check */
	index -= 1;
	sensor_value = convert_to_value(index);

	return sensor_value;
}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int value)
{
	uint8_t pin_mask = GPIO_PIN_MASK(value);

	if((type != ONBOARD_LDR_SENSOR) && (type != ADC_PINOUT_ONE) &&
			(type != RESISTOR_SENSOR) && (type != SOUND_SENSOR) && 
			(type != ONBOARD_TOUCH_SENSOR) ) {
		PRINTF("ADC sensors: sensor not supported, check adc_wrapper.h header\n");
		return ADC_WRAPPER_ERROR;
	}
	if(sensors.sensors_num >= ADC_SENSORS_MAX) {
		PRINTF("ADC sensors: all adc channels available have been assigned\n");
		return ADC_WRAPPER_ERROR;
	}
	switch(type) {
		/* V+3.3 sensors */
		case ONBOARD_LDR_SENSOR:
		case ADC_PINOUT_ONE:
		case RESISTOR_SENSOR:
			if(pin_configure(SENSORS_HW_INIT, pin_mask) == OWINOS_SENSORS_ERROR) {
				return ADC_WRAPPER_ERROR;
			}
			sensors.sensor[sensors.sensors_num].type = type;
			sensors.sensor[sensors.sensors_num].pin_mask = pin_mask;
			sensors.sensor[sensors.sensors_num].vdd3 = 1;
			break;
			/*V+5 sensors*/
		case SOUND_SENSOR:
		case ONBOARD_TOUCH_SENSOR:
			if(pin_configure(SENSORS_HW_INIT, pin_mask) == OWINOS_SENSORS_ERROR) {
				return ADC_WRAPPER_ERROR;
			}
			sensors.sensor[sensors.sensors_num].type = type;
			sensors.sensor[sensors.sensors_num].pin_mask = pin_mask;
			sensors.sensor[sensors.sensors_num].vdd3 = 0;
			break;
		default:
			return ADC_WRAPPER_ERROR;
	}
	sensors.sensors_num++;
	sensors.sensors_ports |= pin_mask;
	return ADC_WRAPPER_SUCCESS;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(adc_sensors, ADC_SENSORS, value, configure, NULL);
/*---------------------------------------------------------------------------*/
/** @} */

