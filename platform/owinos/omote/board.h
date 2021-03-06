/*
 * Copyright (c) 2020, Oppila - http://www.oppila.in/
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
 *    contributors may be used to endorse or promote products derived
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
/**
 * \addtogroup owinos-platforms
 * @{
 *
 * \defgroup omote OMote platform 
 *
 * This file provides connectivity information on LEDs, Buttons, UART and
 * other OMote peripherals
 *
 * This file can be used as the basis to configure other platforms using the
 * cc2538 SoC.
 * @{
 *
 * \file
 * Header file with definitions related to the I/O connections on the Owinos
 * OMote platform, cc2538-based
 *
 * \note   Do not include this file directly. It gets included by contiki-conf
 *         after all relevant directives have been set.
 */
#ifndef BOARD_H_
#define BOARD_H_

#include "dev/gpio.h"
#include "dev/nvic.h"
/*---------------------------------------------------------------------------*/
#undef LEDS_GREEN
#undef LEDS_GREEN1

#define LEDS_GREEN               1           /**< LED1 (Green)   -> PD0 */
#define LEDS_GREEN_PIN_MASK     (1 << 0)
#define LEDS_GREEN_PORT_BASE    GPIO_D_BASE

#define LEDS_GREEN1           1           /**< LED2 (Green1) -> PD4 */
#define LEDS_GREEN1_PIN_MASK   (1 << 4)
#define LEDS_GREEN1_PORT_BASE  GPIO_D_BASE


/* Notify various examples that we have LEDs */
#define PLATFORM_HAS_LEDS        1
/** @} */
/*---------------------------------------------------------------------------*/
/** \name USB configuration
 *
 * The USB pullup is enabled by an external resistor, not mapped to a GPIO
 */
#ifdef USB_PULLUP_PORT
#undef USB_PULLUP_PORT
#endif
#ifdef USB_PULLUP_PIN
#undef USB_PULLUP_PIN
#endif
/** @} */
/*---------------------------------------------------------------------------*/
/** \name UART configuration
 *
 * On the OMote, the UARTs are connected to the following ports/pins:
 *
 * - UART0:
 *   - RX:  PA0
 *   - TX:  PA1
 * - UART1:
 *   - RX:  PC1
 *   - TX:  PC0
 *   - CTS: disabled as default
 *   - RTS: disabled as default
 *
 * @{
 */
#define UART0_RX_PORT            GPIO_A_NUM
#define UART0_RX_PIN             0
#define UART0_TX_PORT            GPIO_A_NUM
#define UART0_TX_PIN             1

#define UART1_RX_PORT            GPIO_C_NUM
#define UART1_RX_PIN             1
#define UART1_TX_PORT            GPIO_C_NUM
#define UART1_TX_PIN             0
#define UART1_CTS_PORT           (-1)
#define UART1_CTS_PIN            (-1)
#define UART1_RTS_PORT           (-1)
#define UART1_RTS_PIN            (-1)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name ADC configuration
 *
 * These values configure which CC2538 pins and ADC channels to use for the ADC
 * inputs. By default the OMote allows four out-of-the-box ADC ports with a
 *
 * The OMote allows both 3.3V  analogue sensors as follow:
 *
 */
#define ADC_SENSORS_PORT         GPIO_A_NUM    /**< ADC GPIO control port */

#ifndef ADC_SENSORS_CONF_ADC1_PIN
#define ADC_SENSORS_ADC1_PIN     5             /**< LDR sensor is connected    */
#else
#if ((ADC_SENSORS_CONF_ADC1_PIN != -1) && (ADC_SENSORS_CONF_ADC1_PIN != 5))
#error "ADC1 channel should be mapped to PA5 or disabled with -1"
#else
#define ADC_SENSORS_ADC1_PIN ADC_SENSORS_CONF_ADC1_PIN
#endif
#endif

#ifndef ADC_SENSORS_CONF_ADC3_PIN
#define ADC_SENSORS_ADC3_PIN     2             /**< ADC3 to PA2, 5V     */
#else
#if ((ADC_SENSORS_CONF_ADC3_PIN != -1) && (ADC_SENSORS_CONF_ADC3_PIN != 2))
#error "ADC3 channel should be mapped to PA2 or disabled with -1"
#else
#define ADC_SENSORS_ADC3_PIN ADC_SENSORS_CONF_ADC3_PIN
#endif
#endif

#ifndef ADC_SENSORS_CONF_ADC2_PIN
#define ADC_SENSORS_ADC2_PIN     4             /**< ADC2 to PA4     */
#else
#if ((ADC_SENSORS_CONF_ADC2_PIN != -1) && (ADC_SENSORS_CONF_ADC2_PIN != 4))
#error "ADC2 channel should be mapped to PA2 or disabled with -1"
#else
#define ADC_SENSORS_ADC2_PIN ADC_SENSORS_CONF_ADC2_PIN
#endif
#endif

#ifndef ADC_SENSORS_CONF_ADC4_PIN
#define ADC_SENSORS_ADC4_PIN     6             /**< ADC4 to PA6     */
#else
#if ((ADC_SENSORS_CONF_ADC4_PIN != -1) && (ADC_SENSORS_CONF_ADC4_PIN != 6))
#error "ADC2 channel should be mapped to PA2 or disabled with -1"
#else
#define ADC_SENSORS_ADC4_PIN ADC_SENSORS_CONF_ADC4_PIN
#endif
#endif

#ifndef ADC_SENSORS_CONF_ADC5_PIN
#define ADC_SENSORS_ADC5_PIN     7             /**< ADC5 to PA7     */
#else
#if ((ADC_SENSORS_CONF_ADC5_PIN != -1) && (ADC_SENSORS_CONF_ADC5_PIN != 7))
#error "ADC2 channel should be mapped to PA2 or disabled with -1"
#else
#define ADC_SENSORS_ADC5_PIN ADC_SENSORS_CONF_ADC5_PIN
#endif
#endif

#ifndef ADC_SENSORS_CONF_MAX
#define ADC_SENSORS_MAX          4             /**< Maximum sensors    */
#else
#define ADC_SENSORS_MAX          ADC_SENSORS_CONF_MAX
#endif
/** @} */
/*---------------------------------------------------------------------------*/
/** \name OMote Button configuration
 *
 * Buttons on the OMote are connected as follows:
 * - BUTTON_USER  -> PB0, S1 user button
 * - BUTTON_RESET -> RESET_N line, S2 reset the CC2538
 * - BUTTON_BSL   -> PA3, bootloader button
 * @{
 */
#define BUTTON_USER_PORT       GPIO_B_NUM
#define BUTTON_USER_PIN        0
#define BUTTON_USER_VECTOR     GPIO_B_IRQn

/* Notify various examples that we have an user button.
 * If ADC6 channel is used, then disable the user button
 */
#define PLATFORM_HAS_BUTTON  1
#define PLATFORM_SUPPORTS_BUTTON_HAL  1
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name SPI (SSI1) configuration
 *
 * These values configure which CC2538 pins to use for the SPI (SSI1) lines,
 * shared with the microSD and exposed over JP5 connector.
 * TX -> MOSI, RX -> MISO
 * @{
 */
#define SPI1_CLK_PORT            GPIO_C_NUM
#define SPI1_CLK_PIN             4
#define SPI1_TX_PORT             GPIO_C_NUM
#define SPI1_TX_PIN              5
#define SPI1_RX_PORT             GPIO_C_NUM
#define SPI1_RX_PIN              6
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name I2C configuration
 *
 * These values configure which CC2538 pins to use for the I2C lines
 * The I2C is exposed over the J6 header, using a 2-pin connector 
 * @{
 */
#define I2C_SCL_PORT             GPIO_C_NUM
#define I2C_SCL_PIN              3
#define I2C_SDA_PORT             GPIO_C_NUM
#define I2C_SDA_PIN              2
#define I2C_INT_PORT             GPIO_D_NUM
#define I2C_INT_PIN              1
#define I2C_INT_VECTOR           GPIO_D_IRQn
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Device string used on startup
 * @{
 */
#define BOARD_STRING "Owinos OMote platform"
/** @} */

#endif /* BOARD_H_ */

/**
 * @}
 * @}
 */
