/*
 * Enlightened-Otter  -  Stm32f334 based mobile worklight.
 * Copyright (C) 2019 Jan Henrik Hemsing
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of  MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "stm32f3xx_hal.h"
#include "defines.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;


void enable_OTG(void);
void disable_OTG(void);
void set_pwm(uint8_t timer, float duty);
void set_brightness(uint8_t chan, float brightness, float color, float max_value);
float read_RT_ADC(void);
float ntc_calc(uint16_t adc_value);
float gamma_calc(float target);
uint8_t read_RT_status(uint8_t _mask);
uint8_t read_RT_register(uint8_t _register);
void RT_Init(void);
void configure_RT(uint8_t _register, uint8_t _mask);
void powerdown(void);
void RT_adc_task(void);
#if defined(SCOPE_CHANNELS)
void set_scope_channel(uint8_t ch, int16_t val);
void console_scope(void);
uint8_t uart_buf[(7 * SCOPE_CHANNELS) + 2];
volatile int16_t ch_buf[2 * SCOPE_CHANNELS];
#endif
