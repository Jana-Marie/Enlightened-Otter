/*
 * Enlighted-Otter  -  Stm32f334 based mobile worklight.
 * Copyright (C) 2018 Jan Henrik Hemsing
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

#include <math.h>
#include <string.h>
#include "main.h"
#include "stm32f3xx_hal.h"
#include "init_functions.h"
#include "defines.h"
#include "gamma.h"
#include "variables.h"
#include "utils.h"

extern ADC_HandleTypeDef hadc2;

extern COMP_HandleTypeDef hcomp2;
extern COMP_HandleTypeDef hcomp4;
extern COMP_HandleTypeDef hcomp6;

extern DAC_HandleTypeDef hdac1;
extern DAC_HandleTypeDef hdac2;

extern HRTIM_HandleTypeDef hhrtim1;

extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim2;

extern TSC_HandleTypeDef htscs;        // Touch slider handle
extern TSC_IOConfigTypeDef IoConfigs;

extern TSC_HandleTypeDef htscb;        // Touch button handle
extern TSC_IOConfigTypeDef IoConfigb;

extern UART_HandleTypeDef huart1;

void slider_task(void);
void button_task(void);
void UI_task(void);
void TSC_task(void);
void LED_task(void);
void boost_reg();
extern void enable_OTG(void);
extern void disable_OTG(void);
extern void set_pwm(uint8_t timer, float duty);
extern void set_brightness(uint8_t chan, float brightness, float color, float max_value);
extern uint16_t read_RT_ADC(void);

#if defined(SCOPE_CHANNELS)
extern void set_scope_channel(uint8_t ch, int16_t val);
extern void console_scope();
#endif

struct touch_t t = {.IdxBank = 0, .slider.offsetValue = {1153, 1978, 1962}, .button.offsetValue = {2075, 2131, 2450}};
struct reg_t r = {.Magiekonstante = (KI * (1.0f / (HRTIM_FREQUENCY_KHZ * 1000.0f) * REG_CNT)), .WW.target = 0.0f, .CW.target = 0.0f};
struct UI_t ui;
float vtemp;
uint8_t printCnt = 0;     // debugvalue can be removed later

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  GPIO_Init();
  DMA_Init();
  ADC2_Init();
  COMP2_Init();
  COMP4_Init();
  COMP6_Init();
  HRTIM1_Init();
  TSC_Init();
  I2C1_Init();
  USART1_UART_Init();
  DAC1_Init();
  DAC2_Init();
  TIM2_Init();

  HAL_COMP_Start(&hcomp2);
  HAL_COMP_Start(&hcomp4);
  HAL_COMP_Start(&hcomp6);

  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
  HAL_DAC_Start(&hdac2, DAC_CHANNEL_1);

  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, FAULT_CURRENT);  // set the current for the COMP2,4 to trigger FLT_1
  HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, FAULT_VOLTAGE);  // set the voltage for the COMP6 to trigger FLT_1

  RT_Init();      // mainly sets ILIM
  start_HRTIM1(); // start HRTIM and enable outputs

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  set_pwm(HRTIM_TIMERINDEX_TIMER_D, MIN_DUTY); // clear PWM registers needs to be done, otherwise power failure
  set_pwm(HRTIM_TIMERINDEX_TIMER_C, MIN_DUTY); // clear PWM registers

  HAL_TSC_Start_IT(&htscb);
  HAL_TSC_Start_IT(&htscs);

  while (1)
  {
    if (printCnt++ % 250 == 0) { // print only every n cycle

      set_scope_channel(0, r.CW.iavg);
      set_scope_channel(1, r.WW.iavg);
      set_scope_channel(2, r.CW.target);
      set_scope_channel(3, r.WW.target);
      set_scope_channel(4, 0);
      set_scope_channel(5, 0);
      set_scope_channel(6, ntc_calc(vtemp)); // f(x) = 0.096081461085562x^2 - 4.76256993467882x + 132.372469902458
      console_scope();
      HAL_Delay(5);
      printCnt = 0;
      LED_task();
    }
  }
}

void boost_reg(void) {
  /* Main current regulator */
  r.CW.iout = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2) / 4096.0f * 3.0f * 1000.0f;  // ISensCW - mA
  r.WW.iout = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3) / 4096.0f * 3.0f * 1000.0f;  // ISensWW - mA
  vtemp = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);

  r.CW.iavg = FILT(r.CW.iavg, r.CW.iout, CURRENT_AVERAGING_FILTER); // Moving average filter for CW input current
  r.WW.iavg = FILT(r.WW.iavg, r.WW.iout, CURRENT_AVERAGING_FILTER); // Moving average filter for WW input current

  r.CW.error = r.CW.target - r.CW.iavg;  // Calculate CW-current error
  r.WW.error = r.WW.target - r.WW.iavg;  // Calculate WW-current error

  r.CW.duty += (r.Magiekonstante * r.CW.error);     // Simple I regulator for CW current
  r.CW.duty = CLAMP(r.CW.duty, MIN_DUTY, MAX_DUTY); // Clamp to duty cycle

  r.WW.duty += (r.Magiekonstante * r.WW.error);     // Simple I regulator for WW current
  r.WW.duty = CLAMP(r.WW.duty, MIN_DUTY, MAX_DUTY); // Clamp to duty cycle

  set_pwm(HRTIM_TIMERINDEX_TIMER_D, r.CW.duty);  // Update CW duty cycle
  set_pwm(HRTIM_TIMERINDEX_TIMER_C, r.WW.duty);  // Update WW duty cycle
}

void set_brightness(uint8_t chan, float brightness, float color, float max_value) {
  float target_tmp, color_tmp;

  if (chan)       color_tmp = color;          // set color temperature multiplicator from 0-1 for WW
  else if (!chan) color_tmp = (1.0f - color); // and 1-0 fow CW

  target_tmp = CLAMP((brightness * color_tmp), 0.0f, max_value);  // calculate brightness accordingly and clamp

  if (chan) r.WW.target = gammaTable[(int)(target_tmp * 2)];        // apply gamma corection - gamma correction array position
  else if (!chan) r.CW.target = gammaTable[(int)(target_tmp * 2)];  // needs to be multiplied by 2 as we have 1000 gamma values for a current from 0-500mA
}

void TSC_task(void) {

  if (HAL_TSC_GroupGetStatus(&htscs, TSC_GROUP1_IDX) == TSC_GROUP_COMPLETED)
  {

    t.slider.acquisitionValue[t.IdxBank] = HAL_TSC_GroupGetValue(&htscs, TSC_GROUP1_IDX);
    t.slider.acquisitionValue[t.IdxBank] = t.slider.acquisitionValue[t.IdxBank] - t.slider.offsetValue[t.IdxBank];

    slider_task();

    HAL_TSC_IOConfig(&htscb, &IoConfigb);
    HAL_TSC_IODischarge(&htscb, ENABLE);
    __HAL_TSC_CLEAR_FLAG(&htscb, (TSC_FLAG_EOA | TSC_FLAG_MCE));
    HAL_TSC_Start_IT(&htscb);
  } else if (HAL_TSC_GroupGetStatus(&htscb, TSC_GROUP5_IDX) == TSC_GROUP_COMPLETED)
  {
    t.button.acquisitionValue[t.IdxBank] = HAL_TSC_GroupGetValue(&htscb, TSC_GROUP5_IDX);
    t.button.acquisitionValue[t.IdxBank] = t.button.acquisitionValue[t.IdxBank] - t.button.offsetValue[t.IdxBank];

    button_task();

    HAL_TSC_IOConfig(&htscs, &IoConfigs);
    HAL_TSC_IODischarge(&htscs, ENABLE);
    __HAL_TSC_CLEAR_FLAG(&htscs, (TSC_FLAG_EOA | TSC_FLAG_MCE));
    HAL_TSC_Start_IT(&htscs);
  }

  switch (t.IdxBank)
  {
  case 0:
    IoConfigb.ChannelIOs = TSC_GROUP5_IO2;
    IoConfigs.ChannelIOs = TSC_GROUP1_IO1;
    t.IdxBank = 1;
    break;
  case 1:
    IoConfigb.ChannelIOs = TSC_GROUP5_IO3;
    IoConfigs.ChannelIOs = TSC_GROUP1_IO2;
    t.IdxBank = 2;
    break;
  case 2:
    IoConfigb.ChannelIOs = TSC_GROUP5_IO4;
    IoConfigs.ChannelIOs = TSC_GROUP1_IO3;
    t.IdxBank = 0;
    break;
  default:
    break;
  }
}

void button_task(void) {
  uint8_t _powButton;

  if (t.button.acquisitionValue[1] < BUTTON_THRESHOLD) t.button.CBSwitch = 0;       // switch color or brightness selector
  else if (t.button.acquisitionValue[2] < BUTTON_THRESHOLD) t.button.CBSwitch = 1;
  else;
  if (t.button.acquisitionValue[0] < BUTTON_THRESHOLD) _powButton = 1;  // if the power button is pressed set to 1
  else _powButton = 0;

  if (t.button.isReleased) {                       // power button state maschine start if button was released, waiting for the next press
    if (_powButton == 1 && t.button.state == 0) {  // if powerbutton is pressed and device is off, turn on and reset "is released flag"
      t.button.state = 1;
      t.button.isReleased = 0;
    } else if (_powButton == 1 && t.button.state == 1) { // if powerbutton is pressed and device is on, turn off and reset "is released flag"
      t.button.state = 0;
      t.button.isReleased = 0;
      start_HRTIM1();                             // for now lets reset the flt state in the power off state
    }
  } else if (!t.button.isReleased && _powButton == 0) t.button.isReleased = 1; // set isReleased flag if powerbutton was released

  UI_task();
}

void slider_task(void) {
  if (t.IdxBank == 2) t.slider.acquisitionValue[t.IdxBank] = t.slider.acquisitionValue[t.IdxBank] * 2;

  t.slider.acquisitionValue[t.IdxBank] = CLAMP(t.slider.acquisitionValue[t.IdxBank], -2000, 0);

  int16_t x = ((t.slider.acquisitionValue[0] + t.slider.acquisitionValue[1]) / 2) - t.slider.acquisitionValue[2];
  int16_t y = ((t.slider.acquisitionValue[0] + t.slider.acquisitionValue[2]) / 2) - t.slider.acquisitionValue[1];
  int16_t z = ((t.slider.acquisitionValue[1] + t.slider.acquisitionValue[2]) / 2) - t.slider.acquisitionValue[0];

  if      (x < y && x < z && y < z) t.slider.pos = 2 * TOUCH_SCALE - ((z * TOUCH_SCALE) / (y + z));
  else if (x < y && x < z && y > z) t.slider.pos = ((y * TOUCH_SCALE) / (y + z)) + TOUCH_SCALE;
  else if (z < y && z < x && x < y) t.slider.pos = 5 * TOUCH_SCALE - ((y * TOUCH_SCALE) / (y + x));
  else if (z < y && z < x && x > y) t.slider.pos = ((x * TOUCH_SCALE) / (y + x)) + 4 * TOUCH_SCALE;
  else if (y < x && y < z && z < x) t.slider.pos = 8 * TOUCH_SCALE - ((x * TOUCH_SCALE) / (x + z));
  else if (y < x && y < z && z > x) t.slider.pos = ((z * TOUCH_SCALE) / (x + z)) + 7 * TOUCH_SCALE;

  if (MIN(MIN(t.slider.acquisitionValue[0], t.slider.acquisitionValue[1]), t.slider.acquisitionValue[2]) > SLIDER_THRESHOLD) {
    t.slider.pos = 0;
    t.slider.isTouched = 0;
  } else t.slider.isTouched = 1;

  UI_task();
}

void UI_task(void) {
  uint8_t _enable = 1;

  if (t.button.state) {  // if lamp is turned "soft" on
    if (t.slider.isTouched) {  // check if slider is touched
      if (ui.debounce >= 5) {   // "debounce" slider

        ui.distance += t.slider.pos - ui.distanceOld;             // calculate t.slider.pos delta
        ui.distance = CLAMP(ui.distance, 0.0f, MAX_CURRENT);

        if (t.button.CBSwitch == 0) ui.brightness = ui.distance;          // if color/brightness switch is 0 then change brightness
        if (t.button.CBSwitch == 1) ui.color = ui.distance / MAX_CURRENT; // if color/brightness switch is 1 then change the color

      } else ui.debounce++;

      if (t.button.CBSwitch == 0) ui.distance = ui.brightness;          // prevents jumps when switching between modes
      if (t.button.CBSwitch == 1) ui.distance = ui.color * MAX_CURRENT; // prevents jumps when switching between modes

      ui.distanceOld = t.slider.pos;                                // set ui.distanceOld to current t.slider.pos
    } else ui.debounce = 0;
  } else if ( t.button.state == 0 && ui.brightnessAvg != 0) _enable = 0;

  if ((ui.colorAvg != ui.color || ui.brightnessAvg != ui.brightness) && _enable) {       // smooth out color value until target

    ui.colorAvg = FILT(ui.colorAvg, ui.color, COLOR_FADING_FILTER);      // moving average filter with fixed constants
    ui.brightnessAvg = FILT(ui.brightnessAvg, ui.brightness, BRIGHTNESS_FADING_FILTER); // moving average filter with fixed constants

    set_brightness(CHAN_CW, ui.brightnessAvg, ui.colorAvg, MAX_CURRENT);
    set_brightness(CHAN_WW, ui.brightnessAvg, ui.colorAvg, MAX_CURRENT);
  } else {
    set_brightness(CHAN_CW, 0.0f, ui.colorAvg, MAX_CURRENT);
    set_brightness(CHAN_WW, 0.0f, ui.colorAvg, MAX_CURRENT);
  }
}

void LED_task(void) {
  if (t.button.state == 1) {
    HAL_GPIO_WritePin(GPIOA, LED_Brightness, !t.button.CBSwitch); // set LED "Brightness"
    HAL_GPIO_WritePin(GPIOA, LED_Color, t.button.CBSwitch);       // set LED "Color"
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1024);
  } else {
    HAL_GPIO_WritePin(GPIOA, LED_Brightness, 0);    // clear LED "Brightness"
    HAL_GPIO_WritePin(GPIOA, LED_Color, 0);         // clear LED "Color"
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, POWER_LED_BRIGHTNESS);      //HAL_GPIO_WritePin(GPIOA, LED_Power, 0);
  }
}

void _Error_Handler(char * file, int line)
{
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{

}

#endif
