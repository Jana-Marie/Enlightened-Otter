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

#include <math.h>
#include <string.h>
#include "main.h"
#include "stm32f3xx_hal.h"
#include "init_functions.h"
#include "defines.h"
#include "utils.h"
#include "variables.h"
#include "sk6812.h"

extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc1;

extern COMP_HandleTypeDef hcomp2;
extern COMP_HandleTypeDef hcomp4;
extern COMP_HandleTypeDef hcomp6;

extern DAC_HandleTypeDef hdac1;
extern DAC_HandleTypeDef hdac2;

extern HRTIM_HandleTypeDef hhrtim1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;

extern I2C_HandleTypeDef hi2c1;

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

struct touch_t t = {.IdxBank = 0, .slider.offsetValue = {5100, 5200, 4615}, .button.offsetValue = {5500, 5350, 6650}, .button.CBSwitch = 0};
struct reg_t r = {.Magiekonstante = (KI * (1.0f / (HRTIM_FREQUENCY_KHZ * 1000.0f) * REG_CNT)), .WW.target = 0.0f, .CW.target = 0.0f};
struct UI_t ui = {.colorAvg = 0.7, .color = 0.7, .brightnessAvg = 10, .brightness = 10};
struct status_t stat = {.vBat = 4.2};
extern uint16_t write_buffer[LED_BUFFER_SIZE];

uint16_t otterStat = 0;
hsv otter;
rgb otter_rgb;
uint8_t moodlight = 0;
uint8_t party = 0;
double colorOffset;
uint32_t standby = 0;

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  GPIO_Init();
  DMA_Init();
  ADC1_Init();
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
  TIM1_Init();
  //TIM15_Init();
  TIM16_Init();

  HAL_COMP_Start(&hcomp2);
  HAL_COMP_Start(&hcomp4);
  HAL_COMP_Start(&hcomp6);

  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
  HAL_DAC_Start(&hdac2, DAC_CHANNEL_1);

  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, FAULT_CURRENT);  // set the current for the COMP2,4 to trigger FLT_1
  HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, FAULT_VOLTAGE);  // set the voltage for the COMP6 to trigger FLT_1

  RT_Init();      // initialize the RT9466, mainly sets ILIM
  configure_RT(CHG_CTRL1,0x10);

  start_HRTIM1(); // start HRTIM and enable outputs

  set_pwm(HRTIM_TIMERINDEX_TIMER_D, MIN_DUTY); // clear PWM registers needs to be done, otherwise power failure
  set_pwm(HRTIM_TIMERINDEX_TIMER_C, MIN_DUTY); // clear PWM registers

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TSC_Start_IT(&htscb);   // start the touch button controller
  HAL_TSC_Start_IT(&htscs);   // start the touch slider controller

  HAL_ADC_Start(&hadc1);

  HAL_Delay(100); // measure current offset
  r.CW.ioff = r.CW.iavg;  // IOffsetCW - mA
  r.WW.ioff = r.WW.iavg;  // IOffsetWW - mA

  HAL_TIM_PWM_Start_DMA(&htim16, TIM_CHANNEL_1, write_buffer, LED_BUFFER_SIZE);

  otter.h = 0;
  otter.s = 1.0;
  otter.v = 1.0;

  while (1)
  {
    HAL_Delay(10);
    //otterStat = read_RT_register(0x11);
    //RT_adc_task();
    if(moodlight && !t.slider.isTouched){
      HAL_GPIO_WritePin(GPIOA,SK6812_EN,0);
      for(int i = 0; i <= 5; i++){
        otter.h = (int)((i) + (HAL_GetTick()/600))%360;
        otter.s = 1.0;
        otter.v = 1.0;
        set_pixel(i,hsv2rgb(otter),write_buffer);
      }
    } else if (party && !t.slider.isTouched ){
      HAL_GPIO_WritePin(GPIOA,SK6812_EN,0);
      for(int i = 0; i <= 5; i++){
        otter.h = (int)((i*60) + (HAL_GetTick()/30))%360;
        otter.s = 1.0;
        otter.v = 1.0;
        set_pixel(i,hsv2rgb(otter),write_buffer);
      }
    } else if (!t.button.CBSwitch && t.slider.isTouched && t.button.state && ui.debounce >= UI_DEBOUNCE_VAL){
      HAL_GPIO_WritePin(GPIOA,SK6812_EN,0);
      for(int i = 0; i <= 5; i++){
        otter.h = 180.0;
        if((int)((ui.brightness/MAX_CURRENT)*6.0)>i){
          otter.v = 1.0;
        } else if((int)((ui.brightness/MAX_CURRENT)*6.0)>=i){
          otter.v = ((ui.brightness/MAX_CURRENT)*6.0)-(int)((ui.brightness/MAX_CURRENT)*6.0);
        } else {
          otter.v = 0.0;
        }
        otter.s = 0.5;

        set_pixel(((i+2)%6),hsv2rgb(otter),write_buffer);
      }
    } else if (t.button.CBSwitch && t.slider.isTouched  && t.button.state && ui.debounce >= UI_DEBOUNCE_VAL){
      HAL_GPIO_WritePin(GPIOA,SK6812_EN,0);
      for(int i = 0; i <= 5; i++){

        if(((int)(ui.color*6.0f))>i){
          otter_rgb.r = 0.5;
          otter_rgb.g = 0.7;
          otter_rgb.b = 1;
        }
        else if(((int)(ui.color*6.0f))>=i){
          colorOffset = (ui.color*6.0f)-((int)(ui.color*6.0f));
          otter_rgb.g = 0.5 + (0.5*(colorOffset));
          otter_rgb.r = 0.7 ;//+ -((0.5*(colorOffset)) * (0.5*(1.0-colorOffset)))*2;
          otter_rgb.b = 0.5 + (0.5*(colorOffset-2.0));
        }
        else {
          otter_rgb.r = 1;
          otter_rgb.g = 0.7;
          otter_rgb.b = 0.5;
        }
        set_pixel(((i+2)%6),otter_rgb,write_buffer);
      }
    } else {
      HAL_GPIO_WritePin(GPIOA,SK6812_EN,1);
    }

    if(t.button.state || !party || !moodlight){
      standby = HAL_GetTick();
    } else {
      if(standby + STANDBY_TIME_CALC < HAL_GetTick()){
        stat.state = -1;
      }
    }

    stat.vBat = FILT(ADC2VBAT(HAL_ADC_GetValue(&hadc1)),stat.vBat,0.95);
    HAL_ADC_Start(&hadc1);
    if ((stat.vBat > 1.0f && stat.vBat < 2.9f) || stat.state == -1) powerdown();
  }
}

HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef * hhrtim, uint32_t TimerIdx){
  boost_reg();
}

void boost_reg(void) {
  // Main current regulator
  stat.ledTemp = ntc_calc(HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1));

  r.CW.iout = AMP(HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2),SHUNT_GAIN) * 1000.0f;  // ISensCW - mA
  r.WW.iout = AMP(HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3),SHUNT_GAIN) * 1000.0f;  // ISensWW - mA

  r.CW.iavg = FILT(r.CW.iavg, r.CW.iout, CURRENT_AVERAGING_FILTER); // Moving average filter for CW input current
  r.WW.iavg = FILT(r.WW.iavg, r.WW.iout, CURRENT_AVERAGING_FILTER); // Moving average filter for WW input current

  r.CW.error = r.CW.target - (r.CW.iavg - r.CW.ioff);  // Calculate CW-current error
  r.WW.error = r.WW.target - (r.WW.iavg - r.WW.ioff);  // Calculate WW-current error

  r.CW.duty += (r.Magiekonstante * r.CW.error);     // Simple I regulator for CW current
  r.CW.duty = CLAMP(r.CW.duty, MIN_DUTY, MAX_DUTY); // Clamp to duty cycle

  r.WW.duty += (r.Magiekonstante * r.WW.error);     // Simple I regulator for WW current
  r.WW.duty = CLAMP(r.WW.duty, MIN_DUTY, MAX_DUTY); // Clamp to duty cycle

  if (r.CW.target < CURRENT_CUTOFF) r.CW.duty = MIN_DUTY;
  if (r.WW.target < CURRENT_CUTOFF) r.WW.duty = MIN_DUTY;

  set_pwm(HRTIM_TIMERINDEX_TIMER_D, r.CW.duty);  // Update CW duty cycle
  set_pwm(HRTIM_TIMERINDEX_TIMER_C, r.WW.duty);  // Update WW duty cycle
}

void set_brightness(uint8_t chan, float brightness, float color, float max_value) {
  float target_tmp, color_tmp;

  if (chan)       color_tmp = (1.0f - color);   // set color temperature multiplicator from 0 to 1 for WW
  else if (!chan) color_tmp = color;            // and from 1 to 0 for CW

  target_tmp = CLAMP((brightness * color_tmp), 0.0f, max_value);  // calculate brightness accordingly and clamp it

  if (chan) {
    //r.WW.target = gammaTable[(int)(target_tmp)];  // New gamma calculation
    r.WW.target = gamma_calc(target_tmp);       // possibly replaces this one
    //r.WW.targetNoGamma = target_tmp;              // if needed
  }
  else if (!chan) {
    //r.CW.target = gammaTable[(int)(target_tmp)];  //
    r.CW.target = gamma_calc(target_tmp);  //
    //r.CW.targetNoGamma = target_tmp;  //
  }
}

void TSC_task(void) {

  if (HAL_TSC_GroupGetStatus(&htscs, TSC_GROUP1_IDX) == TSC_GROUP_COMPLETED)
  {
    t.slider.acquisitionValue[t.IdxBank] = HAL_TSC_GroupGetValue(&htscs, TSC_GROUP1_IDX);
    if (t.IdxBank == 1) t.slider.acquisitionValue[t.IdxBank] = t.slider.acquisitionValue[t.IdxBank] * 2;    // outer channel has only half the strenght
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
    t.button.acquisitionValue[t.IdxBank] = FILT(t.button.acquisitionValue[t.IdxBank], t.button.acquisitionValue[t.IdxBank], 0.98); // average/Lowpass filter the touch intesity

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
  UI_task();
}

void button_task(void) {
  uint8_t _powButton;

  if (t.button.acquisitionValue[2] < BUTTON_THRESHOLD && t.button.acquisitionValue[0] > BUTTON_THRESHOLD) {
    t.button.CBSwitch = 0; // switch color or brightness selector
    t.button.isTouchedTime++;
  } else if (t.button.acquisitionValue[1] < BUTTON_THRESHOLD && t.button.acquisitionValue[0] > BUTTON_THRESHOLD) {
    t.button.CBSwitch = 1;
    t.button.isTouchedTime++;
  } else if (t.button.acquisitionValue[0] < BUTTON_THRESHOLD && t.button.acquisitionValue[1] > BUTTON_THRESHOLD && t.button.acquisitionValue[2] > BUTTON_THRESHOLD) {
    _powButton = 1;        // if the power button is pressed set to 1
    t.button.isTouchedTime++;
  } else {
    _powButton = 0;
  }

  // "hard" Off state maschine

  if (t.button.isTouchedTime > TURNOFF_TIME && _powButton) {
    stat.state = -1;// when the counter target is reached, turn off via richtek "shipping" mode
    t.button.isTouchedTime = 0;
  } else if (t.button.isTouchedTime > TURNOFF_TIME && t.button.CBSwitch) {
    moodlight ^= 1;
    party = 0;
    t.button.isTouchedTime = 0;
  } else if (t.button.isTouchedTime > TURNOFF_TIME && !t.button.CBSwitch) {
    party ^= 1;
    moodlight = 0;
    t.button.isTouchedTime = 0;
  }

  // power button state maschine
  if (t.button.isReleased) {                       // power button state maschine start if button was released, waiting for the next press
    if (_powButton == 1 && t.button.state == 0) {  // if powerbutton is pressed and device is off, turn on and reset "is released flag"
      t.button.state = 1;
      t.button.isReleased = 0;
    } else if (_powButton == 1 && t.button.state == 1) {  // if powerbutton is pressed and device is on, turn off and reset "is released flag"
      t.button.state = 0;
      t.button.isReleased = 0;
      start_HRTIM1();                               // for now lets reset the flt state in the power off state
    }
  } else if (!t.button.isReleased && _powButton == 0) t.button.isReleased = 1; // set isReleased flag if powerbutton was released
}

void slider_task(void) {
  t.slider.acquisitionValue[t.IdxBank] = CLAMP(t.slider.acquisitionValue[t.IdxBank], -2000, 0);           // clamp values for position calculation

  t.slider.isTouchedVal = MIN(MIN(t.slider.acquisitionValue[0], t.slider.acquisitionValue[1]), t.slider.acquisitionValue[2]); // Check intensity of touch
  t.slider.isTouchedValAvg = FILT(t.slider.isTouchedValAvg, t.slider.isTouchedVal, TOUCH_THRESHOLD_FILTER); // average/Lowpass filter the touch intesity

  int16_t _isTouchedDelta = t.slider.isTouchedValAvg - t.slider.isTouchedVal; // caltulate delta from current intesity to averaged intesity

  if      (_isTouchedDelta > IS_TOUCHED_DELTA)          t.slider.isTouched = 1; // if delta is larger then x, touch down was detected
  else if (_isTouchedDelta < IS_RELEASED_DELTA)         t.slider.isTouched = 0; // if delta is lower then x, touch up was detected
  else if (t.slider.isTouchedValAvg > IS_RELEASED_ABS)  t.slider.isTouched = 0; // std value, if no touch is present

  if (t.slider.isTouched ) {
    int16_t x = ((t.slider.acquisitionValue[1] + t.slider.acquisitionValue[2]) / 2) - t.slider.acquisitionValue[0];
    int16_t y = ((t.slider.acquisitionValue[1] + t.slider.acquisitionValue[0]) / 2) - t.slider.acquisitionValue[2];
    int16_t z = ((t.slider.acquisitionValue[2] + t.slider.acquisitionValue[0]) / 2) - t.slider.acquisitionValue[1];

    if      (x < y && x < z && y < z) t.slider.pos = 2 * TOUCH_SCALE - ((z * TOUCH_SCALE) / (y + z));
    else if (x < y && x < z && y > z) t.slider.pos = ((y * TOUCH_SCALE) / (y + z)) + TOUCH_SCALE;
    else if (z < y && z < x && x < y) t.slider.pos = 5 * TOUCH_SCALE - ((y * TOUCH_SCALE) / (y + x));
    else if (z < y && z < x && x > y) t.slider.pos = ((x * TOUCH_SCALE) / (y + x)) + 4 * TOUCH_SCALE;
    else if (y < x && y < z && z < x) t.slider.pos = 8 * TOUCH_SCALE - ((x * TOUCH_SCALE) / (x + z));
    else if (y < x && y < z && z > x) t.slider.pos = ((z * TOUCH_SCALE) / (x + z)) + 7 * TOUCH_SCALE;
  }
}

void UI_task(void) {
  uint8_t _enable = 1;

  if (t.button.state) {         // if lamp is turned "soft" on
    if (t.slider.isTouched) {   // check if slider is touched
      if (ui.debounce >= UI_DEBOUNCE_VAL) {  // "debounce" slider

        if (SLIDER_BEHAVIOR == REL) {
          if (ABS(t.slider.pos - ui.distanceOld) < 200) ui.distance += ui.distanceOld - t.slider.pos;        // calculate t.slider.pos delta
        } else if (SLIDER_BEHAVIOR == AB){
          ui.distance = (MAX_CURRENT - (t.slider.pos - 30)) ;
        }

        ui.distance = CLAMP(ui.distance, 0.0f, MAX_CURRENT);  // clamp it to the maximum current

        if (t.button.CBSwitch == 0) ui.brightness = ui.distance;          // if color/brightness switch is 0 then change brightness
        if (t.button.CBSwitch == 1) ui.color = ui.distance / MAX_CURRENT; // if color/brightness switch is 1 then change the color - scale from 0 to 1

      } else ui.debounce++;   // increase debounce counter until counter-target is reached

      if (t.button.CBSwitch == 0) ui.distance = ui.brightness;          // prevents jumps when switching between modes
      if (t.button.CBSwitch == 1) ui.distance = ui.color * MAX_CURRENT; // prevents jumps when switching between modes

      ui.distanceOld = t.slider.pos;        // set ui.distanceOld to current t.slider.pos so the delta will be 0
    } else ui.debounce = 0;                 // clear douncer if slider is not touched
  } else if (!t.button.state) _enable = 0;  // if button state is 0 disable output

  if ((ui.colorAvg != ui.color || ui.brightnessAvg != ui.brightness) && _enable) {      // smooth out color value until target is reached and output is enabled
    ui.colorAvg = FILT(ui.colorAvg, ui.color, COLOR_FADING_FILTER);                     // moving average filter with fixed constants for the color mixing
    ui.brightnessAvg = FILT(ui.brightnessAvg, ui.brightness, BRIGHTNESS_FADING_FILTER); // moving average filter with fixed constants for the brightness

    set_brightness(CHAN_CW, ui.brightnessAvg, ui.colorAvg, MAX_CURRENT);  // set CW brightness accordingly to output
    set_brightness(CHAN_WW, ui.brightnessAvg, ui.colorAvg, MAX_CURRENT);  // set WW brightness accordingly to output
  } else {                                                    // if output is not enabled - brightness and color are ignored here
    set_brightness(CHAN_CW, 0.0f, ui.colorAvg, MAX_CURRENT);  // set CW output to 0
    set_brightness(CHAN_WW, 0.0f, ui.colorAvg, MAX_CURRENT);  // set WW output to 0
  }
  LED_task();
}

void LED_task(void) {
  if (t.button.state == 1) {
    HAL_GPIO_WritePin(GPIOB, LED_Brightness, !t.button.CBSwitch); // set LED "Brightness"
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 4096);           // set LED "Power"
    HAL_GPIO_WritePin(GPIOA, LED_Color, t.button.CBSwitch);       // set LED "Color"
  } else {
    HAL_GPIO_WritePin(GPIOB, LED_Brightness, 0);                  // clear LED "Brightness"
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 512);            // clear LED "Power"
    HAL_GPIO_WritePin(GPIOA, LED_Color, 0);                       // clear LED "Color"
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
