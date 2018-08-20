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

#include "stm32f3xx_hal.h"
#include "init_functions.h"
#include "main.h"
#include <string.h>
#include "defines.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
 
extern COMP_HandleTypeDef hcomp2;
extern COMP_HandleTypeDef hcomp4;
extern COMP_HandleTypeDef hcomp6;
 
extern DAC_HandleTypeDef hdac1;
extern DAC_HandleTypeDef hdac2;
 
extern HRTIM_HandleTypeDef hhrtim1;
 
extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
 
extern TIM_HandleTypeDef htim2;
 
extern TSC_HandleTypeDef htscs;        // Touch slider handle
extern TSC_IOConfigTypeDef IoConfigs;

extern TSC_HandleTypeDef htscb;        // Touch button handle
extern TSC_IOConfigTypeDef IoConfigb;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

extern void boost_reg();
static void enable_OTG(void);

uint16_t read_RT_ADC(void);

void set_pwm(uint8_t timer, float duty);
void primitive_TSC_button_task(uint8_t *colorBrightnessSwitch, uint8_t *powerButton);
void primitive_TSC_slider_task(uint16_t *sPos, uint8_t *isT);
void set_brightness(uint8_t chan, float brightness, float color, float max_value);
#if defined(SCOPE_CHANNELS)
void set_scope_channel(uint8_t ch, int16_t val);
void console_scope();
uint8_t uart_buf[(7 * SCOPE_CHANNELS) + 2];
volatile int16_t ch_buf[2 * SCOPE_CHANNELS];
#endif

__IO int32_t sliderAcquisitionValue[3];                 // register that holds the acquired slider values
__IO int32_t buttonAcquisitionValue[3];                 // register that holds the acquired button values
__IO int32_t sliderOffsetValue[3] = {1945, 1934, 1134}; // offset values which needs to be subtracted from the acquired values
__IO int32_t buttonOffsetValue[3] = {2120, 2433, 2058}; // Todo - make some kind of auto calibration

uint8_t IdxBankS = 0;       // IO indexer for the slider
uint8_t IdxBankB = 0;       // IO indexer for the buttons
uint8_t IdxBank = 0;
uint16_t sliderPos = 0;     // current slider position
uint8_t sliderIsTouched = 0;// 1 if slider is touched
int16_t distanceDelta = 0;      // delta slider position
int16_t brightnessDelta = 0;      // calculated brightness delta
float brightnessDeltaAvg = 0;      // calculated brightness delta
int16_t oldDistance = 0;   // old slider position
float colorProportion = 0;  // a value from 0.0f to 1.0f defining the current color porportions
float colorProportionAvg = 0;  // a value from 0.0f to 1.0f defining the current color porportions

uint8_t colorBrightnessSwitch = 0;       // color or brightness switch
uint8_t powButton = 1;        // power button value
uint8_t powStateHasChanged = 1;        // power button value
uint8_t powState = 1;

float targetCW = 0.0f;  // Coldwhite target current in mA
float targetWW = 0.0f;  // Warmwhite target current in mA

float cycleTime;            // time of one cycle
float MagiekonstanteCycle;  // Ki constant, independent of cycle time
float iavgCW, iavgWW, errorCW, errorWW, vin; // stores the average current
float dutyCW = MIN_DUTY;    // cold white duty cycle
float dutyWW = MIN_DUTY;    // warm white duty cycle

float _v, _i, _w, _wAvg;  // debugvalues to find matching boost frequency will be removed later
uint8_t print = 1;        // debugvalue for alternating reading of current / voltage
uint8_t printCnt = 0;     // debugvalue for delay reading of current / voltage
uint8_t sliderCnt = 0;
uint16_t adcCnt = 0;

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  set_pwm(HRTIM_TIMERINDEX_TIMER_D, MIN_DUTY); // clear PWM registers
  set_pwm(HRTIM_TIMERINDEX_TIMER_C, MIN_DUTY); // clear PWM registers

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

  HAL_TSC_IODischarge(&htscb, ENABLE);
  HAL_TSC_IODischarge(&htscs, ENABLE);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  //HAL_TSC_Start_IT(&htscb);
  //HAL_TSC_Start_IT(&htscs);

  HAL_GPIO_WritePin(GPIOA, LED_Brightness, 0); // clear LED "Brightness"
  HAL_GPIO_WritePin(GPIOA, LED_Color, 0);      // clear LED "Color"
  //HAL_GPIO_WritePin(GPIOA, LED_Power, 1);      // clear LED "Power"

  set_pwm(HRTIM_TIMERINDEX_TIMER_D, MIN_DUTY); // clear PWM registers
  set_pwm(HRTIM_TIMERINDEX_TIMER_C, MIN_DUTY); // clear PWM registers

  cycleTime = 1.0f / (HRTIM_FREQUENCY_KHZ * 1000.0f) * REG_CNT; // calculated cycle time
  MagiekonstanteCycle = KI * cycleTime;                         // calculated Ki independent of cycle time

  while (1)
  {

    if (printCnt % 2 == 0 ) primitive_TSC_slider_task(&sliderPos, &sliderIsTouched); // do the tsc tasks every now and then
    if ((printCnt + 1) % 2 == 0 ) primitive_TSC_button_task(&colorBrightnessSwitch, &powButton);

    if (printCnt++ > 50) { // print only every n cycle

      set_scope_channel(0, iavgWW);
      set_scope_channel(1, iavgCW);
      set_scope_channel(2, targetWW);
      set_scope_channel(3, targetCW);
      set_scope_channel(4, vin);
      set_scope_channel(5, 0);
      set_scope_channel(6, colorProportion * 100.0f);
      console_scope();

      printCnt = 0;
    }

    if (powState == 1) {      // if lamp is turned "soft" on
      if ( sliderPos != 0) {  // check if slider is touched
        if (sliderCnt >= 5) { // "debounce" slider

          distanceDelta += sliderPos - oldDistance;             // calculate sliderPos delta
          distanceDelta = CLAMP(distanceDelta, 0.0f, TOUCH_SCALE_DIVIDER);

          if (colorBrightnessSwitch == 0) brightnessDelta = distanceDelta;          // if color/brightness switch is 0 then change brightness
          if (colorBrightnessSwitch == 1) colorProportion = distanceDelta / TOUCH_SCALE_DIVIDER; // if color/brightness switch is 1 then change the color

        } else sliderCnt++;

        if (colorBrightnessSwitch == 0) distanceDelta = brightnessDelta;          // prevents jumps when switching between modes
        if (colorBrightnessSwitch == 1) distanceDelta = colorProportion * TOUCH_SCALE_DIVIDER; // prevents jumps when switching between modes

        oldDistance = sliderPos;                                // set oldDistance to current sliderPos
      } else sliderCnt = 0;
      // calculate nex value with moving average filter, do this until target is reached
      if (colorProportionAvg != colorProportion) {              // smooth out color value until target

        colorProportionAvg = FILT(colorProportionAvg, colorProportion, COLOR_FADING_FILTER); // moving average filter with fixed constants

        set_brightness(CW, brightnessDeltaAvg, colorProportionAvg, TOUCH_SCALE_DIVIDER);
        set_brightness(WW, brightnessDeltaAvg, colorProportionAvg, TOUCH_SCALE_DIVIDER);
      }
      if (brightnessDeltaAvg != brightnessDelta) {                                // smooth out brightness value until target

        brightnessDeltaAvg = FILT(brightnessDeltaAvg, brightnessDelta, BRIGHTNESS_FADING_FILTER); // moving average filter with fixed constants

        set_brightness(CW, brightnessDeltaAvg, colorProportionAvg, TOUCH_SCALE_DIVIDER);
        set_brightness(WW, brightnessDeltaAvg, colorProportionAvg, TOUCH_SCALE_DIVIDER);
      }
    } else if ( powState == 0) {                        // if lamp is turned "soft" off
      if (brightnessDeltaAvg != 0) {                    // calculate and set until target is reached

        brightnessDeltaAvg = brightnessDeltaAvg * BRIGHTNESS_FADING_FILTER;  // moving average filter with fixed constants and fixed taget

        set_brightness(CW, brightnessDeltaAvg, colorProportionAvg, TOUCH_SCALE_DIVIDER);
        set_brightness(WW, brightnessDeltaAvg, colorProportionAvg, TOUCH_SCALE_DIVIDER);
      }
    }

    if (powStateHasChanged) {                       // power button state maschine start if something has changed
      if (powButton == 1 && powState == 0) {        // if powerbutton is pressed and device is off, turn on and set "has changed flag"
        powState = 1;
        powStateHasChanged = 0;
      } else if (powButton == 1 && powState == 1) { // if powerbutton is pressed and device is on, turn off and set "has changed flag"
        powState = 0;
        powStateHasChanged = 0;
      }
    } else if (!powStateHasChanged && powButton == 0) powStateHasChanged = 1; // else clear flag

    if (powState == 1) {
      HAL_GPIO_WritePin(GPIOA, LED_Brightness, !colorBrightnessSwitch); // set LED "Brightness"
      HAL_GPIO_WritePin(GPIOA, LED_Color, colorBrightnessSwitch);       // set LED "Color"
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1024);
    } else {
      HAL_GPIO_WritePin(GPIOA, LED_Brightness, 0);    // clear LED "Brightness"
      HAL_GPIO_WritePin(GPIOA, LED_Color, 0);         // clear LED "Color"
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, POWER_LED_BRIGHTNESS);      //HAL_GPIO_WritePin(GPIOA, LED_Power, 0);
    }
  }
}

void boost_reg(void) {

  /* VIN ADC not injected mode*/

  if (adcCnt++ >= 2014 && __HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC)) {
    vin = (HAL_ADC_GetValue(&hadc1) * 2.2744f); //  1 / 4096.0f * 1000.0f * 3.0f / 0.475 * 1.475 = 2.2744
    HAL_ADC_Start(&hadc1);
    adcCnt = 0;
  }

  /* Main current regulator */
  float ioutCW, ioutWW;

  ioutCW = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2) / 4096.0f * 3.0f * 1000.0f;  // ISensCW - mA
  ioutWW = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3) / 4096.0f * 3.0f * 1000.0f;  // ISensWW - mA

  iavgCW = FILT(iavgCW, ioutCW, CURRENT_AVERAGING_FILTER); // Moving average filter for CW input current
  iavgWW = FILT(iavgWW, ioutWW, CURRENT_AVERAGING_FILTER); // Moving average filter for WW input current

  errorCW = targetCW - iavgCW;  // Calculate CW-current error
  errorWW = targetWW - iavgWW;  // Calculate WW-current error

  dutyCW += (MagiekonstanteCycle * errorCW);  // Simple I regulator for CW current
  dutyCW = CLAMP(dutyCW, MIN_DUTY, MAX_DUTY); // Clamp to duty cycle

  dutyWW += (MagiekonstanteCycle * errorWW);  // Simple I regulator for WW current
  dutyWW = CLAMP(dutyWW, MIN_DUTY, MAX_DUTY); // Clamp to duty cycle

  set_pwm(HRTIM_TIMERINDEX_TIMER_D, dutyCW);  // Update CW duty cycle
  set_pwm(HRTIM_TIMERINDEX_TIMER_C, dutyWW);  // Update WW duty cycle
}

void set_brightness(uint8_t chan, float brightness, float color, float max_value) {
  float target_temp, color_temp;

  if (chan) color_temp = color;
  else if (!chan) color_temp = (1.0f - color);

  target_temp = CLAMP((brightness * color_temp), 0.0f, max_value);

  if (chan) targetWW = target_temp;
  else if (!chan) targetCW = target_temp;
}

void HAL_TSC_ConvCpltCallback(TSC_HandleTypeDef* htsc)
{

  HAL_GPIO_TogglePin(GPIOA, LED_Color); //debug

  HAL_TSC_IODischarge(&htscb, ENABLE);
  HAL_TSC_IODischarge(&htscs, ENABLE);

  if (HAL_TSC_GroupGetStatus(&htscs, TSC_GROUP1_IDX) == TSC_GROUP_COMPLETED)
  {
    sliderAcquisitionValue[IdxBank] = HAL_TSC_GroupGetValue(&htscs, TSC_GROUP1_IDX);
    sliderAcquisitionValue[IdxBank] = sliderAcquisitionValue[IdxBank] - sliderOffsetValue[IdxBank];
  }
  if (HAL_TSC_GroupGetStatus(&htscb, TSC_GROUP5_IDX) == TSC_GROUP_COMPLETED)
  {
    buttonAcquisitionValue[IdxBank] = HAL_TSC_GroupGetValue(&htscb, TSC_GROUP5_IDX);
    buttonAcquisitionValue[IdxBank] = buttonAcquisitionValue[IdxBank] - buttonOffsetValue[IdxBank];
  }

  switch (IdxBank)
  {
  case 0:
    IoConfigb.ChannelIOs = TSC_GROUP5_IO2;
    IoConfigs.ChannelIOs = TSC_GROUP1_IO1;
    IdxBankB = 1;
    break;
  case 1:
    IoConfigb.ChannelIOs = TSC_GROUP5_IO3;
    IoConfigs.ChannelIOs = TSC_GROUP1_IO2;
    IdxBankB = 2;
    break;
  case 2:
    IoConfigb.ChannelIOs = TSC_GROUP5_IO4;
    IoConfigs.ChannelIOs = TSC_GROUP1_IO3;
    IdxBankB = 0;
    break;
  default:
    break;
  }

  HAL_TSC_IOConfig(&htscb, &IoConfigb);
  HAL_TSC_IOConfig(&htscs, &IoConfigs);

  HAL_TSC_Start_IT(&htscb);
  HAL_TSC_Start_IT(&htscs);
}


void primitive_TSC_button_task(uint8_t *colorBrightnessSwitch, uint8_t *powerButton) {

  int16_t buttonThr = -900;

  switch (IdxBankB)
  {
  case 0:
    IoConfigb.ChannelIOs = TSC_GROUP5_IO2;
    IdxBankB = 1;
    break;
  case 1:
    IoConfigb.ChannelIOs = TSC_GROUP5_IO3;
    IdxBankB = 2;
    break;
  case 2:
    IoConfigb.ChannelIOs = TSC_GROUP5_IO4;
    IdxBankB = 0;
    break;
  default:
    break;
  }

  HAL_TSC_IOConfig(&htscb, &IoConfigb);
  HAL_TSC_IODischarge(&htscb, ENABLE);
  HAL_TSC_Start(&htscb);

  while (HAL_TSC_GetState(&htscb) == HAL_TSC_STATE_BUSY)
  {
    //FIXME INTERRUPT
  }

  __HAL_TSC_CLEAR_FLAG(&htscb, (TSC_FLAG_EOA | TSC_FLAG_MCE)); //idk why were doing this here

  if (HAL_TSC_GroupGetStatus(&htscb, TSC_GROUP5_IDX) == TSC_GROUP_COMPLETED)
  {
    buttonAcquisitionValue[IdxBankB] = HAL_TSC_GroupGetValue(&htscb, TSC_GROUP5_IDX);
    buttonAcquisitionValue[IdxBankB] = buttonAcquisitionValue[IdxBankB] - buttonOffsetValue[IdxBankB];

    if (buttonAcquisitionValue[0] < buttonThr) *colorBrightnessSwitch = 0;
    else if (buttonAcquisitionValue[1] < buttonThr)  *colorBrightnessSwitch = 1;
    else;
    if (buttonAcquisitionValue[2] < buttonThr) *powerButton = 1;
    else *powerButton = 0;
  }
}

void primitive_TSC_slider_task(uint16_t *sPos, uint8_t *isT) {

  switch (IdxBankS)
  {
  case 0:
    IoConfigs.ChannelIOs = TSC_GROUP1_IO1;
    IdxBankS = 1;
    break;
  case 1:
    IoConfigs.ChannelIOs = TSC_GROUP1_IO2;
    IdxBankS = 2;
    break;
  case 2:
    IoConfigs.ChannelIOs = TSC_GROUP1_IO3;
    IdxBankS = 0;
    break;
  default:
    break;
  }

  HAL_TSC_IOConfig(&htscs, &IoConfigs);
  HAL_TSC_IODischarge(&htscs, ENABLE);
  HAL_TSC_Start(&htscs);

  while (HAL_TSC_GetState(&htscs) == HAL_TSC_STATE_BUSY)
  {
    //FIXME ADD INTERRUPT
  }

  __HAL_TSC_CLEAR_FLAG(&htscs, (TSC_FLAG_EOA | TSC_FLAG_MCE)); //idk why were doing this here

  if (HAL_TSC_GroupGetStatus(&htscs, TSC_GROUP1_IDX) == TSC_GROUP_COMPLETED)
  {
    sliderAcquisitionValue[IdxBankS] = HAL_TSC_GroupGetValue(&htscs, TSC_GROUP1_IDX);
    sliderAcquisitionValue[IdxBankS] = sliderAcquisitionValue[IdxBankS] - sliderOffsetValue[IdxBankS];

    if (IdxBankS == 2) sliderAcquisitionValue[IdxBankS] = sliderAcquisitionValue[IdxBankS] * 2;

    sliderAcquisitionValue[IdxBankS] = CLAMP(sliderAcquisitionValue[IdxBankS], -2000, 0);

    int16_t z = ((sliderAcquisitionValue[0] + sliderAcquisitionValue[1]) / 2) - sliderAcquisitionValue[2];
    int16_t x = ((sliderAcquisitionValue[0] + sliderAcquisitionValue[2]) / 2) - sliderAcquisitionValue[1];
    int16_t y = ((sliderAcquisitionValue[1] + sliderAcquisitionValue[2]) / 2) - sliderAcquisitionValue[0];

    if      (x < y && x < z && y < z) *sPos = 2 * TOUCH_SCALE - ((z * TOUCH_SCALE) / (y + z));
    else if (x < y && x < z && y > z) *sPos = ((y * TOUCH_SCALE) / (y + z)) + TOUCH_SCALE;
    else if (z < y && z < x && x < y) *sPos = 5 * TOUCH_SCALE - ((y * TOUCH_SCALE) / (y + x));
    else if (z < y && z < x && x > y) *sPos = ((x * TOUCH_SCALE) / (y + x)) + 4 * TOUCH_SCALE;
    else if (y < x && y < z && z < x) *sPos = 8 * TOUCH_SCALE - ((x * TOUCH_SCALE) / (x + z));
    else if (y < x && y < z && z > x) *sPos = ((z * TOUCH_SCALE) / (x + z)) + 7 * TOUCH_SCALE;

    if (MIN(MIN(sliderAcquisitionValue[0], sliderAcquisitionValue[1]), sliderAcquisitionValue[2]) > -100) {
      *sPos = 0;
      *isT = 0;
    } else {
      *isT = 1;
    }
  }
}

static void enable_OTG(void) {
  configure_RT(CHG_CTRL16, DISABLE_UUG);
  configure_RT(CHG_CTRL1, ENABLE_OTG_MASK);
}

uint16_t read_RT_ADC(void) {
  uint8_t _ADC_H, _ADC_L;
  uint8_t _tmp_data_H = ADC_DATA_H;
  uint8_t _tmp_data_L = ADC_DATA_L;

  HAL_I2C_Master_Transmit(&hi2c1, RT_ADDRESS, &_tmp_data_H, sizeof(_tmp_data_H), 500);
  HAL_I2C_Master_Receive(&hi2c1, RT_ADDRESS, &_ADC_H, 1, 500);
  HAL_I2C_Master_Transmit(&hi2c1, RT_ADDRESS, &_tmp_data_L, sizeof(_tmp_data_L), 500);
  HAL_I2C_Master_Receive(&hi2c1, RT_ADDRESS, &_ADC_L, 1, 500);

  uint16_t _tmp_data = ((_ADC_H << 8) | (_ADC_L & 0xFF));
  return _tmp_data;
}

#if defined(SCOPE_CHANNELS)
void set_scope_channel(uint8_t ch, int16_t val) {
  ch_buf[ch] = val;
}

void console_scope(void) {
  memset(uart_buf, 0, sizeof(uart_buf));

#if (SCOPE_CHANNELS == 1)
  sprintf((char*)uart_buf, "%i\n\r", ch_buf[0]);
#elif (SCOPE_CHANNELS == 2)
  sprintf((char*)uart_buf, "%i\t%i\n\r", ch_buf[0], ch_buf[1]);
#elif (SCOPE_CHANNELS == 3)
  sprintf((char*)uart_buf, "%i\t%i\t%i\n\r", ch_buf[0], ch_buf[1], ch_buf[2]);
#elif (SCOPE_CHANNELS == 4)
  sprintf((char*)uart_buf, "%i\t%i\t%i\t%i\n\r", ch_buf[0], ch_buf[1], ch_buf[2], ch_buf[3]);
#elif (SCOPE_CHANNELS == 5)
  sprintf((char*)uart_buf, "%i\t%i\t%i\t%i\t%i\n\r", ch_buf[0], ch_buf[1], ch_buf[2], ch_buf[3], ch_buf[4]);
#elif (SCOPE_CHANNELS == 6)
  sprintf((char*)uart_buf, "%i\t%i\t%i\t%i\t%i\t%i\n\r", ch_buf[0], ch_buf[1], ch_buf[2], ch_buf[3], ch_buf[4], ch_buf[5]);
#elif (SCOPE_CHANNELS == 7)
  sprintf((char*)uart_buf, "%i\t%i\t%i\t%i\t%i\t%i\t%i\n\r", ch_buf[0], ch_buf[1], ch_buf[2], ch_buf[3], ch_buf[4], ch_buf[5], ch_buf[6]);
#elif (SCOPE_CHANNELS == 8)
  sprintf((char*)uart_buf, "%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\n\r", ch_buf[0], ch_buf[1], ch_buf[2], ch_buf[3], ch_buf[4], ch_buf[5], ch_buf[6], ch_buf[7]);
#endif

  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)uart_buf, strlen((char*)uart_buf));
  huart1.gState = HAL_UART_STATE_READY;
}
#endif

void set_pwm(uint8_t timer, float duty) {

  /* Clamp duty cycle values */
  if (duty < MIN_DUTY) duty = MIN_DUTY;
  if (duty > MAX_DUTY) duty = MAX_DUTY;

  /* Set registers according to duty cycle */
  HRTIM1->sTimerxRegs[timer].CMP1xR = HRTIM_PERIOD * duty;
  HRTIM1->sTimerxRegs[timer].CMP2xR = HRTIM_PERIOD - (HRTIM_PERIOD * duty);
  HRTIM1->sTimerxRegs[timer].SETx1R = HRTIM_SET1R_PER;
  HRTIM1->sTimerxRegs[timer].RSTx1R = HRTIM_RST1R_CMP1;
  HRTIM1->sTimerxRegs[timer].SETx2R = HRTIM_SET2R_CMP2;
  HRTIM1->sTimerxRegs[timer].RSTx2R = HRTIM_RST2R_PER;
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
