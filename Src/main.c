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

#include "main.h"
#include <string.h>
#include "stm32f3xx_hal.h"
#include "defines.h"

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

COMP_HandleTypeDef hcomp2;
COMP_HandleTypeDef hcomp4;
COMP_HandleTypeDef hcomp6;

DAC_HandleTypeDef hdac1;
DAC_HandleTypeDef hdac2;

HRTIM_HandleTypeDef hhrtim1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TSC_HandleTypeDef htscs;        // Touch slider handle
TSC_IOConfigTypeDef IoConfigs;

TSC_HandleTypeDef htscb;        // Touch button handle
TSC_IOConfigTypeDef IoConfigb;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

void SystemClock_Config(void);
static void GPIO_Init(void);
static void DMA_Init(void);
static void ADC1_Init(void);
static void ADC2_Init(void);
static void COMP2_Init(void);
static void COMP4_Init(void);
static void COMP6_Init(void);
static void HRTIM1_Init(void);
static void TSC_Init(void);
static void I2C1_Init(void);
static void USART1_UART_Init(void);
static void DAC1_Init(void);
static void DAC2_Init(void);
static void configure_RT(uint8_t _register, uint8_t _mask);
static void init_RT(void);
static void start_HRTIM1(void);
extern void boost_reg();
static void enable_OTG(void);
uint16_t read_RT_ADC(void);
void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);
void set_pwm(uint8_t timer, float duty);
void primitive_TSC_button_task(uint8_t *colorBrightnessSwitch, uint8_t *powerButton);
void primitive_TSC_slider_task(uint16_t *sPos, uint8_t *isT);
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
float avgConst = 0.99f; // Averaging filter constant closer to 1 => stronger filter

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

  HAL_COMP_Start(&hcomp2);
  HAL_COMP_Start(&hcomp4);
  HAL_COMP_Start(&hcomp6);

  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
  HAL_DAC_Start(&hdac2, DAC_CHANNEL_1);

  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, FAULT_CURRENT);  // set the current for the COMP2,4 to trigger FLT_1
  HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, FAULT_VOLTAGE);  // set the voltage for the COMP6 to trigger FLT_1

  init_RT();      // mainly sets ILIM
  start_HRTIM1(); // start HRTIM and enable outputs

  HAL_TSC_IODischarge(&htscb, ENABLE);
  HAL_TSC_IODischarge(&htscs, ENABLE);

  //HAL_TSC_Start_IT(&htscb);
  //HAL_TSC_Start_IT(&htscs);

  HAL_GPIO_WritePin(GPIOA, LED_Brightness, 0); // clear LED "Brightness"
  HAL_GPIO_WritePin(GPIOA, LED_Color, 0);      // clear LED "Color"
  HAL_GPIO_WritePin(GPIOA, LED_Power, 1);      // clear LED "Power"

  set_pwm(HRTIM_TIMERINDEX_TIMER_D, MIN_DUTY); // clear PWM registers
  set_pwm(HRTIM_TIMERINDEX_TIMER_C, MIN_DUTY); // clear PWM registers

  cycleTime = 1.0f / (HRTIM_FREQUENCY_KHZ * 1000.0f) * REG_CNT; // calculated cycle time
  MagiekonstanteCycle = KI * cycleTime;             // calculated Ki independent of cycle time

  while (1)
  {

    if (printCnt%2 == 0 ) primitive_TSC_slider_task(&sliderPos, &sliderIsTouched); // do the tsc tasks every now and then
    if ((printCnt+1)%2 == 0 ) primitive_TSC_button_task(&colorBrightnessSwitch, &powButton);
    
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
    
    if (powState == 1){                                         // if lamp is turned "soft" on
      if ( sliderPos != 0) {                                    // check if slider is touched
        if (sliderCnt >= 5) {                                   // "debounce" slider

          distanceDelta += sliderPos - oldDistance;                  // calculate sliderPos delta
          distanceDelta = CLAMP(distanceDelta, 0.0f, 287.0f);

          if (colorBrightnessSwitch == 0) brightnessDelta = distanceDelta;                 // if color/brightness switch is 0 then change brightness
          if (colorBrightnessSwitch == 1) colorProportion = distanceDelta / 287.0f; // if color/brightness switch is 1 then change the color
        } else sliderCnt++;

        if (colorBrightnessSwitch == 0) distanceDelta = brightnessDelta;                   // prevents jumps when switching between modes
        if (colorBrightnessSwitch == 1) distanceDelta = colorProportion * 287.0f;   // prevents jumps when switching between modes
        
        oldDistance = sliderPos;                                // set oldDistance to current sliderPos
      } else sliderCnt = 0;

                                                                // calculate nex value with moving average filter, do this until target is reached  
      if (colorProportionAvg != colorProportion){               // smooth out color value until target
          colorProportionAvg = colorProportionAvg * 0.95 + colorProportion * 0.05;  // moving average filter with fixed constants
          
          targetCW = CLAMP((brightnessDeltaAvg * colorProportionAvg), 0.0f, 287.0f);
          targetWW = CLAMP((brightnessDeltaAvg * (1.0f - colorProportionAvg)), 0.0f, 287.0f);
      }
      if(brightnessDeltaAvg != brightnessDelta){                              // smooth out brightness value until target
          brightnessDeltaAvg = brightnessDeltaAvg * 0.9 + brightnessDelta * 0.1;     // moving average filter with fixed constants

          targetCW = CLAMP((brightnessDeltaAvg * colorProportionAvg), 0.0f, 287.0f);
          targetWW = CLAMP((brightnessDeltaAvg * (1.0f - colorProportionAvg)), 0.0f, 287.0f);
      }
    } else if( powState == 0) {                   // if lamp is turned "soft" off
      if(brightnessDeltaAvg != 0){                     // calculate and set until target is reached
        brightnessDeltaAvg = brightnessDeltaAvg * 0.9;        // moving average filter with fixed constants and fixed taget

        targetCW = CLAMP((brightnessDeltaAvg * colorProportionAvg), 0.0f, 287.0f);
        targetWW = CLAMP((brightnessDeltaAvg * (1.0f - colorProportionAvg)), 0.0f, 287.0f);
      }
    }
    
    if (powStateHasChanged){                        // power button state maschine start if something has changed
      if (powButton == 1 && powState == 0){         // if powerbutton is pressed and device is off, turn on and set "has changed flag"
        powState = 1;
        powStateHasChanged = 0;
      } else if (powButton == 1 && powState == 1){  // if powerbutton is pressed and device is on, turn off and set "has changed flag"
        powState = 0;
        powStateHasChanged = 0;
      }
    } else if (!powStateHasChanged && powButton == 0) powStateHasChanged = 1; // else clear flag
    
    if (powState == 1){
      HAL_GPIO_WritePin(GPIOA, LED_Brightness, !colorBrightnessSwitch);  // clear LED "Brightness"
      HAL_GPIO_WritePin(GPIOA, LED_Color, colorBrightnessSwitch);        // clear LED "Color"
      HAL_GPIO_WritePin(GPIOA, LED_Power, powState);      // clear LED "Power"
    else {
      HAL_GPIO_WritePin(GPIOA, LED_Brightness, 0);  // clear LED "Brightness"
      HAL_GPIO_WritePin(GPIOA, LED_Color, 0);        // clear LED "Color"
      HAL_GPIO_WritePin(GPIOA, LED_Power, powState);      // clear LED "Power"
    }
  }
}

void boost_reg() {

  /* VIN ADC not injected mode*/

  if(adcCnt++ >= 2014 && __HAL_ADC_GET_FLAG(&hadc1,ADC_FLAG_EOC)){
    vin = (HAL_ADC_GetValue(&hadc1) * 2.2744f); //  1 / 4096.0f * 1000.0f * 3.0f / 0.475 * 1.475 = 2.2744
    HAL_ADC_Start(&hadc1);
    adcCnt = 0;
  }
  
  /* Main current regulator */
  float ioutCW, ioutWW;

  ioutCW = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2) / 4096.0f * 3.0f * 1000.0f;  // ISensCW - mA
  ioutWW = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3) / 4096.0f * 3.0f * 1000.0f;  // ISensWW - mA

  iavgCW = iavgCW * avgConst + ioutCW * (1.0f - avgConst);  // Moving average filter for CW input current
  iavgWW = iavgWW * avgConst + ioutWW * (1.0f - avgConst);  // Moving average filter for WW input current

  errorCW = targetCW - iavgCW;  // Calculate CW-current error
  errorWW = targetWW - iavgWW;  // Calculate WW-current error

  dutyCW += (MagiekonstanteCycle * errorCW);  // Simple I regulator for CW current
  dutyCW = CLAMP(dutyCW, MIN_DUTY, MAX_DUTY); // Clamp to duty cycle

  dutyWW += (MagiekonstanteCycle * errorWW);  // Simple I regulator for WW current
  dutyWW = CLAMP(dutyWW, MIN_DUTY, MAX_DUTY); // Clamp to duty cycle

  set_pwm(HRTIM_TIMERINDEX_TIMER_D, dutyCW);  // Update CW duty cycle
  set_pwm(HRTIM_TIMERINDEX_TIMER_C, dutyWW);  // Update WW duty cycle
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

static void init_RT(void) {
  /* Configure the RT9466, set currents to maximum */
  configure_RT(CHG_CTRL2, IINLIM_MASK);
  configure_RT(CHG_CTRL3, SET_ILIM_3A);
}

static void enable_OTG(void) {
  configure_RT(CHG_CTRL16, DISABLE_UUG);
  configure_RT(CHG_CTRL1, ENABLE_OTG_MASK);
}

static void configure_RT(uint8_t _register, uint8_t _mask) {
  uint8_t _tmp_data[2] = {_register, _mask};
  HAL_I2C_Master_Transmit(&hi2c1, RT_ADDRESS, _tmp_data, sizeof(_tmp_data), 500);
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

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue      = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType       = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource    = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider   = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider  = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider  = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection  = RCC_PERIPHCLK_HRTIM1 | RCC_PERIPHCLK_USART1
                                        | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection  = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection   = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection    = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Hrtim1ClockSelection  = RCC_HRTIM1CLK_PLLCLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

static void ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

  hadc1.Instance = ADC1;

  /* std ADC config */
  hadc1.Init.ClockPrescaler         = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution             = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign              = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode           = DISABLE;
  hadc1.Init.ContinuousConvMode     = DISABLE;
  hadc1.Init.DiscontinuousConvMode  = DISABLE;
  hadc1.Init.ExternalTrigConvEdge   = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv       = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign              = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion        = 1;
  hadc1.Init.NbrOfDiscConversion    = 1;
  hadc1.Init.DMAContinuousRequests  = DISABLE;
  hadc1.Init.EOCSelection           = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait       = DISABLE;
  hadc1.Init.Overrun                = ADC_OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc1);

  /* Disable DMA mode */
  multimode.DMAAccessMode     = ADC_DMAACCESSMODE_DISABLED;
  multimode.Mode              = ADC_MODE_INDEPENDENT;
  multimode.TwoSamplingDelay  = ADC_TWOSAMPLINGDELAY_1CYCLE;
  HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);

  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  /* Run the ADC calibration in single-ended mode */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  /* Enable End of Injected Conversion interrupt */
  //__HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_EOC);
  HAL_ADC_Start(&hadc1);
}

static void ADC2_Init(void)
{
  ADC_MultiModeTypeDef MultiModeConfig;
  ADC_InjectionConfTypeDef InjectionConfig;

  hadc2.Instance = ADC2;

  /* std ADC config */
  hadc2.Init.ClockPrescaler         = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution             = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign              = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode           = ENABLE;
  hadc2.Init.ContinuousConvMode     = DISABLE;
  hadc2.Init.DiscontinuousConvMode  = DISABLE;
  hadc2.Init.ExternalTrigConvEdge   = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv       = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign              = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion        = 1;
  hadc2.Init.NbrOfDiscConversion    = 1;
  hadc2.Init.DMAContinuousRequests  = DISABLE;
  hadc2.Init.EOCSelection           = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait       = DISABLE;
  hadc2.Init.Overrun                = ADC_OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc2);

  /* Disable DMA mode */
  MultiModeConfig.DMAAccessMode     = ADC_DMAACCESSMODE_DISABLED;
  MultiModeConfig.Mode              = ADC_MODE_INDEPENDENT;
  MultiModeConfig.TwoSamplingDelay  = ADC_TWOSAMPLINGDELAY_1CYCLE;
  HAL_ADCEx_MultiModeConfigChannel(&hadc2, &MultiModeConfig);

  /* Discontinuous injected mode: 1st injected conversion for NTC on Ch12 */
  InjectionConfig.InjectedChannel               = ADC_CHANNEL_12;
  InjectionConfig.InjectedRank                  = ADC_INJECTED_RANK_1;
  InjectionConfig.InjectedSamplingTime          = ADC_SAMPLETIME_601CYCLES_5;
  InjectionConfig.InjectedSingleDiff            = ADC_SINGLE_ENDED;
  InjectionConfig.InjectedOffsetNumber          = ADC_OFFSET_NONE;
  InjectionConfig.InjectedOffset                = 0;
  InjectionConfig.InjectedNbrOfConversion       = 4;
  InjectionConfig.InjectedDiscontinuousConvMode = DISABLE;
  InjectionConfig.AutoInjectedConv              = DISABLE;
  InjectionConfig.QueueInjectedContext          = DISABLE;
  InjectionConfig.ExternalTrigInjecConv         = ADC_EXTERNALTRIGINJECCONV_HRTIM_TRG2;
  InjectionConfig.ExternalTrigInjecConvEdge     = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  HAL_ADCEx_InjectedConfigChannel(&hadc2, &InjectionConfig);

  /* Configure the 2nd injected conversion for ICW on Ch1 */
  InjectionConfig.InjectedChannel       = ADC_CHANNEL_1;
  InjectionConfig.InjectedRank          = ADC_INJECTED_RANK_2;
  InjectionConfig.InjectedSamplingTime  = ADC_SAMPLETIME_601CYCLES_5;
  HAL_ADCEx_InjectedConfigChannel(&hadc2, &InjectionConfig);

  /* Configure the 2nd injected conversion for IWW on Ch2 */
  InjectionConfig.InjectedChannel       = ADC_CHANNEL_2;
  InjectionConfig.InjectedRank          = ADC_INJECTED_RANK_3;
  InjectionConfig.InjectedSamplingTime  = ADC_SAMPLETIME_601CYCLES_5;
  HAL_ADCEx_InjectedConfigChannel(&hadc2, &InjectionConfig);

  /* Configure the 2nd injected conversion for VBAT on Ch3 */
  InjectionConfig.InjectedChannel       = ADC_CHANNEL_3;
  InjectionConfig.InjectedRank          = ADC_INJECTED_RANK_4;
  InjectionConfig.InjectedSamplingTime  = ADC_SAMPLETIME_601CYCLES_5;
  HAL_ADCEx_InjectedConfigChannel(&hadc2, &InjectionConfig);

  /* Run the ADC calibration in single-ended mode */
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  /* Start ADC2 Injected Conversions */
  HAL_ADCEx_InjectedStart(&hadc2);

  /* Enable End of Injected Conversion interrupt */
  __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
}

static void COMP2_Init(void)
{

  hcomp2.Instance = COMP2;

  /* CMP2 config, input ICW and DAC1 channel 2, output HRTIM fault line 1 */
  hcomp2.Init.InvertingInput    = COMP_INVERTINGINPUT_DAC1_CH2;
  hcomp2.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp2.Init.Output            = HRTIM_FAULT_1;
  hcomp2.Init.OutputPol         = COMP_OUTPUTPOL_INVERTED;
  hcomp2.Init.BlankingSrce      = COMP_BLANKINGSRCE_NONE;
  hcomp2.Init.TriggerMode       = COMP_TRIGGERMODE_NONE;
  HAL_COMP_Init(&hcomp2);
}

static void COMP4_Init(void)
{

  hcomp4.Instance = COMP4;

  /* CMP4 config, input IWW and DAC1 channel 2, output HRTIM fault line 1 */
  hcomp4.Init.InvertingInput    = COMP_INVERTINGINPUT_DAC1_CH2;
  hcomp4.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp4.Init.Output            = HRTIM_FAULT_1;
  hcomp4.Init.OutputPol         = COMP_OUTPUTPOL_INVERTED;
  hcomp4.Init.BlankingSrce      = COMP_BLANKINGSRCE_NONE;
  hcomp4.Init.TriggerMode       = COMP_TRIGGERMODE_NONE;
  HAL_COMP_Init(&hcomp4);
}

static void COMP6_Init(void)
{

  hcomp6.Instance = COMP6;

  /* CMP6 config, input VREG and DAC2 channel 1, output HRTIM fault line 1 */
  hcomp6.Init.InvertingInput    = COMP_INVERTINGINPUT_DAC2_CH1;
  hcomp6.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp6.Init.Output            = HRTIM_FAULT_1;
  hcomp6.Init.OutputPol         = COMP_OUTPUTPOL_INVERTED;
  hcomp6.Init.BlankingSrce      = COMP_BLANKINGSRCE_NONE;
  hcomp6.Init.TriggerMode       = COMP_TRIGGERMODE_NONE;
  HAL_COMP_Init(&hcomp6);
}

static void DAC1_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

  /* std DAC config, no changes needed */
  hdac1.Instance = DAC1;
  HAL_DAC_Init(&hdac1);

  /* DAC not routed to output */
  sConfig.DAC_Trigger       = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputSwitch  = DAC_OUTPUTSWITCH_DISABLE;
  HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2);
}

static void DAC2_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

  /* std DAC config, no changes needed */
  hdac2.Instance = DAC2;
  HAL_DAC_Init(&hdac2);

  /* DAC not routed to output */
  sConfig.DAC_Trigger       = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputSwitch  = DAC_OUTPUTSWITCH_DISABLE;
  HAL_DAC_ConfigChannel(&hdac2, &sConfig, DAC_CHANNEL_1);
}

static void HRTIM1_Init(void)
{

  HRTIM_FaultCfgTypeDef pFaultCfg;
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg;
  HRTIM_TimerCfgTypeDef pTimerCfg;
  HRTIM_OutputCfgTypeDef pOutputCfg;
  HRTIM_ADCTriggerCfgTypeDef adc_trigger_config;
  HRTIM_CompareCfgTypeDef compare_config;

  hhrtim1.Instance = HRTIM1;

  /* HRTIM initialised with no Interrupt and synchronisation */
  hhrtim1.Init.HRTIMInterruptResquests  = HRTIM_IT_NONE;
  hhrtim1.Init.SyncOptions              = HRTIM_SYNCOPTION_NONE;
  HAL_HRTIM_Init(&hhrtim1);

  HAL_HRTIM_FaultPrescalerConfig(&hhrtim1, HRTIM_FAULTPRESCALER_DIV1);

  /* Select internal fault source (FLT_1) */
  pFaultCfg.Source    = HRTIM_FAULTSOURCE_INTERNAL;
  pFaultCfg.Polarity  = HRTIM_FAULTPOLARITY_LOW;
  pFaultCfg.Filter    = HRTIM_FAULTFILTER_NONE;
  pFaultCfg.Lock      = HRTIM_FAULTLOCK_READWRITE;
  HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_1, &pFaultCfg);

  HAL_HRTIM_FaultModeCtl(&hhrtim1, HRTIM_FAULT_1, HRTIM_FAULTMODECTL_ENABLED);

  /* Set the frequency and period */
  pTimeBaseCfg.Period             = HRTIM_PERIOD;
  pTimeBaseCfg.RepetitionCounter  = REG_CNT;
  pTimeBaseCfg.PrescalerRatio     = HRTIM_PRESCALERRATIO_MUL32;
  pTimeBaseCfg.Mode               = HRTIM_MODE_CONTINUOUS;
  HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg);

  /* main timer config, mostly std. with no DMA */
  pTimerCfg.DMARequests           = HRTIM_MASTER_DMA_NONE;
  pTimerCfg.DMASrcAddress         = 0x0000;
  pTimerCfg.DMADstAddress         = 0x0000;
  pTimerCfg.DMASize               = 0x0;
  pTimerCfg.HalfModeEnable        = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.StartOnSync           = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync           = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro            = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable         = HRTIM_PRELOAD_ENABLED;
  pTimerCfg.UpdateGating          = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode             = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate      = HRTIM_UPDATEONREPETITION_ENABLED;
  pTimerCfg.ResetUpdate           = HRTIM_TIMUPDATEONRESET_DISABLED;
  pTimerCfg.InterruptRequests     = HRTIM_TIM_IT_REP;
  pTimerCfg.PushPull              = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable           = HRTIM_TIMFAULTENABLE_FAULT1;
  pTimerCfg.FaultLock             = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion     = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.DelayedProtectionMode |= HRTIM_TIMER_D_E_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger         = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger          = HRTIM_TIMRESETTRIGGER_NONE;

  HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimerCfg);

  HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &pTimerCfg);

  /* set output and fault behavioral for HRTIM Timer C & D */
  pOutputCfg.Polarity               = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource              = HRTIM_OUTPUTSET_TIMPER;
  pOutputCfg.ResetSource            = HRTIM_OUTPUTRESET_TIMCMP1;
  pOutputCfg.IdleMode               = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel              = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel             = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable      = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed  = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;

  HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_OUTPUT_TC1, &pOutputCfg);

  HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_OUTPUT_TD1, &pOutputCfg);

  HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_OUTPUT_TC2, &pOutputCfg);

  HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_OUTPUT_TD2, &pOutputCfg);

  /* settings for the injected ADC */
  compare_config.AutoDelayedMode    = HRTIM_AUTODELAYEDMODE_REGULAR;
  compare_config.AutoDelayedTimeout = 0;
  compare_config.CompareValue       = HRTIM_PERIOD / 10 * 8.5;
  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_2, &compare_config);

  adc_trigger_config.Trigger      = HRTIM_ADCTRIGGEREVENT24_TIMERC_CMP2;
  adc_trigger_config.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_C;
  HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_2, &adc_trigger_config);

  /* std. config for Timer C & D */
  HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimeBaseCfg);

  HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &pTimeBaseCfg);

  HAL_HRTIM_MspPostInit(&hhrtim1);

}

static void I2C1_Init(void)
{

  hi2c1.Instance = I2C1;

  /* I2C Master config */
  hi2c1.Init.Timing           = 0x2000090E;
  hi2c1.Init.OwnAddress1      = 0;
  hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2      = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

  HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);

  HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);

}

static void TSC_Init(void)
{

  htscs.Instance = TSC;

  htscs.Init.CTPulseHighLength        = TSC_CTPH_1CYCLE;
  htscs.Init.CTPulseLowLength         = TSC_CTPL_1CYCLE;
  htscs.Init.SpreadSpectrum           = DISABLE;
  htscs.Init.SpreadSpectrumDeviation  = 1;
  htscs.Init.SpreadSpectrumPrescaler  = TSC_SS_PRESC_DIV1;
  htscs.Init.PulseGeneratorPrescaler  = TSC_PG_PRESC_DIV64;
  htscs.Init.MaxCountValue            = TSC_MCV_8191;
  htscs.Init.IODefaultMode            = TSC_IODEF_OUT_PP_LOW;
  htscs.Init.SynchroPinPolarity       = TSC_SYNC_POLARITY_FALLING;
  htscs.Init.AcquisitionMode          = TSC_ACQ_MODE_NORMAL;
  htscs.Init.MaxCountInterrupt        = DISABLE;
  htscs.Init.ChannelIOs               = 0;
  htscs.Init.SamplingIOs              = 0;
  HAL_TSC_Init(&htscs);

  htscb.Instance = TSC;

  htscb.Init.CTPulseHighLength        = TSC_CTPH_1CYCLE;
  htscb.Init.CTPulseLowLength         = TSC_CTPL_1CYCLE;
  htscb.Init.SpreadSpectrum           = DISABLE;
  htscb.Init.SpreadSpectrumDeviation  = 1;
  htscb.Init.SpreadSpectrumPrescaler  = TSC_SS_PRESC_DIV1;
  htscb.Init.PulseGeneratorPrescaler  = TSC_PG_PRESC_DIV64;
  htscb.Init.MaxCountValue            = TSC_MCV_8191;
  htscb.Init.IODefaultMode            = TSC_IODEF_OUT_PP_LOW;
  htscb.Init.SynchroPinPolarity       = TSC_SYNC_POLARITY_FALLING;
  htscb.Init.AcquisitionMode          = TSC_ACQ_MODE_NORMAL;
  htscb.Init.MaxCountInterrupt        = DISABLE;
  htscb.Init.ChannelIOs               = 0;
  htscb.Init.SamplingIOs              = 0;
  HAL_TSC_Init(&htscb);

  IoConfigs.ChannelIOs  = TSC_GROUP1_IO1; /* Start with the first channel */
  IoConfigs.SamplingIOs = TSC_GROUP1_IO4;
  IoConfigs.ShieldIOs   = 0;
  HAL_TSC_IOConfig(&htscs, &IoConfigs);

  IoConfigb.ChannelIOs  = TSC_GROUP5_IO2; /* Start with the first channel */
  IoConfigb.SamplingIOs = TSC_GROUP5_IO1;
  IoConfigb.ShieldIOs   = 0;
  HAL_TSC_IOConfig(&htscb, &IoConfigb);
}

static void USART1_UART_Init(void)
{

  huart1.Instance = USART1;

  /* UART master config */
  huart1.Init.BaudRate    = 115200;
  huart1.Init.WordLength  = UART_WORDLENGTH_8B;
  huart1.Init.StopBits    = UART_STOPBITS_1;
  huart1.Init.Parity      = UART_PARITY_NONE;
  huart1.Init.Mode        = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl   = UART_HWCONTROL_NONE;
  HAL_UART_Init(&huart1);
}

static void DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 6);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 7);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

static void GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable GPIO Bank clocks */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Set output state to LOW */
  HAL_GPIO_WritePin(GPIOA, LED_Brightness | LED_Color | LED_Power, GPIO_PIN_RESET);

  /* Change pin mode to output */
  GPIO_InitStruct.Pin   = LED_Brightness | LED_Color | LED_Power;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void start_HRTIM1(void) {
  /* Enable HRTIM timers */
  __HAL_HRTIM_ENABLE(&hhrtim1, HRTIM_TIMERID_MASTER);
  __HAL_HRTIM_ENABLE(&hhrtim1, HRTIM_TIMERID_TIMER_C);
  __HAL_HRTIM_ENABLE(&hhrtim1, HRTIM_TIMERID_TIMER_D);

  /* Enable outputs */
  HRTIM1->sCommonRegs.OENR = HRTIM_OENR_TD1OEN;
  HRTIM1->sCommonRegs.OENR = HRTIM_OENR_TD2OEN;
  HRTIM1->sCommonRegs.OENR = HRTIM_OENR_TC1OEN;
  HRTIM1->sCommonRegs.OENR = HRTIM_OENR_TC2OEN;
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
