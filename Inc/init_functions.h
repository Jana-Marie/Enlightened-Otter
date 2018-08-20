#pragma once

#include "stm32f3xx_hal.h"
#include "init_functions.h"
#include "defines.h"

void SystemClock_Config(void);
void GPIO_Init(void);
void DMA_Init(void);
void ADC1_Init(void);
void ADC2_Init(void);
void COMP2_Init(void);
void COMP4_Init(void);
void COMP6_Init(void);
void HRTIM1_Init(void);
void TSC_Init(void);
void I2C1_Init(void);
void USART1_UART_Init(void);
void DAC1_Init(void);
void DAC2_Init(void);
void TIM2_Init(void);

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void RT_Init(void);
void configure_RT(uint8_t _register, uint8_t _mask);
void start_HRTIM1(void);

