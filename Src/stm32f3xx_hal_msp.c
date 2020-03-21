/**
  ******************************************************************************
  * File Name          : stm32f3xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization
  *                      and de-Initialization codes.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

extern DMA_HandleTypeDef hdma_adc1;

extern DMA_HandleTypeDef hdma_adc2;

extern DMA_HandleTypeDef hdma_i2c1_rx;

extern DMA_HandleTypeDef hdma_i2c1_tx;

extern DMA_HandleTypeDef hdma_usart1_rx;

extern DMA_HandleTypeDef hdma_usart1_tx;

extern DMA_HandleTypeDef hdma_tim16_ch1_up;

extern void _Error_Handler(char *, int);
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

static uint32_t HAL_RCC_ADC12_CLK_ENABLED = 0;

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if (HAL_RCC_ADC12_CLK_ENABLED == 1) {
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PB1     ------> ADC1_IN12
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  /*
  GPIO_InitTypeDef GPIO_InitStruct;
  if (hadc->Instance == ADC1)
  {
    HAL_RCC_ADC12_CLK_ENABLED++;
    if (HAL_RCC_ADC12_CLK_ENABLED == 1) {
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_NORMAL;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);

    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  }
  */
  else if (hadc->Instance == ADC2)
  {
    /* USER CODE BEGIN ADC2_MspInit 0 */

    /* USER CODE END ADC2_MspInit 0 */
    /* Peripheral clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if (HAL_RCC_ADC12_CLK_ENABLED == 1) {
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    /**ADC2 GPIO Configuration
    PA4     ------> ADC2_IN1
    PA5     ------> ADC2_IN2
    PA6     ------> ADC2_IN3
    PB2     ------> ADC2_IN12
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ADC2 DMA Init */
    /* ADC2 Init */
    hdma_adc2.Instance = DMA1_Channel2;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode = DMA_NORMAL;
    hdma_adc2.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc2);

    /* USER CODE BEGIN ADC2_MspInit 1 */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    /* USER CODE END ADC2_MspInit 1 */
  }

}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{

  if (hadc->Instance == ADC1)
  {
    /* USER CODE BEGIN ADC1_MspDeInit 0 */

    /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    /* Be sure that all peripheral instances that share the same clock need to be disabled */
    /**  HAL_RCC_ADC12_CLK_ENABLED--;
    *  if(HAL_RCC_ADC12_CLK_ENABLED==0){
    *    __HAL_RCC_ADC12_CLK_DISABLE();
    **/

    /**ADC1 GPIO Configuration
    PB1     ------> ADC1_IN12
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);
    /* USER CODE BEGIN ADC1_MspDeInit 1 */

    /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if (hadc->Instance == ADC2)
  {
    /* USER CODE BEGIN ADC2_MspDeInit 0 */

    /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    /* Be sure that all peripheral instances that share the same clock need to be disabled */
    /**  HAL_RCC_ADC12_CLK_ENABLED--;
    *  if(HAL_RCC_ADC12_CLK_ENABLED==0){
    *    __HAL_RCC_ADC12_CLK_DISABLE();
    **/

    /**ADC2 GPIO Configuration
    PA4     ------> ADC2_IN1
    PA5     ------> ADC2_IN2
    PA6     ------> ADC2_IN3
    PB2     ------> ADC2_IN12
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4 | GPIO_PIN_5);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_2);

    /* ADC2 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);
    /* USER CODE BEGIN ADC2_MspDeInit 1 */

    /* USER CODE END ADC2_MspDeInit 1 */
  }

}

void HAL_COMP_MspInit(COMP_HandleTypeDef* hcomp)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if (hcomp->Instance == COMP2)
  {
    /* USER CODE BEGIN COMP2_MspInit 0 */

    /* USER CODE END COMP2_MspInit 0 */

    /**COMP2 GPIO Configuration
    PA7     ------> COMP2_INP
    PA12     ------> COMP2_OUT
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_GPCOMP2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN COMP2_MspInit 1 */

    /* USER CODE END COMP2_MspInit 1 */
  }
  else if (hcomp->Instance == COMP4)
  {
    /* USER CODE BEGIN COMP4_MspInit 0 */

    /* USER CODE END COMP4_MspInit 0 */

    /**COMP4 GPIO Configuration
    PB0     ------> COMP4_INP
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN COMP4_MspInit 1 */

    /* USER CODE END COMP4_MspInit 1 */
  }
  else if (hcomp->Instance == COMP6)
  {
    /* USER CODE BEGIN COMP6_MspInit 0 */

    /* USER CODE END COMP6_MspInit 0 */

    /**COMP6 GPIO Configuration
    PB11     ------> COMP6_INP
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN COMP6_MspInit 1 */

    /* USER CODE END COMP6_MspInit 1 */
  }

}

void HAL_COMP_MspDeInit(COMP_HandleTypeDef* hcomp)
{

  if (hcomp->Instance == COMP2)
  {
    /* USER CODE BEGIN COMP2_MspDeInit 0 */

    /* USER CODE END COMP2_MspDeInit 0 */

    /**COMP2 GPIO Configuration
    PA7     ------> COMP2_INP
    PA12     ------> COMP2_OUT
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7 | GPIO_PIN_12);

    /* USER CODE BEGIN COMP2_MspDeInit 1 */

    /* USER CODE END COMP2_MspDeInit 1 */
  }
  else if (hcomp->Instance == COMP4)
  {
    /* USER CODE BEGIN COMP4_MspDeInit 0 */

    /* USER CODE END COMP4_MspDeInit 0 */

    /**COMP4 GPIO Configuration
    PB0     ------> COMP4_INP
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0);

    /* USER CODE BEGIN COMP4_MspDeInit 1 */

    /* USER CODE END COMP4_MspDeInit 1 */
  }
  else if (hcomp->Instance == COMP6)
  {
    /* USER CODE BEGIN COMP6_MspDeInit 0 */

    /* USER CODE END COMP6_MspDeInit 0 */

    /**COMP6 GPIO Configuration
    PB11     ------> COMP6_INP
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);

    /* USER CODE BEGIN COMP6_MspDeInit 1 */

    /* USER CODE END COMP6_MspDeInit 1 */
  }

}

void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac)
{

  if (hdac->Instance == DAC1)
  {
    /* USER CODE BEGIN DAC1_MspInit 0 */

    /* USER CODE END DAC1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_DAC1_CLK_ENABLE();
    /* USER CODE BEGIN DAC1_MspInit 1 */

    /* USER CODE END DAC1_MspInit 1 */
  }
  else if (hdac->Instance == DAC2)
  {
    /* USER CODE BEGIN DAC2_MspInit 0 */

    /* USER CODE END DAC2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_DAC2_CLK_ENABLE();
    /* USER CODE BEGIN DAC2_MspInit 1 */

    /* USER CODE END DAC2_MspInit 1 */
  }

}

void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac)
{

  if (hdac->Instance == DAC1)
  {
    /* USER CODE BEGIN DAC1_MspDeInit 0 */

    /* USER CODE END DAC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DAC1_CLK_DISABLE();
    /* USER CODE BEGIN DAC1_MspDeInit 1 */

    /* USER CODE END DAC1_MspDeInit 1 */
  }
  else if (hdac->Instance == DAC2)
  {
    /* USER CODE BEGIN DAC2_MspDeInit 0 */

    /* USER CODE END DAC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DAC2_CLK_DISABLE();
    /* USER CODE BEGIN DAC2_MspDeInit 1 */

    /* USER CODE END DAC2_MspDeInit 1 */
  }

}

void HAL_HRTIM_MspInit(HRTIM_HandleTypeDef* hhrtim)
{

  if (hhrtim->Instance == HRTIM1)
  {
    /* USER CODE BEGIN HRTIM1_MspInit 0 */

    /* USER CODE END HRTIM1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_HRTIM1_CLK_ENABLE();
    /* USER CODE BEGIN HRTIM1_MspInit 1 */

    /* USER CODE END HRTIM1_MspInit 1 */
  }

}

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef* hhrtim)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if (hhrtim->Instance == HRTIM1)
  {
    /* USER CODE BEGIN HRTIM1_MspPostInit 0 */

    /* USER CODE END HRTIM1_MspPostInit 0 */

    /**HRTIM1 GPIO Configuration
    PB12     ------> HRTIM1_CHC1
    PB14     ------> HRTIM1_CHD1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN HRTIM1_MspPostInit 1 */

    /* USER CODE END HRTIM1_MspPostInit 1 */
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
  else if(htim_base->Instance==TIM15)
  {
  /* USER CODE BEGIN TIM15_MspInit 0 */

  /* USER CODE END TIM15_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM15_CLK_ENABLE();
    /* TIM15 interrupt Init */

  /* USER CODE BEGIN TIM15_MspInit 1 */

  /* USER CODE END TIM15_MspInit 1 */
  }
  else if(htim_base->Instance==TIM16)
  {
  /* USER CODE BEGIN TIM16_MspInit 0 */

  /* USER CODE END TIM16_MspInit 0 */
  /* Peripheral clock enable */
  __HAL_RCC_TIM16_CLK_ENABLE();
  hdma_tim16_ch1_up.Instance = DMA1_Channel3;
  hdma_tim16_ch1_up.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tim16_ch1_up.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim16_ch1_up.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim16_ch1_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_tim16_ch1_up.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_tim16_ch1_up.Init.Mode = DMA_CIRCULAR;
  hdma_tim16_ch1_up.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  HAL_DMA_Init(&hdma_tim16_ch1_up);

  /* Several peripheral DMA handle pointers point to the same DMA handle.
   Be aware that there is only one channel to perform all the requested DMAs. */
  //__HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_CC1],hdma_tim16_ch1_up);
  __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_CC1],hdma_tim16_ch1_up);
  /* USER CODE BEGIN TIM16_MspInit 1 */

  /* USER CODE END TIM16_MspInit 1 */
  }
}

void HAL_HRTIM_MspDeInit(HRTIM_HandleTypeDef* hhrtim)
{

  if (hhrtim->Instance == HRTIM1)
  {
    /* USER CODE BEGIN HRTIM1_MspDeInit 0 */

    /* USER CODE END HRTIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_HRTIM1_CLK_DISABLE();
    /* USER CODE BEGIN HRTIM1_MspDeInit 1 */

    /* USER CODE END HRTIM1_MspDeInit 1 */
  }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if (hi2c->Instance == I2C1)
  {
    /* USER CODE BEGIN I2C1_MspInit 0 */

    /* USER CODE END I2C1_MspInit 0 */

    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* I2C1 DMA Init */
    /* I2C1_RX Init */
    hdma_i2c1_rx.Instance = DMA1_Channel7;
    hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_DMA_REMAP_CHANNEL_ENABLE(HAL_REMAPDMA_I2C1_RX_DMA1_CH3);

    __HAL_LINKDMA(hi2c, hdmarx, hdma_i2c1_rx);

    /* I2C1_TX Init */
    hdma_i2c1_tx.Instance = DMA1_Channel6;
    hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_i2c1_tx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(hi2c, hdmatx, hdma_i2c1_tx);

    /* USER CODE BEGIN I2C1_MspInit 1 */

    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);

    /* USER CODE END I2C1_MspInit 1 */
  }

}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{

  if (htim_pwm->Instance == TIM1)
  {
    /* USER CODE BEGIN TIM2_MspInit 0 */

    /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    //__HAL_RCC_TIM1_CLK_ENABLE();
    /* USER CODE BEGIN TIM2_MspInit 1 */

    /* USER CODE END TIM2_MspInit 1 */
  }

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */

    /**TIM1 GPIO Configuration
    PA11     ------> TIM1_CH4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }
  else if(htim->Instance==TIM16)
  {
  /* USER CODE BEGIN TIM16_MspPostInit 0 */

  /* USER CODE END TIM16_MspPostInit 0 */

    //__HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM16 GPIO Configuration
    PA6     ------> TIM16_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM16;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM16_MspPostInit 1 */

  /* USER CODE END TIM16_MspPostInit 1 */
  }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{

  if (htim_pwm->Instance == TIM1)
  {
    /* USER CODE BEGIN TIM2_MspDeInit 0 */

    /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();
    /* USER CODE BEGIN TIM2_MspDeInit 1 */

    /* USER CODE END TIM2_MspDeInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{

  if (hi2c->Instance == I2C1)
  {
    /* USER CODE BEGIN I2C1_MspDeInit 0 */

    /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);

    /* I2C1 DMA DeInit */
    HAL_DMA_DeInit(hi2c->hdmarx);
    HAL_DMA_DeInit(hi2c->hdmatx);
    /* USER CODE BEGIN I2C1_MspDeInit 1 */

    /* USER CODE END I2C1_MspDeInit 1 */
  }

}

void HAL_TSC_MspInit(TSC_HandleTypeDef* htsc)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if (htsc->Instance == TSC)
  {
    /* USER CODE BEGIN TSC_MspInit 0 */


    /* USER CODE END TSC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TSC_CLK_ENABLE();

    /**TSC GPIO Configuration
    PA0     ------> TSC_G1_IO1
    PA1     ------> TSC_G1_IO2
    PA2     ------> TSC_G1_IO3
    PA3     ------> TSC_G1_IO4
    PB3     ------> TSC_G5_IO1
    PB4     ------> TSC_G5_IO2
    PB6     ------> TSC_G5_IO3
    PB7     ------> TSC_G5_IO4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN TSC_MspInit 1 */
    HAL_NVIC_SetPriority(EXTI2_TSC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);

    /* USER CODE END TSC_MspInit 1 */
  }

}

void HAL_TSC_MspDeInit(TSC_HandleTypeDef* htsc)
{

  if (htsc->Instance == TSC)
  {
    /* USER CODE BEGIN TSC_MspDeInit 0 */

    /* USER CODE END TSC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TSC_CLK_DISABLE();

    /**TSC GPIO Configuration
    PA0     ------> TSC_G1_IO1
    PA1     ------> TSC_G1_IO2
    PA2     ------> TSC_G1_IO3
    PA3     ------> TSC_G1_IO4
    PB3     ------> TSC_G5_IO1
    PB4     ------> TSC_G5_IO2
    PB6     ------> TSC_G5_IO3
    PB7     ------> TSC_G5_IO4
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7);

    /* USER CODE BEGIN TSC_MspDeInit 1 */

    /* USER CODE END TSC_MspDeInit 1 */
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if (huart->Instance == USART1)
  {
    /* USER CODE BEGIN USART1_MspInit 0 */
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /* USER CODE END USART1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(huart, hdmarx, hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(huart, hdmatx, hdma_usart1_tx);

    /* USER CODE BEGIN USART1_MspInit 1 */

    /* USER CODE END USART1_MspInit 1 */
  }

}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  if (huart->Instance == USART1)
  {
    /* USER CODE BEGIN USART1_MspDeInit 0 */

    /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmarx);
    HAL_DMA_DeInit(huart->hdmatx);
    /* USER CODE BEGIN USART1_MspDeInit 1 */

    /* USER CODE END USART1_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
