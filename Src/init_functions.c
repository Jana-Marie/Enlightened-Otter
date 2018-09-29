/*
* Enlighted - Otter  -  Stm32f334 based mobile worklight.
* Copyright (C) 2018 Jan Henrik Hemsing
*
* This program is free software: you can redistribute it and / or modify it
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
* this program.  If not, see < http : //www.gnu.org/licenses/>.
*/

#include "init_functions.h"
#include "variables.h"

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

TIM_HandleTypeDef htim2;

TSC_HandleTypeDef htscs;        // Touch slider handle
TSC_IOConfigTypeDef IoConfigs;

TSC_HandleTypeDef htscb;        // Touch button handle
TSC_IOConfigTypeDef IoConfigb;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

extern struct reg_t r;

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

void ADC2_Init(void)
{
  ADC_MultiModeTypeDef MultiModeConfig;
  ADC_InjectionConfTypeDef InjectionConfig;

  hadc2.Instance = ADC2;

  // std ADC config
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

  // Disable DMA mode
  MultiModeConfig.DMAAccessMode     = ADC_DMAACCESSMODE_DISABLED;
  MultiModeConfig.Mode              = ADC_MODE_INDEPENDENT;
  MultiModeConfig.TwoSamplingDelay  = ADC_TWOSAMPLINGDELAY_1CYCLE;
  HAL_ADCEx_MultiModeConfigChannel(&hadc2, &MultiModeConfig);

  // Discontinuous injected mode: 1st injected conversion for NTC on Ch12
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

  // Configure the 2nd injected conversion for ICW on Ch1
  InjectionConfig.InjectedChannel       = ADC_CHANNEL_1;
  InjectionConfig.InjectedRank          = ADC_INJECTED_RANK_2;
  InjectionConfig.InjectedSamplingTime  = ADC_SAMPLETIME_601CYCLES_5;
  HAL_ADCEx_InjectedConfigChannel(&hadc2, &InjectionConfig);

  // Configure the 2nd injected conversion for IWW on Ch2
  InjectionConfig.InjectedChannel       = ADC_CHANNEL_2;
  InjectionConfig.InjectedRank          = ADC_INJECTED_RANK_3;
  InjectionConfig.InjectedSamplingTime  = ADC_SAMPLETIME_601CYCLES_5;
  HAL_ADCEx_InjectedConfigChannel(&hadc2, &InjectionConfig);

  // Configure the 2nd injected conversion for VBAT on Ch3
  InjectionConfig.InjectedChannel       = ADC_CHANNEL_3;
  InjectionConfig.InjectedRank          = ADC_INJECTED_RANK_4;
  InjectionConfig.InjectedSamplingTime  = ADC_SAMPLETIME_601CYCLES_5;
  HAL_ADCEx_InjectedConfigChannel(&hadc2, &InjectionConfig);

  // Run the ADC calibration in single-ended mode
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  // Start ADC2 Injected Conversions
  HAL_ADCEx_InjectedStart(&hadc2);

  // Enable End of Injected Conversion interrupt
  __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
}

void COMP2_Init(void)
{

  hcomp2.Instance = COMP2;

  // CMP2 config, input ICW and DAC1 channel 2, output HRTIM fault line 1
  hcomp2.Init.InvertingInput    = COMP_INVERTINGINPUT_DAC1_CH2;
  hcomp2.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp2.Init.Output            = HRTIM_FAULT_1;
  hcomp2.Init.OutputPol         = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.BlankingSrce      = COMP_BLANKINGSRCE_NONE;
  hcomp2.Init.TriggerMode       = COMP_TRIGGERMODE_NONE;
  HAL_COMP_Init(&hcomp2);
}

void COMP4_Init(void)
{

  hcomp4.Instance = COMP4;

  // CMP4 config, input IWW and DAC1 channel 2, output HRTIM fault line 1
  hcomp4.Init.InvertingInput    = COMP_INVERTINGINPUT_DAC1_CH2;
  hcomp4.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp4.Init.Output            = HRTIM_FAULT_2;
  hcomp4.Init.OutputPol         = COMP_OUTPUTPOL_NONINVERTED;
  hcomp4.Init.BlankingSrce      = COMP_BLANKINGSRCE_NONE;
  hcomp4.Init.TriggerMode       = COMP_TRIGGERMODE_NONE;
  HAL_COMP_Init(&hcomp4);
}

void COMP6_Init(void)
{

  hcomp6.Instance = COMP6;

  // CMP6 config, input VREG and DAC2 channel 1, output HRTIM fault line 1
  hcomp6.Init.InvertingInput    = COMP_INVERTINGINPUT_DAC2_CH1;
  hcomp6.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp6.Init.Output            = HRTIM_FAULT_3;
  hcomp6.Init.OutputPol         = COMP_OUTPUTPOL_NONINVERTED;
  hcomp6.Init.BlankingSrce      = COMP_BLANKINGSRCE_NONE;
  hcomp6.Init.TriggerMode       = COMP_TRIGGERMODE_NONE;
  HAL_COMP_Init(&hcomp6);
}

void DAC1_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

  // std DAC config, no changes needed
  hdac1.Instance = DAC1;
  HAL_DAC_Init(&hdac1);

  // DAC not routed to output
  sConfig.DAC_Trigger       = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputSwitch  = DAC_OUTPUTSWITCH_DISABLE;
  HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2);
}

void DAC2_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

  // std DAC config, no changes needed
  hdac2.Instance = DAC2;
  HAL_DAC_Init(&hdac2);

  // DAC not routed to output
  sConfig.DAC_Trigger       = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputSwitch  = DAC_OUTPUTSWITCH_DISABLE;
  HAL_DAC_ConfigChannel(&hdac2, &sConfig, DAC_CHANNEL_1);
}

void HRTIM1_Init(void)
{

  HRTIM_FaultCfgTypeDef pFaultCfg;
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg;
  HRTIM_TimerCfgTypeDef pTimerCfg;
  HRTIM_OutputCfgTypeDef pOutputCfg;
  HRTIM_ADCTriggerCfgTypeDef adc_trigger_config;
  HRTIM_CompareCfgTypeDef compare_config;

  hhrtim1.Instance = HRTIM1;

  // HRTIM initialised with no Interrupt and synchronisation
  hhrtim1.Init.HRTIMInterruptResquests  = HRTIM_IT_NONE;
  hhrtim1.Init.SyncOptions              = HRTIM_SYNCOPTION_NONE;
  HAL_HRTIM_Init(&hhrtim1);

  HAL_HRTIM_FaultPrescalerConfig(&hhrtim1, HRTIM_FAULTPRESCALER_DIV1);

  // Select internal fault source (FLT_1)
  pFaultCfg.Source    = HRTIM_FAULTSOURCE_INTERNAL;
  pFaultCfg.Polarity  = HRTIM_FAULTPOLARITY_HIGH;
  pFaultCfg.Filter    = HRTIM_FAULTFILTER_NONE;
  pFaultCfg.Lock      = HRTIM_FAULTLOCK_READWRITE;

  HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_1, &pFaultCfg);

  HAL_HRTIM_FaultModeCtl(&hhrtim1, HRTIM_FAULT_1, HRTIM_FAULTMODECTL_ENABLED);

  HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_2, &pFaultCfg);

  HAL_HRTIM_FaultModeCtl(&hhrtim1, HRTIM_FAULT_2, HRTIM_FAULTMODECTL_ENABLED);

  HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_3, &pFaultCfg);

  HAL_HRTIM_FaultModeCtl(&hhrtim1, HRTIM_FAULT_3, HRTIM_FAULTMODECTL_ENABLED);

  // Set the frequency and period
  pTimeBaseCfg.Period             = HRTIM_PERIOD;
  pTimeBaseCfg.RepetitionCounter  = REG_CNT;
  pTimeBaseCfg.PrescalerRatio     = HRTIM_PRESCALERRATIO_MUL32;
  pTimeBaseCfg.Mode               = HRTIM_MODE_CONTINUOUS;
  HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg);

  // main timer config, mostly std. with no DMA
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
  pTimerCfg.FaultEnable           = HRTIM_TIMFAULTENABLE_FAULT1 | HRTIM_TIMFAULTENABLE_FAULT2 |  HRTIM_TIMFAULTENABLE_FAULT3;
  pTimerCfg.FaultLock             = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion     = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.DelayedProtectionMode |= HRTIM_TIMER_D_E_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger         = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger          = HRTIM_TIMRESETTRIGGER_NONE;

  HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimerCfg);

  HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &pTimerCfg);

  // set output and fault behavioral for HRTIM Timer C & D
  pOutputCfg.Polarity               = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource              = HRTIM_OUTPUTSET_TIMPER;
  pOutputCfg.ResetSource            = HRTIM_OUTPUTRESET_TIMCMP1;
  pOutputCfg.IdleMode               = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel              = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel             = HRTIM_OUTPUTFAULTLEVEL_INACTIVE;
  pOutputCfg.ChopperModeEnable      = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed  = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;

  HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_OUTPUT_TC1, &pOutputCfg);

  HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_OUTPUT_TD1, &pOutputCfg);

  HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_OUTPUT_TC2, &pOutputCfg);

  HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_OUTPUT_TD2, &pOutputCfg);

  // settings for the injected ADC
  compare_config.AutoDelayedMode    = HRTIM_AUTODELAYEDMODE_REGULAR;
  compare_config.AutoDelayedTimeout = 0;
  compare_config.CompareValue       = HRTIM_PERIOD / 100.0f * 99.0f;
  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_2, &compare_config);

  adc_trigger_config.Trigger      = HRTIM_ADCTRIGGEREVENT24_TIMERC_CMP2;
  adc_trigger_config.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_C;
  HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_2, &adc_trigger_config);

  // std. config for Timer C & D
  HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimeBaseCfg);

  HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &pTimeBaseCfg);

  HAL_HRTIM_MspPostInit(&hhrtim1);

}

void TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1024;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_PWM_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = POWER_LED_BRIGHTNESS;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_MspPostInit(&htim2);
}

void I2C1_Init(void)
{

  hi2c1.Instance = I2C1;

  // I2C Master config
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

  HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 2);

}

void TSC_Init(void)
{

  htscs.Instance = TSC;

  // TSC slider common config
  htscs.Init.CTPulseHighLength        = TSC_CTPH_1CYCLE;
  htscs.Init.CTPulseLowLength         = TSC_CTPL_1CYCLE;
  htscs.Init.SpreadSpectrum           = ENABLE;
  htscs.Init.SpreadSpectrumDeviation  = 127;
  htscs.Init.SpreadSpectrumPrescaler  = TSC_SS_PRESC_DIV2;
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

  // TSC button common config
  htscb.Init.CTPulseHighLength        = TSC_CTPH_1CYCLE;
  htscb.Init.CTPulseLowLength         = TSC_CTPL_1CYCLE;
  htscb.Init.SpreadSpectrum           = ENABLE;
  htscb.Init.SpreadSpectrumDeviation  = 127;
  htscb.Init.SpreadSpectrumPrescaler  = TSC_SS_PRESC_DIV2;
  htscb.Init.PulseGeneratorPrescaler  = TSC_PG_PRESC_DIV64;
  htscb.Init.MaxCountValue            = TSC_MCV_8191;
  htscb.Init.IODefaultMode            = TSC_IODEF_OUT_PP_LOW;
  htscb.Init.SynchroPinPolarity       = TSC_SYNC_POLARITY_FALLING;
  htscb.Init.AcquisitionMode          = TSC_ACQ_MODE_NORMAL;
  htscb.Init.MaxCountInterrupt        = DISABLE;
  htscb.Init.ChannelIOs               = 0;
  htscb.Init.SamplingIOs              = 0;
  HAL_TSC_Init(&htscb);

  // TSC slider IO config
  IoConfigs.ChannelIOs  = TSC_GROUP1_IO1; // Start with the first channel
  IoConfigs.SamplingIOs = TSC_GROUP1_IO4;
  IoConfigs.ShieldIOs   = 0;
  HAL_TSC_IOConfig(&htscs, &IoConfigs);

  // TSC button IO config
  IoConfigb.ChannelIOs  = TSC_GROUP5_IO2; // Start with the first channel
  IoConfigb.SamplingIOs = TSC_GROUP5_IO1;
  IoConfigb.ShieldIOs   = 0;
  HAL_TSC_IOConfig(&htscb, &IoConfigb);
}

void USART1_UART_Init(void)
{

  huart1.Instance = USART1;

  // UART master config
  huart1.Init.BaudRate    = 115200;
  huart1.Init.WordLength  = UART_WORDLENGTH_8B;
  huart1.Init.StopBits    = UART_STOPBITS_1;
  huart1.Init.Parity      = UART_PARITY_NONE;
  huart1.Init.Mode        = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl   = UART_HWCONTROL_NONE;
  HAL_UART_Init(&huart1);
}

void DMA_Init(void)
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

void GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  // Enable GPIO Bank clocks
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Set output state to LOW
  HAL_GPIO_WritePin(GPIOA, LED_Brightness | LED_Color | LED_Power, GPIO_PIN_RESET);

  // Change pin mode to output
  GPIO_InitStruct.Pin   = LED_Brightness | LED_Color | LED_Power;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void start_HRTIM1(void) {

  // Clear all fault flags and reset target values, start_HRTIM1() clears also faults
  __HAL_HRTIM_CLEAR_IT(&hhrtim1, HRTIM_IT_FLT1);
  __HAL_HRTIM_TIMER_CLEAR_IT(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_IT_FLT1);
  __HAL_HRTIM_TIMER_CLEAR_IT(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_IT_FLT1);

  __HAL_HRTIM_CLEAR_IT(&hhrtim1, HRTIM_IT_FLT2);
  __HAL_HRTIM_TIMER_CLEAR_IT(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_IT_FLT2);
  __HAL_HRTIM_TIMER_CLEAR_IT(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_IT_FLT2);

  r.CW.target = 0.0f;
  r.CW.targetNoGamma = 0.0f;
  r.WW.target = 0.0f;
  r.WW.targetNoGamma = 0.0f;

  // Enable HRTIM timers
  __HAL_HRTIM_ENABLE(&hhrtim1, HRTIM_TIMERID_MASTER);
  __HAL_HRTIM_ENABLE(&hhrtim1, HRTIM_TIMERID_TIMER_C);
  __HAL_HRTIM_ENABLE(&hhrtim1, HRTIM_TIMERID_TIMER_D);

  // Enable outputs
  HRTIM1->sCommonRegs.OENR = HRTIM_OENR_TD1OEN;
  HRTIM1->sCommonRegs.OENR = HRTIM_OENR_TD2OEN;
  HRTIM1->sCommonRegs.OENR = HRTIM_OENR_TC1OEN;
  HRTIM1->sCommonRegs.OENR = HRTIM_OENR_TC2OEN;
}

void RT_Init(void) {
  // Configure the RT9466, set currents to maximum
  configure_RT(CHG_CTRL2, IINLIM_MASK);
  configure_RT(CHG_CTRL3, SET_ILIM_3A);
}

void configure_RT(uint8_t _register, uint8_t _mask) {
  uint8_t _tmp_data[2] = {_register, _mask};
  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
  HAL_I2C_Master_Transmit_DMA(&hi2c1, RT_ADDRESS, _tmp_data, 2);
  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
}