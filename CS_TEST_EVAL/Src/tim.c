/**
  ******************************************************************************
  * File Name          : TIM.c
  * Date               : 26/02/2015 01:43:58
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "tim.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim4_up;
DMA_HandleTypeDef hdma_tim5_ch1;

/* TIM4 init function */
void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  HAL_TIM_OnePulse_Init(&htim4, TIM_OPMODE_SINGLE);

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  HAL_TIM_SlaveConfigSynchronization(&htim4, &sSlaveConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

}
/* TIM5 init function */
void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_IC_InitTypeDef sConfigIC;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim5);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig);

  HAL_TIM_IC_Init(&htim5);

  HAL_TIM_OnePulse_Init(&htim5, TIM_OPMODE_SINGLE);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1);

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  HAL_TIM_SlaveConfigSynchronization(&htim5, &sSlaveConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim_base->Instance==TIM4)
  {
    /* Peripheral clock enable */
    __TIM4_CLK_ENABLE();

    /* Peripheral DMA init*/
  
    hdma_tim4_up.Instance = DMA1_Stream6;
    hdma_tim4_up.Init.Channel = DMA_CHANNEL_2;
    hdma_tim4_up.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_tim4_up.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim4_up.Init.MemInc = DMA_MINC_DISABLE;
    hdma_tim4_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tim4_up.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_tim4_up.Init.Mode = DMA_NORMAL;
    hdma_tim4_up.Init.Priority = DMA_PRIORITY_LOW;
    hdma_tim4_up.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_tim4_up);

    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_UPDATE],hdma_tim4_up);

  }
  else if(htim_base->Instance==TIM5)
  {
    /* Peripheral clock enable */
    __TIM5_CLK_ENABLE();
  
    /**TIM5 GPIO Configuration    
    PH10     ------> TIM5_CH1 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    /* Peripheral DMA init*/
  
    hdma_tim5_ch1.Instance = DMA1_Stream2;
    hdma_tim5_ch1.Init.Channel = DMA_CHANNEL_6;
    hdma_tim5_ch1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_tim5_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim5_ch1.Init.MemInc = DMA_MINC_DISABLE;
    hdma_tim5_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tim5_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_tim5_ch1.Init.Mode = DMA_NORMAL;
    hdma_tim5_ch1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_tim5_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_tim5_ch1);

    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_CC1],hdma_tim5_ch1);

    /* Peripheral interrupt init*/
    /* Sets the priority grouping field */
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
    HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM4)
  {
    /* Peripheral clock disable */
    __TIM4_CLK_DISABLE();

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_UPDATE]);
  }
  else if(htim_base->Instance==TIM5)
  {
    /* Peripheral clock disable */
    __TIM5_CLK_DISABLE();
  
    /**TIM5 GPIO Configuration    
    PH10     ------> TIM5_CH1 
    */
    HAL_GPIO_DeInit(GPIOH, GPIO_PIN_10);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_CC1]);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIM5_IRQn);
  }
} 

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
