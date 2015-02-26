/**
  ******************************************************************************
  * @file    stm32f2xx_it.c
  * @date    26/02/2015 01:43:59
  * @brief   Interrupt Service Routines.
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
#include "stm32f2xx_hal.h"
#include "stm32f2xx.h"
#include "stm32f2xx_it.h"

/* External variables --------------------------------------------------------*/

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern SD_HandleTypeDef hsd;
extern TIM_HandleTypeDef htim5;
extern WWDG_HandleTypeDef hwwdg;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles SDIO global interrupt.
*/
void SDIO_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(SDIO_IRQn);
  HAL_SD_IRQHandler(&hsd);
}

/**
* @brief This function handles I2C2 event interrupt.
*/
void I2C2_EV_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(I2C2_EV_IRQn);
  HAL_I2C_EV_IRQHandler(&hi2c2);
}

/**
* @brief This function handles Window Watchdog interrupt.
*/
void WWDG_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(WWDG_IRQn);
  HAL_WWDG_IRQHandler(&hwwdg);
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/**
* @brief This function handles I2C1 event interrupt.
*/
void I2C1_EV_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
  HAL_I2C_EV_IRQHandler(&hi2c1);
}

/**
* @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
*/
void ADC_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(ADC_IRQn);
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
}

/**
* @brief This function handles TIM5 global interrupt.
*/
void TIM5_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(TIM5_IRQn);
  HAL_TIM_IRQHandler(&htim5);
}

/**
* @brief This function handles RCC global interrupt.
*/
void RCC_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(RCC_IRQn);
  /* USER CODE BEGIN 0 */

  /* USER CODE END 0 */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
