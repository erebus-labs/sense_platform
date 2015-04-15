/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 04/04/2015 22:33:50
  * Description        : Main program body
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
#include "ff.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h" /* defines SD_Driver as external */
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "sdio.h"
#include "usart.h"
#include "gpio.h"
#include "SENSORS.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE BEGIN Includes */
#define DEBUG
#define MAX_LEN 100
#define __USE_IRQ
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
uint8_t retSD;    /* Return value for SD */
char SD_Path[4];  /* SD logical drive path */
volatile uint32_t adc1;
volatile uint32_t I2C_value;

typedef struct t_settings{
	uint8_t ACCEL_SET;
	uint8_t ADC1_SET;
	uint8_t ADC2_SET;
	uint8_t ADC3_SET;
	uint8_t ADC4_SET;
}T_SET;

static uint8_t ACCEL_COUNTER;
static uint8_t ACCEL_SCHEDULE=10;

static uint8_t ADC1_COUNTER;
static uint8_t ADC1_SCHEDULE=0;

static uint8_t ADC2_COUNTER;
static uint8_t ADC2_SCHEDULE=30;

static uint8_t ADC3_COUNTER;
static uint8_t ADC3_SCHEDULE=60;

static uint8_t ADC4_COUNTER;
static uint8_t ADC4_SCHEDULE=255;

uint8_t data_to_read;

uint64_t SD_write_address=0;

static DMA_HandleTypeDef huart_rx;
static DMA_HandleTypeDef huart_tx;
static DMA_HandleTypeDef hdma_adc;
static DMA_HandleTypeDef hdma_i2c1;
static DMA_HandleTypeDef hdma_sdio;
/*
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;

extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;

extern DMA_HandleTypeDef hdma_sdio;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef huart_rx;
extern DMA_HandleTypeDef huart_tx;
extern DMA_HandleTypeDef hdma_i2c1;
*/
SD_HandleTypeDef SD_CARD; 
ADC_ChannelConfTypeDef sConfig;
__IO uint16_t uhADCxConvertedValue = 0;
volatile uint32_t ADC_TEMP;
volatile uint32_t STAMPER=0;

/*Variables for string receive over USART:*/
uint8_t rxBuffer = '\000';
uint8_t rxString[MAX_LEN];
uint8_t txBuffer[] = "SERIAL PRINT\n";
int rx_index=0;

/*Functions to read from sensors:*/
uint32_t READ_ADC(ADC_HandleTypeDef *);
uint32_t READ_I2C(I2C_HandleTypeDef *);

/*Peripheral functions:*/
void SystemClock_Config(void);
void GPIO_INIT(void);
void ADC_INIT(void);
void I2C_INIT(void);
void IRQ_INIT(void);
void USART_INIT(void);
void ARBITRATE(uint32_t);

void I2C_DEVICE_INIT(I2C_HandleTypeDef *, uint8_t dev_id);
void SET_RTC_ALARM(RTC_HandleTypeDef *, uint32_t);

void LIGHT_LEDS(uint32_t);
void HAL_DMA_XferCpltCallback(void);

int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  ADC_INIT();
  I2C_INIT();
  USART_INIT();
  GPIO_INIT();

  IRQ_INIT();
  //I2C_DEVICE_INIT(&hi2c1, ACCEL_ADDR);

  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SD_Path);

  /* Infinite loop */
  while (1)
  {     
	  __WFI();
  }
}
/******************************************************************************
* @brief  SystemClock_Config(void)
* @param  NONE
* @retval NONE
******************************************************************************/
void SystemClock_Config(void){

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 6;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 20;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK| RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}
/* USER CODE BEGIN 4 */
/******************************************************************************
* @brief  print(char string[])
* @param  string to print
* @retval NONE
******************************************************************************/
void print(uint8_t string[]){
	uint8_t index;
	for(index=0; index < sizeof(uint8_t); index++){
		//HAL_UART_Transmit_IT(&huart1, (uint8_t*)&string[index], 10);
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&string[index],1);	
	}	
	//HAL_UART_Transmit(&huart1, (uint8_t *)string, strlen(string), 1);
	//HAL_DMA_Start_IT(huart1.hdmatx, *(uint32_t*)string, (uint32_t)huart1.Instance->DR, sizeof(uint32_t*)); 
	//HAL_DMA_Start_IT(huart1->hdmatx, *(uint32_t*)tmp, (uint32_t)&huart->Instance->DR, Size);
}
/******************************************************************************
* @brief  ADC_INIT(void)
* @param  NONE
* @retval NONE
******************************************************************************/
void ADC_INIT(void){
	/**Configure the global features of the ADC*/
	__ADC1_CLK_ENABLE();

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION12b;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;//ENABLE DISABLE
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.NbrOfDiscConversion = 0;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;	
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;//ENABLE;
	hadc1.Init.EOCSelection = EOC_SINGLE_CONV; //DISABLE;//
	HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
	sConfig.Channel = ADC_CHANNEL_1; //changed from 0
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;//changed from 3 cycles
	sConfig.Offset = 0;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	
	hdma_adc.Instance = DMA2_Stream0;
	hdma_adc.Init.Channel = DMA_CHANNEL_0;
	hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_adc.Init.MemInc = DMA_MINC_DISABLE;
	hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_adc.Init.Mode = DMA_NORMAL;
	hdma_adc.Init.Priority = DMA_PRIORITY_LOW;
	hdma_adc.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&hdma_adc);
	
	__HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc);

	//HAL_DMA_Start_IT(&hdma_adc, (uint32_t)&hadc1.Instance->DR, (uint32_t)&uhADCxConvertedValue, sizeof(uint32_t));
	//HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&uhADCxConvertedValue, 1);
}

/******************************************************************************
* @brief  I2C_INIT(void)
* @param  NONE
* @retval NONE
******************************************************************************/
void I2C_INIT(void){
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;//0x3E;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;	
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
	
	HAL_I2C_Init(&hi2c1);
/*
	__DMA1_CLK_ENABLE();
	//I2C RX:
	hdma_i2c1.Instance = DMA1_Stream5;
	hdma_i2c1.Init.Channel = DMA_CHANNEL_1;
	hdma_i2c1.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_i2c1.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_i2c1.Init.MemInc = DMA_MINC_DISABLE;
	hdma_i2c1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_i2c1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_i2c1.Init.Mode = DMA_NORMAL;
	hdma_i2c1.Init.Priority = DMA_PRIORITY_LOW;
	hdma_i2c1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&hdma_i2c1);
	
	__HAL_LINKDMA(&hi2c1, hdmarx, hdma_i2c1);
	
	//I2C TX:
	hdma_i2c1.Instance = DMA1_Stream6;
	hdma_i2c1.Init.Channel = DMA_CHANNEL_1;
	hdma_i2c1.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_i2c1.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_i2c1.Init.MemInc = DMA_MINC_DISABLE;
	hdma_i2c1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_i2c1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_i2c1.Init.Mode = DMA_NORMAL;
	hdma_i2c1.Init.Priority = DMA_PRIORITY_LOW;
	hdma_i2c1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&hdma_i2c1);
	
	__HAL_LINKDMA(&hi2c1, hdmatx, hdma_i2c1);
	*/
	HAL_I2C_IsDeviceReady(&hi2c1, ACCEL_ADDR, 3, 10);
	
	HAL_I2C_Mem_Write_IT(&hi2c1, ACCEL_ADDR, ACCEL_POWER_CTRL, 2, (uint8_t *)ACCEL_WAKE, 2);
	HAL_I2C_Mem_Write_IT(&hi2c1, ACCEL_ADDR, ACCEL_DATA_FMT, 2, (uint8_t *)SELF_TEST_OFF, 2);
	HAL_I2C_Mem_Write_IT(&hi2c1, ACCEL_ADDR, ACCEL_FIFO_CTRL, 2, (uint8_t *)0x0, 2);
	
}
/******************************************************************************
* @brief  IRQ_INIT(void)
* @param  NONE
* @retval NONE
******************************************************************************/
void IRQ_INIT(void){
	/* Peripheral clock enable */
	__HAL_RCC_RTC_ENABLE();

	/* Peripheral interrupt init*/
	HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
	
	HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
	
	/**Enable ADC Interrupts:*/
	HAL_NVIC_SetPriority(ADC_IRQn, 0,2);
	HAL_NVIC_EnableIRQ(ADC_IRQn);
	
	/* ENABLE USART INTERRUPTS:*/
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);	
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	
	/*ENABLE I2C INTERRUPTS:*/
	HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
	
	/*ENABLE I2C DMA INTERRUPTS:*/
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 2);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 2);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	
	/*Enable ADC DMA Conversion:*/
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 2);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	
	/*Enable DMA Interrupts for RX:*/
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	
	/*Enable DMA Interrupts for TX:*/
	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}
/******************************************************************************
* @brief  GPIO_INIT(void)
* @param  NONE
* @retval NONE
******************************************************************************/
void GPIO_INIT(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	__GPIOA_CLK_ENABLE();
	__USART1_CLK_ENABLE();
	/**USART1 GPIO Configuration    
	PA9     ------> USART1_TX
	PA10     ------> USART1_RX 
	*/
	GPIO_InitStructure.Pin = GPIO_PIN_9|GPIO_PIN_10;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
	GPIO_InitStructure.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*Initialize ADC:*/
	__ADC1_CLK_ENABLE();
	/**ADC1 GPIO Configuration    
	PA0-WKUP     ------> ADC1_IN0
	PA1     ------> ADC1_IN1
	PA2     ------> ADC1_IN2
	PA4     ------> ADC1_IN4 
	*/
	GPIO_InitStructure.Pin = GPIO_PIN_1|GPIO_PIN_2| GPIO_PIN_3|GPIO_PIN_4;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	//GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*Initialize I2C1*/
	 __I2C1_CLK_ENABLE();
	/**I2C1 GPIO Configuration    
	PB6     ------> I2C1_SCL
	PB7     ------> I2C1_SDA 
	*/
	GPIO_InitStructure.Pin = GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
	GPIO_InitStructure.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
		
	/*Initialize GPIO for LEDs:*/
	GPIO_InitStructure.Pin = GPIO_PIN_10 | GPIO_PIN_9 | GPIO_PIN_8 | GPIO_PIN_7;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);
}
/******************************************************************************
* @brief  USART_INIT(void)
* @param  NONE
* @retval NONE
******************************************************************************/
void USART_INIT(void){	
	__USART1_CLK_ENABLE();
	__HAL_UART_ENABLE(&huart1);
	
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart1);
	
	/*Initialize DMA2 for USART:*/
	/*
	__DMA2_CLK_ENABLE();

	//USART DMA RX
	huart_rx.Instance = DMA2_Stream2;
	huart_rx.Init.Channel = DMA_CHANNEL_4;//BOTH THE SAME?...
	huart_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	huart_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	huart_rx.Init.MemInc = DMA_MINC_DISABLE;
	huart_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	huart_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	huart_rx.Init.Mode = DMA_CIRCULAR;
	huart_rx.Init.Priority = DMA_PRIORITY_HIGH;
	huart_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&huart_rx);
	
	__HAL_LINKDMA(&huart1, hdmarx, huart_rx);

	//USART DMA TX
	huart_tx.Instance = DMA2_Stream7;
	huart_tx.Init.Channel = DMA_CHANNEL_4;//TODO: CHECK THIS!
	huart_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	huart_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	huart_tx.Init.MemInc = DMA_MINC_DISABLE;
	huart_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	huart_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	huart_tx.Init.Mode = DMA_CIRCULAR;
	huart_tx.Init.Priority = DMA_PRIORITY_HIGH;
	huart_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&huart_tx);
	
	__HAL_LINKDMA(&huart1, hdmatx, huart_tx);
	*/
	//print(txBuffer);

	//print("Serial started!\n");
	//__HAL_UART_ENABLE_IT(&huart1, HAL_UART_STATE_READY);

	//HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxString, 1);
}/*END INIT USART*/

/******************************************************************************
* @brief  Alarm A callback.
* @param  hrtc: RTC handle
* @retval None
******************************************************************************/
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc){
	ARBITRATE(++STAMPER);
}
/******************************************************************************
* @brief  Alarm A callback.
* @param  hrtc: RTC handle
* @retval None
******************************************************************************/
void HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtc)
{
  /* NOTE : This function is defined in ..._hal_rtc.c, to not modify the
	library file it has been implemented here:
   */
	//print("I2C Sample from Alarm B");
	//READ_I2C(&hi2c1);
		
}

void ARBITRATE(uint32_t CRAMPER){
	
	if(ADC1_COUNTER == ADC1_SCHEDULE){
		HAL_ADC_Start_IT(&hadc1);//WORKS!
		ADC1_COUNTER=0;
		//HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&uhADCxConvertedValue, 1);//WORKS with DMA
	}
	else ++ADC1_COUNTER;
	
	if(ACCEL_COUNTER == ACCEL_SCHEDULE){
		data_to_read=0;
		ACCEL_COUNTER=0;
		HAL_I2C_IsDeviceReady(&hi2c1, ACCEL_ADDR, 3, 10);
		HAL_I2C_Mem_Read_IT(&hi2c1, ACCEL_ADDR , ACCEL_DATA_X1, 1, (uint8_t *)&data_to_read, 1);//I2C_MEMADD_SIZE_8BIT);
	}
	else ++ACCEL_COUNTER;
}



void HAL_DMA_XferCpltCallback(void){
	LIGHT_LEDS((uint32_t)data_to_read);
}
/******************************************************************************
* @abstract HAL_ADC_ConvCpltCallback function reads from specific ADC device
* @ref HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc1)
* @param ADC_HandleTypeDef * hadc1
* @retval NONE
******************************************************************************/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc){
	__IO uint16_t adc_val=0;//WORKS with IT
	adc_val = HAL_ADC_GetValue(&hadc1);//WORKS with IT
	LIGHT_LEDS(adc_val);//WORKS with IT
	//HAL_UART_Transmit(&huart1, (uint8_t *)&adc_val, sizeof(adc_val), 1);
	adc_val=(adc_val/100);
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)&adc_val, 1);//8);
	//HAL_SD_WriteBlocks_DMA(&sdio
}
/******************************************************************************
* @abstract READ_I2C function reads from I2C channel based on passed arg
TODO: Pass device ADDRESS? Pass HandleType?
* @ref READ_I2C(I2C_HandleTypeDef *I2C_device)
* @param I2C_device - WHICH I2C device to read from...
******************************************************************************/
uint32_t READ_I2C(I2C_HandleTypeDef * I2C_device){		
	//HAL_I2C_IsDeviceReady(&hi2c1, ACCEL_ADDR, PHY_FULLDUPLEX_100M, 10);
	data_to_read=0;
	HAL_I2C_Mem_Read_IT(&hi2c1, ACCEL_READ_ADDR, ACCEL_DATA_X0, sizeof(uint8_t), &data_to_read, sizeof(uint8_t)); 
	//HAL_I2C_Master_Receive_IT(&hi2c1, ACCEL_ADDR, &data_to_read, sizeof(data_to_read));
	//HAL_I2C_Mem_Read_IT(&hi2c1, ACCEL_ADDR, ACCEL_DATA_X0, sizeof(uint8_t), &data_to_read, sizeof(data_to_read));
	return I2C_value;
}
/******************************************************************************
* @abstract READ_ADC function reads from specific ADC channel
* @ref ADC_READ(ADC_HandleTypeDef * adc_num)
* @param ADC_HandleTypeDef * adc_num
* @retval ADC value read from channel
******************************************************************************/
uint32_t READ_ADC(ADC_HandleTypeDef * adc_num){	
	volatile uint32_t adc_value;
	//uint32_t * pass_to_SD;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);
	adc_value=0;
	adc_value = HAL_ADC_GetValue(&hadc1);
	//*pass_to_SD = adc_value;
	#ifdef DEBUG
		LIGHT_LEDS(adc_value);
	#endif
	print("Lighting LEDs");
	MX_RTC_Init();
	/*TODO: THIS IS THE DEFINITION OF A KLUDGE, SHOULD SET IRQ PRIORITY DIFFERENTLY
	OR SIMPLY RESET THE SINGLE TIMER, NOT THE ENTIRE ROUTINE, PLUS IT VERY WELL COULD F*CK UP THE
	RTC ITSELF, THUS RUINING OUR TIMERS ALTOGETHER!*/
	//HAL_SD_WriteBlocks_DMA(&hsd, pass_to_SD, SD_write_address, SDIO_DATABLOCK_SIZE_512B, 1);
	//HAL_SD_CheckWriteOperation(&hsd, 1);
return adc_value;
}
/******************************************************************************
* @brief  SD_SDIO_DMA_IRQHANDLER(void)
* @param  NONE
* @retval NONE
******************************************************************************/
void SD_SDIO_DMA_IRQHANDLER(void){
//	HAL_NVIC_ClearPendingIRQ(DMA2_Stream3_IRQn);
}
/******************************************************************************
* @brief  HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
* @param  UART_HandleTypeDef * huart
* @retval NONE
******************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//HAL_NVIC_ClearPendingIRQ(DMA2_Stream2_IRQn);	
	/*
	if(rxBuffer =='\n' ||rxBuffer == '\r'){
		print("User wrote: %s");
		print((char *)&rxBuffer);
	}
	else{
		rxString[rx_index] = rxBuffer;
		rx_index++;
		if(rx_index > MAX_LEN){
			
		}	
	}
	*/
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
}
/******************************************************************************
* @brief  HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart)
* @param  UART_HandleTypeDef * huart
* @retval NONE
******************************************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	//__HAL_UART_CLEAR_FLAG(huart, HAL_UART_STATE_ERROR);
	//HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_7);
}
/******************************************************************************
* @brief  HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
* @param  UART_HandleTypeDef * huart
* @retval NONE
******************************************************************************/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	__HAL_UART_CLEAR_FLAG(huart, HAL_UART_STATE_ERROR);
	HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
	//print("\n\nSERIAL ERROR!\n");
}
/******************************************************************************
* @brief  HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef * hi2c)
* @param  I2C_HandleTypeDef * hi2c
* @retval NONE
******************************************************************************/
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);
	LIGHT_LEDS(hi2c1.Instance->DR);
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)&data_to_read, 1);
}
/******************************************************************************
* @brief  HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef * hi2c)
* @param  I2C_HandleTypeDef * hi2c
* @retval NONE
******************************************************************************/
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef * hi2c){
	#ifdef DEBUG
		LIGHT_LEDS(hi2c1.Instance->DR);
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)&data_to_read, 1);
	#endif
}
/******************************************************************************
* @abstract SET_RTC_ALARM sets corresponding alarm based on passed args
TODO: Pass device ADDRESS? Pass HandleType?
* @ref SET_RTC_ALARM(RTC_HandleTypeDef *which_alarm)
* @param which_alarm - WHICH alarm to set, A or B really...
* @param alarm_time - 32 bit value packed as Alarm register format:
TODO: ADD ASCII DIAGRAM/ ILLUSTRATION
******************************************************************************/
void SET_RTC_ALARMS(RTC_HandleTypeDef * which_alarm, uint32_t alarm_time){
	/*Set Alarm-A and Alarm-B, TODO: SET REC CLOCK FROM DFU or HID mode, (programming)*/
	/* RTC.C starting on line: 77 sets ALARMA--->MASKS DEFINED in AN3371.pdf, P. 12, Table 5!*/
	/*        M3__M2__M1__M0_  
	EXAMPLE: | 1 | 1 | 1 | 0 | -> 0xD would set off the alarm every minute...0xF every second...crontab!
	AN3371.pdf P. 21, Table 11 shows RTC Timestamp based Interrupts Init , etc.
	*/
	//which_alarm.
    
	//HAL_RTC_SetAlarm(&hrtc, &which_alarm.AlarmA, FORMAT_BIN);
}

/******************************************************************************
* @abstract LIGHT_LEDS function lights corresponding LEDS based on passed arg
* @ref LIGHT_LEDS(uint32_t value)
* @param uint32_t value - determines which LED(s) to light, ADC debugging!
******************************************************************************/
void LIGHT_LEDS(uint32_t value){
	if(value<1024){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);	
	}
	else if(value>=1024&&value<2048){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
	}
	else if(value>=2048&&value<3072){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);	
	}
	else if(value>=3072&& value<4096){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);	
	}
	else{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);		
	}
}

/*
(#)Initialize the SDIO low level resources by implementing the HAL_SD_MspInit() API:
        (##) Enable the SDIO interface clock using __SDIO_CLK_ENABLE(); 
        (##) SDIO pins configuration for SD card
            (+++) Enable the clock for the SDIO GPIOs using the functions __GPIOx_CLK_ENABLE();   
            (+++) Configure these SDIO pins as alternate function pull-up using HAL_GPIO_Init()
                  and according to your pin assignment;
        (##) DMA Configuration if you need to use DMA process (HAL_SD_ReadBlocks_DMA()
             and HAL_SD_WriteBlocks_DMA() APIs).
            (+++) Enable the DMAx interface clock using __DMAx_CLK_ENABLE(); 
            (+++) Configure the DMA using the function HAL_DMA_Init() with predeclared and filled. 
        (##) NVIC configuration if you need to use interrupt process when using DMA transfer.
            (+++) Configure the SDIO and DMA interrupt priorities using functions
                  HAL_NVIC_SetPriority(); DMA priority is superior to SDIO's priority
            (+++) Enable the NVIC DMA and SDIO IRQs using function HAL_NVIC_EnableIRQ()
            (+++) SDIO interrupts are managed using the macros __HAL_SD_SDIO_ENABLE_IT() 
                  and __HAL_SD_SDIO_DISABLE_IT() inside the communication process.
            (+++) SDIO interrupts pending bits are managed using the macros __HAL_SD_SDIO_GET_IT()
                  and __HAL_SD_SDIO_CLEAR_IT()
    (#) At this stage, you can perform SD read/write/erase operations after SD card initialization  
*/
void SD_CARD_INIT(void){
	__SDIO_CLK_ENABLE();
	__HAL_SD_SDIO_ENABLE();
	__HAL_SD_SDIO_DMA_ENABLE();
	
	HAL_SD_CardInfoTypedef SD_info;
	
	SD_info.CardBlockSize = SDIO_DATABLOCK_SIZE_512B;
	//SD_info.CardCapacity = SDIO_
	
	HAL_SD_Init(&SD_CARD, &SD_info);
	
}

/******************************************************************************
* @abstract I2C_INIT function initializes I2C Accelerometer sensor
* @ref ADC_READ(I2C_HandleTypeDef * I2C_device, uint8_t dev_id)
* @param I2C_HandleTypeDef * I2C_device
* @retval NONE
******************************************************************************/
void I2C_DEVICE_INIT(I2C_HandleTypeDef * I2C_device, uint8_t dev_id){
	uint8_t * to_send;//, to_receive;
	uint8_t samples;
	uint8_t * X_ST;
	uint8_t * Y_ST;
	uint8_t * Z_ST;
	uint8_t * X_ST_ON;
	uint8_t * Y_ST_ON;
	uint8_t * Z_ST_ON;
	uint8_t * X_SAMPLES;
	uint8_t * Y_SAMPLES;
	uint8_t * Z_SAMPLES;
	uint8_t * X_SAMPLES_ON;
	uint8_t * Y_SAMPLES_ON;
	uint8_t * Z_SAMPLES_ON;

	switch(dev_id){
		case ACCEL_ADDR:
			to_send = ACCEL_RATE_100KHZ;/*SET ACCEL TO 100KHZ CLOCK FREQUENCY, SAME AS IS I2C SET TO*/
			HAL_I2C_IsDeviceReady(I2C_device, ACCEL_ADDR, 1, 10);
			HAL_I2C_Mem_Write(I2C_device, ACCEL_ADDR, ACCEL_BW_RATE, sizeof(uint8_t), to_send, sizeof(uint8_t), 10);
		
			to_send = SELF_TEST_OFF;
			HAL_I2C_IsDeviceReady(I2C_device, ACCEL_ADDR, 1, 10);
			HAL_I2C_Mem_Write(I2C_device, ACCEL_ADDR, ACCEL_DATA_FMT, sizeof(uint8_t), to_send, sizeof(uint8_t), 10);
			for(samples=0; samples<NUM_SAMPLES; samples++){
				HAL_I2C_IsDeviceReady(I2C_device, ACCEL_ADDR, 1, 10);
				HAL_I2C_Mem_Read(I2C_device, ACCEL_ADDR, ACCEL_DATA_X0, sizeof(uint8_t), &X_SAMPLES[samples], sizeof(uint8_t), 10);
				HAL_I2C_IsDeviceReady(I2C_device, ACCEL_ADDR, 1, 10);
				HAL_I2C_Mem_Read(I2C_device, ACCEL_ADDR, ACCEL_DATA_Y0, sizeof(uint8_t), &Y_SAMPLES[samples], sizeof(uint8_t), 10);
				HAL_I2C_IsDeviceReady(I2C_device, ACCEL_ADDR, 1, 10);
				HAL_I2C_Mem_Read(I2C_device, ACCEL_ADDR, ACCEL_DATA_Z0, sizeof(uint8_t), &Z_SAMPLES[samples], sizeof(uint8_t), 10);
			}//end for
			for(samples=0;samples<NUM_SAMPLES; samples++){
				X_ST += X_SAMPLES[samples]; 
				Y_ST += Y_SAMPLES[samples];
				Y_ST += Z_SAMPLES[samples];
			}//end for
			
			*X_ST = *X_ST/NUM_SAMPLES; /*AVERAGE THE NON-SELF TEST VALUES*/
			*Y_ST = *Y_ST/NUM_SAMPLES;
			*Z_ST = *Z_ST/NUM_SAMPLES;
			
			to_send = SELF_TEST_ON; /*TURN ON SELF-TEST MODE:*/
			HAL_Delay(1);
			HAL_I2C_IsDeviceReady(I2C_device, ACCEL_ADDR, 1, 10);
			HAL_I2C_Mem_Write(I2C_device, ACCEL_ADDR, ACCEL_DATA_FMT, sizeof(uint8_t), to_send, sizeof(uint8_t), 100);
			
			for(samples=0; samples<NUM_SAMPLES; samples++){/*TAKE NUM_SAMPLES*/
				HAL_I2C_IsDeviceReady(I2C_device, ACCEL_ADDR, 1, 10);
				HAL_I2C_Mem_Read(I2C_device, ACCEL_ADDR, ACCEL_DATA_X0, sizeof(uint8_t), &X_SAMPLES_ON[samples], sizeof(uint8_t), 10);
				HAL_I2C_IsDeviceReady(I2C_device, ACCEL_ADDR, 1, 10);
				HAL_I2C_Mem_Read(I2C_device, ACCEL_ADDR, ACCEL_DATA_Y0, sizeof(uint8_t), &Y_SAMPLES_ON[samples], sizeof(uint8_t), 10);
				HAL_I2C_IsDeviceReady(I2C_device, ACCEL_ADDR, 1, 10);
				HAL_I2C_Mem_Read(I2C_device, ACCEL_ADDR, ACCEL_DATA_Z0, sizeof(uint8_t), &Z_SAMPLES_ON[samples], sizeof(uint8_t), 10);
			}//end for
			for(samples=0;samples<NUM_SAMPLES; samples++){/*SUM SAMPLES, CURRENTLY 10*/
				X_ST_ON += X_SAMPLES_ON[samples]; 
				Y_ST_ON += Y_SAMPLES_ON[samples];
				Z_ST_ON += Z_SAMPLES_ON[samples];
			}//end for
			*X_ST_ON = *X_ST_ON/NUM_SAMPLES; /*AVERAGE THE SAMPLES*/
			*Y_ST_ON = *Y_ST_ON/NUM_SAMPLES;
			*Z_ST_ON = *Z_ST_ON/NUM_SAMPLES;
			
			*X_ST_ON -= *X_ST; /*TAKE THE DIFFERENCE OF THE NORMAL ANDSELF TEST VALUES*/
			*Y_ST_ON -= *Y_ST;
			*Z_ST_ON -= *Z_ST;
			//if()	/*IF THE CALIBRATION VALUE IS NO GOOD, RESET AND CALL INIT ROUTINE AGAIN*/		
			to_send = ACCEL_FIFO_INIT;
			HAL_I2C_IsDeviceReady(I2C_device, ACCEL_ADDR, 1, 10);
			HAL_I2C_Mem_Write(&hi2c1, ACCEL_ADDR, ACCEL_FIFO_CTRL, sizeof(uint8_t), to_send, sizeof(uint8_t), 100); 
			HAL_I2C_IsDeviceReady(I2C_device, ACCEL_ADDR, 1, 10);
			HAL_I2C_Mem_Read(I2C_device, ACCEL_ADDR, ACCEL_DATA_X0, sizeof(uint8_t), &Z_SAMPLES_ON[samples], sizeof(uint8_t), 100);

		break;
		case GYRO_ADDR:
			
		break;
		case PRESSURE_ADDR:
			
		break;
		
	}


}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
