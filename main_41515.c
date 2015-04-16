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
#include <string.h>

/* USER CODE BEGIN Includes */
#define DEBUG
#define MAX_LEN 2//100
#define __USE_IRQ

#define ADC_TYPE 0
#define I2C_TYPE 1
#define UNUSED '\0'

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
uint8_t retSD;    /* Return value for SD */
char SD_Path[4];  /* SD logical drive path */
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
FRESULT res;                                          /* FatFs function common result code */
uint32_t byteswritten, bytesread;                     /* File write/read counts */
uint8_t wtext[] = "CAPSTONE SD Test1:"; /* File write buffer */
uint8_t rtext[100];                                   /* File read buffer */

volatile uint32_t adc1;
volatile uint32_t I2C_value;

typedef struct t_settings{
	uint8_t ACCEL_SET;
	uint8_t ADC1_SET;
	uint8_t ADC2_SET;
	uint8_t ADC3_SET;
	uint8_t ADC4_SET;
}T_SET;
T_SET timers;

typedef struct sensor{
	char * name;
	uint8_t type;
	uint8_t array_index;
	uint32_t alarm;
	uint32_t data;
	//need to add a "conversion factor" entry, may define as a macro...
}SENS;
SENS sensors[10];

static uint32_t ACCEL_COUNTER;
static uint32_t ACCEL_SCHEDULE=10;

static uint32_t ADC1_COUNTER;
static uint32_t ADC1_SCHEDULE=0;

static uint32_t ADC2_COUNTER;
static uint32_t ADC2_SCHEDULE=30;

static uint32_t ADC3_COUNTER;
static uint32_t ADC3_SCHEDULE=60;

static uint32_t ADC4_COUNTER;
static uint32_t ADC4_SCHEDULE=255;

uint8_t data_to_read;

uint64_t SD_write_address=0;

static DMA_HandleTypeDef huart_rx;
static DMA_HandleTypeDef huart_tx;
static DMA_HandleTypeDef hdma_adc;
static DMA_HandleTypeDef hdma_i2c1;
static DMA_HandleTypeDef hdma_sdio;

SD_HandleTypeDef SD_CARD;

ADC_ChannelConfTypeDef sConfig;

/*Variables for string receive over USART:*/
uint8_t rxBuffer = '\000';
uint8_t rxString[MAX_LEN];
uint8_t txBuffer[] = "SERIAL PRINT\n";
int rx_index=0;

/*Peripheral functions:*/
void SystemClock_Config(void);
void GPIO_INIT(void);
void ADC_INIT(void);
void I2C_INIT(void);
void IRQ_INIT(void);
void USART_INIT(void);
void ARBITRATE(void);
void SD_write_test(void);
void Error_Handler(void);
void print(uint8_t string[]);

void I2C_DEVICE_INIT(I2C_HandleTypeDef *, uint8_t dev_id);
void LIGHT_LEDS(uint32_t);

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
  //HAL_UART_MspInit(&huart1);
  USART_INIT();
  GPIO_INIT();
  //HAL_SD_Init(&SD_CARD, &SD_CardInfo);
  IRQ_INIT();
  //I2C_DEVICE_INIT(&hi2c1, ACCEL_ADDR);
  SD_write_test();
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
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)&string[index],1);	
	}	
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
	hadc1.Init.DMAContinuousRequests = ENABLE;//ENABLE;
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
	__GPIOB_CLK_ENABLE();
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
	
	/*Initialize GPIO for FS LEDs:*/
	GPIO_InitStructure.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
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

	HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxString, 1);
}/*END INIT USART*/

/******************************************************************************
* @brief  Alarm A callback.
* @param  hrtc: RTC handle
* @retval None
******************************************************************************/
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc){
	ARBITRATE();
}
/******************************************************************************
* @brief  Arbitration function for the Alarm A:
*	  -Determines which sensor to sample based on the timer counter
* @param  hrtc: RTC handle
* @retval None
******************************************************************************/
void ARBITRATE(void){
	
	if(ADC1_COUNTER == ADC1_SCHEDULE){
		HAL_ADC_Start_IT(&hadc1);//WORKS!
		//HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&uhADCxConvertedValue, 1);//WORKS with DMA
		ADC1_COUNTER=0;
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
/******************************************************************************
* @abstract HAL_ADC_ConvCpltCallback function reads from specific ADC device
* @ref HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc1)
* @param ADC_HandleTypeDef * hadc1
* @retval NONE
******************************************************************************/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc){
	__IO uint32_t adc_val=0;
	__IO uint16_t adc_temp=0;//WORKS with IT
	adc_val = HAL_ADC_GetValue(&hadc1);//WORKS with IT
	LIGHT_LEDS(adc_val);//WORKS with IT
	adc_temp=(adc_val/1000);
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)&adc_temp, 1);//8);
	//HAL_SD_WriteBlocks_DMA(&SD_CARD, (uint32_t *)adc_val, 0x0, SDIO_DATABLOCK_SIZE_512B, 1);
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
	
	if(rxBuffer =='\n' || rxBuffer == '\r'){
		print((uint8_t*)&rxBuffer);
	}
	else{
		rxString[rx_index] = rxBuffer;
		ADC1_SCHEDULE = rxString[rx_index];
		rx_index++;
		if(rx_index >= MAX_LEN){
			return;
		}	
	}
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
	Error_Handler();
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
/******************************************************************************
* @abstract Initialize SD Card
* @ref SD_CARD_INIT(void)
* @param NONE
******************************************************************************/
void SD_CARD_INIT(void){
	__SDIO_CLK_ENABLE();
	__HAL_SD_SDIO_ENABLE();
	__HAL_SD_SDIO_DMA_ENABLE();
	
	HAL_SD_CardInfoTypedef SD_info;
	
	SD_info.CardBlockSize = SDIO_DATABLOCK_SIZE_512B;
	//if(HAL_GPIO_ReadPin(GPIO_PIN_, MICROPY_HW_SDCARD_DETECT_PIN.pin_mask))
	//	return 1;
	//SD_info.CardCapacity = SDIO_
	
	HAL_SD_Init(&SD_CARD, &SD_info);
	
}
/******************************************************************************
* @abstract Trap the error condition and blink the RED LED on 
* @ref Error_Handler(void)
* @param uint32_t value - determines which LED(s) to light, ADC debugging!
******************************************************************************/
void Error_Handler(void){
	while(1){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		HAL_Delay(10);
	}
}
/******************************************************************************
* @abstract Initialize the filesystem and do a test write to the card
* @ref SD_write_test(void)
* @param NONE
******************************************************************************/
void SD_write_test(void){
	
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  /*##-1- Link the micro SD disk I/O driver ##################################*/
  if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
  {
    /*##-2- Register the file system object to the FatFs module ##############*/
    if(f_mount(&SDFatFs, (TCHAR const*)SD_Path, 0) != FR_OK)
    {
      /* FatFs Initialization Error */
      Error_Handler();
    }
    else
    {
      /*##-3- Create a FAT file system (format) on the logical drive #########*/
      /* WARNING: Formatting the uSD card will delete all content on the device */
      if(f_mkfs((TCHAR const*)SD_Path, 0, 0) != FR_OK)
      {
        /* FatFs Format Error */
        Error_Handler();
      }
      else
      {       
        /*##-4- Create and Open a new text file object with write access #####*/
        if(f_open(&MyFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
        {
          /* 'STM32.TXT' file Open for write Error */
          Error_Handler();
        }
        else
        {
          /*##-5- Write data to the text file ################################*/
          res = f_write(&MyFile, wtext, sizeof(wtext), (void *)&byteswritten);
          
          if((byteswritten == 0) || (res != FR_OK))
          {
            /* 'STM32.TXT' file Write or EOF Error */
            Error_Handler();
          }
          else
          {
            /*##-6- Close the open text file #################################*/
            f_close(&MyFile);
            
            /*##-7- Open the text file object with read access ###############*/
            if(f_open(&MyFile, "STM32.TXT", FA_READ) != FR_OK)
            {
              /* 'STM32.TXT' file Open for read Error */
              Error_Handler();
            }
            else
            {
              /*##-8- Read data from the text file ###########################*/
              res = f_read(&MyFile, rtext, sizeof(rtext), (UINT*)&bytesread);
              
              if((bytesread == 0) || (res != FR_OK))
              {
                /* 'STM32.TXT' file Read or EOF Error */
                Error_Handler();
              }
              else
              {
                /*##-9- Close the open text file #############################*/
                f_close(&MyFile);
                
                /*##-10- Compare read data with the expected data ############*/
                if((bytesread != byteswritten))
                {                
                  /* Read data is different from the expected data */
                  Error_Handler();
                }
                else
                {
                  /* Success of the demo: no error occurrence */
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
                }
              }
            }
          }
        }
      }
    }
  }
  
  /*##-11- Unlink the RAM disk I/O driver ####################################*/
  FATFS_UnLinkDriver(SD_Path);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);//Turn LED off on card release

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
