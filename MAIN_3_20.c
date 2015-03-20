/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 28/02/2015 02:37:08
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
#define DEBUG
#define PRESSURE_ADDR 0x77 /*ADDRESS FOR BMP085 BOSCH BAROMETRIC PRESSURE SENSOR*/
#define GYRO_ADDR 0x69 /*ADDRESS FOR L3G4200D GYROSCOPE FROM ST MICRO*/
#define ACCEL_ADDR 0x53 /*ADDRESS FOR ADXL345 FROM ANALOG DEVICES*/
/* Includes -----------------------------------------------------------------*/
#include "stm32f2xx_hal.h"
#include "ff.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h" /* defines SD_Driver as external */
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "sdio.h"
#include "usb_device.h"
#include "gpio.h"
#include "I2C_MODULES.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

uint8_t retSD;    /* Return value for SD */
char SD_Path[4];  /* SD logical drive path */

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
volatile uint32_t adc1;
static uint16_t RTC_addr = 0xD0;

static uint16_t data_size = sizeof(char);
static uint8_t * data_to_read;
uint64_t SD_write_address=0;

extern volatile uint32_t READ_ADC(ADC_HandleTypeDef *);
void LIGHT_LEDS(uint32_t);
extern uint32_t READ_I2C(I2C_HandleTypeDef *);
void SET_RTC_ALARM(RTC_HandleTypeDef *, uint32_t);
void SYSTEM_INIT();
void I2C_DEVICE_INIT(I2C_HandleTypeDef *, uint8_t dev_id);

/* USER CODE END 0 */

//int main(void)
int main(int argc, char * argv[])
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_ClkInitTypeDef RCC_ClockFreq;
	ErrorStatus HSEStartUpStatus;
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	
	
	/*Initialize GPIO:*/
	GPIO_InitStructure.Pin = GPIO_PIN_10 | GPIO_PIN_9 | GPIO_PIN_8 | GPIO_PIN_7;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	
	/*Initialize ADC:*/
	GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
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
	
	
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;	
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
	HAL_I2C_Init(&hi2c1);
	
	HAL_I2C_MspInit(&hi2c1);
	I2C_DEVICE_INIT(&hi2c1, ACCEL_ADDR);

  /* USER CODE END 2 */

  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SD_Path);

  /* USER CODE BEGIN 3 */
  /*Initialize SD Card*/
		
		
  /* Infinite loop */
  while (1)
  {
	/*	
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);
	adc1=0;
	adc1 = HAL_ADC_GetValue(&hadc1);
        */
	__WFI();
	//adc1 = READ_ADC(&hadc1);
	//LIGHT_LEDS(adc1);
  /* USER CODE END 3 */
  }  
}
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
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
* @brief  Alarm A callback.
* @param  hrtc: RTC handle
* @retval None
******************************************************************************/
__weak void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  /* NOTE : This function is defined in ..._hal_rtc.c, to not modify the
	library file it has been implemented here:
   */
	READ_ADC(&hadc1);
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
	READ_I2C(&hi2c1);
}

/******************************************************************************
* @abstract READ_ADC function reads from specific ADC channel
* @ref ADC_READ(ADC_HandleTypeDef * adc_num)
* @param ADC_HandleTypeDef * adc_num
* @retval ADC value read from channel
******************************************************************************/
volatile uint32_t READ_ADC(ADC_HandleTypeDef * adc_num){	
	volatile uint32_t adc_value;
	uint32_t * pass_to_SD;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);
	adc_value=0;
	adc_value = HAL_ADC_GetValue(&hadc1);
	*pass_to_SD = adc_value;
	#ifdef DEBUG
		LIGHT_LEDS(adc_value);
	#endif
	 /**Enable the Alarm B 
    */
	
	/*sAlarm.Alarm = RTC_ALARM_B;
	sAlarm.AlarmMask = RTC_ALARMMASK_SECONDS;//TODO: MASK ALL BUT HOURS? MINS?->DEBUG: SECONDS!
	HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, FORMAT_BCD);
*/
	MX_RTC_Init();/*TODO: THIS IS THE DEFINITION OF A KLUDGE, SHOULD SET IRQ PRIORITY DIFFERENTLY
	OR SIMPLY RESET THE SINGLE TIMER, NOT THE ENTIRE ROUTINE, PLUS IT VERY WELL COULD F*CK UP THE
	RTC ITSELF, THUS RUINING OUR TIMERS ALTOGETHER!*/
	HAL_SD_WriteBlocks(&hsd, pass_to_SD, SD_write_address, SDIO_DATABLOCK_SIZE_512B, 1);
return adc_value;
}
SD_SDIO_DMA_IRQHANDLER
//DMA2_Stream3_IRQHandler(
/******************************************************************************
* @abstract READ_ADC function reads from specific ADC channel
* @ref ADC_READ(ADC_HandleTypeDef * adc_num)
* @param ADC_HandleTypeDef * adc_num
* @retval ADC value read from channel
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
			HAL_I2C_Mem_Write(I2C_device, ACCEL_ADDR, ACCEL_BW_RATE, sizeof(uint8_t), to_send, sizeof(uint8_t), 100);
			HAL_I2C_IsDeviceReady(I2C_device, ACCEL_ADDR, 1, 100);
		
			to_send = SELF_TEST_OFF;
			HAL_I2C_Mem_Write(I2C_device, ACCEL_ADDR, ACCEL_DATA_FMT, sizeof(uint8_t), to_send, sizeof(uint8_t), 100);
			HAL_I2C_IsDeviceReady(I2C_device, ACCEL_ADDR, 1, 100);
			for(samples=0; samples<NUM_SAMPLES; samples++){
				HAL_I2C_Mem_Read(I2C_device, ACCEL_ADDR, ACCEL_DATA_X0, sizeof(uint8_t), &X_SAMPLES[samples], sizeof(uint8_t), 100);
				HAL_I2C_IsDeviceReady(I2C_device, ACCEL_ADDR, 1, 100);
				HAL_I2C_Mem_Read(I2C_device, ACCEL_ADDR, ACCEL_DATA_Y0, sizeof(uint8_t), &Y_SAMPLES[samples], sizeof(uint8_t), 100);
				HAL_I2C_IsDeviceReady(I2C_device, ACCEL_ADDR, 1, 100);
				HAL_I2C_Mem_Read(I2C_device, ACCEL_ADDR, ACCEL_DATA_Z0, sizeof(uint8_t), &Z_SAMPLES[samples], sizeof(uint8_t), 100);
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
			HAL_I2C_Mem_Write(I2C_device, ACCEL_ADDR, ACCEL_DATA_FMT, sizeof(uint8_t), to_send, sizeof(uint8_t), 100);
			HAL_I2C_IsDeviceReady(I2C_device, ACCEL_ADDR, 1, 100);
			
			for(samples=0; samples<NUM_SAMPLES; samples++){/*TAKE NUM_SAMPLES*/
				HAL_I2C_Mem_Read(I2C_device, ACCEL_ADDR, ACCEL_DATA_X0, sizeof(uint8_t), &X_SAMPLES_ON[samples], sizeof(uint8_t), 100);
				HAL_I2C_IsDeviceReady(I2C_device, ACCEL_ADDR, 1, 100);
				HAL_I2C_Mem_Read(I2C_device, ACCEL_ADDR, ACCEL_DATA_Y0, sizeof(uint8_t), &Y_SAMPLES_ON[samples], sizeof(uint8_t), 100);
				HAL_I2C_IsDeviceReady(I2C_device, ACCEL_ADDR, 1, 100);
				HAL_I2C_Mem_Read(I2C_device, ACCEL_ADDR, ACCEL_DATA_Z0, sizeof(uint8_t), &Z_SAMPLES_ON[samples], sizeof(uint8_t), 100);
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
			HAL_I2C_IsDeviceReady(I2C_device, ACCEL_ADDR, 1, 100);
			HAL_I2C_Mem_Write(&hi2c1, ACCEL_ADDR, ACCEL_FIFO_CTRL, sizeof(uint8_t), to_send, sizeof(uint8_t), 100); 
			HAL_I2C_IsDeviceReady(I2C_device, ACCEL_ADDR, 1, 100);
			HAL_I2C_Mem_Read(I2C_device, ACCEL_ADDR, ACCEL_DATA_X0, sizeof(uint8_t), &Z_SAMPLES_ON[samples], sizeof(uint8_t), 100);

		break;
		case GYRO_ADDR:
			
		break;
		case PRESSURE_ADDR:
			
		break;
		
	}


}
/******************************************************************************
* @abstract READ_I2C function reads from I2C channel based on passed arg
TODO: Pass device ADDRESS? Pass HandleType?
* @ref READ_I2C(I2C_HandleTypeDef *I2C_device)
* @param I2C_device - WHICH I2C device to read from...
******************************************************************************/
uint32_t READ_I2C(I2C_HandleTypeDef * I2C_device){		
	volatile uint32_t I2C_value;
	HAL_I2C_IsDeviceReady(&hi2c1, ACCEL_ADDR, PHY_FULLDUPLEX_100M, 100); 
	HAL_I2C_Mem_Read(&hi2c1, ACCEL_ADDR, ACCEL_DATA_X0, sizeof(char), data_to_read, sizeof(char), 10);
	I2C_value = *data_to_read;
	#ifdef DEBUG
		LIGHT_LEDS(I2C_value*100);
	#endif
	return I2C_value;
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
    
	//HAL_RTC_SetAlarm(&hrtc, &sAlarm.AlarmA, FORMAT_BIN);
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
