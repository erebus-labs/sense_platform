/**
  ******************************************************************************
  * @file    Project/STM32F2xx_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    13-April-2012
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>

#define DEBUG
#define ACCEL_ADDR 0x53

volatile char tx_buffer[] = "USART3 ACTIVE\n";
volatile char rx_buffer[100];
static int tx_index=0;
static int rx_index=0;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t TimingDelay;
uint32_t ADC1_SCHEDULE=0;
uint32_t ADC1_COUNTER=0;
uint32_t ADC2_SCHEDULE=0;
uint32_t ADC2_COUNTER=0;
uint32_t ADC3_SCHEDULE=0;
uint32_t ADC3_COUNTER=0;

uint32_t I2C1_SCHEDULE=0;
uint32_t I2C1_COUNTER=0;
uint32_t I2C2_SCHEDULE=0;
uint32_t I2C2_COUNTER=0;
uint32_t I2C3_SCHEDULE=0;
uint32_t I2C3_COUNTER=0;
uint32_t I2C4_SCHEDULE=0;
uint32_t I2C4_COUNTER=0;

/* Private function prototypes -----------------------------------------------*/
  void GPIO_INIT(void);/*GPIO Initialization*/
  void USART3_INIT(void);/*USART3 System Initialization*/
  void EXTI_INIT(void);/*External Interrupt Initialization*/
  void DMA_INIT(void);/*DMA System Initialization*/
  void SDIO_INIT(void);/*SD Card System Initialization*/
  void ADC_INIT(void);/*ADC System Initialization*/  
  void I2C_INIT(void);/*I2C System Initialization*/
  
  void I2C_DEVICE_INIT(uint16_t address);//TODO: add handle and sensor struct args
  void I2C_WRITE(uint16_t address, uint8_t offset, uint8_t to_write);
  uint8_t I2C_READ(uint16_t address, uint8_t offset, uint8_t to_read);
  
  void RCC_Config(void);/*RCC (Restet and Clock Controller) Initialization*/
  void NVIC_Config(void);/*Non-Vectored Interrupt Controller Initialization*/
  void RTC_Config(void);/*Real Time Clock Initialization*/
  
  void ARBITRATE(void);/*Keeps track of sensor sampling, called by RTC Alarm*/
/********************************************************************************
  * @brief  Main program.
  * @param  None
  * @retval None
********************************************************************************/
int main(void)
{
  RCC_ClocksTypeDef RCC_Clocks;

  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f2xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f2xx.c file
     */  

  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
  
  RCC_Config();
  GPIO_INIT();
  USART3_INIT();
  NVIC_Config();

  
  /* Infinite loop */
  while (1)
  {
    GPIOC->ODR ^= GPIO_Pin_15;
      Delay(10);
    
  
  //__WFI();
  }
}
/********************************************************************************
********************************************************************************/
void RCC_Config(void){
  /*ENABLE CLOCKS FOR GPIO[A,B,C]*/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOA, ENABLE);
  /*ENABLE CLOCKS FOR USART3, I2C1:*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 | RCC_APB1Periph_I2C1, ENABLE);
  /*ENABLE CLOCKS FOR ADC[1,2,3]*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_ADC2|RCC_APB2Periph_ADC3, ENABLE);
}
/********************************************************************************
********************************************************************************/
void ADC_INIT(void){
  /*ADC1 Configuration:*/
  ADC_InitTypeDef ADC_InitStructure;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_15Cycles);
  ADC_Cmd(ADC1, ENABLE);
  
  /*ADC2 Configuration:*/ 
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC2, &ADC_InitStructure);
  
  ADC_RegularChannelConfig(ADC2, ADC_Channel_2, 1, ADC_SampleTime_15Cycles);
  ADC_Cmd(ADC2, ENABLE);

  /*ADC3 Configuration:*/
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC3, &ADC_InitStructure);
  
  ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 1, ADC_SampleTime_15Cycles);
  ADC_Cmd(ADC3, ENABLE);
  
}
/********************************************************************************
********************************************************************************/
void GPIO_INIT(void){
  GPIO_InitTypeDef GPIO_InitStructure;
  /*USART TX PIN CONFIG:*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
    
  /*USART RX PIN CONFIG:*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /*SET USART3 PINS TO Alternate Function (AF)*/
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

  /*LED RED0 and RED1, RESPECTIVELY*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /*GREEN LED:*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /*EXTI PROGRAM BUTTON INPUT:*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /*I2C1 AF PIN CONFIG: PIN[6] --> SCL*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /*I2C1 AF PIN CONFIG: PIN[7] --> SDA*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C1);
  
  /*ADC1: PA1*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /*ADC2: PA2*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /*ADC3: PA3*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
}
/********************************************************************************
********************************************************************************/
void I2C_INIT(void){//TODO: set to pass in sensor struct!
  I2C_InitTypeDef I2C_InitStructure;
  
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x0;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = 100000;
  I2C_Init(I2C1, &I2C_InitStructure);
  I2C_Cmd(I2C1, ENABLE);
  
}
/********************************************************************************
********************************************************************************/
void USART3_INIT(void){
  
  USART_InitTypeDef USART_InitStructure;
  /*
  USART_ClockInitTypeDef USART_ClockInitStructure;
  USART_ClockInitStructure.USART_Clock = USART_Clock_Enable;
  USART_ClockStructInit(&USART_ClockInitStructure);
  USART_ClockInit(USART3, &USART_ClockInitStructure);
  */

  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure);
  
  USART_Cmd(USART3, ENABLE);

  USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}
/********************************************************************************
********************************************************************************/
void NVIC_Config(void){
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;/*EXTI:External Interrupt : Buttons*/
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
  /*USART3 IRQ Configuration:*/
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_EnableIRQ(USART3_IRQn);
  
  /*ADC IRQ Configuration:*/
  NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_EnableIRQ(ADC_IRQn);
  

  //NVIC_InitStructure.NVIC_IRQChannel = EXTI;

}
/******************************************************************************
* @brief  Arbitration function for the Alarm A:
*   -Determines which sensor to sample based on the timer counter
* @param  hrtc: RTC handle
* @retval None
******************************************************************************/
void ARBITRATE(void){
  
  if(ADC1_COUNTER == ADC1_SCHEDULE){
  
    
    ADC1_COUNTER=0;
  }
  else ++ADC1_COUNTER;
  if(ADC2_COUNTER == ADC2_SCHEDULE){

    ADC2_COUNTER=0;
  }
  else ++ADC2_COUNTER;
  if(ADC3_COUNTER == ADC3_SCHEDULE){

    ADC3_COUNTER=0;
  }
  else ++ADC3_COUNTER;
  
  if(I2C1_COUNTER == I2C1_SCHEDULE){
    uint8_t data_to_read=0;
    I2C1_COUNTER=0;
  }
  else ++I2C1_COUNTER;

}
/********************************************************************************
********************************************************************************/
void USART3_IRQHandler(void){

  GPIO_ToggleBits(GPIOB, GPIO_Pin_9);
  if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET){
    
    USART_SendData(USART3, tx_buffer[tx_index++]);
    if(tx_index>=(sizeof(tx_buffer)-1))
      tx_index=0;
  }
  if(USART_GetITStatus(USART3, USART_IT_RXNE != RESET)){
    rx_buffer[rx_index++] = USART_ReceiveData(USART3);
    if(rx_index >= (sizeof(rx_buffer)-1))
      rx_index=0;
  }
}
/********************************************************************************
********************************************************************************/
void RTC_WKUP_IRQHandler(void){
  
}
/********************************************************************************
********************************************************************************/
void ADC_IRQHandler(void){
  
  
}
/********************************************************************************
********************************************************************************/
void I2C1_EV_IRQHandler(void){
  
}
/********************************************************************************
********************************************************************************/
void I2C1_ER_IRQHandler(void){
  
}
/********************************************************************************
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
********************************************************************************/
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
