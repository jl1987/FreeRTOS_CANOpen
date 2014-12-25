/**
  ******************************************************************************
  * @file    serial_debug.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-October-2011
  * @brief   This file provide functions to retarget the C library printf function
  *          to the USART. 
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
//#include "stm324xg_eval.h"
#include "serial_debug.h"
#include <stdio.h>

#ifdef SERIAL_DEBUG_ON

#define RCC_DEBUG_GPIO    RCC_AHB1Periph_GPIOA
#define RCC_DEBUG_USART   RCC_APB2Periph_USART1
#define DEBUG_USART_PORT  GPIOA
#define DEBUG_USART_TX    GPIO_Pin_9
#define DEBUG_USART_RX    GPIO_Pin_10

u8 MSG_U1_R[256];
u8 MSG_U1_T[256];

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize COM1 interface for serial debug
  * @note   COM1 interface is defined in stm3210g_eval.h file (under Utilities\STM32_EVAL\STM324xG_EVAL)  
  * @param  None
  * @retval None
  */
void DebugComPort_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef 	NVIC_InitStructure; 
	DMA_InitTypeDef 	DMA_InitStruct;
	
  /* Clock For Debug GPIOA */
  RCC_AHB1PeriphClockCmd(RCC_DEBUG_GPIO, ENABLE);
  /* Clock For Debug USART1 */
  RCC_APB2PeriphClockCmd(RCC_DEBUG_USART,ENABLE);

  /* Configuration for GPIO */
  GPIO_InitStructure.GPIO_Pin   = DEBUG_USART_TX|DEBUG_USART_RX;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); 
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);


  /* USARTx configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
 
  USART_DeInit(USART1);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure); 
  /*Enable Interrupt*/
  USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
  /*Enable USART1*/
  USART_Cmd(USART1, ENABLE);
	
	
	/*Configuration For DMA*/
	DMA_DeInit(DMA2_Stream5);
  DMA_DeInit(DMA2_Stream7);
	
	while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE);	
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;			   //S5C4
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(USART1->DR));
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)MSG_U1_R;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;             //RX
	DMA_InitStruct.DMA_BufferSize = (u32)0;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream5, &DMA_InitStruct);
	//DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA2_Stream5, ENABLE);  //接收通道
	
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);	
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;			   //S7C4
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(USART1->DR));
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)MSG_U1_T;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;            //TX
	DMA_InitStruct.DMA_BufferSize = (u32)0;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream7, &DMA_InitStruct);
	//DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA2_Stream7, ENABLE);  //接收通道
	//U1
	
	
	/*Configuration For NVIC*/
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}



#endif /* SERIAL_DEBUG */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
