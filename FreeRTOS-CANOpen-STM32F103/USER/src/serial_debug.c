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
#include "stm32f10x.h"
//#include "stm324xg_eval.h"
#include "serial_debug.h"
#include <stdio.h>

#ifdef SERIAL_DEBUG_ON

#ifdef ARM_BMS
	#define RCC_DEBUG_GPIO    				RCC_APB2Periph_GPIOA
	#define RCC_DEBUG_USART   				RCC_APB2Periph_USART1
	#define DEBUG_USART_PORT  				GPIOA
	#define DEBUG_USART_TX    				GPIO_Pin_9
	#define DEBUG_USART_RX    				GPIO_Pin_10
	#define DEBUG_USART_TX_PinSource  GPIO_PinSource9
	#define DEBUG_USART_RX_PinSource  GPIO_PinSource10
	#define DEBUG_USART_AF 						GPIO_AF_USART1
	#define DEBUG_USARTx 							USART1
	#define DEBUG_USART_IQRn 					USART1_IRQn
#endif

#ifdef ARM_LIFTER
	#define RCC_DEBUG_GPIO    				RCC_APB2Periph_GPIOB
	#define RCC_DEBUG_USART   				RCC_APB1Periph_USART3
	#define DEBUG_USART_PORT  				GPIOB
	#define DEBUG_USART_TX    				GPIO_Pin_10
	#define DEBUG_USART_RX    				GPIO_Pin_11
	#define DEBUG_USART_TX_PinSource  GPIO_PinSource10
	#define DEBUG_USART_RX_PinSource  GPIO_PinSource11
	#define DEBUG_USART_AF 						GPIO_AF_USART3
	#define DEBUG_USARTx 							USART3
	#define DEBUG_USART_IQRn 					USART3_IRQn
#endif

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
  /* init structure for usart */
  USART_InitTypeDef USART_InitStructure;
  /* init structure for NVIC */
  NVIC_InitTypeDef NVIC_InitStructure; 
	/* Clock For Debug GPIO */
	RCC_APB2PeriphClockCmd(RCC_DEBUG_GPIO | RCC_APB2Periph_AFIO, ENABLE);
	/* Clock For Debug USARTx */
#ifdef ARM_BMS
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#endif

#ifdef ARM_LIFTER
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 , ENABLE);
#endif
	
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX;	         //TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(DEBUG_USART_PORT, &GPIO_InitStructure);		   
	
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX;	         //RX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(DEBUG_USART_PORT, &GPIO_InitStructure);
	
/* USARTx configured as follow:
			- BaudRate = 115200 baud  
			- Word Length = 8 Bits
			- One Stop Bit
			- No parity
			- Hardware flow control disabled (RTS and CTS signals)
			- Receive and transmit enabled
*/
	USART_DeInit(DEBUG_USARTx);
	USART_InitStructure.USART_BaudRate 		= 115200;
	USART_InitStructure.USART_WordLength 	= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits 		= USART_StopBits_1;
	USART_InitStructure.USART_Parity 			= USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode 				= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(DEBUG_USARTx, &USART_InitStructure);
	/*Enable Interrupt*/
  USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);
	/*Enable USARTx*/
  USART_Cmd(DEBUG_USARTx, ENABLE);

  /*Configuration For NVIC*/
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IQRn; 
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
  USART_SendData(DEBUG_USARTx, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TC) == RESET)
  {}

  return ch;
}



#endif /* SERIAL_DEBUG */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
