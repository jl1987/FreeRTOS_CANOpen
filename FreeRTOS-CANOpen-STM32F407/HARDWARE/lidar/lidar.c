/**
  ******************************************************************************
  * @file    lidar.c
  * @author  Jim
  * @version V1.0
  * @date    01-Sep-2014
  * @brief   This file provides lidar functions 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */
#include "lidar.h"
#include "stm32f4xx.h"

void Lidar_Init()
{	
	USART_SendData(UART4, 0xA5);                                     //?癳?誹
	while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET){}//单?癳?

	USART_SendData(UART4, 0x20);                                     //?癳?誹
	while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET){}//单?癳?
	
	USART_SendData(USART1, 0xA5);                                     //?癳?誹
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}//单?癳?

	USART_SendData(USART1, 0x20);                                     //?癳?誹
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}//单?癳?
		
}

void Lidar_Stop()
{
	USART_SendData(UART4, 0xA5);                                     //?癳?誹
	while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET){}//单?癳?

	USART_SendData(UART4, 0x25);                                     //?癳?誹
	while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET){}//单?癳?	
}

void Lidar_Reset()
{
	USART_SendData(UART4, 0xA5);                                     //?癳?誹
	while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET){}//单?癳?

	USART_SendData(UART4, 0x40);                                     //?癳?誹
	while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET){}//单?癳?	
}



void SENSOR_Lidar_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure; 
	
	/* GPIO Config ------------------------------------------------------------------*/
	/* Enable GPIOF Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	/* Connect USART pins to AF7 */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

	/* Configure USART Tx and Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	/* UART4 Config -----------------------------------------------------------------*/
	/* Enable UART4 Clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	/* Config USAR4 Detail */
  USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);
	/* Enable USART */
	/* Enable USART4 Receive Data Read To Be Read Interrupt */
	USART_Cmd(UART4, ENABLE);
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	
	/* NVIC Config ------------------------------------------------------------------*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
	




/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
