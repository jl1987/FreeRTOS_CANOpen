/**
  ******************************************************************************
  * @file    motor.c
  * @author  Jim
  * @version V1.0
  * @date    16-Jan-2015
  * @brief   This file provides motor thread functions 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */

#include "motor.h"
#include "stm32f4xx.h"

u8 MSG_U2_R[8];
u8 MSG_U2_T[16];
u8 MSG_U3_R[8];
u8 MSG_U3_T[16];

/**
 * Moter of Chassis initial
 * @brief CANOpen Data Scan, if CANbus got a CAN message, then push the message into
 *        queue(xQ_CAN_MSG), this thread scan the queue at CANOpen_THREAD_SCAN_TIMER(
 *        default:20ms) rate.
 */
void Motor_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	DMA_InitTypeDef 	DMA_InitStruct;
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
  TIM_OCInitTypeDef TIM_OCInitStructure; 
	
	NVIC_InitTypeDef 	NVIC_InitStructure;
	
	/* GPIO Config ------------------------------------------------------------------*/
	/* Enable GPIO Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|
												 RCC_AHB1Periph_GPIOB|
												 RCC_AHB1Periph_GPIOC|
												 RCC_AHB1Periph_GPIOD|
												 RCC_AHB1Periph_GPIOE, ENABLE);
	/* Moter FAULT */
	GPIO_InitStructure.GPIO_Pin = Channel1_FLT|Channel2_FLT|Channel3_FLT;		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(Channel1_FLTPORT, &GPIO_InitStructure);
	
	/* Moter RST */
	GPIO_InitStructure.GPIO_Pin = Channel1_RST;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Channel1_RSTPORT, &GPIO_InitStructure);
	GPIO_WriteBit(Channel1_RSTPORT,Channel1_RST, Bit_SET);
	
	GPIO_InitStructure.GPIO_Pin = Channel2_RST|Channel3_RST;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Channel2_RSTPORT, &GPIO_InitStructure);
	GPIO_WriteBit(Channel2_RSTPORT,Channel2_RST|Channel2_RST, Bit_SET);
	
	/* Moter 1 --- U2 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;	  //tx
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);	
	
	/* Moter 2 --- U3 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;	  //tx
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);	

	/* Moter 3 --- U5 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	  //TX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	    //RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,  GPIO_AF_UART5);	
	
	/* Moter PWM */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
  GPIO_Init(GPIOE, &GPIO_InitStructure); 
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
	
	
	/* UART Config ------------------------------------------------------------------*/
	/* Enable USART Clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2|
												 RCC_APB1Periph_USART3|
												 RCC_APB1Periph_UART5,ENABLE);
	/* Moter 1 */
	
	USART_DeInit(USART2);
	USART_DeInit(USART3);
	USART_DeInit(UART5);
	
	USART_InitStructure.USART_BaudRate = 19200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure); 
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
  USART_DMACmd(USART2,USART_DMAReq_Rx, ENABLE);
	USART_DMACmd(USART2,USART_DMAReq_Tx, ENABLE);
  USART_Cmd(USART2, ENABLE);
	/* Moter 2 */
	USART_InitStructure.USART_BaudRate = 19200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure); 
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
  USART_DMACmd(USART3,USART_DMAReq_Rx, ENABLE);
	USART_DMACmd(USART3,USART_DMAReq_Tx, ENABLE);
  USART_Cmd(USART3, ENABLE);
	/* Moter 3 */  
	USART_InitStructure.USART_BaudRate = 19200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(UART5, &USART_InitStructure); 
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
// 	USART_DMACmd(UART5,USART_DMAReq_Rx, ENABLE);
// 	USART_DMACmd(UART5,USART_DMAReq_Tx, ENABLE);
  USART_Cmd(UART5, ENABLE);


	/* DMA Config ------------------------------------------------------------------*/
	/* Enable DMA1 Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
	DMA_DeInit(DMA1_Stream1);
	DMA_DeInit(DMA1_Stream3);
	DMA_DeInit(DMA1_Stream5);
	DMA_DeInit(DMA1_Stream6);
	
	/* Moter 1 --- U2 */
	while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);	
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;			   //S5C4
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(USART2->DR));
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)MSG_U2_R;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;             //RX
	DMA_InitStruct.DMA_BufferSize = (u32)10;
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
	
	DMA_Init(DMA1_Stream5, &DMA_InitStruct);
	DMA_Cmd(DMA1_Stream5, ENABLE);  //接收通道
	
	while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE);	
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;			   //S6C4
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(USART2->DR));
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)MSG_U2_T;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;            //TX
	DMA_InitStruct.DMA_BufferSize = (u32)10;
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
	
	DMA_Init(DMA1_Stream6, &DMA_InitStruct);
	DMA_Cmd(DMA1_Stream6, ENABLE); 
	
	/* Moter 1 --- U3 */
	while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);	
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;			   //S1C4
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(USART3->DR));
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)MSG_U3_R;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;             //RX
	DMA_InitStruct.DMA_BufferSize = (u32)10;
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
	
	DMA_Init(DMA1_Stream1, &DMA_InitStruct);
	DMA_Cmd(DMA1_Stream1, ENABLE);  //接收通道
	
	while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE);	
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;			   //S3C4
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(USART3->DR));
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)MSG_U3_T;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;            //TX
	DMA_InitStruct.DMA_BufferSize = (u32)10;
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
	
	DMA_Init(DMA1_Stream3, &DMA_InitStruct);
	DMA_Cmd(DMA1_Stream3, ENABLE); 
	
  /* PWM Config -------------------------------------------------------------------*/
	/* Enable TIM1 Clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	/* TIM1 Config */
  TIM_TimeBaseStructure.TIM_Prescaler = 100;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = HALFPWMTIMERPERIOD*2;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	/* PWM1 Config */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	  //还没匹配，高!
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;  //匹配之后，低!
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
  //TIM_OC1Init(TIM1, &TIM_OCInitStructure); 
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);
  TIM_Cmd(TIM1, ENABLE);
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
	
	/* NVIC Config ------------------------------------------------------------------*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
}

void MotorFault(void)
{
	//处理DRIVER报错  
	//处理DRIVER报错 
	//处理DRIVER报错 
  if(!GPIO_ReadInputDataBit(Channel1_FLTPORT, Channel1_FLT))
	{	GPIO_ResetBits(Channel1_RSTPORT, Channel1_RST);
	  delay(100);
		//OSTimeDly(2);
  }
	else
	{GPIO_SetBits(Channel1_RSTPORT, Channel1_RST);}
	
	if(!GPIO_ReadInputDataBit(Channel2_FLTPORT, Channel2_FLT))
	{	GPIO_ResetBits(Channel2_RSTPORT, Channel2_RST);
		delay(100);
		//OSTimeDly(2);
  }
		else
	{GPIO_SetBits(Channel2_RSTPORT, Channel2_RST);}
	
	if(!GPIO_ReadInputDataBit(Channel3_FLTPORT, Channel3_FLT))
	{	GPIO_ResetBits(Channel3_RSTPORT, Channel3_RST);
		delay(100);
		//OSTimeDly(2);
  }
		else
	{GPIO_SetBits(Channel3_RSTPORT, Channel3_RST);}
	//处理DRIVER报错 
	//处理DRIVER报错 
	//处理DRIVER报错 
}


void chassis_move(double wheel1, double wheel2, double wheel3)
{
	if ((Channel_PWMOUTPUT(wheel1)>0)&&(Channel_PWMOUTPUT(wheel1)<(HALFPWMTIMERPERIOD*2)))
	Channel1_PWM=Channel_PWMOUTPUT(wheel1);
	
	if ((Channel_PWMOUTPUT(wheel2)>0)&&(Channel_PWMOUTPUT(wheel2)<(HALFPWMTIMERPERIOD*2)))
	Channel2_PWM=Channel_PWMOUTPUT(wheel2);
	
	if ((Channel_PWMOUTPUT(wheel3)>0)&&(Channel_PWMOUTPUT(wheel3)<(HALFPWMTIMERPERIOD*2)))
	Channel3_PWM=Channel_PWMOUTPUT(wheel3);
}


void delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}



/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
