/**
  ******************************************************************************
  * @file    lifter_control.c
  * @author  Jim
  * @version V1.0
  * @date    01-Sep-2014
  * @brief   This file provides LIFTER control thread functions 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */

#include "chassis_control.h"
#include "stdio.h"

#include "CHASSIS_OD.h"

xQueueHandle xQ_CHASSIS_MSG = NULL;
xTaskHandle	 xT_CHASSIS 	   = NULL;

Chassis_Data CHASSIS_D;

DRIVE DRIVE1,DRIVE2,DRIVE3;


volatile int lidar_offset_count = 0;
volatile int lidar_message[10] = {0,0,0,0,0,0,0,0,0,0};


volatile int RP_Lidar_Buf[360][1];
volatile int RP_Lidar_Duf_Index = 0;
volatile double repul_force_x[360];
volatile double repul_force_y[360];


void chassis_control_thread(void * pvParameters)
{
	printf("start the lifter control\r\n");
	
	Chassis_Init(&CHASSIS_D);
	
	while(1)
	{
		/* Check Motor Fault */
		ChassisFault();
		
		/* Lidar ??? */
		if(CHASSIS_D.Lidar_delay<100)	
		{
			CHASSIS_D.Lidar_delay++;
		}
		if(CHASSIS_D.lidar_init_P == FALSE && CHASSIS_D.Lidar_delay>=100)
		{
			Lidar_Stop();
			vTaskDelay(20);	
			Lidar_Init();
			CHASSIS_D.lidar_init_P = true;
		}

		/* The stepwise acceleration/deceleration function */
		StepwiseFunction(&CHASSIS_D);
		
		/* Chassis Motion Control */
		ChassisMotionCtrl(&CHASSIS_D);
		
		printf("chassis control is running\r\n");
		vTaskDelay(CHASSIS_CONTROL_THREAD_DELAY_TIMER);
	}

}





// void CAN1Master_ConfigureNode(CO_Data* d)
// {
// // 	  d->heartbeatError   = CAN1Master_heartbeatError;
// // 	  d->initialisation   = CAN1Master_initialisation;
// // 	  d->preOperational   = CAN1Master_preOperational;
// // 	  d->operational      = CAN1Master_operational;
// // 	  d->stopped	      = CAN1Master_stopped;
// // 	  d->post_SlaveBootup = CAN1Master_SlaveBootup;
// // 	  d->post_emcy        = CAN1Master_emcy;
// // 	  setState(d,Initialisation);
// }




void start_chassis_control(void)
{
	xTaskCreate(chassis_control_thread, "lifter_control", CHASSIS_CONTROL_THREAD_STACK, NULL,CHASSIS_CONTROL_THREAD_PRIO, &xT_CHASSIS);
	if(NULL == xT_CHASSIS)
	{
	 	printf("chassis control thread creat failed!\r\n");
	}else
	{
		printf("chassis control thread creat successfully!\r\n");
	}
}




void Chassis_Init(Chassis_Data *ch)
{
	Motor_Init();
	
	ch->Lidar_delay = 0;  
	ch->lidar_init_P = false;
	ch->lidar_init_ok_P = false;
	
	ch->target_speed1 = 0;					// Angular speed of wheel 1 [target]
	ch->target_speed2 = 0;
	ch->target_speed3 = 0;
	ch->drive_speed1 = 0;					// Angular speed of wheel 1 [driving]
	ch->drive_speed2 = 0;
	ch->drive_speed3 = 0;
	ch->motion_command = 30;						// the motion command of the chassis
	
	
	
	ch->angle = 0;
	ch->angle_diff = 0;
	ch->prev_angle = 0;
	ch->prev_angle_diff = 0;
	ch->magnitude = 0;
	ch->magnitude_diff = 0;
	ch->prev_magnitude = 0;
	ch->prev_magnitude_diff = 0;
	ch->Kp = 60;
	ch->Kd = 1;
}







double chassis_drive(double X_out,double Y_out,double Theta_out,u8 wheel)
{
 double VALUE[3];
	
	VALUE[0] = X_out*sin(Alpha1) + Y_out*cos(Alpha1) + L1O*Theta_out*Pi/180;
  VALUE[1] = X_out*sin(Alpha2) + Y_out*cos(Alpha2) + L2O*Theta_out*Pi/180;
  VALUE[2] = X_out*sin(Alpha3) + Y_out*cos(Alpha3) + L3O*Theta_out*Pi/180;
	
	
 if (wheel==1)
  return VALUE[0];
 else
 if (wheel==2)
	return VALUE[1];	
 else
 if (wheel==3)
	return VALUE[2];
 else
	return 0; 
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










void ChassisFault(void)
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
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE);
	/* Moter FAULT */
	GPIO_InitStructure.GPIO_Pin = Channel1_FLT|Channel2_FLT|Channel3_FLT;		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	        //需要配置的引脚
	GPIO_Init(Channel1_FLTPORT, &GPIO_InitStructure);
	
	/* Moter RST */
	GPIO_InitStructure.GPIO_Pin = Channel1_RST;								//需要配置的引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	GPIO_Init(Channel1_RSTPORT, &GPIO_InitStructure);
	GPIO_WriteBit(Channel1_RSTPORT,Channel1_RST, Bit_SET);
	
	GPIO_InitStructure.GPIO_Pin = Channel2_RST|Channel3_RST;								//需要配置的引脚
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
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB1Periph_USART3|RCC_APB1Periph_UART5,ENABLE);
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
	
	//No UART5??????????????????
	//No UART5??????????????????
	//No UART5??????????????????
	//No UART5??????????????????
	//No UART5??????????????????
	//No UART5??????????????????
	//No UART5??????????????????
	//No UART5??????????????????
	//No UART5??????????????????
	//No UART5??????????????????
	
}

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





void StepwiseFunction(Chassis_Data *chassis)
{
	chassis->drive_diff1 = chassis->drive_speed1 - chassis->target_speed1; 
	chassis->drive_diff2 = chassis->drive_speed2 - chassis->target_speed2; 
	chassis->drive_diff3 = chassis->drive_speed3 - chassis->target_speed3;
	
	if(chassis->drive_diff1>0)
	{
		chassis->drive_speed1 -= abs(chassis->drive_diff1)/SPEED_STEP;
	}
	else if(chassis->drive_diff1<0)
	{
		chassis->drive_speed1 += abs(chassis->drive_diff1)/SPEED_STEP;
	}

	if(abs(chassis->drive_speed1)<SPEED_CONSTANT_MIN)
	{
		chassis->drive_speed1 = 0;
	}

	if(chassis->drive_diff2>0)
	{
		chassis->drive_speed2 -= abs(chassis->drive_diff2)/SPEED_STEP;			
	}
	else if(chassis->drive_diff2<0)
	{
		chassis->drive_speed2 += abs(chassis->drive_diff2)/SPEED_STEP;
	}
	if(abs(chassis->drive_speed2)<SPEED_CONSTANT_MIN)
	{
		chassis->drive_speed2 = 0;
	}

	if(chassis->drive_diff3>0)
	{
		chassis->drive_speed3 -= abs(chassis->drive_diff3)/SPEED_STEP;
	}
	else if(chassis->drive_diff3<0)
	{
		chassis->drive_speed3 += abs(chassis->drive_diff3)/SPEED_STEP;
	}
	if(abs(chassis->drive_speed3)<SPEED_CONSTANT_MIN)
	{
		chassis->drive_speed3 = 0;
	}
}




void ChassisMotionCtrl(Chassis_Data *ch)
{
	int ii;	
	switch(ch->motion_command)
	{
		case 0:		// No keys is pressed --> stop the chassis
			ch->target_speed1  = 0; ch->target_speed2  = 0; ch->target_speed3  = 0;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			break;
		case 1:		// Key D is pressed	--> right turning
			ch->target_speed1  = 0; ch->target_speed2  = 0; ch->target_speed3  = SPEED_CONSTANT/2;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			break;
		case 2:		// Key S is pressed	--> move backward
			// compute the resultant speed
			ch->result_speed_x = 0;
			ch->result_speed_y = 0;
			for(ii=0;ii<360;ii++)
			{
				if(ii>89 && ii<271)
				{
					ch->result_speed_x += repul_force_x[ii];
					ch->result_speed_y += repul_force_y[ii];
				}
			}
			ch->result_speed_y += -SPEED_CONSTANT;
				
			// reverse motion filtering
			if(ch->result_speed_y>0)
			{
				ch->result_speed_y = 0;
			}
			
			ch->result_speed = sqrt(ch->result_speed_x * ch->result_speed_x + ch->result_speed_y * ch->result_speed_y);
			if(ch->result_speed > SPEED_CONSTANT)
			{
					ch->result_speed_x = ch->result_speed_x*SPEED_CONSTANT/ch->result_speed;
					ch->result_speed_y = ch->result_speed_y*SPEED_CONSTANT/ch->result_speed;
			}
			else if(ch->result_speed<SPEED_CONSTANT_MIN)
			{
				ch->result_speed_x = 0;
				ch->result_speed_y = 0;
			}
					
			ch->target_speed1  = ch->result_speed_x; 
			ch->target_speed2  = ch->result_speed_y; 
			ch->target_speed3  = 0;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);
			
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			break;
		case 4:		// Key A is pressed	--> left turning
			ch->target_speed1  = 0; ch->target_speed2  = 0; ch->target_speed3  = -SPEED_CONSTANT/2;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			break;
		case 8:		// Key E is pressed	--> move to right
			
			// compute the resultant speed
			ch->result_speed_x = 0;
			ch->result_speed_y = 0;
			for(ii=0;ii<360;ii++)
			{
				if(ii>-1 && ii<181)
				{
					ch->result_speed_x += repul_force_x[ii];
					ch->result_speed_y += repul_force_y[ii];
				}
			}
			ch->result_speed_x += SPEED_CONSTANT;

			// reverse motion filtering
			if(ch->result_speed_x<0)
			{
				ch->result_speed_x = 0;
			}		
			ch->result_speed = sqrt(ch->result_speed_x*ch->result_speed_x + ch->result_speed_y*ch->result_speed_y);
			if(ch->result_speed > SPEED_CONSTANT)
			{
					ch->result_speed_x = ch->result_speed_x*SPEED_CONSTANT/ch->result_speed;
					ch->result_speed_y = ch->result_speed_y*SPEED_CONSTANT/ch->result_speed;
			}
			else if(ch->result_speed<SPEED_CONSTANT_MIN)
			{
				ch->result_speed_x = 0;
				ch->result_speed_y = 0;
			}

			
			ch->target_speed1  = ch->result_speed_x; 
			ch->target_speed2  = ch->result_speed_y; 
			ch->target_speed3  = 0;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);
			
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			break;
		case 16:		// Key W is pressed	--> move forward
			// compute the resultant speed
			ch->result_speed_x = 0;
			ch->result_speed_y = 0;
			for(ii=0;ii<360;ii++)
			{
				if((ii>-1&&ii<91)||(ii>269&&ii<360))
				{
					ch->result_speed_x += repul_force_x[ii];
					ch->result_speed_y += repul_force_y[ii];
				}
			}
			ch->result_speed_y += SPEED_CONSTANT;
			
			// reverse motion filtering
			if(ch->result_speed_y<0)
			{
				ch->result_speed_y = 0;	
			}
		
			ch->result_speed = sqrt(ch->result_speed_x*ch->result_speed_x + ch->result_speed_y*ch->result_speed_y);
			if(ch->result_speed > SPEED_CONSTANT)
			{
					ch->result_speed_x = ch->result_speed_x*SPEED_CONSTANT/ch->result_speed;
					ch->result_speed_y = ch->result_speed_y*SPEED_CONSTANT/ch->result_speed;
			}
			else if(ch->result_speed<SPEED_CONSTANT_MIN)
			{
				ch->result_speed_x = 0;
				ch->result_speed_y = 0;
			}
			
			
			ch->target_speed1  = ch->result_speed_x; ch->target_speed2  = ch->result_speed_y; ch->target_speed3  = 0;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);
						
			 
// 			sprintf(message,"%d %d %d \r\n",target_speed1, target_speed2,123);
// 			USART1_Puts(message);
			
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);		
			break;
		case 17:		// Keys W + D are pressed --> turn right [forward]
			
			// compute the resultant speed
			ch->result_speed_x = 0;
			ch->result_speed_y = 0;
			for(ii=0;ii<360;ii++)
			{
				if((ii>-1&&ii<136)||(ii>314&&ii<360))
				{
					ch->result_speed_x += repul_force_x[ii];
					ch->result_speed_y += repul_force_y[ii];
				}
			}
			ch->result_speed_y += SPEED_CONSTANT;

			// reverse motion filtering
			if(ch->result_speed_y<0)
			{
				ch->result_speed_y = 0;	
			}
			if(ch->result_speed > SPEED_CONSTANT)
			{
					ch->result_speed_x = ch->result_speed_x*SPEED_CONSTANT/ch->result_speed;
					ch->result_speed_y = ch->result_speed_y*SPEED_CONSTANT/ch->result_speed;
			}
			else if(ch->result_speed<SPEED_CONSTANT_MIN)
			{
				ch->result_speed_x = 0;
				ch->result_speed_y = 0;
			}

			ch->target_speed1  = ch->result_speed_x; ch->target_speed2  = ch->result_speed_y; ch->target_speed3  = 0;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);
			
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			break;
		case 20:		// Keys W + A are pressed --> turn left [forward]
			
			// compute the resultant speed
			ch->result_speed_x = 0;
			ch->result_speed_y = 0;
			for(ii=0;ii<360;ii++)
			{
				if((ii>-1 && ii<46)||(ii>224&&ii<360))
				{
					ch->result_speed_x += repul_force_x[ii];
					ch->result_speed_y += repul_force_y[ii];
				}
			}
			ch->result_speed_y += SPEED_CONSTANT;
			
			// reverse motion filtering
			if(ch->result_speed_y<0)
			{
				ch->result_speed_y = 0;	
			}

			ch->result_speed = sqrt(ch->result_speed_x*ch->result_speed_x + ch->result_speed_y*ch->result_speed_y);
			if(ch->result_speed > SPEED_CONSTANT)
			{
					ch->result_speed_x = ch->result_speed_x*SPEED_CONSTANT/ch->result_speed;
					ch->result_speed_y = ch->result_speed_y*SPEED_CONSTANT/ch->result_speed;
			}
			else if(ch->result_speed<SPEED_CONSTANT_MIN)
			{
				ch->result_speed_x = 0;
				ch->result_speed_y = 0;
			}

			ch->target_speed1  = ch->result_speed_x; ch->target_speed2  = ch->result_speed_y; ch->target_speed3  = 0;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);

			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			
			break;
		case 24:		  // Keys W + E are pressed	--> move to right [forward]
			
			// compute the resultant speed
			ch->result_speed_x = 0;
			ch->result_speed_y = 0;
			for(ii=0;ii<360;ii++)
			{
				if((ii>-1 && ii<136)||(ii>314&&ii<360))
				{
					ch->result_speed_x += repul_force_x[ii];
					ch->result_speed_y += repul_force_y[ii];
				}
			}
			ch->result_speed_y += SPEED_CONSTANT;
			
			// reverse motion filtering
			if(ch->result_speed_y<0)
			{
				ch->result_speed_y = 0;	
			}
			
			ch->result_speed = sqrt(ch->result_speed_x*ch->result_speed_x + ch->result_speed_y*ch->result_speed_y);
			if(ch->result_speed > SPEED_CONSTANT)
			{
					ch->result_speed_x = ch->result_speed_x*SPEED_CONSTANT/ch->result_speed;
					ch->result_speed_y = ch->result_speed_y*SPEED_CONSTANT/ch->result_speed;
			}
			else if(ch->result_speed<SPEED_CONSTANT_MIN)
			{
				ch->result_speed_x = 0;
				ch->result_speed_y = 0;
			}

			ch->target_speed1  = ch->result_speed_x; ch->target_speed2  = ch->result_speed_y; ch->target_speed3  = 0;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);
	
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			break;
		case 30:
		break;
		case 32:		// Key Q is pressed --> move to left
			
			// compute the resultant speed
			ch->result_speed_x = 0;
			ch->result_speed_y = 0;
			for(ii=0;ii<360;ii++)
			{
				if(ii>179&&ii<360)
				{
					ch->result_speed_x += repul_force_x[ii];
					ch->result_speed_y +=repul_force_y[ii];
				}
			}
			ch->result_speed_x += -SPEED_CONSTANT;
			
			// reverse motion filtering
			if(ch->result_speed_x>0)
			{
				ch->result_speed_x = 0;
			}

			ch->result_speed = sqrt(ch->result_speed_x*ch->result_speed_x + ch->result_speed_y*ch->result_speed_y);
			if(ch->result_speed > SPEED_CONSTANT)
			{
					ch->result_speed_x = ch->result_speed_x*SPEED_CONSTANT/ch->result_speed;
					ch->result_speed_y = ch->result_speed_y*SPEED_CONSTANT/ch->result_speed;
			}
			else if(ch->result_speed<SPEED_CONSTANT_MIN)
			{
				ch->result_speed_x = 0;
				ch->result_speed_y = 0;
			}

			ch->target_speed1  = ch->result_speed_x; ch->target_speed2  = ch->result_speed_y; ch->target_speed3  = 0;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);
		
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			break;
		case 48:		// Keys W + Q are pressed --> move to left [forward]
			
			// compute the resultant speed
			ch->result_speed_x = 0;
			ch->result_speed_y = 0;
			for(ii=0;ii<360;ii++)
			{
				if((ii>-1 && ii<46)||(ii>224 && ii<360))
				{
					ch->result_speed_x += repul_force_x[ii];
					ch->result_speed_y += repul_force_y[ii];
				}
			}
			ch->result_speed_y += SPEED_CONSTANT;
			
			// reverse motion filtering
			if(ch->result_speed_y<0)
			{
				ch->result_speed_y = 0;	
			}

			ch->result_speed = sqrt(ch->result_speed_x*ch->result_speed_x + ch->result_speed_y*ch->result_speed_y);
			if(ch->result_speed > SPEED_CONSTANT)
			{
					ch->result_speed_x = ch->result_speed_x*SPEED_CONSTANT/ch->result_speed;
					ch->result_speed_y = ch->result_speed_y*SPEED_CONSTANT/ch->result_speed;
			}
			else if(ch->result_speed<SPEED_CONSTANT_MIN)
			{
				ch->result_speed_x = 0;
				ch->result_speed_y = 0;
			}

			ch->target_speed1  = ch->result_speed_x; ch->target_speed2  = ch->result_speed_y; ch->target_speed3  = 0;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);
		
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			break;
		default:
			DRIVE1.V_SET=chassis_drive(0,0,0,1);
			DRIVE2.V_SET=chassis_drive(0,0,0,2);
			DRIVE3.V_SET=chassis_drive(0,0,0,3);
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			break;
	}
}










void delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}










void CAN1Master_heartbeatError(CO_Data* d,  UNS8 heartbeatID)
{
	d->NMTable[heartbeatID]=Unknown_state;
    printf(" HeartBeat not received from node : %d \r\n",heartbeatID);
}

void CAN1Master_initialisation(CO_Data* d)
{
  	printf(" CAN1 Entering in INIT \r\n");
}
void CAN1Master_preOperational(CO_Data* d)
{
	printf(" CAN1 Entering in PRE-OPERATIONAL \r\n");
	(void)masterSendNMTstateChange(d, 0, NMT_Reset_Node);
	//BodyMaster_boottimeInit(d);  //start a timer f or boot time
}
void CAN1Master_operational(CO_Data* d)
{
 	printf(" CAN1 Entering in OPERATIONAL \r\n");
	 
}
void CAN1Master_stopped(CO_Data* d)
{
    printf(" CAN1 Entering in STOPPED \r\n");
}
void CAN1Master_SlaveBootup(CO_Data* d, UNS8 SlaveID)
{
    printf(" receive Bootup SlaveId:%d \r\n",SlaveID);
	(void)masterSendNMTstateChange (d, SlaveID, NMT_Start_Node);
}
void CAN1Master_emcy(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg)
{
    printf(" there is node in emcy state\r\n");
}

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
