/**
  ******************************************************************************
  * @file    BMS2.0_RC/USER/src/initial.c 
  * @author  Intelligent Robotics Team
  * @version V2.0
  * @date    26-March-2014
  * @brief   Program Initial Function
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "system_stm32f10x.c"
//#include "stm32f10x_conf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USART1_DR_Addr    (u32)(&(USART1->DR))
#define USART2_DR_Addr    (u32)(&(USART2->DR))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
RCC_ClocksTypeDef RCC_Clocks; 
ADC_InitTypeDef   ADC_InitStructure;
DMA_InitTypeDef   DMA_InitStructure;

extern u8 CAN_init_SorF;  //bms_tasks.c

/* Variables for test */
unsigned char Message2[8] = {1,0,0,0,0,0,0,0};//For Test, Return to PC
unsigned char Message3[8] = {1,0,0,0,0,0,0,0};//For Test, Return to PC


ErrorStatus HSEStartUpStatus;

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  RCC system reset(for debug purpose)
  * @param  
  * @retval None
  */
void Debug_RCCReset(void)
{
	RCC_DeInit();
	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);
	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	if(HSEStartUpStatus == SUCCESS)
	{
			/* Enable Prefetch Buffer */
			FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
			/* Flash 2 wait state */
			FLASH_SetLatency(FLASH_Latency_2);
			/* HCLK = SYSCLK */
			RCC_HCLKConfig(RCC_SYSCLK_Div1);
			/* PCLK2 = HCLK */
			RCC_PCLK2Config(RCC_HCLK_Div1);
			/* PCLK1 = HCLK/2 */
			RCC_PCLK1Config(RCC_HCLK_Div2);
			/* ADCCLK = PCLK2/4 */
			RCC_ADCCLKConfig(RCC_PCLK2_Div4);
			/* PLLCLK = 8MHz * 9 = 72 MHz */
			RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
			/* Enable PLL */
			RCC_PLLCmd(ENABLE);
			/* Wait till PLL is ready */
			while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
			/* Select PLL as system clock source */
			RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
			/* Wait till PLL is used as system clock source */
			while(RCC_GetSYSCLKSource() != 0x08);
	}
}

/**
  * @brief  Clock Initial
  * @param  file: pointer to the source file name
  * @retval None
  */
void RCCInit(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/* 36 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_CAN1, ENABLE);
	/* 72 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO  | RCC_APB2Periph_GPIOA |
												 RCC_APB2Periph_GPIOB | RCC_APB2Periph_USART1|
												 RCC_APB2Periph_TIM1, ENABLE);
}
/**
  * @brief  NVIC Initial Program
  * @param  Including:
  * 		TIM2_IRQn				      1s
  * 		USB_LP_CAN1_RX0_IRQn	CAN RX Interupt
  *			EXTI15_10_IRQn        
  * @retval None
  */
void NVICInit(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
// 	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
// 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
// 	NVIC_InitStructure.NVIC_IRQChannelSubPriority  = 0;
// 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
// 	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
/**
  * @brief  EXTI Initial Program
  * @param  Including:
			Using the Power_Det PA6 as EXTI Pin to Run into NVIC
  * @retval None
  */
void EXTIInit(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
	
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource6);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line6;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  //EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //PA6 LOW->HIGH
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

/**
  * @brief  DMA Initial Program
  * @param  Including:
			USART1-DMA   
			Buffer: 115200
  * @retval None
  */
void DMAInit(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	DMA_DeInit(DMA1_Channel4);
	DMA_DeInit(DMA1_Channel5);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Addr; 
	DMA_InitStructure.DMA_MemoryBaseAddr     = (u32)Message2;
	DMA_InitStructure.DMA_BufferSize         = T_trsbytes;	                //DMA要传输的字节数
	DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralDST;	    	//方向：从内存读
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;  	//外设地址增量模式：Disable
	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;       	//主机地址增量模式：Enable 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据宽度
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;    	//主机数据宽度
	DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;            	//循环模式    非循环模式
	DMA_InitStructure.DMA_Priority           = DMA_Priority_High;          	//高优先值 
	DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;            	//Disable存储器到存储器模式
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel4, ENABLE);	                                      	//发送通道 7 通过串口2发出去
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Addr; 
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)Message3;
	DMA_InitStructure.DMA_BufferSize = T_trsbytes;	 //DNA要传输的字节数
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	 //方向：从外设读
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 	//外设地址增量模式：Disable
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //主机地址增量模式：Enable 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //外设数据宽度
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 	//主机数据宽度
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 	  //循环模式    非循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;  //高优先值 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //Disable存储器到存储器模式
	DMA_Init(DMA1_Channel5, &DMA_InitStructure); 
	//DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE); //原来就有的
	DMA_Cmd(DMA1_Channel5, ENABLE);  //接收通道*/
}
/**
  * @brief  GPIO Initial Program
  * @param  Including:
						USART1      PA9/TX 		PA10/RX
						LED         PA0			  PA1				PA2
						CHARGELED		

  * @retval None
  */
void GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOC);

	/* Power Saving */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* U1_Tx */	  	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;						  				  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				   
	GPIO_Init(GPIOA, &GPIO_InitStructure);						   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	

  /* LED LED LED LED LED LED LED LED LED LED LED LED LED LED LED LED LED LED */
	//STATUS LED: 1-Work | 2-Charge | 3-CAN |
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
 
	//Battery LED
//	GPIO_InitStructure.GPIO_Pin = REDPIN;  //LED-RED
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
//	GPIO_Init(REDPORT, &GPIO_InitStructure);
// 
//	GPIO_InitStructure.GPIO_Pin = BLUEPIN; //LED-BLUE
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
//	GPIO_Init(BLUEPORT, &GPIO_InitStructure);
	
  //Battery LED - PWM GPIO Init
  GPIO_InitStructure.GPIO_Pin = REDPIN|BLUEPIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(REDPORT, &GPIO_InitStructure);
  GPIO_Init(BLUEPORT, &GPIO_InitStructure);

  //Chagre LED
	GPIO_InitStructure.GPIO_Pin = CHARGE_LED_PIN; //LED
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(CHARGE_LED_PORT, &GPIO_InitStructure);
	
	/* CHARGE_DET */
	GPIO_InitStructure.GPIO_Pin = CHARGE_DET_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //Notice here
	GPIO_Init(CHARGE_DET_PORT, &GPIO_InitStructure);
	/* CHARGE_DET */
	
	/* DC-DC Charging Control*/  
	GPIO_InitStructure.GPIO_Pin = DCDC_PIN;       
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //Notice here
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DCDC_PORT, &GPIO_InitStructure);
	/* DC-DC Charging Control*/
	
	//CHARGE_Switch
	GPIO_InitStructure.GPIO_Pin = CHARGE_SWITCH_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(CHARGE_SWTICH_PORT, &GPIO_InitStructure);
	//CHARGE_Switch
 
//   //testtesttesttest
//   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);
// 	GPIO_ResetBits(GPIOA,GPIO_Pin_6);  //控制DC-DC  初始化拉低，充电时延迟1.5s后拉高，不充电时拉低
//   //testtesttesettest
 
 
 
	//MOS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;       //LED
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//MOS
 
	//IIC
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;  //4 SDA
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //5 SCL
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//IIC


//   //CAN PIN
// 	GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);
// 	/* Configure CAN pin: RX */ 
//   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
//   GPIO_Init(GPIOB, &GPIO_InitStructure); 
// 	
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //CAN RX PIN
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
// 	GPIO_Init(GPIOB, &GPIO_InitStructure);			
//   //CAN PIN





   //CAN PIN
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; //CAN RX PIN
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	//CAN TX PIN
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					     
  //CAN PIN
	
	/*  */
	GPIO_ResetBits(GPIOA,GPIO_Pin_0); 
	GPIO_SetBits(GPIOA,GPIO_Pin_1);
	//GPIO_SetBits(GPIOA,GPIO_Pin_2);
	
	//GPIO_SetBits(GPIOA,GPIO_Pin_2);  //test-DC-DC
	//Test 2nd Time: 
	//GPIO_SetBits(GPIOA,GPIO_Pin_6);  //控制DC-DC  初始化拉高，充电时延迟1.5s后拉低，不充电时拉高
	GPIO_SetBits(GPIOA,GPIO_Pin_2);  //test-DC-DC
}
/**
  * @brief  Serial/USART Initial
  * @retval None
  */
void USARTInit(void)
{
    //U1
	USART_InitTypeDef USART_InitStructure;		
	USART_DeInit(USART1);
  USART_DeInit(USART3);
	
	USART_InitStructure.USART_BaudRate            = BaudRate; 
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b; 
	USART_InitStructure.USART_StopBits            = USART_StopBits_1; 
	USART_InitStructure.USART_Parity              = USART_Parity_No ; 
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
	USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx; 
	
	USART_Init(USART1, &USART_InitStructure);
	USART_DMACmd(USART1,USART_DMAReq_Tx, ENABLE);	 //Only TX
	USART_DMACmd(USART1,USART_DMAReq_Rx, ENABLE);  //Test-USART-RX
	USART_Cmd(USART1, ENABLE);
		
  USART_InitStructure.USART_BaudRate            = BaudRate; 
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b; 
	USART_InitStructure.USART_StopBits            = USART_StopBits_1; 
	USART_InitStructure.USART_Parity              = USART_Parity_No ; 
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
	USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx; 
	
	USART_Init(USART3, &USART_InitStructure);
	USART_DMACmd(USART3,USART_DMAReq_Tx, ENABLE);	 //Only TX
	USART_Cmd(USART3, ENABLE);
}

/**
  * @brief  TIMER Initial
  * @param  Clock: 
					TIM1: 
					TIM2: 8M ->AHB/1->8M->APB/2->4M->x2->8M
  * @retval None
  */		
void TIMInit(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
	TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	
  //TIM2 - Basic Timer For BMS
  TIM_DeInit(TIM2);
  TIM_TimeBaseStructure.TIM_Prescaler   = 320;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	
  //TIM_TimeBaseStructure.TIM_Period      = 4000;
	TIM_TimeBaseStructure.TIM_Period      = 25000;    //1s
  TIM_TimeBaseStructure.TIM_ClockDivision     = 0; 
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; 
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM2, ENABLE);  // Enable TIM2
	
	
	//TIM1 - PWM Clock For Battery LED
  TIM_TimeBaseStructure.TIM_Prescaler         = 0;
  TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period            = 9000-1;
  TIM_TimeBaseStructure.TIM_ClockDivision     = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;  //RED -x1 PA8
  TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse        = 0;
  TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;	 
  TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_Low;  
  TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	
  TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;  //GREEN -x2 PB15
  TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse        = 0;
  TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	
  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);
  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
  //---------------TIM1
}
/**
  * @brief  bxCAN Intial
  * @retval None
  */
void CAN_Configuration(void)
{
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;

  /* CAN register init */
  CAN_ITConfig(CAN1,CAN_IT_FMP0, DISABLE);    //CAN RX Interupt Disable
  CAN_DeInit(CAN1);													  //CAN1 Initial
  CAN_StructInit(&CAN_InitStructure);
  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = ENABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = ENABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;		            //FIFO Lock Mode
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; 
  CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;     // 1 . for //          
  CAN_InitStructure.CAN_BS1  = CAN_BS1_4tq;	    // 4 .  1  //           
  CAN_InitStructure.CAN_BS2  = CAN_BS2_3tq;     // 3 .  M  //             
  CAN_InitStructure.CAN_Prescaler = 1;          // 1 . bit //CAN = RCC_APB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler;)
  CAN_init_SorF = CAN_Init(CAN1 ,&CAN_InitStructure);
	/* CAN filter init */

	/* Only Recive CAN Message of 0x015  */
//   CAN_FilterInitStructure.CAN_FilterNumber     = 0;                     //0~13
//   CAN_FilterInitStructure.CAN_FilterMode       = CAN_FilterMode_IdMask; //ID MASK MODE
//   CAN_FilterInitStructure.CAN_FilterScale      = CAN_FilterScale_32bit; //Filter Scale 32bit
//   CAN_FilterInitStructure.CAN_FilterIdHigh     = (((u32)0x015<<21)&0xFFFF0000)>>16;
//   CAN_FilterInitStructure.CAN_FilterIdLow      = (((u32)0x015<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF;
//   CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;  //  FxR1: xxxx xxx0 101x xxxx xxxx xxxx xxxx x00x
//   CAN_FilterInitStructure.CAN_FilterMaskIdLow  = 0xFFFF;  //  FxR2: 0000 0001 1110 0000 0000 0000 0000 0110
//   CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
//   CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
//   CAN_FilterInit(&CAN_FilterInitStructure);		
	
 	/* Only Recive CAN Message of 0x015 & 0x035  */
  CAN_FilterInitStructure.CAN_FilterNumber     = 0;                     //0~13
  CAN_FilterInitStructure.CAN_FilterMode       = CAN_FilterMode_IdMask; //ID MASK MODE
  CAN_FilterInitStructure.CAN_FilterScale      = CAN_FilterScale_32bit; //Filter Scale 32bit
  CAN_FilterInitStructure.CAN_FilterIdHigh     = (((u32)0x015<<21)&0xFFFF0000)>>16;
  CAN_FilterInitStructure.CAN_FilterIdLow      = (((u32)0x015<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x01EF;  //  FxR1: xxxx xxx0 101x xxxx xxxx xxxx xxxx x00x
  CAN_FilterInitStructure.CAN_FilterMaskIdLow  = 0xFFFF;  //  FxR2: 0000 0001 1110 0000 0000 0000 0000 0110
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);		
	
// 	/* TEST */
// 	CAN_FilterInitStructure.CAN_FilterNumber     = 0;                     //0~13
//   CAN_FilterInitStructure.CAN_FilterMode       = CAN_FilterMode_IdMask; //ID MASK MODE
//   CAN_FilterInitStructure.CAN_FilterScale      = CAN_FilterScale_32bit; //Filter Scale 32bit
//   CAN_FilterInitStructure.CAN_FilterIdHigh     = 0x0000;
//   CAN_FilterInitStructure.CAN_FilterIdLow      = 0x0000;
//   CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;  //  FxR1: xxxx xxx0 101x xxxx xxxx xxxx xxxx x00x
//   CAN_FilterInitStructure.CAN_FilterMaskIdLow  = 0x0000;  //  FxR2: 0000 0001 1110 0000 0000 0000 0000 0110
//   CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
//   CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
//   CAN_FilterInit(&CAN_FilterInitStructure);		

  CAN_ITConfig(CAN1 ,CAN_IT_FMP0, ENABLE);
  /* initialize the value that will be returned */
}	

/**
  * @brief  Control Command to BQ34z100 Block Initial 
  * @retval None
  */
void DataBlockInit()
{
  /* BQ34z100->BQZ Struct Init */	
  //BQZ.CNTL 	= 0x00;			//Control()               	  0x00/0x01 N/A
  BQZ.SOC 		= 0x02;			//StateOfCharge()      			  0x02/0x03	%
  BQZ.RM			= 0x04;			//RemainingCapacity()  			  0x04/0x05	mAh
  BQZ.FCC			= 0x06;			//FullChargeCapacity() 			  0x06/0x07	mAh
  BQZ.VOLT		= 0x08;			//Voltage()                   0x08/0x09	mV
  BQZ.AI			= 0x0a;			//AverageCurrent()            0x0a/0x0b	mA
  BQZ.TEMP		= 0x0c;			//Temperature()               0x0c/0x0d	0.1°K
  BQZ.FLAGS		= 0x0e;			//Flags()             			  0x0e/0x0f	N/A
  BQZ.AR			= 0x10;  		//AtRate()             			  0X10/0x11	mA
  BQZ.ARTTE		= 0x12;			//AtRateTimeToEmpty() 			  0x12/0x13 Minutes
  BQZ.NAC			= 0x14;			//NominalAvailableCapacity() 	0x14/0x15 mAh
  BQZ.FAC			= 0x16;			//FullAvailableCapacity()   	0x16/0x17 mAh
  BQZ.TTE			= 0x18;			//TimeToEmpty()  				      0x18/0x19 Minutes
  BQZ.TTF			= 0x1a;			//TimeToFull()  				      0x1a/0x1b Minutes
  BQZ.SI			= 0x1c;			//StandbyCurrent()				    0x1c/0x1d	mA
  BQZ.STTE		= 0x1e;     //StandbyTimeToEmpty()			  0x1e/0x1f Minutes
  BQZ.MLI			= 0x20;			//MaxLoadCurrent()				    0x20/0x21 mA
  BQZ.MLTTE		= 0x22;			//MaxLoadTimeToEmpty()			  0x22/0x23 Minutes
  BQZ.AE			= 0x24;     //AvailableEnergy()           0x24/0x25 10 mWhr
  BQZ.AP			= 0x26;   	//AveragePower()				      0x26/0x27 10 mW
  BQZ.TTECP		= 0x28;			//TTEatConstantPower()  		  0x128/0x29 Minutes
  BQZ.INTTEMP	= 0x2a;		 	//Internal_Temp()				      0x2a/0x2b 0.1°K
  BQZ.CC			= 0x2c;			//CycleCount()       			    0x2c/0x2d Counts
  BQZ.SOH			= 0x2e; 		//StateOfHealth()				      0x2e/0x2f	%/num
  BQZ.CHGV		= 0x30;			//ChargeVoltage()				      0x30/0x31 mV
  BQZ.CHGI		= 0x32;   	//ChargeCurrent()  				    0x32/0x33 mA
  BQZ.PCHG		= 0x34;			//PassedCharge()				      0x34/0x35 mAh
  BQZ.DOD0		= 0x36;			//DOD0()						          0x36/0x37 HEX#
  BQZ.SDSG		= 0x38;			//SelfDischargeCurrent			  0x38/0x39 mA
  BQZ.PKCFG		= 0x3a;			//PackConfiguration()  			  0x3a/0x3b N/A
  BQZ.DCAP		= 0x3c;			//DesignCapacity() 				    0x3c/0x3d mAh
  
  /* TIMER Struct Init */
  TimerFlag.LED = 0;
  TimerFlag.BQZ = 0;
  TimerFlag.ARM = 0;
	TimerFlag.CHASSIS = 0;
  /* TIMER Struct Init */
}


/*** (C) COPYRIGHT 2014 CSST R&D Intelligent Robotics Research Centre *END OF FILE*/
