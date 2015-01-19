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


/*TIM2 variables*/
TIM_TimeBaseInitTypeDef  TIM2_TimeBaseStructure;
TIM_OCInitTypeDef  TIM2_OCInitStructure;

__IO uint16_t CCR1_Val = 54618;
__IO uint16_t CCR2_Val = 27309;
__IO uint16_t CCR3_Val = 13654;
__IO uint16_t CCR4_Val = 6826;

uint16_t PrescalerValue = 0;



/**
 * Moter of Chassis initial
 * @brief CANOpen Data Scan, if CANbus got a CAN message, then push the message into
 *        queue(xQ_CAN_MSG), this thread scan the queue at CANOpen_THREAD_SCAN_TIMER(
 *        default:20ms) rate.
 */
void Motor_Init(void)
{
	RCC_Configuration();
	GPIO_Configuration();
	NVIC_Configuration(); 
	TIM3_Configuration(); 
	TIM4_Configuration();
	TIM5_Configuration(); 	  
	TIM2_Config();
	TIM2_CCR_CALC();
}


/**
  * @brief  RCC configuration
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	
	/* GPIOC clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	/* TIM4 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	
	/* GPIOD|GPIOB clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOB, ENABLE);	 
	
	/* TIM5 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	
	/* GPIOA clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	
}
			    
/**
  * @brief  NVIC configuration
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
   NVIC_InitTypeDef NVIC_InitStructure;
 
   /* Configure the NVIC Preemption Priority Bits */
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
    
   /* Enable the TIM3 Interrupt */
   NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

   /* Enable the TIM4 Interrupt */
   NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure); 	
   
   /* Enable the TIM5 Interrupt */
   NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);   
}
 
/**
  * @brief  GPIO configuration
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* TIM3 *****************************************************************************/
	/* GPIOC Configuration: TIM3 CH1 (PC6) | TIM3 CH2 (PC7) | TIM3 CH3 (PC8) | TIM3 CH4 (PC9), 12162013 new version*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOC, &GPIO_InitStructure); 		
	/* Connect TIM3 pins to AF2 */ 
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);

	/* TIM4 *****************************************************************************/
	/* GPIOD Configuration: TIM4 CH3 (PD14) and TIM4 CH4 (PB9) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOD, &GPIO_InitStructure); 		
	/* Connect TIM4 pins to AF2 */ 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	
	/* GPIOB Configuration: TIM4 CH3 (PD14) and TIM4 CH4 (PB9) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 		
	/* Connect TIM4 pins to AF2 */ 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4); 
 
	/* TIM5 *****************************************************************************/
	/* GPIOA Configuration: TIM5 CH1 (PA0) | TIM5 CH2 (PA1) | TIM5 CH3 (PA2) | TIM5 CH4 (PA3) | */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 		
	/* Connect TIM5 pins to AF2 */ 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5); 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5); 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5); 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5); 
 
}
 
 
/**
  * @brief  TIM3 configuration
  * @param  None
  * @retval None
  */
void TIM3_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	// Simplify the math here so TIM_Pulse has 1 us units 
	// Use (24-1) for VL @ 24 MHz
	// Use (72-1) for STM32 @ 72 MHz
	// Use (84-1) for STM32 @ 84 MHz 
	//TIM_TimeBaseStructure.TIM_Prescaler = 24 - 1;  // 24 MHz / 24 = 1 MHz, original  
	//TIM_TimeBaseStructure.TIM_Prescaler = 16 - 1;  // 24 MHz / 24 = 1 MHz, used		 
	//TIM_TimeBaseStructure.TIM_Period = 20000 - 1; // 1 MHz / 20000 = 50 Hz (20 ms)  

	/*TIM_TimeBaseStructure.TIM_Prescaler = 3;  // 24 MHz / 24 = 1 MHz, used	
	TIM_TimeBaseStructure.TIM_Period = 1049; // 1 MHz / 50 = 20 kHz , for pump*/	
	
	TIM_TimeBaseStructure.TIM_Prescaler = 0;  
	TIM_TimeBaseStructure.TIM_Period = (SystemCoreClock / 40000 ) - 1 - 1; //20 kHz , for pump	
	
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	  
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	// Set up 4 channel servo 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  
	//TIM_OCInitStructure.TIM_Pulse = 600 + 900; // 1500 us - Servo Top Centre 
	TIM_OCInitStructure.TIM_Pulse = (SystemCoreClock / 40000 ) - 1 - 1;     
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);    // Channel 1 configuration = PC.06 TIM3_CH1
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);    // Channel 2 configuration = PC.07 TIM3_CH2
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);    // Channel 3 configuration = PC.08 TIM3_CH3
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);    // Channel 4 configuration = PC.09 TIM3_CH4	  	
	
	// turning on TIM3 and PWM outputs  
	TIM_Cmd(TIM3, ENABLE);			 
	TIM_CtrlPWMOutputs(TIM3, ENABLE);  
	// TIM IT enable
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}

/**
  * @brief  TIM4 configuration
  * @param  None
  * @retval None
  */
void TIM4_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	// Simplify the math here so TIM_Pulse has 1 us units 
	// Use (24-1) for VL @ 24 MHz
	// Use (72-1) for STM32 @ 72 MHz
	//TIM_TimeBaseStructure.TIM_Prescaler = 24 - 1;  // 24 MHz / 24 = 1 MHz, original  
	//TIM_TimeBaseStructure.TIM_Prescaler = 16 - 1;  // 24 MHz / 24 = 1 MHz, used		 
	//TIM_TimeBaseStructure.TIM_Period = 20000 - 1; // 1 MHz / 20000 = 50 Hz (20 ms)  

	/*TIM_TimeBaseStructure.TIM_Prescaler = 3;  // 24 MHz / 24 = 1 MHz, used	
	TIM_TimeBaseStructure.TIM_Period = 1049; // 1 MHz / 50 = 20 kHz , for pump*/	
	
	TIM_TimeBaseStructure.TIM_Prescaler = 0;  
	TIM_TimeBaseStructure.TIM_Period = (SystemCoreClock / 40000 ) - 1 - 1; //20 kHz , for pump	
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	  
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	// Set up 4 channel servo 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  
	//TIM_OCInitStructure.TIM_Pulse = 600 + 900; // 1500 us - Servo Top Centre 
	//TIM_OCInitStructure.TIM_Pulse = 1049; //  
	TIM_OCInitStructure.TIM_Pulse = (SystemCoreClock / 40000 ) - 1 - 1;     	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);    // Channel 3 configuration = PD14 TIM4_CH3
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);    // Channel 4 configuration = PB9 TIM4_CH4
	
	// turning on TIM4 and PWM outputs  
	TIM_Cmd(TIM4, ENABLE);			 
	TIM_CtrlPWMOutputs(TIM4, ENABLE);  
	// TIM IT enable
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
}

/**
  * @brief  TIM5 configuration
  * @param  None
  * @retval None
  */
void TIM5_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	// Simplify the math here so TIM_Pulse has 1 us units 
	// Use (24-1) for VL @ 24 MHz
	// Use (72-1) for STM32 @ 72 MHz
	//TIM_TimeBaseStructure.TIM_Prescaler = 24 - 1;  // 24 MHz / 24 = 1 MHz, original  
	//TIM_TimeBaseStructure.TIM_Prescaler = 16 - 1;  // 24 MHz / 24 = 1 MHz, used		 
	//TIM_TimeBaseStructure.TIM_Period = 20000 - 1; // 1 MHz / 20000 = 50 Hz (20 ms)  

	/*TIM_TimeBaseStructure.TIM_Prescaler = 3;  // 24 MHz / 24 = 1 MHz, used	
	TIM_TimeBaseStructure.TIM_Period = 1049; // 1 MHz / 50 = 20 kHz , for pump*/	
	
	TIM_TimeBaseStructure.TIM_Prescaler = 0;  
	TIM_TimeBaseStructure.TIM_Period = (SystemCoreClock / 40000 ) - 1 - 1; //20 kHz , for pump	
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	  
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	
	// Set up 4 channel servo 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  
	//TIM_OCInitStructure.TIM_Pulse = 600 + 900; // 1500 us - Servo Top Centre 
	//TIM_OCInitStructure.TIM_Pulse = 1049; //  
	TIM_OCInitStructure.TIM_Pulse = (SystemCoreClock / 40000 ) - 1 - 1;       
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 	
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);    // Channel 1 configuration = PA0 TIM5_CH1
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);    // Channel 2 configuration = PA1 TIM5_CH2
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);    // Channel 3 configuration = PA2 TIM5_CH3
	TIM_OC4Init(TIM5, &TIM_OCInitStructure);    // Channel 4 configuration = PA3 TIM5_CH4  	

	// turning on TIM5 and PWM outputs  
	TIM_Cmd(TIM5, ENABLE);			 
	TIM_CtrlPWMOutputs(TIM5, ENABLE);  
	// TIM IT enable
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
}  

 
/**
  * @brief  TIM2 configuration
  * @param  None
  * @retval None
  */
void TIM2_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* TIM2 clock enable */   
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);		 

  /* Enable the TIM2 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;	//error  TIM1_IRQn
  //NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);							 
}

/**
  * @brief  TIM2 CCR CALC
  * @param  None
  * @retval None
  */
void TIM2_CCR_CALC(void)
{ 
	/* -----------------------------------------------------------------------
	TIM3 Configuration: Output Compare Timing Mode:
	
	In this example TIM4 input clock (TIM4CLK) is set to 2 * APB1 clock (PCLK1), 
	since APB1 prescaler is different from 1.   
	  TIM4CLK = 2 * PCLK1  
	  PCLK1 = HCLK / 4 
	  => TIM4CLK = HCLK / 2 = SystemCoreClock /2
	      
	To get TIM4 counter clock at 50 MHz, the prescaler is computed as follows:
	   Prescaler = (TIM4CLK / TIM4 counter clock) - 1
	   Prescaler = ((SystemCoreClock /2) /50 MHz) - 1
	                                          
	CC1 update rate = TIM4 counter clock / CCR1_Val = 9.154 Hz
	==> Toggling frequency = 4.57 Hz
	
	C2 update rate = TIM4 counter clock / CCR2_Val = 18.31 Hz
	==> Toggling frequency = 9.15 Hz
	
	CC3 update rate = TIM4 counter clock / CCR3_Val = 36.62 Hz
	==> Toggling frequency = 18.31 Hz
	
	CC4 update rate = TIM4 counter clock / CCR4_Val = 73.25 Hz
	==> Toggling frequency = 36.62 Hz
	
	Note: 
	 SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
	 Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
	 function to update SystemCoreClock variable value. Otherwise, any configuration
	 based on this variable will be incorrect.    
	----------------------------------------------------------------------- */  
	
	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 500000) - 1;
	
	/* Time base configuration */
	TIM2_TimeBaseStructure.TIM_Period = 65535;
	TIM2_TimeBaseStructure.TIM_Prescaler = 0;
	TIM2_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM2_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	
	TIM_TimeBaseInit(TIM2, &TIM2_TimeBaseStructure);
	
	/* Prescaler configuration */
	TIM_PrescalerConfig(TIM2, PrescalerValue, TIM_PSCReloadMode_Immediate);
	
	/* Output Compare Timing Mode configuration: Channel1 */
	TIM2_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM2_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM2_OCInitStructure.TIM_Pulse = CCR1_Val;
	TIM2_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  	
	TIM_OC1Init(TIM2, &TIM2_OCInitStructure);	   	
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);
	
	/* Output Compare Timing Mode configuration: Channel2 */
	TIM2_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM2_OCInitStructure.TIM_Pulse = CCR2_Val;	  	
	TIM_OC2Init(TIM2, &TIM2_OCInitStructure);		
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);
	
	/* Output Compare Timing Mode configuration: Channel3 */
	TIM2_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM2_OCInitStructure.TIM_Pulse = CCR3_Val;		 	
	TIM_OC3Init(TIM2, &TIM2_OCInitStructure);	   	
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable);
	
	/* Output Compare Timing Mode configuration: Channel4 */
	TIM2_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM2_OCInitStructure.TIM_Pulse = CCR4_Val;  													 
	TIM_OC4Init(TIM2, &TIM2_OCInitStructure);   	
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);
	
	/* TIM Interrupts enable */
	TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
	
	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}




void MotorFault(void)
{
	
}


void delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}



/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
