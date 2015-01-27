/*
*********************************************************************************************************
*
*	模块名称 : BSP-TIM模块
*	文件名称 : bsp_led.c
* 作    者 ：Jim
* 版    本 ：1.0
* 日    期 ：2014.08.25
*
*	说    明 : 
* 	该程序适用于CSST替身机器人ARM控制板，如果用于其它硬件，请修改相应定义
*		CSST-STM32F407 开发板LED口线分配：
*		LED1	:	PE2		低电平点亮，高电平熄灭
*		LED2	: PE3		低电平点亮，高电平熄灭
*		LED3	: PE4		低电平点亮，高电平熄灭
*		LED4	: PE5		低电平点亮，高电平熄灭
*
* 修改记录 :
* 	版本号		日期				作者				说明
* 	V1.0		2014-08-24		Jim				创建文件，添加基本功能描述
*
*	Copyright (C), 2013-2014
*
*********************************************************************************************************
*/

#include "bsp.h"

/* 定义对应的RCC时钟及端口 */
// #define RCC_LED RCC_AHB1Periph_GPIOE
// #define LEDPORT GPIOE
// #define LED1    GPIO_Pin_2
// #define LED2    GPIO_Pin_3
// #define LED3    GPIO_Pin_4
// #define LED4    GPIO_Pin_5

/* ARM开发板指示灯定义 */
#define RCC_LED RCC_AHB1Periph_GPIOA
#define LEDPORT GPIOA
#define LED1    GPIO_Pin_4
#define LED2    GPIO_Pin_5
#define LED3    GPIO_Pin_6
#define LED4    GPIO_Pin_7

/**
  * @brief  Sets the TIMx Capture Compare1 Register value
  * @param  TIMx: where x can be 1 to 14 except 6 and 7, to select the TIM peripheral.
  * @param  Compare1: specifies the Capture Compare1 register new value.
  * @retval None
  */
void bsp_InitTim(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 打开TIM时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);
	
	//T1PWM
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIO_InitStructure); 
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
	//T1PWM
	
	
	
// 	uint16_t PrescalerValue = 0;	// Prescaler of the Timer5

//   TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
//   TIM_OCInitTypeDef TIM_OCInitStructure;   
// 	//--------------TIM1的配置
//   TIM_TimeBaseStructure.TIM_Prescaler = 100;
//   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//   TIM_TimeBaseStructure.TIM_Period = HALFPWMTIMERPERIOD*2;
//   TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//   TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//   TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
//   
//   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//   TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
//   TIM_OCInitStructure.TIM_Pulse = 0;
//   TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	  //还没匹配，高!
//   TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;  //匹配之后，低!
//   TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//   TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
//   //TIM_OC1Init(TIM1, &TIM_OCInitStructure); 
//   TIM_OC2Init(TIM1, &TIM_OCInitStructure);
// 	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
//   TIM_OC4Init(TIM1, &TIM_OCInitStructure);
//   TIM_Cmd(TIM1, ENABLE);
//   TIM_CtrlPWMOutputs(TIM1, ENABLE);
//   //---------------TIM1的配置

//   //--------------TIM5的配置
//   /* Compute the prescaler value */
//   // System clock 100MHz
//   PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 24000000) - 1;


//   /* TIM5 clock enable */
//   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
//   /* Time base configuration */
//   TIM_TimeBaseStructure.TIM_Period = 800000;  //    Timer5: 24000000/1000000 = 24 hz
//   TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue; 
//   TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//   TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
//   /* TIM IT enable */
//   TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
//   /* TIM5 enable counter */
//   TIM_Cmd(TIM5, ENABLE);
//   //--------------TIM5的配置
  

	/*
		配置所有的LED指示灯GPIO为推挽输出模式
		由于将GPIO设置为输出时，GPIO输出寄存器的值缺省是0，因此会驱动LED点亮.
		这是我不希望的，因此在改变GPIO为输出前，先关闭LED指示灯
	*/
// 	bsp_LedOff(1);
// 	bsp_LedOff(2);
// 	bsp_LedOff(3);
// 	bsp_LedOff(4);
	
	bsp_LedOn(1);
	bsp_LedOn(2);
	bsp_LedOn(3);
	bsp_LedOn(4);
	
	GPIO_InitStructure.GPIO_Pin = LED1|LED2|LED3|LED4;		 /* 设为输出口 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;/* 设为推挽模式 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; /* 上下拉电阻不使能 */
	GPIO_Init(LEDPORT, &GPIO_InitStructure);
	//GPIO_WriteBit(LEDPORT, LED1|LED2|LED3|LED4, Bit_SET);
}

/***********************************************************************************************/
