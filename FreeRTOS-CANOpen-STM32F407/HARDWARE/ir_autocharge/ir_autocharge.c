/**
  ******************************************************************************
  * @file    ir_autocharge.c
  * @author  Jim
  * @version V1.0
  * @date    16-Jan-2015
  * @brief   This file provides ir_autocharge functions 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */
#include "ir_autocharge.h"
#include "stm32f4xx.h"

AUTO_CHARGE Auto_charge;

void SENSOR_IR_Autocharge_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure; 
	EXTI_InitTypeDef	EXTI_InitStructure;
	
	/* GPIO Config ------------------------------------------------------------------*/
	/* Enable GPIOG Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	//EXTI 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;       //输入模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;   //上拉
  GPIO_Init(GPIOG, &GPIO_InitStructure);           
  //EXTI
	

	/* NVIC Config ------------------------------------------------------------------*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	  
	/* EXTI Config ------------------------------------------------------------------*/
  SYSCFG_EXTILineConfig(IRPORT,IRPIN0);
  SYSCFG_EXTILineConfig(IRPORT,IRPIN1);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line0|EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}


/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
