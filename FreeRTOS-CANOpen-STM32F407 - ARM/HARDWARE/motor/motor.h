/**
  ******************************************************************************
  * @file    motor.h
  * @author  Jim
  * @version V1.0
  * @date    01-Sep-2014
  * @brief   This file provides motor variable definitions. 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "globalstruct.h"
#include "data.h"
#include "functions.h"

extern __IO uint16_t CCR1_Val;
extern __IO uint16_t CCR2_Val;
extern __IO uint16_t CCR3_Val;
extern __IO uint16_t CCR4_Val;




void Motor_Init(void);

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void); 
void TIM3_Configuration(void); 
void TIM4_Configuration(void);
void TIM5_Configuration(void); 	  
void TIM2_Config(void);
void TIM2_CCR_CALC(void);






void MotorFault(void);

void delay(__IO uint32_t nCount);

#endif

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
