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

#define HALFPWMTIMERPERIOD 6980

#define Channel1_U    USART2
#define Channel2_U    UART5
#define Channel3_U    USART3
#define Channel1_PWM  TIM1->CCR3
#define Channel2_PWM  TIM1->CCR4
#define Channel3_PWM  TIM1->CCR2
#define Channel_PWMOUTPUT(VALUE)  HALFPWMTIMERPERIOD+VALUE

#define Channel1_RST  		GPIO_Pin_15
#define Channel2_RST  		GPIO_Pin_11
#define Channel3_RST  		GPIO_Pin_9
#define Channel1_RSTPORT  GPIOB
#define Channel2_RSTPORT  GPIOD
#define Channel3_RSTPORT  GPIOD

#define Channel1_FLT  		GPIO_Pin_8
#define Channel2_FLT  		GPIO_Pin_12
#define Channel3_FLT  		GPIO_Pin_10
#define Channel1_FLTPORT  GPIOD
#define Channel2_FLTPORT  GPIOD
#define Channel3_FLTPORT  GPIOD




void chassis_move(double wheel1, double wheel2, double wheel3);


void Motor_Init(void);

void MotorFault(void);

void delay(__IO uint32_t nCount);

#endif

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
