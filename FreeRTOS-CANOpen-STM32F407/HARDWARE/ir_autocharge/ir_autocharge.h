/**
  ******************************************************************************
  * @file    ir_autocharge.c
  * @author  Jim
  * @version V1.0
  * @date    16-Jan-2015
  * @brief   This file provides ir_autocharge variable definitions. 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef __IR_AUTOCHARGE_H__
#define __IR_AUTOCHARGE_H__

#include "globalstruct.h"
#include "data.h"

#define IRPIN0 EXTI_PinSource0
#define IRPIN1 EXTI_PinSource1
#define IRPORT EXTI_PortSourceGPIOG

typedef struct
{
  u16 Infared0_time,Infared1_time;
	u16 Infared0_time_end,Infared1_time_end;
	u16 Infared0_collected,Infared1_collected;
	u16 receive_status;
	u16 nosignal_counting;
	
	u16 Infared0_width,Infared1_width;
	
}AUTO_CHARGE;


void SENSOR_IR_Autocharge_Init(void);
#endif

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
