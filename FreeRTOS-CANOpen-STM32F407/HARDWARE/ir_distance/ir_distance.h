/**
  ******************************************************************************
  * @file    ir_distance.c
  * @author  Jim
  * @version V1.0
  * @date    16-Jan-2015
  * @brief   This file provides ir_distance variable definitions. 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef __IR_DISSTANCE_H__
#define __IR_DISSTANCE_H__

#include "globalstruct.h"
#include "data.h"

#define ADC1_DR_Address  ((uint32_t)0X4001204C)


void SENSOR_IR_Distance_Init(void);

#endif

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
