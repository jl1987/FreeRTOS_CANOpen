/**
  ******************************************************************************
  * @file    force_sensor.c
  * @author  Jim
  * @version V1.0
  * @date    16-Jan-2015
  * @brief   This file provides force_sensor variable definitions. 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef __FORCE_SENSOR_H__
#define __FORCE_SENSOR_H__

#include "globalstruct.h"
#include "data.h"


#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)

void SENSOR_Force_Init(void);

#endif

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
