/**
  ******************************************************************************
  * @file    ./HARDWAR/touch_sensor/touch_sensor.c
  * @author  Ronghuai.Qi, Jim
  * @version V1.0
  * @date    18-Jan-2015
  * @brief   This file provides touch_sensor variable definitions. 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */

#ifndef __TOUCH_SENSOR_H__
#define __TOUCH_SENSOR_H__

#include "globalstruct.h"
#include "data.h"


#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)

void TouchSensor_Init(void);
void GPIO_TouchSensor_Config(void);

#endif

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
