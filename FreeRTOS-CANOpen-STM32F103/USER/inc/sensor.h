/**
  ******************************************************************************
  * @file    lifter_control.c
  * @author  Jim
  * @version V1.0
  * @date    01-Sep-2014
  * @brief   This file provides LIFTER variable definitions. 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 CSST Robot Research Center</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "can_STM32.h"
#include "canfestival.h"
#include "data.h"




void sensor_thread(void * pvParameters);
void start_sensor(void);

void SENSOR_Init(void);

// void GetDataFromSENSOR(SENSOR_STRUCT* sensor);

//UNS32 StoreSENSORDataToOD(SENSOR_STRUCT *sensor,CO_Data *d);


#endif

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
