/**
  ******************************************************************************
  * @file    lidar.c
  * @author  Jim
  * @version V1.0
  * @date    16-Jan-2015
  * @brief   This file provides LIFTER variable definitions. 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef __LIDAR_H__
#define __LIDAR_H__

#include "globalstruct.h"
#include "data.h"


void SENSOR_Lidar_Init(void);
void Lidar_Init(void);
void Lidar_Stop(void);
void Lidar_Reset(void);


#endif

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
