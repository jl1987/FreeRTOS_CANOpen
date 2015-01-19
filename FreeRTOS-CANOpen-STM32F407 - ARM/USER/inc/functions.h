/**
  ******************************************************************************
  * @file    ./USER/inc/functions.h
  * @author  Jim
  * @version V1.0
  * @date    01-Sep-2014
  * @brief   This file provides function variable definitions. 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  *
  ******************************************************************************
  */
	
#ifndef __FUNCTIONS_H__
#define __FUNCTIONS_H__

#include "stm32f4xx.h"
#include "stdio.h"	
#include "math.h" 

#define REAL   float

REAL my_abs(REAL f);
int32_t my_abs_int(int32_t f); 
REAL my_sqrt(REAL number);
int32_t my_max_int_abs_6d(int32_t a[6], int32_t b[6]);





#endif

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
