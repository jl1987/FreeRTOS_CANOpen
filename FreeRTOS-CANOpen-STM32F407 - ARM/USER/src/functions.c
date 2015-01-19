/**
  ******************************************************************************
  * @file    ./USER/src/functions.c
  * @author  Jim
  * @version V1.0
  * @date    01-Sep-2014
  * @brief   This file provides functions personal defined. 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  *
  ******************************************************************************
  */
#include "functions.h"

float my_abs(float f)
{
    if (f >= (float)0.0)
    {
        return f;
    } 

    return -f;
}

int32_t my_abs_int(int32_t f)
{
    if (f >= 0)
    {
        return f;
    } 

    return -f;
} 


int32_t my_max_int_abs_6d(int32_t a[6], int32_t b[6])
{
	uint8_t i,j;
	int32_t c[6];  
	int32_t d; 	

	for(i=0; i<6; i++)
	{
		c[i] = a[i] - b[i]; 
		c[i] = my_abs_int(c[i]);
	}

	d = c[0];
	for(j=0; j<6; j++)
	{
	  if(d>c[j]) d=c[j];
	}
		  
	return d;
} 


REAL my_sqrt(REAL number)
{ 
    long i;
    float x, y;
    const float f = 1.5F;
    x = number * 0.5F;
    y = number;
    i = * ( long * ) &y;
    i = 0x5f3759df - ( i >> 1 ); 

    y = * ( float * ) &i;
    y = y * ( f - ( x * y * y ) );
    y = y * ( f - ( x * y * y ) );
    return number * y;
}





















/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
