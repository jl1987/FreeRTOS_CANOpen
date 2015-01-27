/**
  ******************************************************************************
  * @file    ./HARDWARE/pneumatic/pneumatic.c
  * @author  Ronghuai.Qi, Jim
  * @version V1.0
  * @date    16-Jan-2015
  * @brief   This file provides pneumatic variable definitions. 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */

#ifndef __PNEUMATIC_H__
#define __PNEUMATIC_H__

#include "globalstruct.h"
#include "data.h"

/*ADC*/ 
#define ADC1_DR_Address    ((u32)0x4001204C)
#define ADC3_DR_Address    ((u32)0x4001224C)  


extern vu16 ADC1_ConvertedValue[5];
extern vu16 ADC3_ConvertedValue[5];

void Pneumatic_Init(void);
void pneumatic_IO_initial(void);
void ADC1_CH6_DMA_Config(void);
void ADC3_CH6_DMA_Config(void);	

#endif

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
