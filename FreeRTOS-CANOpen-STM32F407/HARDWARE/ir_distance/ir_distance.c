/**
  ******************************************************************************
  * @file    ir_distance.c
  * @author  Jim
  * @version V1.0
  * @date    16-Jan-2015
  * @brief   This file provides ir_distance functions 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */
#include "ir_distance.h"
#include "stm32f4xx.h"

__IO uint16_t ADC1ConvertedValue[8] = {0,0,0,0,0,0,0,0};	// the ADC value of the infrared distance sensor

void SENSOR_IR_Distance_Init(void)
{	
	GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	/* GPIO Config ------------------------------------------------------------------*/
	/* Enable GPIOA Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Configure ADC1 Channel 0,1,4,5,6,7,12,13 pins as analog input for infrared 
		 distance sensors 1-6 respectively*/
  GPIO_StructInit(&GPIO_InitStructure);//Reset init structure, if not it can cause issues...
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	

	/* DMA Config ------------------------------------------------------------------*/
	/* Enable DMA2 Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);    //DMA2,GPIO use AHB1 BUS
  
	DMA_DeInit(DMA2_Stream0);
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;      
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;  
	//ADC1 DATA ADDRESS    
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC1ConvertedValue;    
	//DMA memory base address    
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;    
	DMA_InitStructure.DMA_BufferSize = 8;    
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;   
	//periphera inc mode: when have multi peripheras need to use DMA    
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;      
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;   
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;   
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  
	//need  continue visit   
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;    
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;            
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;   
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;   
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);   
	DMA_Cmd(DMA2_Stream0, ENABLE); 

	/* ADC Config ------------------------------------------------------------------*/
	/* Enable ADC1 Clock */ 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);//ADC1 in DMA2 channel_0, Stream0 
  /* ADC Common Init */
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; 
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
  //ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; 
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_15Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC1 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  //ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE; 
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  //gtz02nov  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; //gtz01nov 
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 
  //ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_InitStructure.ADC_NbrOfConversion = 8;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel12 configuration *************************************/
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_144Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_144Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_144Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_144Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 5, ADC_SampleTime_144Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 6, ADC_SampleTime_144Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12,7, ADC_SampleTime_144Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13,8, ADC_SampleTime_144Cycles);

  /* Enable DMA request after last transfer (Single-ADC mode) */
 	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);//gtz01nov 
	ADC_ContinuousModeCmd(ADC1, ENABLE);
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConv(ADC1);
}













/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
