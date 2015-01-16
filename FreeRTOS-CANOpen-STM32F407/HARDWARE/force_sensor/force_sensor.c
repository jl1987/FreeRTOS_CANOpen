/**
  ******************************************************************************
  * @file    force_sensor.c
  * @author  Jim
  * @version V1.0
  * @date    16-Jan-2015
  * @brief   This file provides force_sensor functions 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */
#include "force_sensor.h"
#include "stm32f4xx.h"



__IO uint16_t ADC3ConvertedValue[8] = {0,0,0,0,0,0,0,0};	// the ADC value of the new force sensors


void SENSOR_Force_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
	DMA_InitTypeDef DMA_InitStructure;
	
	/* GPIO Config ------------------------------------------------------------------*/
	/* Enable GPIOF Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	
  // Configure ADC3 Channel13 pin as analog input for New Force sensors' (1,2,3,4) I/Os 
  GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
	//ADC3 														CH9  			CH14			CH15				CH4					CH5				CH6					CH7				CH8
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|
																GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;		
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOF, &GPIO_InitStructure);	
	
	/* DMA2 Config ------------------------------------------------------------------*/
	/* Enable DMA2 Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	/* DMA2 Stream0 channel0 configuration */ 
	
	DMA_DeInit(DMA2_Stream1);
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_2;      
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;   
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;    
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; 
	DMA_InitStructure.DMA_BufferSize = 8;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;    
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;   
	
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;    
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;            
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;   
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;   
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);    
	DMA_Cmd(DMA2_Stream1, ENABLE); 

	/* ADC Config -------------------------------------------------------------------*/
	/* Enable ADC3 Clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3,ENABLE);
  /* ADC Common Init */
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;    //
  //ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; 
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_15Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC1 Init */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  //ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
// gtz02nov  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;  //gtz01nov 
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  //ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_InitStructure.ADC_NbrOfConversion = 8;
  ADC_Init(ADC3, &ADC_InitStructure);

  /* ADC3 regular channelx configuration */
  ADC_RegularChannelConfig(ADC3, ADC_Channel_9,  1, ADC_SampleTime_144Cycles);// PF3
  ADC_RegularChannelConfig(ADC3, ADC_Channel_14, 2, ADC_SampleTime_144Cycles);// PF4
  ADC_RegularChannelConfig(ADC3, ADC_Channel_15, 3, ADC_SampleTime_144Cycles);// PF5
  ADC_RegularChannelConfig(ADC3, ADC_Channel_4,  4, ADC_SampleTime_144Cycles);// PF6
  ADC_RegularChannelConfig(ADC3, ADC_Channel_5,  5, ADC_SampleTime_144Cycles);// PF7
  ADC_RegularChannelConfig(ADC3, ADC_Channel_6,  6, ADC_SampleTime_144Cycles);// PF8
  ADC_RegularChannelConfig(ADC3, ADC_Channel_7,  7, ADC_SampleTime_144Cycles);// PF9
  ADC_RegularChannelConfig(ADC3, ADC_Channel_8,  8, ADC_SampleTime_144Cycles);// PF10

 /* Enable DMA request after last transfer (Single-ADC mode) */
 	ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);//gtz01nov 

	ADC_ContinuousModeCmd(ADC3, ENABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC3, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);

  /* Start ADC3 Software Conversion */ 
  ADC_SoftwareStartConv(ADC3);
}

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
