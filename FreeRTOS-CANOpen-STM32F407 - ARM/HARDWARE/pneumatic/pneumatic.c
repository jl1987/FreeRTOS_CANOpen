/**
  ******************************************************************************
  * @file    ./HARDWARE/pneumatic/pneumatic.c
  * @author  Ronghuai.Qi, Jim
  * @version V1.0
  * @date    19-Jan-2015
  * @brief   This file provides pneumatic functions 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */
#include "pneumatic.h"
#include "stm32f4xx.h"

vu16 ADC1_ConvertedValue[5];
vu16 ADC3_ConvertedValue[5];

void Pneumatic_Init(void)
{
	/* IO initial */
	pneumatic_IO_initial();

 	/* ADC initial */  	
	ADC3_CH6_DMA_Config();	
	ADC_SoftwareStartConv(ADC3);
	ADC1_CH6_DMA_Config();	
	ADC_SoftwareStartConv(ADC1);
}



/**
  * @brief  Pneumatic GPIO configuration
  * @param  None
  * @retval None
  */
void pneumatic_IO_initial(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 

	/* Enable the GPIOG Clock ****************************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	/* GPIOG Configuration */
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;										
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;									
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;								
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	/* Enable the GPIOB Clock ****************************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* GPIOB Configuration */
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;										
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;									
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;									
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	/* Enable the GPIOE Clock ****************************************************/
	/* GPIOE Configuration */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	
	GPIO_InitStructure.GPIO_Pin	 	= GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;										
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;									
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;									
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	/* Enable the GPIOC Clock ****************************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	
	/* GPIOC Configuration */
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;										
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;									
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;								
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	/* Enable the GPIOF Clock ****************************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);	
	/* GPIOF Configuration */
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_0 |GPIO_Pin_1 |GPIO_Pin_2 |GPIO_Pin_3| 
																	GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;										
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;									
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;								
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);		
}


/******************************************/
/*ADCs prog */
/******************************************/
/**
  * @brief  ADC1 channel6 with DMA configuration
  * @param  None
  * @retval None
  */
void ADC1_CH6_DMA_Config(void)
{
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef       DMA_InitStructure;
	GPIO_InitTypeDef      GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2|RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	// DMA_DeInit(DMA2_Stream0);
	/* DMA2 Stream0 channe0 configuration **************************************/
	DMA_InitStructure.DMA_Channel 						= DMA_Channel_0;  
	DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)ADC1_DR_Address;
	DMA_InitStructure.DMA_Memory0BaseAddr 		= (uint32_t)&ADC1_ConvertedValue;
	DMA_InitStructure.DMA_DIR 								= DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize 					= 5;
	DMA_InitStructure.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc 					= DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize 			= DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode 								= DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority 						= DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode 						= DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold 			= DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst 				= DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);

  /* Configure ADC1 Channel6 pin as analog input ******************************/
	//mapping => ADC12_IN4,ADC12_IN5 11282013 based on new pcb config	
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	//mapping => ADC123_IN10,ADC123_IN12,ADC123_IN13 12112013 based on new pcb config	
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure); 

	/* ADC Common Init **********************************************************/
	ADC_CommonInitStructure.ADC_Mode 						 = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler 			 = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode		 = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	/* ADC1 Init ****************************************************************/
	ADC_InitStructure.ADC_Resolution 					 = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode 				 = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode	 = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign 					 = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion 		 = 5;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channe6 configuration *************************************/ 
	// 12112013 based on new pcb config		
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_3Cycles); // left thumb 左大拇指
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_3Cycles); // right thumb 右大拇指
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10,3, ADC_SampleTime_3Cycles); // right ring finger 右无名指
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12,4, ADC_SampleTime_3Cycles); // left little finger 左小指
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13,5, ADC_SampleTime_3Cycles); // right little finger 右小指

	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	
	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

}

/**
  * @brief  ADC3 channel6 with DMA configuration
  * @param  None
  * @retval None
  */
// note: verify later if need, 11/18/2013
void ADC3_CH6_DMA_Config(void)
{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOF, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

  //  DMA_DeInit(DMA2_Stream0);
  /* DMA2 Stream0 channe0 configuration **************************************/
  DMA_InitStructure.DMA_Channel 						= DMA_Channel_2;  
  DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)ADC3_DR_Address;
  DMA_InitStructure.DMA_Memory0BaseAddr 		= (uint32_t)&ADC3_ConvertedValue;
  DMA_InitStructure.DMA_DIR 								= DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize 					= 5;
  DMA_InitStructure.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc 					= DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize 			= DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode 								= DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority 						= DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode 						= DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold 			= DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst 				= DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream1, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream1, ENABLE);

  /* Configure ADC3 Channel6 pin as analog input ******************************/
  //mapping => ADC3_IN4,ADC3_IN5,ADC3_IN8,ADC3_IN14,ADC3_IN15
  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_10|GPIO_Pin_4|GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIO_InitStructure);    

  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode 							= ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler 				= ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode 		= ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay 	= ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);	

  /* ADC3 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution 						= ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode 					= ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode 		= ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge 	= ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign							= ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion 			= 5;
  ADC_Init(ADC3, &ADC_InitStructure);

  /* ADC3 regular channe6 configuration *************************************/ 
  ADC_RegularChannelConfig(ADC3, ADC_Channel_14,1, ADC_SampleTime_3Cycles); //左食指
  ADC_RegularChannelConfig(ADC3, ADC_Channel_15,2, ADC_SampleTime_3Cycles); //右食指
  ADC_RegularChannelConfig(ADC3, ADC_Channel_4, 3, ADC_SampleTime_3Cycles); //左中指
  ADC_RegularChannelConfig(ADC3, ADC_Channel_5, 4, ADC_SampleTime_3Cycles); //右中指
  ADC_RegularChannelConfig(ADC3, ADC_Channel_8, 5, ADC_SampleTime_3Cycles); //左无名指

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC3, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);

}

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
