/**
  ******************************************************************************
  * @file    bms_control.c
  * @author  Jim
  * @version V1.0
  * @date    01-Sep-2014
  * @brief   This file provides BMS control thread functions 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */

#include "sensor.h"
#include "stdio.h"
#include "globalstruct.h"

/* BSP */
#include "bsp_led.h"

/* Object Dictionary */
#include "CHASSIS_OD.h"

xTaskHandle	 xT_SENSOR 	  = NULL;

//SENSOR_STRUCT 	SENSOR;  //make it your own sensor struct

void sensor_thread(void * pvParameters)
{
	UNS32 ret_store;
	
	SENSOR_Init();
	
	printf("start the sensor \r\n");
	
	while(1)
	{ 
		/* Get Data From SENSOR using XXX */
		//GetDataFromSENSOR(&SENSOR);  //make it your own sensor function
		
		/* Copy Data to Object Dictionary */
		//ret_store = StoreSENSORDataToOD(&SENSOR,&SENSOR_OD_Data);  //make it your own sensor store function
		
	  vTaskDelay(SENSOR_CONTROL_THREAD_DELAY_TIMER);
		bsp_LedToggle(3,100);
		
	}
}


void start_sensor(void)
{
	xTaskCreate(sensor_thread, "sensor", SENSOR_THREAD_STACK, NULL,SENSOR_THREAD_PRIO, &xT_SENSOR);
	if(NULL == xT_SENSOR)
	{
	 	printf("sensor data thread creat failed!\r\n");
	}else
	{
		printf("sensor data thread creat successfully!\r\n");
	}
}

void SENSOR_Init(void)
{
	SENSOR_Force_Init();
	SENSOR_Lidar_Init();
	SENSOR_IR_Distance_Init();
	SENSOR_IR_Autocharge_Init();
	SENSOR_Anticollision();
}


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
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;		
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
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;   //gtz01nov 
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  //ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_InitStructure.ADC_NbrOfConversion = 8;
  ADC_Init(ADC3, &ADC_InitStructure);

  /* ADC3 regular channelx configuration */
  ADC_RegularChannelConfig(ADC3, ADC_Channel_9,  1, ADC_SampleTime_144Cycles);		// PF3
  ADC_RegularChannelConfig(ADC3, ADC_Channel_14, 2, ADC_SampleTime_144Cycles);		// PF4
  ADC_RegularChannelConfig(ADC3, ADC_Channel_15, 3, ADC_SampleTime_144Cycles);		// PF5
  ADC_RegularChannelConfig(ADC3, ADC_Channel_4,  4, ADC_SampleTime_144Cycles);		// PF6
  ADC_RegularChannelConfig(ADC3, ADC_Channel_5,  5, ADC_SampleTime_144Cycles);		// PF7
  ADC_RegularChannelConfig(ADC3, ADC_Channel_6,  6, ADC_SampleTime_144Cycles);		// PF8
  ADC_RegularChannelConfig(ADC3, ADC_Channel_7,  7, ADC_SampleTime_144Cycles);		// PF9
  ADC_RegularChannelConfig(ADC3, ADC_Channel_8,  8, ADC_SampleTime_144Cycles);		// PF10

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

void SENSOR_Lidar_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure; 
	
	/* GPIO Config ------------------------------------------------------------------*/
	/* Enable GPIOF Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	/* Connect USART pins to AF7 */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

	/* Configure USART Tx and Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	/* UART4 Config -----------------------------------------------------------------*/
	/* Enable UART4 Clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	/* Config USAR4 Detail */
  USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);
	/* Enable USART */
	/* Enable USART4 Receive Data Read To Be Read Interrupt */
	USART_Cmd(UART4, ENABLE);
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	
	/* NVIC Config ------------------------------------------------------------------*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}
	


void SENSOR_IR_Distance_Init(void)
{	
	GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	/* GPIO Config ------------------------------------------------------------------*/
	/* Enable GPIOA Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// Configure ADC1 Channel 0, 1, 4, 5, 6, 7, 12, 13 pins as analog input for infrared distance sensors 1-6 respectively
  GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 	//ADC1 in DMA2 channel_0 , Stream0 
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
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;   //gtz01nov 
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


void SENSOR_IR_Autocharge_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure; 
	EXTI_InitTypeDef	EXTI_InitStructure;
	
	/* GPIO Config ------------------------------------------------------------------*/
	/* Enable GPIOG Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	//EXTI 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;       //输入模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;   //上拉
  GPIO_Init(GPIOG, &GPIO_InitStructure);           
  //EXTI
	

	/* NVIC Config ------------------------------------------------------------------*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	  
	/* EXTI Config ------------------------------------------------------------------*/
  SYSCFG_EXTILineConfig(IRPORT,IRPIN0);
  SYSCFG_EXTILineConfig(IRPORT,IRPIN1);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line0|EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

void SENSOR_Anticollision(void)
{
	
}


// void GetDataFromSENSOR(SENSOR_STRUCT* sensor)
// {

// }

// UNS32 StoreSENSORDataToOD(SENSOR_STRUCT *sensor,CO_Data *d)
// {
// 	UNS32 size;
// 	UNS32 errorCode;
// 	
// 	size = sizeof(UNS16);
// 	
// 	errorCode = setODentry(d, (UNS16)0x6200, (UNS8)1,&(sensor->data), &size, 0);

// 	return errorCode;
// 	
// }




/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
