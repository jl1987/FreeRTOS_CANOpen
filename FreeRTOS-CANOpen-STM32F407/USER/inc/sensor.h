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

#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"

#define ADC1_DR_Address  ((uint32_t)0X4001204C)
#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)
// extern __IO uint16_t ADC1ConvertedValue[];
// extern __IO uint16_t ADC3ConvertedValue[];

__IO uint16_t ADC1ConvertedValue[8] = {0,0,0,0,0,0,0,0};	// the ADC value of the infrared distance sensor
__IO uint16_t ADC3ConvertedValue[8] = {0,0,0,0,0,0,0,0};	// the ADC value of the new force sensors


/* SENSOR ON-OFF -------------------------------------------------------------*/
#define SENSOR_FORCE					1
#define SENSOR_LIDAR					1
#define SENSOR_COLLISION 			1
#define SENSOR_IR_DISTANCE 		1
#define SENSOR_IR_AUTOCHARGE	1


#define HALFPWMTIMERPERIOD 6980

#define Channel1_U    USART2
#define Channel2_U    UART5
#define Channel3_U    USART3
#define Channel1_PWM  TIM1->CCR3
#define Channel2_PWM  TIM1->CCR4
#define Channel3_PWM  TIM1->CCR2
#define Channel_PWMOUTPUT(VALUE)  HALFPWMTIMERPERIOD+VALUE

#define Channel1_RST  		GPIO_Pin_15
#define Channel2_RST  		GPIO_Pin_11
#define Channel3_RST  		GPIO_Pin_9
#define Channel1_RSTPORT  GPIOB
#define Channel2_RSTPORT  GPIOD
#define Channel3_RSTPORT  GPIOD

#define IRPIN0 EXTI_PinSource0
#define IRPIN1 EXTI_PinSource1
#define IRPORT EXTI_PortSourceGPIOG

#define Channel1_FLT  		GPIO_Pin_8
#define Channel2_FLT  		GPIO_Pin_12
#define Channel3_FLT  		GPIO_Pin_10
#define Channel1_FLTPORT  GPIOD
#define Channel2_FLTPORT  GPIOD
#define Channel3_FLTPORT  GPIOD
#define infrared_PORT  		GPIOG

#define Auto_charge_Infared0_time_MIN 	3
#define Auto_charge_Infared0_time_MAX 	6
#define Auto_charge_Infared1_time_MIN 	3
#define Auto_charge_Infared1_time_MAX 	6
#define Auto_charge_Infared_collected_valve 3

#define Pi 		3.14159265358979
#define L1O 	100
#define L2O 	100
#define L3O 	100
#define Alpha1 Pi*0.166666667
#define Alpha2 Pi*0.833333333
#define Alpha3 Pi*1.5


typedef struct
{
  s32 V_SET;
	s32 V_GET;
	u8 fault_msg;
	u8 temperture;
}DRIVE;

typedef struct
{
  u8 Infared0_time,Infared1_time;
	u8 Infared0_time_end,Infared1_time_end;
	u8 Infared0_collected,Infared1_collected;
	u8 receive_status;
	u16 nosignal_counting;
}AUTO_CHARGE;

u8 MSG_U1_R[256];
u8 MSG_U1_T[256];
u8 MSG_U2_R[8];
u8 MSG_U2_T[16];
u8 MSG_U3_R[8];
u8 MSG_U3_T[16];
u8 MSG_U5_R[8];
u8 MSG_U5_T[16];
u32 mask=0;





void RCCInit(void)
{
  /*
  *            @arg RCC_AHB1Periph_GPIOA:       GPIOA clock
  *            @arg RCC_AHB1Periph_GPIOB:       GPIOB clock 
  *            @arg RCC_AHB1Periph_GPIOC:       GPIOC clock
  *            @arg RCC_AHB1Periph_GPIOD:       GPIOD clock
  *            @arg RCC_AHB1Periph_GPIOE:       GPIOE clock
  *            @arg RCC_AHB1Periph_GPIOF:       GPIOF clock
  *            @arg RCC_AHB1Periph_GPIOG:       GPIOG clock
  *            @arg RCC_AHB1Periph_GPIOH:       GPIOH clock
  *            @arg RCC_AHB1Periph_GPIOI:       GPIOI clock
  *            @arg RCC_AHB1Periph_CRC:         CRC clock
  *            @arg RCC_AHB1Periph_BKPSRAM:     BKPSRAM interface clock
  *            @arg RCC_AHB1Periph_CCMDATARAMEN CCM data RAM interface clock
  *            @arg RCC_AHB1Periph_DMA1:        DMA1 clock
  *            @arg RCC_AHB1Periph_DMA2:        DMA2 clock
  *            @arg RCC_AHB1Periph_ETH_MAC:     Ethernet MAC clock
  *            @arg RCC_AHB1Periph_ETH_MAC_Tx:  Ethernet Transmission clock
  *            @arg RCC_AHB1Periph_ETH_MAC_Rx:  Ethernet Reception clock
  *            @arg RCC_AHB1Periph_ETH_MAC_PTP: Ethernet PTP clock
  *            @arg RCC_AHB1Periph_OTG_HS:      USB OTG HS clock
  *            @arg RCC_AHB1Periph_OTG_HS_ULPI: USB OTG HS ULPI clock
  *
  *            @arg RCC_APB1Periph_TIM2:   TIM2 clock
  *            @arg RCC_APB1Periph_TIM3:   TIM3 clock
  *            @arg RCC_APB1Periph_TIM4:   TIM4 clock
  *            @arg RCC_APB1Periph_TIM5:   TIM5 clock
  *            @arg RCC_APB1Periph_TIM6:   TIM6 clock
  *            @arg RCC_APB1Periph_TIM7:   TIM7 clock
  *            @arg RCC_APB1Periph_TIM12:  TIM12 clock
  *            @arg RCC_APB1Periph_TIM13:  TIM13 clock
  *            @arg RCC_APB1Periph_TIM14:  TIM14 clock
  *            @arg RCC_APB1Periph_WWDG:   WWDG clock
  *            @arg RCC_APB1Periph_SPI2:   SPI2 clock
  *            @arg RCC_APB1Periph_SPI3:   SPI3 clock
  *            @arg RCC_APB1Periph_USART2: USART2 clock
  *            @arg RCC_APB1Periph_USART3: USART3 clock
  *            @arg RCC_APB1Periph_UART4:  UART4 clock
  *            @arg RCC_APB1Periph_UART5:  UART5 clock
  *            @arg RCC_APB1Periph_I2C1:   I2C1 clock
  *            @arg RCC_APB1Periph_I2C2:   I2C2 clock
  *            @arg RCC_APB1Periph_I2C3:   I2C3 clock
  *            @arg RCC_APB1Periph_CAN1:   CAN1 clock
  *            @arg RCC_APB1Periph_CAN2:   CAN2 clock
  *            @arg RCC_APB1Periph_PWR:    PWR clock
  *            @arg RCC_APB1Periph_DAC:    DAC clock

  *            @arg RCC_APB2Periph_TIM1:   TIM1 clock
  *            @arg RCC_APB2Periph_TIM8:   TIM8 clock
  *            @arg RCC_APB2Periph_USART1: USART1 clock
  *            @arg RCC_APB2Periph_USART6: USART6 clock
  *            @arg RCC_APB2Periph_ADC1:   ADC1 clock
  *            @arg RCC_APB2Periph_ADC2:   ADC2 clock
  *            @arg RCC_APB2Periph_ADC3:   ADC3 clock
  *            @arg RCC_APB2Periph_SDIO:   SDIO clock
  *            @arg RCC_APB2Periph_SPI1:   SPI1 clock
  *            @arg RCC_APB2Periph_SYSCFG: SYSCFG clock
  *            @arg RCC_APB2Periph_TIM9:   TIM9 clock
  *            @arg RCC_APB2Periph_TIM10:  TIM10 clock
  *            @arg RCC_APB2Periph_TIM11:  TIM11 clock */
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|
                        RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|
	                      RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_DMA1|
												RCC_AHB1Periph_DMA2|RCC_AHB1Periph_GPIOG
												, ENABLE);
												
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB1Periph_USART3| RCC_APB1Periph_UART4| 
                        RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3|
	                      RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM5|
												RCC_APB1Periph_TIM7|RCC_APB1Periph_PWR|
												RCC_APB1Periph_CAN1|RCC_APB1Periph_CAN2|
												RCC_APB1Periph_UART5
												,ENABLE);
												
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1|RCC_APB2Periph_ADC1|
                        RCC_APB2Periph_ADC2|RCC_APB2Periph_ADC3|RCC_APB2Periph_SPI1|
                        RCC_APB2Periph_SYSCFG|RCC_APB2Periph_USART1|
												RCC_APB2Periph_USART6
												,ENABLE);
 RCC_LSICmd(ENABLE);//打开LSI
 while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY)==RESET);
}
void GPIOInit(void)			
{
 GPIO_InitTypeDef  GPIO_InitStructure;
 GPIO_DeInit(GPIOA);
 GPIO_DeInit(GPIOB);
 GPIO_DeInit(GPIOC);
 GPIO_DeInit(GPIOD);
 GPIO_DeInit(GPIOE);
 GPIO_DeInit(GPIOG);

 RCC_LSICmd(ENABLE);//打开LSI
 while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY)==RESET);
	
	//FAULT
	GPIO_InitStructure.GPIO_Pin = Channel1_FLT|Channel2_FLT|Channel3_FLT;		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	        //需要配置的引脚
	GPIO_Init(Channel1_FLTPORT, &GPIO_InitStructure);
	//FAULT
	
	//RST
	GPIO_InitStructure.GPIO_Pin = Channel1_RST;								//需要配置的引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Channel1_RSTPORT, &GPIO_InitStructure);
	GPIO_WriteBit(Channel1_RSTPORT,Channel1_RST, Bit_SET);
	
	GPIO_InitStructure.GPIO_Pin = Channel2_RST|Channel3_RST;								//需要配置的引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Channel2_RSTPORT, &GPIO_InitStructure);
	GPIO_WriteBit(Channel2_RSTPORT,Channel2_RST|Channel2_RST, Bit_SET);
	//RST
	
	//U1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;	  //tx
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);	
	//U1
	
	//U2-m
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;	  //tx
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);	
	//U2-m
	//U3-m
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;	  //tx
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);	
	//U3-m



	//U5-m
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	  //TX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	    //RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,  GPIO_AF_UART5);	
	//U5-m
	//U6
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	  //tx
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);	
	//U6
	
	//T1PWM
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIO_InitStructure); 
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
	//T1PWM
	

	
	//EXTI 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;       //输入模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;   //上拉
  GPIO_Init(GPIOG, &GPIO_InitStructure);           
  //EXTI

  // Configure ADC1 Channel 12, 13 pins as analog input for future use
  GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2|GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // Configure ADC3 Channel13 pin as analog input for New Force sensors' (1,2,3,4) I/Os 
  GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOF, &GPIO_InitStructure);	
	
	

}

void ADC1_DMA_Config(void){
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	/* Enable ADC1, DMA2  and GPIO clocks ****************************************/   
	//ADC1 in DMA2 channel_0 , Stream0    
	//DMA2,GPIO use AHB1 BUS     
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);                  
	//ADC use APB2 BUS        
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


  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;       //?ミ家Α
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;         //?だ?
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;    //
  //ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; 
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_15Cycles;  //?┑?
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC1 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;    //?竚12だ?瞯
  //ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;   //?磞家Α?硄笵㎝硄笵??だ
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;      //??家Α
// gtz02nov  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;   //场郉?窽ゎ
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;   //gtz01nov 
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;         //?誹??
  //ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_InitStructure.ADC_NbrOfConversion = 8;          //?竚ADC??硄笵?
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel12 configuration *************************************/
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_144Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_144Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_144Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_144Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 5, ADC_SampleTime_144Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 6, ADC_SampleTime_144Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 7, ADC_SampleTime_144Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 8, ADC_SampleTime_144Cycles);

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


void DMAInit(void)
{
	DMA_InitTypeDef DMA_InitStruct;
	DMA_DeInit(DMA1_Stream1);
	DMA_DeInit(DMA1_Stream3);
	DMA_DeInit(DMA1_Stream5);
	DMA_DeInit(DMA1_Stream6);
	DMA_DeInit(DMA1_Stream7);
	DMA_DeInit(DMA2_Stream5);

	DMA_DeInit(DMA2_Stream0);
	DMA_DeInit(DMA2_Stream1);
	DMA_DeInit(DMA2_Stream5);
  	DMA_DeInit(DMA2_Stream7);
	
	//U1
	while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE);	
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;			   //S5C4
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(USART1->DR));
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)MSG_U1_R;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;             //RX
	DMA_InitStruct.DMA_BufferSize = (u32)0;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream5, &DMA_InitStruct);
	//DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA2_Stream5, ENABLE);  //接收通道
	
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);	
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;			   //S7C4
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(USART1->DR));
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)MSG_U1_T;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;            //TX
	DMA_InitStruct.DMA_BufferSize = (u32)0;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream7, &DMA_InitStruct);
	//DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA2_Stream7, ENABLE);  //接收通道
	//U1
	
	//U2
	while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);	
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;			   //S5C4
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(USART2->DR));
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)MSG_U2_R;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;             //RX
	DMA_InitStruct.DMA_BufferSize = (u32)10;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream5, &DMA_InitStruct);
	DMA_Cmd(DMA1_Stream5, ENABLE);  //接收通道
	
	while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE);	
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;			   //S6C4
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(USART2->DR));
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)MSG_U2_T;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;            //TX
	DMA_InitStruct.DMA_BufferSize = (u32)10;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream6, &DMA_InitStruct);
	DMA_Cmd(DMA1_Stream6, ENABLE);  //接收通道
	//U2
	
	//U3
	while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);	
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;			   //S1C4
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(USART3->DR));
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)MSG_U3_R;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;             //RX
	DMA_InitStruct.DMA_BufferSize = (u32)10;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream1, &DMA_InitStruct);
	DMA_Cmd(DMA1_Stream1, ENABLE);  //接收通道
	
	while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE);	
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;			   //S3C4
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(USART3->DR));
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)MSG_U3_T;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;            //TX
	DMA_InitStruct.DMA_BufferSize = (u32)10;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream3, &DMA_InitStruct);
	DMA_Cmd(DMA1_Stream3, ENABLE);  //接收通道
	//U3

}


void USARTInit(void)
{
 	USART_InitTypeDef USART_InitStructure;

	USART_DeInit(USART1);
	USART_DeInit(USART2);
	USART_DeInit(USART3);
	USART_DeInit(UART5);
	
	USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure); 
  //  USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
//  USART_DMACmd(USART1,USART_DMAReq_Rx, ENABLE);
//	USART_DMACmd(USART1,USART_DMAReq_Tx, ENABLE);

// Enable Interrupt
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	
  USART_Cmd(USART1, ENABLE);

	USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART6, &USART_InitStructure); 
  USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
  USART_DMACmd(USART6,USART_DMAReq_Rx, ENABLE);
	USART_DMACmd(USART6,USART_DMAReq_Tx, ENABLE);
  USART_Cmd(USART6, ENABLE);
	
	//M1
	USART_InitStructure.USART_BaudRate = 19200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure); 
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
  USART_DMACmd(USART2,USART_DMAReq_Rx, ENABLE);
	USART_DMACmd(USART2,USART_DMAReq_Tx, ENABLE);
  USART_Cmd(USART2, ENABLE);
	//M1
	
	//M2
	USART_InitStructure.USART_BaudRate = 19200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure); 
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
  USART_DMACmd(USART3,USART_DMAReq_Rx, ENABLE);
	USART_DMACmd(USART3,USART_DMAReq_Tx, ENABLE);
  USART_Cmd(USART3, ENABLE);
	//M2
	
	//M3  
	USART_InitStructure.USART_BaudRate = 19200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(UART5, &USART_InitStructure); 
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
//   USART_DMACmd(UART5,USART_DMAReq_Rx, ENABLE);
// 	USART_DMACmd(UART5,USART_DMAReq_Tx, ENABLE);
  USART_Cmd(UART5, ENABLE);
	//M3

	// Lidar
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
}
void TIMInit(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
  TIM_OCInitTypeDef TIM_OCInitStructure;   
	//--------------TIM1的配置
  TIM_TimeBaseStructure.TIM_Prescaler = 100;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = HALFPWMTIMERPERIOD*2;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	  //还没匹配，高!
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;  //匹配之后，低!
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
  //TIM_OC1Init(TIM1, &TIM_OCInitStructure); 
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);
  TIM_Cmd(TIM1, ENABLE);
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
  //---------------TIM1的配置
}



void NVICInit(void)
{
 NVIC_InitTypeDef NVIC_InitStructure; 

 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
 //******************************************************************
 
//  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
	
 NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 NVIC_Init(&NVIC_InitStructure);
	
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
 
 NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
 NVIC_Init(&NVIC_InitStructure);

 NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; 
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
 NVIC_Init(&NVIC_InitStructure);
 
 NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; 
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
 NVIC_Init(&NVIC_InitStructure);
 
 /* Enable the USARTx Interrupt */
 NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 NVIC_Init(&NVIC_InitStructure);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}



void EXTIInit(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
	
  SYSCFG_EXTILineConfig(IRPORT,IRPIN0);
  SYSCFG_EXTILineConfig(IRPORT,IRPIN1);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line0|EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}




void sensor_thread(void * pvParameters);
void start_sensor(void);

void SENSOR_Init(void);


void SENSOR_Force_Init(void);


// void GetDataFromSENSOR(SENSOR_STRUCT* sensor);

//UNS32 StoreSENSORDataToOD(SENSOR_STRUCT *sensor,CO_Data *d);


#endif

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
