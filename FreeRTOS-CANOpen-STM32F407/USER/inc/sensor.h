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




#define IRPIN0 EXTI_PinSource0
#define IRPIN1 EXTI_PinSource1
#define IRPORT EXTI_PortSourceGPIOG


#define infrared_PORT  		GPIOG

#define Auto_charge_Infared0_time_MIN 	3
#define Auto_charge_Infared0_time_MAX 	6
#define Auto_charge_Infared1_time_MIN 	3
#define Auto_charge_Infared1_time_MAX 	6
#define Auto_charge_Infared_collected_valve 3







typedef struct
{
  u8 Infared0_time,Infared1_time;
	u8 Infared0_time_end,Infared1_time_end;
	u8 Infared0_collected,Infared1_collected;
	u8 receive_status;
	u16 nosignal_counting;
}AUTO_CHARGE;


u8 MSG_U5_R[8];
u8 MSG_U5_T[16];
u32 mask=0;


void RCCInit(void)
{
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
 RCC_LSICmd(ENABLE);//´ò¿ªLSI
 while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY)==RESET);
}
void GPIOInit(void)			
{
 GPIO_InitTypeDef  GPIO_InitStructure;
	
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

  // Configure ADC1 Channel 12, 13 pins as analog input for future use
  GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2|GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void USARTInit(void)
{
 	USART_InitTypeDef USART_InitStructure;

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

}



void sensor_thread(void * pvParameters);
void start_sensor(void);

void SENSOR_Init(void);


void SENSOR_Force_Init(void);
void SENSOR_Lidar_Init(void);
void SENSOR_IR_Distance_Init(void);
void SENSOR_IR_Autocharge_Init(void);
void SENSOR_Anticollision(void);

// void GetDataFromSENSOR(SENSOR_STRUCT* sensor);

//UNS32 StoreSENSORDataToOD(SENSOR_STRUCT *sensor,CO_Data *d);


#endif

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
