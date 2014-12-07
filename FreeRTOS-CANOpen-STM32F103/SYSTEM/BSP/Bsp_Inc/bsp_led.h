/**
  ******************************************************************************
  * @file    bsp_led.c
  * @author  Jim
  * @version V1.0
  * @date    01-Sep-2014
  * @brief   This is the led driver function.
  * @brief   ARM_BMS: 
	*										LED  	 PORT		ACTION 	
  *          					LED1 : PA0		Low-On, High-Off
  *          					LED2 : PA1		Low-On, High-Off
  *          					LED3 : PA2		Low-On, High-Off
	* 			   ARM_LIFTER: 
	*										LED  	 PORT		ACTION 	
  *          					GREEN: PA0		Low-On, High-Off
  *          					RED  : PA1		Low-On, High-Off
  *          					BLUE : PA2		Low-On, High-Off
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */
 

#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "bsp.h"

#ifdef ARM_BMS
	/* BMS control borad */
	#define RCC_LED RCC_APB2Periph_GPIOA
	#define LEDPORT GPIOA
	#define LED1    GPIO_Pin_0
	#define LED2    GPIO_Pin_1
	#define LED3    GPIO_Pin_2
#endif

#ifdef ARM_LIFTER
	/* Lifter control board */
	#define RCC_LED 	RCC_APB2Periph_GPIOA
	#define LEDPORT   GPIOB
	#define LED1 			GPIO_Pin_5
	#define LED2   		GPIO_Pin_6
	#define LED3  		GPIO_Pin_7
	#define LED_GREEN GPIO_Pin_5
	#define LED_RED   GPIO_Pin_6
	#define LED_BLUE  GPIO_Pin_7
#endif

void bsp_InitLed(void);
extern void bsp_LedOn(uint8_t _no);
extern void bsp_LedOff(uint8_t _no);
extern void bsp_LedToggle(uint8_t _no, uint32_t _us);
uint8_t bsp_IsLedOn(uint8_t _no);

#endif

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
