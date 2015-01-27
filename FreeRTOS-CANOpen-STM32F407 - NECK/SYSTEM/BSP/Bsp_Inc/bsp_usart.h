/**
  ******************************************************************************
  * @file    bsp_usart.c
  * @author  Jim
  * @version V1.0
  * @date    01-Sep-2014
  * @brief   This is the led driver function.
  * @brief   LED  	PORT	ACTION 	
  *          LED1 : PE3		低电平点亮，高电平熄灭
  *          LED2 : PE4		低电平点亮，高电平熄灭
  *          LED3 : PE5		低电平点亮，高电平熄灭
  *          LED4 : PE5		低电平点亮，高电平熄灭
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

/* 供外部调用的函数声明 */
void bsp_InitLed(void);
void bsp_LedOn(uint8_t _no);
void bsp_LedOff(uint8_t _no);
void bsp_LedToggle(uint8_t _no);
uint8_t bsp_IsLedOn(uint8_t _no);

#endif

/***********************************************************************************************/
