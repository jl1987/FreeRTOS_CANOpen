/**
  ******************************************************************************
  * @file    ./HARDWAR/touch_sensor/touch_sensor.c
  * @author  Ronghuai.Qi, Jim
  * @version V1.0
  * @date    19-Jan-2015
  * @brief   This file provides touch_sensor functions 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */
#include "touch_sensor.h"
#include "stm32f4xx.h"



void TouchSensor_Init(void)
{
	GPIO_TouchSensor_Config();
}


/**
  * @brief  Touch Sensor GPIO configuration,Configure PA6<->Left hand, PA6<->Right hand
  * @param  None
  * @retval None
  */
void GPIO_TouchSensor_Config(void)	   
{	
	/* Enable the GPIOA Clock ****************************************************/
	GPIO_InitTypeDef GPIO_InitStructure;	
	/* GPIOA Configuration */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}  

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
