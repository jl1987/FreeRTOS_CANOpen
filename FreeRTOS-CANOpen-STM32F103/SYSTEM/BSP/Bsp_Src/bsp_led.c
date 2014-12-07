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

#include "bsp.h"

/**
 * LED Init
 * @param LED Init Program
 * @param 
 */
void bsp_InitLed(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

#ifdef ARM_BMS
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
#endif

#ifdef ARM_LIFTER
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
#endif
	
	GPIO_InitStructure.GPIO_Pin = LED1|LED2|LED3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LEDPORT, &GPIO_InitStructure);
	
	bsp_LedOff(1);
	bsp_LedOff(2);
	bsp_LedOff(3);
}


/**
 * Turn LED On Function
 * @param _no LED Number: 1 - 3
 */
void bsp_LedOn(uint8_t _no)
{
	if (_no == 1)
	{
		GPIO_WriteBit(LEDPORT, LED1, Bit_RESET);
	}
	else if (_no == 2)
	{
		GPIO_WriteBit(LEDPORT, LED2, Bit_RESET);
	}
	else if (_no == 3)
	{
		GPIO_WriteBit(LEDPORT, LED3, Bit_RESET);
	}
}

/**
 * Turn LED OFF 
 * @param _no LED Nubmer: 1 - 4
 */
void bsp_LedOff(uint8_t _no)
{
	if (_no == 1)
	{
		GPIO_WriteBit(LEDPORT, LED1, Bit_SET);
	}
	else if (_no == 2)
	{
		GPIO_WriteBit(LEDPORT, LED2, Bit_SET);
	}
	else if (_no == 3)
	{
		GPIO_WriteBit(LEDPORT, LED3, Bit_SET);
	}
}



/**
 * Turn LED Toggle
 * @param _no LED Number: 1 - 3
 */
void bsp_LedToggle(uint8_t _no, uint32_t _us)
{
	if (_no == 1)
	{
		LEDPORT->ODR ^= LED1;
	}
	else if (_no == 2)
	{
		LEDPORT->ODR ^= LED2;
	}
	else if (_no == 3)
	{
		LEDPORT->ODR ^= LED3;
	}
	vTaskDelay(_us);
}


/**
 * Is LED On or OFF
 * @param  _no LED Number: 1 - 3
 * @return     1: LED is On 
 *             2: LED is Off
 */
uint8_t bsp_IsLedOn(uint8_t _no)
{
	if (_no == 1)
	{
		if ((LEDPORT->ODR & LED1) != 0)
		{
			return 1;
		}
		return 0;
	}
	else if (_no == 2)
	{
		if ((LEDPORT->ODR & LED2) != 0)
		{
			return 1;
		}
		return 0;
	}
	else if (_no == 3)
	{
		if ((LEDPORT->ODR & LED3) != 0)
		{
			return 1;
		}
		return 0;
	}

	return 0;
}

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
