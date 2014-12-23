/**
  ******************************************************************************
  * @file    bsp_led.c
  * @author  Jim
  * @version V1.0
  * @date    01-Sep-2014
  * @brief   This is the led driver function.
  * @brief   LED  	PORT	ACTION 	
  *          LED1 : PE2		低电平点亮，高电平熄灭
  *          LED2 : PE3		低电平点亮，高电平熄灭
  *          LED3 : PE4		低电平点亮，高电平熄灭
  *          LED4 : PE5		低电平点亮，高电平熄灭
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

	/* 打开GPIO时钟 */
	RCC_AHB1PeriphClockCmd(RCC_LED, ENABLE);

	bsp_LedOff(1);
	bsp_LedOff(2);
	bsp_LedOff(3);
	bsp_LedOff(4);
	
	GPIO_InitStructure.GPIO_Pin = LED1|LED2|LED3|LED4;		 /* 设为输出口 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;/* 设为推挽模式 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; /* 上下拉电阻不使能 */
	GPIO_Init(LEDPORT, &GPIO_InitStructure);

	#ifdef ARM_CSST
	GPIO_WriteBit(LEDPORT, LED1|LED2|LED3|LED4, Bit_SET);
	#endif
	
	#ifdef ARM_ORIGINAL
	GPIO_WriteBit(LEDPORT, LED1|LED2|LED3|LED4, Bit_RESET);
	#endif
}

#ifdef ARM_CSST
/**
 * Turn LED On Function
 * @param _no LED Number: 1 - 4
 */
void bsp_LedOn(uint8_t _no)
{
	_no--;

	if (_no == 0)
	{
		GPIO_WriteBit(LEDPORT, LED1, Bit_RESET);
	}
	else if (_no == 1)
	{
		GPIO_WriteBit(LEDPORT, LED2, Bit_RESET);
	}
	else if (_no == 2)
	{
		GPIO_WriteBit(LEDPORT, LED3, Bit_RESET);
	}
	else if (_no == 3)
	{
		GPIO_WriteBit(LEDPORT, LED4, Bit_RESET);
	}
}

/**
 * Turn LED OFF 
 * @param _no LED Nubmer: 1 - 4
 */
void bsp_LedOff(uint8_t _no)
{
	_no--;

	if (_no == 0)
	{
		GPIO_WriteBit(LEDPORT, LED1, Bit_SET);
	}
	else if (_no == 1)
	{
		GPIO_WriteBit(LEDPORT, LED2, Bit_SET);
	}
	else if (_no == 2)
	{
		GPIO_WriteBit(LEDPORT, LED3, Bit_SET);
	}
	else if (_no == 3)
	{
		GPIO_WriteBit(LEDPORT, LED4, Bit_SET);
	}
}
#endif


#ifdef ARM_ORIGINAL
/**
 * Turn LED ON (ARM_ORIGINAL)
 * @param _no LED Nubmer: 1 - 4
 */
void bsp_LedOn(uint8_t _no)
{
	_no--;

	if (_no == 0)
	{
		GPIO_WriteBit(LEDPORT, LED1, Bit_SET);
	}
	else if (_no == 1)
	{
		GPIO_WriteBit(LEDPORT, LED2, Bit_SET);
	}
	else if (_no == 2)
	{
		GPIO_WriteBit(LEDPORT, LED3, Bit_SET);
	}
	else if (_no == 3)
	{
		GPIO_WriteBit(LEDPORT, LED4, Bit_SET);
	}
}

/**
 * Turn LED OFF Function
 * @param _no LED Number: 1 - 4
 */
void bsp_LedOff(uint8_t _no)
{
	_no--;

	if (_no == 0)
	{
		GPIO_WriteBit(LEDPORT, LED1, Bit_RESET);
	}
	else if (_no == 1)
	{
		GPIO_WriteBit(LEDPORT, LED2, Bit_RESET);
	}
	else if (_no == 2)
	{
		GPIO_WriteBit(LEDPORT, LED3, Bit_RESET);
	}
	else if (_no == 3)
	{
		GPIO_WriteBit(LEDPORT, LED4, Bit_RESET);
	}
}

#endif




/**
 * Turn LED Toggle
 * @param _no LED Number: 1 - 4
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
	else if (_no == 4)
	{
		LEDPORT->ODR ^= LED4;
	}
	
	vTaskDelay(_us);
}


/**
 * Is LED On or OFF
 * @param  _no LED Number: 1 - 4
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
	else if (_no == 4)
	{
		if ((LEDPORT->ODR & LED4) != 0)
		{
			return 1;
		}
		return 0;
	}

	return 0;
}

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
