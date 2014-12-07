/**
  ******************************************************************************
  * @file    bsp.c
  * @author  Jim
  * @version V1.0
  * @date    01-Sep-2014
  * @brief   This is the bsp driver function for STM32F407ZGT6.
  * @brief 	 bsp = Borad surport packet 板级支持包
  *          这是硬件底层驱动程序的主文件。每个c文件可以 #include "bsp.h" 
  *          来包含所有的外设驱动模块
  * 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */


#include "bsp.h"
#include "stm32f10x_conf.h"


void bsp_Init(void)
{
	/*
		由于ST固件库的启动文件已经执行了CPU系统时钟的初始化，所以不必再次重复配置系统时钟。
		启动文件配置了CPU主时钟频率、内部Flash访问速度和可选的外部SRAM FSMC初始化。

		系统时钟缺省配置为168MHz，如果需要更改，可以修改 system_stm32f4xx.c 文件
	*/
   /* Enable CRC clock */
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
	
	
	 /* 针对不同的应用程序，添加需要的底层驱动模块初始化函数 */
	 bsp_InitLed(); 		/* 初始LED指示灯端口 */
}


/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
