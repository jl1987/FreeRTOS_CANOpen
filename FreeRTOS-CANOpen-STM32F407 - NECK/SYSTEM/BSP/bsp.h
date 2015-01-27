/*
*********************************************************************************************************
*
*	模块名称 : BSP模块
*	文件名称 : bsp.h
* 作    者 ：Jim
* 版    本 ：1.0
* 日    期 ：2014.08.25
*
*	说    明 : 
* 	这是底层驱动模块所有的h文件的汇总文件。
* 	应用程序只需 #include bsp.h 即可，不需要#include 每个模块的 h 文件。
*
* 修改记录 :
* 	版本号		日期				作者				说明
* 	V1.0		2014-08-24		Jim				创建文件，添加基本功能描述
*
*	Copyright (C), 2013-2014, CSST
*********************************************************************************************************
*/

#ifndef _BSP_H_
#define _BSP_H

/* 定义 BSP 版本号 */
#define __STM32F4_BSP_VERSION		"1.0"

// /* 开关全局中断的宏 */
// #define ENABLE_INT()	__set_PRIMASK(0)	/* 使能全局中断 */
// #define DISABLE_INT()	__set_PRIMASK(1)	/* 禁止全局中断 */

/* 这个宏仅用于调试阶段排错 */
//#define BSP_Printf	printf


/* STM32 includes. */
#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

#include "main.h"



/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


#include "serial_debug.h"

/* CANOpen includes. */
#include "canopen_thread.h"

/* Robot Control includes. */
#include "chassis_control.h"
#include "sensor.h"


#ifndef TRUE
	#define TRUE  1
#endif

#ifndef FALSE
	#define FALSE 0
#endif


/* BSP Driver includes */
#include "bsp_led.h"
// #include "bsp_can.h"
// #include "bsp_usart.h"
// #include "bsp_tim.h"
// #include "bsp_exti.h"
// #include "bsp_adc.h"


/* 提供给其他C文件调用的函数 */
void bsp_Init(void);

#endif


