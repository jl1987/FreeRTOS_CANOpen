/**********************************************************************************************************
*	                                  
*	模块名称 : CANOpen通讯模块
*	文件名称 : canopen_thread.h
* 作    者 ：Jim
* 版    本 ：1.0
* 日    期 ：2014.08.24
*
*	说    明 : 
* 	升降台控制程序
*
* 修改记录 :
* 	版本号		日期				作者				说明
* 	V1.0		2014-08-24		Jim				创建文件，添加基本功能描述
*
*	Copyright (C), 2013-2014, CSST
**********************************************************************************************************/

#ifndef __CANOPEN_THREAD_H__
#define __CANOPEN_THREAD_H__

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "can_STM32.h"
#include "canfestival.h"
#include "data.h"


typedef struct _struct_co_data_2_can{
	CO_Data *CO_CAN1;
	CO_Data *CO_CAN2;
} CO_DATA_STRUCT;

extern CO_DATA_STRUCT  CO_D;

void canopen_init(void);

#endif

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
