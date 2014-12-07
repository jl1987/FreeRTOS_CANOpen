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

#ifndef __LIFTER_CONTROL_H__
#define __LIFTER_CONTROL_H__

//#include "canfestival.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "can_STM32.h"
#include "canfestival.h"
#include "data.h"

/* BSP about LIFTER */
#define RCC_LIFTER      RCC_AHB1Periph_GPIOE
#define LIFTER_PORT     GPIOE
#define LIFTER_PIN_INA GPIO_Pin_11
#define LIFTER_PIN_INB GPIO_Pin_13

/* Control word */
#define CONTROLWORD_OPERATION_UP 		0x01 //上升
#define CONTROLWORD_OPERATION_DOWM 	0x02 //下降
#define CONTROLWORD_OPERATION_STOP 	0x03 //停止

/* Status word */
#define STATUSWORD_SPEED_DEFAULT    0x01 //默认速度
#define STATUSWORD_SPEED_HIGH       0x02 //快速
#define STATUSWORD_SPEED_LOW        0x03 //慢速



#ifdef DEBUG_USART
  /* Debug Message */
  extern char lifter_thread_debugMSG[50]; //array for error message printing (for usart debugging)
#endif /* DEBUT_USART */

typedef struct _struct_lifter{
    CO_Data *p_od;
    UNS8 canBus;
    UNS8 nodeId;
    UNS16 controlword;
    UNS16 statusword;
    // INTEGER8 mode_setting;
    // INTEGER8 mode_actual;
    // INTEGER8 polarity;
    // INTEGER16 current_setting;
    // INTEGER16 current_actual;
    // INTEGER32 velocity_setting;
    // INTEGER32 velocity_actual;
    // INTEGER32 position_setting;
    // INTEGER32 position_actual;
    // UNS16 current_max;
    // UNS16 velocity_max;
    // UNS16 supple_voltage;
    // e_nodeState operate_state;
    UNS16 ErrorCode;
    UNS8 ErrorReg;
    char ErrorFlag;
}LIFTER_STRUCT;


void lifter_control_thread(void *arg);

void lifter_up(void);

void lifter_down(void);

void lifter_stop(void);

void start_lifter_control(void);




#endif

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
