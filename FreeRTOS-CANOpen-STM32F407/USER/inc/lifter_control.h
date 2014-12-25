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

#include "math.h"


#define ONSTACLE_DISTANCE 300	// distance for the assisted region for the robot
#define PI 3.14159
#define SPEED_CONSTANT 4500		// maximum speed of the chassis	1700
#define SPEED_CONSTANT_MIN 100		// minimum avoidance speed of the chassis
#define SPEED_STEP 10				// no of steps for acceleration / deceleration





#define HALFPWMTIMERPERIOD 6980

#define Channel1_U    USART2
#define Channel2_U    UART5
#define Channel3_U    USART3
#define Channel1_PWM  TIM1->CCR3
#define Channel2_PWM  TIM1->CCR4
#define Channel3_PWM  TIM1->CCR2
#define Channel_PWMOUTPUT(VALUE)  HALFPWMTIMERPERIOD+VALUE

#define Channel1_RST  		GPIO_Pin_15
#define Channel2_RST  		GPIO_Pin_11
#define Channel3_RST  		GPIO_Pin_9
#define Channel1_RSTPORT  GPIOB
#define Channel2_RSTPORT  GPIOD
#define Channel3_RSTPORT  GPIOD

#define Channel1_FLT  		GPIO_Pin_8
#define Channel2_FLT  		GPIO_Pin_12
#define Channel3_FLT  		GPIO_Pin_10
#define Channel1_FLTPORT  GPIOD
#define Channel2_FLTPORT  GPIOD
#define Channel3_FLTPORT  GPIOD


#define Pi 		3.14159265358979
#define L1O 	100
#define L2O 	100
#define L3O 	100
#define Alpha1 Pi*0.166666667
#define Alpha2 Pi*0.833333333
#define Alpha3 Pi*1.5


typedef struct
{
  s32 V_SET;
	s32 V_GET;
	u8 fault_msg;
	u8 temperture;
}DRIVE;



u8 MSG_U2_R[8];
u8 MSG_U2_T[16];
u8 MSG_U3_R[8];
u8 MSG_U3_T[16];



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


void start_lifter_control(void);

void Moter_Init(void);

void Lidar_Init(void);
void Lidar_Stop(void);
void Lidar_Reset(void);

double chassis_drive(double X_out,double Y_out,double Theta_out,u8 wheel);
void chassis_move(double wheel1, double wheel2, double wheel3);

void delay(__IO uint32_t nCount);

#endif

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
