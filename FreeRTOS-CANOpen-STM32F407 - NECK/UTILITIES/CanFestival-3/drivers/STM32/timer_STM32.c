/**
  ******************************************************************************
  * @file    timer_STM32.c
  * @author  Jim
  * @version V1.0
  * @date    01-Sep-2014
  * @brief   This file is edited as a part of CanFestival for STM32F407ZGT6.
  *
  *  @verbatim
  *  
  *        
  *          ===================================================================
  *                                 How to use this 
  *          ===================================================================  
  *           
  *          1. In this driver CANOpen get the TIMER from FreeRTOS:
  *                 void  vApplicationTickHook(void);
  *                 TimeDispatch();
  *
  *          2. FreeRTOS timer configuration: 
  *             - #define configUSE_TICK_HOOK 1 
  *                This means the OS use SYSTEM TICK as HOOK function.
  *             - #define portTICK_PERIOD_MS  ((TickType_t )1000/configTICK_RATE_HZ)
  *                This means the OS mini tick is 1ms.
  *
  *          3. The whole project configuration:
  *             CORE           PCB       LIB Version  CANFestival  FreeRTOS
  *             STM32F407ZGT6  CSST-ARM  FWLib1.0.2   3-7740       V8.0.1
  *  
  *   
  *  @note  Still have some doults about the CANOpen time in file timerscfg.h:
  *         #define TIMEVAL UNS32
  *         #define TIMEVAL_MAX 0xFFFF 
  *         #define MS_TO_TIMEVAL(ms) ((ms) )
  *         #define US_TO_TIMEVAL(us) ((us) / 1000)
  *        
  *  @endverbatim   
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */



/* Includes for the Canfestival driver */
#include "canfestival.h"
#include "timer.h"


void timer_can_irq_handler(void);
/* 时间片处理的钩子函数，每隔时间片处理一次 */
void vApplicationTickHook(void);

//为方便不同STM32之间的移植，采用FreeRTOS SYSTEM TICK时钟作为定时器，1ms一个中断
/* Define the timer registers */
/* 定义CANOpen闹钟定时器 */
UNS32 TimerAlarm_CAN;
/* 定会CANOpen计时定时器 */
UNS32 TimerCounter_CAN;
 

/************************** Modul variables **********************************/
/* Store the last timer value to calculate the elapsed time */
UNS32 last_time_set = 0; //TIMEVAL_MAX;

/**
 * Initializes the timer, turn on the interrupt and put the interrupt time to zero
 * 
 */
void initTimer(void)
{
  /* 初始化计时器为0 */
 	TimerCounter_CAN = 0;
	TimerAlarm_CAN   = 0;
}

/**
 * Set the timer for the next alarm.
 * @param   value TIMEVAL (unsigned long)
 * @return  void
 */
void setTimer(TIMEVAL value)
{
  /* Add the desired time to timer interrupt time */
  TimerAlarm_CAN += value;
}

/**
 * Return the elapsed time to tell the stack how much time is spent since last call.
 * @return  TIMEVAL (unsigned long) the elapsed time
 */
TIMEVAL getElapsedTime(void)
{
  unsigned long timer = TimerCounter_CAN;	// Copy the value of the running timer
  // Calculate the time difference
  return timer > last_time_set ? timer - last_time_set : last_time_set - timer;
}


/**
 * 处理定时器的函数
 * @brief 由于FreeRTOS系统定义中configUSE_TICK_HOOK为1，使用SYSTEM TICK。其调用
 *        顺序为：
 *        SysTick_Handler(已被定义为xPortSysTickHandler)->xTaskIncrementTick->
 *        vApplicationTickHook，也就是此处需要自己定义的函数，用作CANOpen定时器
 *
 *        注意：在portmacro.h中，#define portTICK_PERIOD_MS  ((TickType_t)1000/
 *        configTICK_RATE_HZ )，也就是说，间隔是1ms？
 */
void  vApplicationTickHook(void)
{
  /* CANOpen 定时器+1 */
  TimerCounter_CAN++;
  /* ALARM */
  if(TimerCounter_CAN == TimerAlarm_CAN)
  {
    last_time_set = TimerCounter_CAN;
    /* Call the time handler of the stack to adapt the elapsed time  */
    TimeDispatch();
    /* This is important!!!!! */
  }
}

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/

