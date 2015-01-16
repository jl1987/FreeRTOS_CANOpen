/**
  ******************************************************************************
  * @file    main.c
  * @author  Jim
  * @version V1.0
  * @date    01-Sep-2014
  * @brief   This is the main program.
  * @brief   CANOpen-FreeRTOS Configuration:
  *          CORE            PCB       LIB Version  CANFestival  FreeRTOS
  *          STM32F407ZGT6   CSST-ARM  FWLib1.0.2   3-7740       V8.0.1
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */

//#include "bsp.h"  ////////////////////

#include "stm32f4xx.h"
#include <string.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "serial_debug.h"

#include "bsp_led.h"

#include "canopen_thread.h"
#include "chassis_control.h"
#include "sensor.h"

#include "stm32f4xx_conf.h"

/*--------------- Tasks Priority -------------*/     
//#define LED_TASK_PRIO    ( tskIDLE_PRIORITY + 1 )

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void ToggleLed1(void * pvParameters);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
 
  /*Initialize ARM BSP (LED) */ 
  bsp_InitLed();

	/* SERIAL_DEBUG */
  #ifdef SERIAL_DEBUG_ON
    DebugComPort_Init();
  #endif /* SERIAL_DEBUG */
  
//  xTaskCreate(pvTaskCode,   /*指向任务的实现函数的指针(效果上仅仅是函数名)*/
//              pcName,       /*具有描述性的任务名。这个参数不会被FreeRTOS使用*/
//              usStackDepth, /*栈空间大小，单位是字*/
//              pvParameters, /*任务函数接受一个指向void的指针(void*)*/
//              uxPriority,   /*指定任务执行的优先级，0到最高优先级(configMAX_PRIORITIES–1)*/
//              pxCreatedTask /*用于传出任务的句柄，不用则NULL代替。*/)
  /* Start toogleLed4 task : Toggle LED4  every 1s */
 // xTaskCreate(ToggleLed1, "LED1", configMINIMAL_STACK_SIZE, NULL, LED_TASK_PRIO, NULL);

  /* Start The CANOpen Communication */
  canopen_init();

  /* Start The Lifter Control */
  start_chassis_control();
	
	start_sensor();

  /* Start scheduler */
  vTaskStartScheduler();

	/* NEVER GET HERE as control is now taken by the SCHEDULER */
	while(1){}

}

/**
  * @brief  Toggle Led4 task
  * @param  None
  * @retval None
  */
// void ToggleLed1(void * pvParameters)
// {
// 	/* Toggle LED1 each 500s */
//   for( ;; )
//   {
// 		//printf("I'm the LED1 blinking");
// 		bsp_LedToggle(1,1000);
//   }
// 	
// }



/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
