/**
  ******************************************************************************
  * @file    sensor.c
  * @author  Jim
  * @version V1.0
  * @date    16-Jan-2015
  * @brief   This file provides sensor control thread functions 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */
#include "sensor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "canfestival.h"
#include "can_STM32.h"

/* BSP */
#include "bsp_led.h"

/* Object Dictionary */
#include "ARM_OD.h"

xTaskHandle	 xT_SENSOR 	  = NULL;

u8 MSG_U5_R[8];
u8 MSG_U5_T[16];
u32 mask=0;


void sensor_thread(void * pvParameters)
{
	
	SENSOR_Init();
	
	printf("start the sensor \r\n");
	
	while(1)
	{
		/* Get Data From SENSOR using XXX */
		//GetDataFromSENSOR(&SENSOR);  //make it your own sensor function
		
		/* Copy Data to Object Dictionary */
		//ret_store = StoreSENSORDataToOD(&SENSOR,&SENSOR_OD_Data);  //make it your own sensor store function
		//printf("Sensor is working...");
	  vTaskDelay(SENSOR_CONTROL_THREAD_DELAY_TIMER);
		bsp_LedToggle(3,100);
		
	}
}


void start_sensor(void)
{
	xTaskCreate(sensor_thread, "sensor", SENSOR_THREAD_STACK, NULL,SENSOR_THREAD_PRIO, &xT_SENSOR);
	if(NULL == xT_SENSOR)
	{
	 	printf("sensor data thread creat failed!\r\n");
	}else
	{
		printf("sensor data thread creat successfully!\r\n");
	}
}

void SENSOR_Init(void)
{
	Pneumatic_Init();
	TouchSensor_Init();
}




// void GetDataFromSENSOR(SENSOR_STRUCT* sensor)
// {

// }

// UNS32 StoreSENSORDataToOD(SENSOR_STRUCT *sensor,CO_Data *d)
// {
// 	UNS32 size;
// 	UNS32 errorCode;
// 	
// 	size = sizeof(UNS16);
// 	
// 	errorCode = setODentry(d, (UNS16)0x6200, (UNS8)1,&(sensor->data), &size, 0);

// 	return errorCode;
// 	
// }




/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
