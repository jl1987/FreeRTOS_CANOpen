/**
  ******************************************************************************
  * @file    globalstruct.h
  * @author  Jim
  * @version V1.0
  * @date    01-Sep-2014
  * @brief   This is the global variable definition file.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */
 
#ifndef __GLOBAL_STRUCT_H__
#define	__GLOBAL_STRUCT_H__

/* Priority Definition */
#define CANOpen_THREAD_PRIO 		(configMAX_PRIORITIES-1) 		//CANopen数据处理任务定为最高优先级
//#define TIMER_THEAD_PRIO 			(configMAX_PRIORITIES-2) 		//定时器任务优先级
//#define BMS_CONTROL_THREAD_PRIO 	(configMAX_PRIORITIES-3) 		//BMS任务优先级
//#define CHASSIS_CONTROL_THREAD_PRIO (configMAX_PRIORITIES-4) 	//底盘任务优先级
//#define ARM_CONTROL_THREAD_PRIO 	(configMAX_PRIORITIES-5) 		//手臂任务优先级
//#define NECK_CONTROL_THREAD_PRIO 	(configMAX_PRIORITIES-6) 		//颈部任务优先级
#define LIFTER_CONTROL_THREAD_PRIO 	(configMAX_PRIORITIES-2) 		//升降台任务优先级
//#define POWERA_CONTROL_THREAD_PRIO 	(configMAX_PRIORITIES-8) 	//电源控制A(上身)  
//#define POWERB_CONTROL_THREAD_PRIO 	(configMAX_PRIORITIES-9) 	//电源控制B(下身)

#define SENSOR_THREAD_PRIO      (configMAX_PRIORITIES-3)  //Sensor Priority


/* Stack SIZE Definition */
#define CANOPEN_THREAD_STACK          500
#define TIMER_THEAD_STACK             500
#define BMS_CONTROL_THREAD_STACK      2000
#define CHASSIS_CONTROL_THREAD_STACK  2000
#define ARM_CONTROL_THREAD_STACK      2000
#define NECK_CONTROL_THREAD_STACK     2000
#define LIFTER_CONTROL_THREAD_STACK   1000
#define POWERA_CONTROL_THREAD_STACK   500
#define POWERB_CONTROL_THREAD_STACK   500
#define SENSOR_THREAD_STACK   				500

/*  Thread Delay Timer (if avilable) unit:ms */
#define CANOpen_THREAD_DELAY_TIMER           20
//#define TIMER_THEAD_DELAY_TIMER            20
//#define BMS_CONTROL_THREAD_DELAY_TIMER     20
//#define CHASSIS_CONTROL_THREAD_DELAY_TIMER 20
//#define ARM_CONTROL_THREAD_DELAY_TIMER     20
//#define NECK_CONTROL_THREAD_DELAY_TIMER    20
#define LIFTER_CONTROL_THREAD_DELAY_TIMER    20
//#define POWERA_CONTROL_THREAD_DELAY_TIMER  20
//#define POWERB_CONTROL_THREAD_DELAY_TIMER  20
#define SENSOR_CONTROL_THREAD_DELAY_TIMER		 20

/*  BSP_ID : NodeID used in CANOpen */
//#define BMS_ID     0x02
//#define CHASSIS_ID 0x03
//#define ARM_ID     0x04
//#define NECK_ID    0x05
#define LIFTER_ID    0x06
//#define POWERA_ID  0x07
//#define POWERB_ID  0x08




//extern xQueueHandle xQ_DRIVE_COMM, xQ_FLIP_COMM, xQ_ARM_COMM,xQ_HAND_COMM;
//extern const char* gNetCommandResStr[];
//extern char gNetBuffer[NET_BUFFER_MAX_NUMBER];
//extern int gNetDataSize;
#endif

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
