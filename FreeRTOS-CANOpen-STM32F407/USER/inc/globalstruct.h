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

typedef enum {false = 0, true = !false}bool;

/* Priority Definition */
#define CANOpen_THREAD_PRIO 		(configMAX_PRIORITIES-1) 		//CANopen数据处理任务定为最高优先级
//#define TIMER_THEAD_PRIO 			(configMAX_PRIORITIES-2) 		//定时器任务优先级
//#define BMS_CONTROL_THREAD_PRIO 	(configMAX_PRIORITIES-3) 		//BMS任务优先级
#define CHASSIS_CONTROL_THREAD_PRIO (configMAX_PRIORITIES-2) 	//底盘任务优先级
//#define ARM_CONTROL_THREAD_PRIO 	(configMAX_PRIORITIES-5) 		//手臂任务优先级
//#define NECK_CONTROL_THREAD_PRIO 	(configMAX_PRIORITIES-6) 		//颈部任务优先级
//#define LIFTER_CONTROL_THREAD_PRIO 	(configMAX_PRIORITIES-2) 		//升降台任务优先级
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
#define CHASSIS_CONTROL_THREAD_DELAY_TIMER	 20
//#define ARM_CONTROL_THREAD_DELAY_TIMER     20
//#define NECK_CONTROL_THREAD_DELAY_TIMER    20
#define LIFTER_CONTROL_THREAD_DELAY_TIMER    20
//#define POWERA_CONTROL_THREAD_DELAY_TIMER  20
//#define POWERB_CONTROL_THREAD_DELAY_TIMER  20
#define SENSOR_CONTROL_THREAD_DELAY_TIMER		 20

/*  BSP_ID : NodeID used in CANOpen */
//#define BMS_ID     0x02
#define CHASSIS_ID 0x03
//#define ARM_ID     0x04
//#define NECK_ID    0x05
//#define LIFTER_ID    0x06
//#define POWERA_ID  0x07
//#define POWERB_ID  0x08









typedef struct struct_Chassis_Data Chassis_Data;


struct struct_Chassis_Data {
	/* Drive Par */
	u8 motion_command;						// the motion command of the chassis
	
	s32 V_SET;
	s32 V_GET;
	u8 fault_msg;
	u8 temperture;
	
	long double result_speed, result_speed_x, result_speed_y, result_direction;	// resltant speed and direction of the chassis [obstacle avoidance]
	int target_speed1;					// Angular speed of wheel 1 [target]
	int target_speed2;
	int target_speed3;
	int drive_speed1;					// Angular speed of wheel 1 [driving]
	int drive_speed2;
	int drive_speed3;
	int drive_diff1;
	int drive_diff2;
	int drive_diff3;
	
	/* Lidar Par */
	int Lidar_delay;  
	bool lidar_init_P;
	bool lidar_init_ok_P;
	
	
	
	
	
	
	/* Force Sensor Par */
	int x_value, y_value;
	double angle;
	double angle_diff;
	double prev_angle;
	double prev_angle_diff;
	double magnitude;
	double magnitude_diff;
	double prev_magnitude;
	double prev_magnitude_diff;
	double Kp;
	double Kd;

};















//extern xQueueHandle xQ_DRIVE_COMM, xQ_FLIP_COMM, xQ_ARM_COMM,xQ_HAND_COMM;
//extern const char* gNetCommandResStr[];
//extern char gNetBuffer[NET_BUFFER_MAX_NUMBER];
//extern int gNetDataSize;
#endif

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
