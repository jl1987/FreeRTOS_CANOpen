/**
  ******************************************************************************
  * @file    lifter_control.c
  * @author  Jim
  * @version V1.0
  * @date    01-Sep-2014
  * @brief   This file provides CHASSIS variable definitions. 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef __CHASSIS_CONTROL_H__
#define __CHASSIS_CONTROL_H__

//#include "canfestival.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "can_STM32.h"
#include "canfestival.h"
#include "data.h"

#include "string.h"
#include "stdio.h"
#include "globalstruct.h"



#include "canopen_thread.h"
#include "bsp_led.h"
#include "motor.h"
#include "sensor.h"


// #include "FreeRTOS.h"
// #include "task.h"
// #include "queue.h"
// #include "can_STM32.h"
// #include "canfestival.h"

// //#include "globalstruct.h"
 #include "data.h"


typedef enum {false = 0, true = !false}bool;

#define ONSTACLE_DISTANCE 300	// distance for the assisted region for the robot
#define PI 3.14159
#define SPEED_CONSTANT 4500		// maximum speed of the chassis	1700
#define SPEED_CONSTANT_MIN 100		// minimum avoidance speed of the chassis
#define SPEED_STEP 10				// no of steps for acceleration / deceleration


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


/* Control Command Word */
#define COMMANDWORD_OPERATION_STOP 								0   //NO Key
#define COMMANDWORD_OPERATION_MOVE_FORWARD 				16	//W
#define COMMANDWORD_OPERATION_MOVE_BACKWARD 			2		//S
#define COMMANDWORD_OPERATION_MOVE_RIGHT	 				8		//E
#define COMMANDWORD_OPERATION_MOVE_RIGHT_FORWARD	24	//W+E
#define COMMANDWORD_OPERATION_MOVE_LEFT		 				32	//Q
#define COMMANDWORD_OPERATION_MOVE_LEFT_FORWARD		48	//W+Q

#define COMMANDWORD_OPERATION_TURN_RIGHT 					1		//D
#define COMMANDWORD_OPERATION_TURN_RIGHT_FORWARD 	17	//W+D
#define COMMANDWORD_OPERATION_TURN_LEFT						4		//A
#define COMMANDWORD_OPERATION_TURN_LEFT_FORWARD		20	//W+A


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



void chassis_control_thread(void *arg);

void start_chassis_control(void);

void Chassis_Init(Chassis_Data *ch);

double chassis_drive(double X_out,double Y_out,double Theta_out,u8 wheel);

void StepwiseFunction(Chassis_Data *chassis);
void ChassisMotionCtrl(Chassis_Data *ch);

UNS32 OnChassisCommandWordUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);


#endif

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
