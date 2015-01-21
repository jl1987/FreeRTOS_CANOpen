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

#ifndef __ARM_CONTROL_H__
#define __ARM_CONTROL_H__

//#include "canfestival.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "can_STM32.h"
#include "canfestival.h"
#include "data.h"
#include "functions.h"

#include "string.h"
#include "stdio.h"
#include "globalstruct.h"

#include "canopen_thread.h"
#include "bsp_led.h"
#include "motor.h"
#include "sensor.h"
#include "pneumatic.h"
#include "touch_sensor.h"


#define SAT_FILTER(val, min, max)   val > max ? max : (val < min ? min : val)
#define THETA3F_CALC_TIMES 				61  						// fixed theta3f calc times, 08092013 update
#define pi 												3.14f 					//3.14159265 
#define SQRT3_DIV2  							0.8660254 			// sqrt(3)/2
#define SQRT3 										1.73205081 			// sqrt(3)
#define MOTOR_TRANSFER_1X 				195.37860814f		//1x
#define PRESS_COEFFICIENT_1 			0.16663525f    	//04232013
#define PRESS_COEFFICIENT_2 			-1.69013382f   	//04232013
#define ANGLE_2_MOTOR_POS_RADIO 	195.37860814  	//(1023/300)*(180/pi)
//#define THETA3F_CALC_TIMES 			58  						// fixed theta3f calc times
#define THETA3F_CALC_TIMES 				61  						// fixed theta3f calc times, 08092013 update
#define uart1_rx_len 							4 							// if char set 8, if Hex set 4 
//#define uart1_rx_len 						7 							// 06212013 input cable length test 
#define ctrl_mode_swtich_len 			11	  					// single motor or all
#define read_motor_len 						11	      			// single motor
#define single_joint_ctrl_len 		11  // single motor or all
#define single_speed_ctrl_len 		9	  // single motor or all
#define single_maxtorque_ctrl_len 9	  // single motor or all
#define single_torque_enable_len 	8  	// singer motor torque on/off
#define syn_5dof_torque_enable_len 30 // 5dof motors torque on/off
#define syn_joint_ctrl_len 				38
#define syn_joint_ctrl_len_4dof	 	28
#define syn_joint_ctrl_len_5dof 	33
#define syn_speed_ctrl_len 				26
#define clear_ax12_error_len 			8  		// clear single or all motors errors
#define command_HL_len 						2
#define	arm_suitable_voltage	 		335   // stop and hold on 
//#define	arm_suitable_voltage	 	320   // stop and hold on 
#define	uart_speed_map_radio 			10 		// old 10.23f 	
#define	MAX_TORQUE_LIMITED 				300 	// MAX torque limit
#define	CTRL_SPD_CCW 							250  	// speed control with fixed speed 
#define	CTRL_SPD_ID_1 						10   	// speed control with fixed speed
#define	CTRL_SPD_ID_2 						12   	// speed control with fixed speed
#define CTRL_SPD_DELAY 						0x2ffffff // speed control move time

#define traj_buffer_len 					100 		// important here, 05292013
#define SPEED_PERCENT 						0.3f  	// move percent for trajectory
#define MIN_JUMP_DISTANCE 				50 			// min jump distance  
#define  MAX_DELTA_POS  					15 			// for planning method 2, 05282013 


static float ZERO_f = 0.0; 
static float PI_DIV_36 = 0.08726646;  	//pi/36
static float EPSILON = 1e-6; //Min value

//typedef enum {false = 0, true = !false}bool;


/*PID parameters*/ 
struct pid_t {
  float e;      /**< Error, SP(set point) - PV(process value) */
  float i;      /**< Error Integral variable */
  float d;      /**< Error derivative variable */   
  float Kp;     /**< Proportional gain, a tuning parameter */
  float Ki;     /**< Integral gain, a tuning parameter */
  float Kd;     /**< Derivative gain, a tuning parameter */
	float ScallFactor;  /**< Apply to the pwm duty region */
	int Max_v;	/**< max filter limited value */
	int Min_v;	/**< min filter limited value */
};

typedef struct struct_Arm_Data Arm_Data;
struct struct_Arm_Data {
	/* Drive Par */
	u8 pneumatic_control_enable; //pneumatic ON or OFF
	
	uint8_t gb_left_touchsensor_switch_cmd;  // 08/12/2014 added
	uint8_t gb_right_touchsensor_switch_cmd; 	// 08/12/2014 added
	
	
	uint8_t gb_hand_index;       //choose 
	uint8_t gb_hand_pose_index; 
	uint8_t gb_hand_pose_enable;  
	
	int8_t gb_current_moving_mode; 		 	// 0 is homing mode, 1 is teach mode, 2 is tracking mode
	int8_t gb_previous_moving_mode;  			// 0 is homing mode, 1 is teach mode, 2 is tracking mode, added 11/06/2014
	
	
	volatile uint8_t command_hand; 
	
	volatile uint8_t breakForLoop;  // break for loop
	
	uint8_t gb_arm_index;
	
	volatile uint8_t preMoveCommand;	
	
	
	volatile uint8_t gb_rx_px; //reveived positions
	volatile uint8_t gb_rx_py;
	volatile uint8_t gb_rx_pz; 
	volatile uint8_t gb_rx_az; 

	volatile uint8_t gb_pre_rx_px;
	volatile uint8_t gb_pre_rx_py;
	volatile uint8_t gb_pre_rx_pz;   

	volatile int32_t gb_arm_px; // target px 
	volatile int32_t gb_arm_py; // target py 
	volatile int32_t gb_arm_pz; // target pz 

	volatile int32_t pre_arm_px; // previous px, or initial pos
	volatile int32_t pre_arm_py; // previous py
	volatile int32_t pre_arm_pz; // previous pz 

	volatile int32_t delta_arm_px; // delta_arm_px
	volatile int32_t delta_arm_py; // delta_arm_py
	volatile int32_t delta_arm_pz; // delta_arm_pz

	volatile uint32_t delta_arm_pos; // delta_arm pos
	
	float theta_solution_buffer[THETA3F_CALC_TIMES][5];
	volatile float theta3f_solution[4];
	volatile float theta4f_solution[4];
	volatile float theta3f_abs_diff;
	volatile float theta4f_abs_diff; 
	
	float gb_theta3f_var; 
	float gb_theta4f_var; 
	int8_t gb_theta3f_t; 
	int8_t gb_theta4f_t;
	uint8_t gb_try_theta3f_ok; //debug
	uint8_t gb_theta3f_calc_times; 	//debug
	uint8_t gb_try_theta4f_ok; 	//debug
	
	
	uint8_t gb_theta3f_exist;
	uint8_t gb_theta4f_exist;
	uint8_t gb_theta_s_index;
	uint8_t gb_theta_s_index2; 
	uint8_t gb_best_s_index; 
	
	
	volatile float gb_fwd_pos[3]; // global t01 position, for selection best solution, update 08072013
	volatile float gb_fwd_px_temp;
	volatile float gb_fwd_px_max; 
	
	
	float theta_solution[5]; 
	
	

	int32_t gb_right_m_pos[5];       // target glob_motor_pos
	int32_t gb_left_m_pos[5];       // target glob_motor_pos
	
	
	uint8_t motor_pos_low[6]; // fixed;
	uint8_t motor_pos_high[6]; 						// fixed; 
	uint8_t motor_speed_low[6]; 			// fixed
	uint8_t motor_speed_high[6]; 						// fixed
	int32_t motor_fixed_speed[6]; 					// fixed tracking speed better
	int32_t rigid_arm_init_motor_pos[5]; 	// fixed, update 09172013 new, ok	test kinematis intial										
	int32_t right_arm_home_motor_pos[5]; 	// fixed, update 09052013	  // 818 should ref motor direction
	int32_t left_arm_init_motor_pos[5]; 	// fixed, 09172013 new, ok	test kinematis intial	
	int32_t left_arm_home_motor_pos[5]; 	// fixed, update 09052013, 	// 818 should ref motor direction
	
	uint32_t left_fist_desired_press[5];
	uint32_t right_fist_desired_press[5];
	uint32_t left_plam_desired_press[5];
	uint32_t right_plam_desired_press[5];
	
	uint8_t gb_monitor_counter;  // 10/24/2014 added 
};

/*ARM parameters*/ 



int pid_update(int sp, int pv, int dt,struct pid_t *pid);
void pid_init(float Kp, float Ki, float Kd, float ScallFactor, int Max_v, int Min_v,struct pid_t *pid);


uint8_t ikine_4dof_rigid_arm_theta3f(uint8_t arm_index, int32_t px, int32_t py, int32_t pz, float theta3f, uint8_t s_index,Arm_Data *arm);
void hand_desired_angle_crl(uint8_t enable_angle_crl, uint8_t desired_rx_angle,Arm_Data *arm);
void angle_2_motor_pos(uint8_t arm_index, float angle[5],Arm_Data *arm);

void AX_12_Syn_Ctrl_5DOF_Rigid_Arm(uint8_t arm_index, int32_t position_in[5], int32_t speed_in[5]);


void Homing(Arm_Data *arm);


void Joint_Teach_DEMO_1(uint8_t arm_index,Arm_Data *arm);
void Joint_Teach_DEMO_2(uint8_t arm_index,Arm_Data *arm);
void Joint_Teach_DEMO_3(uint8_t arm_index,Arm_Data *arm);
void Joint_Teach_DEMO_4(uint8_t arm_index,Arm_Data *arm);
void Joint_Teach_DEMO_5(uint8_t arm_index,Arm_Data *arm);
void Joint_Teach_DEMO_6(uint8_t arm_index,Arm_Data *arm);



void arm_control_thread(void *arg);

void start_arm_control(void);

void Arm_Init(Arm_Data *ch);

void AX_12_Ctrl(uint8_t id_index,uint8_t posLow, uint8_t posHigh);
void AX_12_Syn_Ctrl(uint8_t arm_index,int32_t position_in[6], int32_t speed_in[6]);


void AX_12_CLEAR_ERROR(uint8_t id_index);
void AX_12_Ctrl_Pos_and_Speed(uint8_t id_index, int position_in, int speed_in);
void AX_12_Moving_Mode_Swtich(uint8_t id_index, uint8_t mode);
uint8_t AX_12_Read(uint8_t id_index);
void AX_12_Speed_Ctrl(uint8_t id_index, int goal_speed_in);
void AX_12_Speed_Syn_Ctrl(uint8_t arm_index, int speed_in[6]);
void AX_12_MaxTorque_Ctrl(uint8_t id_index, int max_torque);
void AX_12_Torque_Enable(uint8_t id_index, uint8_t torque_enable);
void AX_12_5DOF_Syn_Torque_Enable(uint8_t arm_index, uint8_t torque_enable);

uint8_t testNot1(int32_t XDec);	 
uint8_t XDec2Low(int32_t XDec);	  
uint8_t XDec2High(int32_t XDec);	
int Hex2Dec(unsigned char Hex);  //uint8_t Hex2Dec(unsigned char Hex); 

	
uint8_t ikine_theta3f(uint8_t arm_index, int32_t px, int32_t py, int32_t pz, float theta3f, uint8_t s_index,Arm_Data *arm);
uint8_t ikine_theta4f(uint8_t arm_index, int32_t px_in, int32_t py_in, int32_t pz_in, float theta4f, uint8_t s_index,Arm_Data *arm);	


void HomingFast(Arm_Data *arm);
void SingleArmHoming(uint8_t arm_index,Arm_Data *arm);
void InitialPos(Arm_Data *arm);
void InitialPosTest(Arm_Data *arm); 
void infltableHandMotionCtrl(uint8_t pneumatic_enable, uint8_t arm_index,Arm_Data *arm);
void infltableFingerMotionCtrl(uint8_t hand_index, uint8_t finger_index, int8_t motion_direction,Arm_Data *arm);

void infltableHandPose(uint8_t pneumatic_enable, uint8_t hand_index, uint8_t pose_index);

void CAN2_RX_Hand_Pose_Ctrl(uint8_t hand_pose, uint8_t arm_motion, Arm_Data *arm);

void TouchSensorBasedArmTorqueCtrl(uint8_t hand_index);


UNS32 OnArmCommandWordUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex);

void CAN2_RX0(void);


void Delay(u32 n);
void delay_t(unsigned int dl);
void Delay_ms(uint32_t nTime);
void delay_t(unsigned int dl);

#endif

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
