/**
  ******************************************************************************
  * @file    lifter_control.c
  * @author  Jim
  * @version V1.0
  * @date    01-Sep-2014
  * @brief   This file provides LIFTER control thread functions 
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */

#include "arm_control.h"
#include "math.h"

/* Object Dictionary */
#include "ARM_OD.h"

/* Private variables ---------------------------------------------------------*/
xQueueHandle xQ_ARM_MSG = NULL;
xTaskHandle	 xT_ARM 	  = NULL;

Arm_Data ARM_D;


struct pid_t pid_per;
int 	 pid_mv = 0;

volatile uint8_t left_hand_pump_pwm_duty[5] 		= {0,0,0,0,0};	//[0 100]
volatile uint8_t right_hand_pump_pwm_duty[5] 		= {0,0,0,0,0};	//[0 100]

static __IO uint32_t TimingDelay;

__IO uint32_t left_hand_actual_AD_value[5] = {0,0,0,0,0};
__IO uint32_t right_hand_actual_AD_value[5] = {0,0,0,0,0}; 

__IO uint32_t AD_value_1=0;	
__IO uint32_t AD_value_2=0;
__IO uint32_t AD_value_3=0;	

volatile int32_t motor_speed_input					= 0;	   
volatile int32_t motor_speed_percent_input	= 0;

volatile uint8_t pump_pwm_duty 	= 0;			//[0 100]

uint16_t capture = 0;	

uint8_t 	uart1_rx_index=0;
volatile 	uint8_t uart1_rx_buffer[uart1_rx_len];  // uart buffer


/* Private functions ---------------------------------------------------------*/
void arm_control_thread(void * pvParameters)
{
	printf("start the chassis control\r\n");
	
	Arm_Init(&ARM_D);
	
	RegisterSetODentryCallBack(CO_D.CO_CAN2, 0x2000, 0x00, &OnArmCommandWordUpdate);	//Control word 
	
	while(1)
	{
		/* Check Motor Fault */
		
		
		//printf("chassis control is running\r\n");
		printf("ODODODODO:0x2000|00: %x \r\n",motion_command);
		bsp_LedToggle(2,100);
		vTaskDelay(ARM_CONTROL_THREAD_DELAY_TIMER);
	}
}





// void CAN1Master_ConfigureNode(CO_Data* d)
// {
// // 	  d->heartbeatError   = CAN1Master_heartbeatError;
// // 	  d->initialisation   = CAN1Master_initialisation;
// // 	  d->preOperational   = CAN1Master_preOperational;
// // 	  d->operational      = CAN1Master_operational;
// // 	  d->stopped	      = CAN1Master_stopped;
// // 	  d->post_SlaveBootup = CAN1Master_SlaveBootup;
// // 	  d->post_emcy        = CAN1Master_emcy;
// // 	  setState(d,Initialisation);
// }


void start_arm_control(void)
{
	xTaskCreate(arm_control_thread, "arm_control", ARM_CONTROL_THREAD_STACK, NULL,ARM_CONTROL_THREAD_PRIO, &xT_ARM);
	if(NULL == xT_ARM)
	{
	 	printf("arm control thread creat failed!\r\n");
	}else
	{
		printf("arm control thread creat successfully!\r\n");
	}
}



void Arm_Init(Arm_Data *arm)
{
	int i;
	
	Motor_Init();
	
	arm->pneumatic_control_enable = 0;
	
	arm->gb_left_touchsensor_switch_cmd	 =	0;
	arm->gb_right_touchsensor_switch_cmd =	0;
	
	arm->gb_current_moving_mode	 = -1; 		// 0 is homing mode, 1 is teach mode, 2 is tracking mode
	arm->gb_previous_moving_mode = -2;  	// 0 is homing mode, 1 is teach mode, 2 is tracking mode, added 11/06/2014
	
	
	arm->gb_hand_index 			= 0; 
	arm->gb_hand_pose_index 	= 0; 
	arm->gb_hand_pose_enable = 0;
	
	arm->command_hand=0; 
	
	arm->preMoveCommand = 0;
	
	arm->gb_rx_px=0; //reveived positions
	arm->gb_rx_py=0;
	arm->gb_rx_pz=0; 
	arm->gb_rx_az=0; 
	
	arm->gb_pre_rx_px=0;
	arm->gb_pre_rx_py=0;
	arm->gb_pre_rx_pz=0;
	

	arm->gb_arm_px=0; // target px 
	arm->gb_arm_py=0; // target py 
	arm->gb_arm_pz=0; // target pz 

	arm->pre_arm_px=440; 	// previous px, or initial pos
	arm->pre_arm_py=0; 		// previous py
	arm->pre_arm_pz=0; 		// previous pz 

	arm->delta_arm_px=0; // delta_arm_px
	arm->delta_arm_py=0; // delta_arm_py
	arm->delta_arm_pz=0; // delta_arm_pz

	arm->delta_arm_pos=0; // delta_arm pos
	
	
	arm->gb_arm_index = 254; 
	
	

	arm->theta3f_solution[0]=0;
	arm->theta3f_solution[1]=0;
	arm->theta3f_solution[2]=0;
	arm->theta3f_solution[3]=0;

	
	arm->theta4f_solution[0]=0;
	arm->theta4f_solution[1]=0;
	arm->theta4f_solution[2]=0;
	arm->theta4f_solution[3]=0;
	
	arm->theta3f_abs_diff = 0;
	arm->theta4f_abs_diff = 0; 
	arm->gb_theta3f_var	= 0; 
	arm->gb_theta4f_var	= 0; 
	arm->gb_theta3f_t 	= 0; 
	arm->gb_theta4f_t		= 0;
	
	arm->gb_try_theta3f_ok = 0; 			//debug
	arm->gb_theta3f_calc_times = 0; 	//debug
	arm->gb_try_theta4f_ok 	 = 0; 		//debug
	
	
	arm->gb_theta3f_exist = 0;
	arm->gb_theta4f_exist = 0;
	arm->gb_theta_s_index = 0;
	arm->gb_theta_s_index2 = 0; 
	arm->gb_best_s_index = 0; 
	
	arm->gb_fwd_pos[0]	=0; // global t01 position, for selection best solution, update 08072013
	arm->gb_fwd_pos[1]	=0;
	arm->gb_fwd_pos[2]	=0;
	
	arm->gb_fwd_px_temp = 0;
	arm->gb_fwd_px_max 	= 0; 
	
	arm->theta_solution[0]=0; 
	arm->theta_solution[1]=0; 
	arm->theta_solution[2]=0; 
	arm->theta_solution[3]=0; 
	arm->theta_solution[4]=0; 
	
	
	
	arm->gb_right_m_pos[0] = 0;       // target glob_motor_pos
	arm->gb_right_m_pos[1] = 0; 
	arm->gb_right_m_pos[2] = 0; 
	arm->gb_right_m_pos[3] = 0; 
	arm->gb_right_m_pos[4] = 0; 
	
	
	arm->gb_left_m_pos[0]  = 0;       // target glob_motor_pos
	arm->gb_left_m_pos[1]  = 0; 
	arm->gb_left_m_pos[2]  = 0; 
	arm->gb_left_m_pos[3]  = 0; 
	arm->gb_left_m_pos[4]  = 0; 
	
	for (i=0;i<6;i++)
	{
		arm->motor_pos_low[i] 		= 255;
		arm->motor_pos_high[i]	  = 3;
		arm->motor_speed_low[i]   =30;
		arm->motor_speed_high[i]  = 0;
		arm->motor_fixed_speed[i] = 50;
	}
	
	arm->rigid_arm_init_motor_pos[0] = 205; 	// fixed, update 09172013 new, ok	test kinematis intial			
	arm->rigid_arm_init_motor_pos[1] = 205; 
	arm->rigid_arm_init_motor_pos[2] = 818; 
	arm->rigid_arm_init_motor_pos[3] = 512; 
	arm->rigid_arm_init_motor_pos[4] = 512; 



	arm->right_arm_home_motor_pos[0] = 205; 	// fixed, update 09052013	  // 818 should ref motor direction
	arm->right_arm_home_motor_pos[1] = 205; 
	arm->right_arm_home_motor_pos[2] = 512; 
	arm->right_arm_home_motor_pos[3] = 512; 
	arm->right_arm_home_motor_pos[4] = 512; 
	
	
	arm->left_arm_init_motor_pos[0]  = 818; 	// fixed, 09172013 new, ok	test kinematis intial	
	arm->left_arm_init_motor_pos[1]  = 818;
	arm->left_arm_init_motor_pos[2]  = 205;
	arm->left_arm_init_motor_pos[3]  = 512;
	arm->left_arm_init_motor_pos[4]  = 512;
	
	arm->left_arm_home_motor_pos[0]  = 818; 	// fixed, update 09052013, 	// 818 should ref motor direction
	arm->left_arm_home_motor_pos[1]  = 818; 
	arm->left_arm_home_motor_pos[2]  = 512; 
	arm->left_arm_home_motor_pos[3]  = 512; 
	arm->left_arm_home_motor_pos[4]  = 512; 
	
	arm->breakForLoop = 0;
	
	for (i=0;i<5;i++)
	{
		arm->left_fist_desired_press[i] 	= 900;
		arm->right_fist_desired_press[i] 	= 900;
		arm->left_plam_desired_press[i] 	= 220;
		arm->right_plam_desired_press[i] 	= 220;
	}
	
	arm->gb_monitor_counter = 0;
	
}


void ArmMotionCtrl(Arm_Data *arm)
{
		if (arm->pneumatic_control_enable==1)  
		{
			/* gb_current_moving_mode commands */
			switch (arm->gb_current_moving_mode)
			{
				case 1: /* Joint space teach progam, without inverse kinematic, 05312013*/
					if ((arm->command_hand==1)||(arm->command_hand==2)||(arm->command_hand==5))	
					{
					  if((arm->gb_rx_px==0)&&(arm->gb_rx_py==0)&&(arm->gb_rx_pz==1))
						{
							Joint_Teach_DEMO_2(1,&ARM_D);		// wave hand, ok
						}
						else if((arm->gb_rx_px==0)&&(arm->gb_rx_py==1)&&(arm->gb_rx_pz==0))
						{
							 Joint_Teach_DEMO_3(1,&ARM_D);	// shake hand, ok
						}
						else if((arm->gb_rx_px==1)&&(arm->gb_rx_py==0)&&(arm->gb_rx_pz==0))
						{
							 Joint_Teach_DEMO_4(1,&ARM_D); 	// go style, ok
						}
						else if((arm->gb_rx_px==0)&&(arm->gb_rx_py==1)&&(arm->gb_rx_pz==1))
						{
							 Joint_Teach_DEMO_6(1,&ARM_D); 	// dancing, ok
						}
					}
					else if ((arm->command_hand==3)||(arm->command_hand==4)||(arm->command_hand==6))
					{
						if((arm->gb_rx_px==0)&&(arm->gb_rx_py==0)&&(arm->gb_rx_pz==1))
						{
							Joint_Teach_DEMO_2(2,&ARM_D);   // wave hand, ok
						}
						else if((arm->gb_rx_px==0)&&(arm->gb_rx_py==1)&&(arm->gb_rx_pz==0))
						{
							 Joint_Teach_DEMO_3(2,&ARM_D);	// shake hand, ok
						}
						else if((arm->gb_rx_px==1)&&(arm->gb_rx_py==0)&&(arm->gb_rx_pz==0))
						{
							 Joint_Teach_DEMO_4(2,&ARM_D); 	// go style, ok
						}
						else if((arm->gb_rx_px==0)&&(arm->gb_rx_py==1)&&(arm->gb_rx_pz==1))
						{
							 Joint_Teach_DEMO_6(2,&ARM_D); 	// dancing, ok
						}
					}
					break;

				case 2:	  /* vision tracking mode, with inverse kinematic, 060752013*/
					arm->gb_arm_index = 254; // initialize
					if ((arm->command_hand==1)||(arm->command_hand==2)||(arm->command_hand==5))	
					{
						arm->gb_arm_index = 1; // left arm
					}
					else if ((arm->command_hand==3)||(arm->command_hand==4)||(arm->command_hand==6))
					{
						arm->gb_arm_index = 2; // right arm
					}					


					/*Enable arm inverse kinematics*/
					if ((arm->gb_arm_index ==1)||(arm->gb_arm_index ==2))
					{
						/*inverse kinematics*/
						/* new version 2, 08072013*/
						arm->gb_theta3f_exist 		 = 0; 			//initialize
						arm->gb_theta_s_index 		 = 0; 			// solution index						
						arm->gb_theta3f_calc_times = 0; 			//debug
						arm->gb_theta3f_var				 = ZERO_f; 	//initialize
						/* theta3f is variable*/
						for(arm->gb_theta3f_t=-30;arm->gb_theta3f_t<31;arm->gb_theta3f_t++) // calc 29*2 times, scale 5 degree,08092013 update
						{
							arm->gb_theta3f_calc_times = arm->gb_theta3f_calc_times+1; //debug	
							arm->gb_theta3f_var 		   = (float)(arm->gb_theta3f_t*PI_DIV_36); //pi/30, region [-150 150] degree,08092013 update								
							arm->gb_theta3f_exist 		 = ikine_4dof_rigid_arm_theta3f(arm->gb_arm_index, arm->gb_arm_px, arm->gb_arm_py, arm->gb_arm_pz,arm->gb_theta3f_var,0,&ARM_D);		
							if(arm->gb_theta3f_exist>=1) // when exist solution, record solution
							{								
								arm->theta_solution_buffer[arm->gb_theta_s_index][0]= arm->theta3f_solution[0];
								arm->theta_solution_buffer[arm->gb_theta_s_index][1]= arm->theta3f_solution[1];
								arm->theta_solution_buffer[arm->gb_theta_s_index][2]= arm->theta3f_solution[2];
								arm->theta_solution_buffer[arm->gb_theta_s_index][3]= arm->theta3f_solution[3];
								arm->theta_solution_buffer[arm->gb_theta_s_index][4]= arm->gb_fwd_pos[0]; //t01, px
								arm->gb_theta_s_index = arm->gb_theta_s_index + 1;
							  //break;
							}		 
						}

						/*When exist solution, get the best solution*/
						if(arm->gb_theta_s_index>0)
						{							
							/*get best solution, by max gb_fwd_pos[0] to judge solution*/					
							arm->gb_fwd_px_temp =  arm->theta_solution_buffer[0][4];
							arm->gb_best_s_index = 0;
							for(arm->gb_theta_s_index2=0;arm->gb_theta_s_index2<arm->gb_theta_s_index;arm->gb_theta_s_index2++) // calc 29*2 times, scale 5 degree
							{
								if (arm->theta_solution_buffer[arm->gb_theta_s_index2][4]>arm->gb_fwd_px_temp)
								{
								   arm->gb_fwd_px_temp = arm->theta_solution_buffer[arm->gb_theta_s_index2][4];
								   arm->gb_best_s_index = arm->gb_theta_s_index2;
								}   
							}
							// choose the crospronding solution as the best solution 
							arm->theta_solution[0] = arm->theta_solution_buffer[arm->gb_best_s_index][0];
							arm->theta_solution[1] = arm->theta_solution_buffer[arm->gb_best_s_index][1]; // initial offset
							arm->theta_solution[2] = arm->theta_solution_buffer[arm->gb_best_s_index][2];
							arm->theta_solution[3] = arm->theta_solution_buffer[arm->gb_best_s_index][3]; 
						    
							/*calc hand rotation angle, 12/05/2013 new version*/ 
							hand_desired_angle_crl(1,arm->gb_rx_az,&ARM_D);	
							
							
							/*calc motor position*/
							angle_2_motor_pos(arm->gb_arm_index,arm->theta_solution,&ARM_D);
							
							/*send positions to motor*/
							if (arm->gb_arm_index==1) // left arm
							{								
							 	AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm->gb_arm_index,arm->gb_left_m_pos, arm->motor_fixed_speed); 
							} 		
							if (arm->gb_arm_index==2) // right arm
							{								
								AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm->gb_arm_index,arm->gb_right_m_pos, arm->motor_fixed_speed);
							}										
					}
				}
					break;			
				default: 				
					break;
			}
	}
}

void pid_init(float Kp, float Ki, float Kd, float ScallFactor, int Max_v, int Min_v,struct pid_t *pid)
{
    pid->e = 0;
    pid->i = 0;
    pid->d = 0;	    
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;  
		pid->ScallFactor = ScallFactor;  
		pid->Max_v = Max_v;
    pid->Min_v = Min_v;	
}

int pid_update(int sp, int pv, int dt,struct pid_t *pid)
{
    float error;
    int mv;
    
    /* calculate error */
    error = sp - pv;
    /* error integration */
    pid->i += error * dt;
    /* error derivation */
    pid->d = (error - pid->e) / dt;
    /* update error */
    pid->e = error;
    
    /* calculate output */
    mv = (pid->Kp * pid->e) + (pid->Ki * pid->i) + (pid->Kd * pid->d);
	 
		mv = mv*pid->ScallFactor; // [0 100] duty
			/* saturation filter */
		mv = SAT_FILTER(mv, pid->Min_v, pid->Max_v);	
	 
    return mv;
}

/*Joint_Teach_DEMO_1, 05312013*/
void Joint_Teach_DEMO_1(uint8_t arm_index, Arm_Data *arm)
{	

	arm->breakForLoop = 0; // release break, 06052013
}



/*Joint_Teach_DEMO_2, 08/04/2014 new version*/
/* wave hand*/
void Joint_Teach_DEMO_2(uint8_t arm_index,Arm_Data *arm)
{
	uint8_t i;
	int32_t l_teach_pos[5],r_teach_pos[5];
	int32_t l_teach_spd[5],r_teach_spd[5];
	 	
	if(arm_index ==1) // left arm
	{		
		// P1
		l_teach_pos[0]=818-357; l_teach_spd[0]=50;
		l_teach_pos[1]=818-10; l_teach_spd[1]=40;
		l_teach_pos[2]=512+50; l_teach_spd[2]=50;
		//l_teach_pos[3]=512+250; l_teach_spd[3]=50;// for 1006
		l_teach_pos[3]=512-250; l_teach_spd[3]=50;// for 1007
		l_teach_pos[4]=512+235; l_teach_spd[4]=50; 
		AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,l_teach_pos,l_teach_spd);	
		Delay_ms(3000);

		// repeat
		for(i=0;i<10;i++)
		{
			// P1			
			l_teach_pos[0]=818-357; l_teach_spd[0]=50;
			l_teach_pos[1]=818-10; l_teach_spd[1]=40;
			l_teach_pos[2]=512+50; l_teach_spd[2]=50;
			//l_teach_pos[3]=512+250; l_teach_spd[3]=50;// for 1006
			l_teach_pos[3]=512-250; l_teach_spd[3]=50;// for 1007
			l_teach_pos[4]=512+235; l_teach_spd[4]=50; 
			AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,l_teach_pos,l_teach_spd);	
			Delay_ms(900);

			// P2
			l_teach_pos[0]=818-357; l_teach_spd[0]=50;
			l_teach_pos[1]=818-90; l_teach_spd[1]=40;
			l_teach_pos[2]=512-30; l_teach_spd[2]=50;			
			//l_teach_pos[3]=512+250; l_teach_spd[3]=50;// for 1006
			l_teach_pos[3]=512-250; l_teach_spd[3]=50;// for 1007
			l_teach_pos[4]=512+235; l_teach_spd[4]=50; 
			AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,l_teach_pos,l_teach_spd);	
			Delay_ms(900);			
		}

		// P0, Home pos				
		Homing(&ARM_D);		
		Delay_ms(5000); /// exactly delay, ms*/
	}
	if(arm_index ==2) // right arm
	{		
		// P1
		r_teach_pos[0]=205+357; 	r_teach_spd[0]=50;
		r_teach_pos[1]=205+10; 		r_teach_spd[1]=40;
		r_teach_pos[2]=512-50; 		r_teach_spd[2]=50;
		//r_teach_pos[3]=512-250; r_teach_spd[3]=50;// for 1006
		r_teach_pos[3]=512+250; 	r_teach_spd[3]=50;// for 1007		
		r_teach_pos[4]=512-235; 	r_teach_spd[4]=50; 
		
		AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,r_teach_pos,r_teach_spd);	
		Delay_ms(3000);

		// repeat
		for(i=0;i<10;i++)
		{
			// P1			
			r_teach_pos[0]=205+357; 	r_teach_spd[0]=50;
			r_teach_pos[1]=205+10; 		r_teach_spd[1]=40;
			r_teach_pos[2]=512-50; 		r_teach_spd[2]=50;
			//r_teach_pos[3]=512-250; r_teach_spd[3]=50;// for 1006
			r_teach_pos[3]=512+250; 	r_teach_spd[3]=50;// for 1007		
			r_teach_pos[4]=512-235; 	r_teach_spd[4]=50; 
			
			AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,r_teach_pos,r_teach_spd);	
			Delay_ms(900);

			// P2
			r_teach_pos[0]=205+357; 	r_teach_spd[0]=50;
			r_teach_pos[1]=205+90; 		r_teach_spd[1]=40;
			r_teach_pos[2]=512+30; 		r_teach_spd[2]=50;
			//r_teach_pos[3]=512-250; r_teach_spd[3]=50;// for 1006
			r_teach_pos[3]=512+250; 	r_teach_spd[3]=50;// for 1007		
			r_teach_pos[4]=512-235; 	r_teach_spd[4]=50; 
			
			AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,r_teach_pos,r_teach_spd);	
			Delay_ms(900);			
		}

		// P0, Home pos				
		Homing(&ARM_D);		
		Delay_ms(5000); /// exactly delay, ms*/
	}

	arm->breakForLoop = 0; // release break, 06052013
}


/*Joint_Teach_DEMO_3, 08/12/2014 new*/
/* shake hand*/
void Joint_Teach_DEMO_3(uint8_t arm_index,Arm_Data *arm)
{	
	int32_t l_teach_pos[5],r_teach_pos[5];
	int32_t l_teach_spd[5],r_teach_spd[5];
	
	
	/*Touch senson control left arm*/
	if (arm_index==1) //left arm
	{
		if(arm->gb_left_touchsensor_switch_cmd == 0)
		{
			// P3
			l_teach_pos[0]=1023-305-50; 	l_teach_spd[0]=50;
			l_teach_pos[1]=1023-205; 			l_teach_spd[1]=50;
			l_teach_pos[2]=1023-512; 			l_teach_spd[2]=50;
			//l_teach_pos[3]=1023-420; 		l_teach_spd[3]=50;	
			l_teach_pos[3]=512-150; 			l_teach_spd[3]=50; // New configure for TeleRobII arm (three), 05/20/2014 added	
			l_teach_pos[4]=512; 					l_teach_spd[4]=50;	
			
			AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,l_teach_pos,l_teach_spd);								
			Delay_ms(1000); // exactly delay, ms, v2 version, 01/04/2014
		}

		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) == 1) // Left hand is touched
		{
			arm->gb_left_touchsensor_switch_cmd = arm->gb_left_touchsensor_switch_cmd + 1; // break P3
			
			AX_12_Torque_Enable(254, 0); // servo releases torque 

			Delay_ms(50); // exactly delay, ms
			/*hand pose control*/
			CAN2_RX_Hand_Pose_Ctrl(2,0,&ARM_D); // left fist		
		}
		else if((GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) != 1) && (arm->gb_left_touchsensor_switch_cmd != 0))
		{
			AX_12_Torque_Enable(254, 1); // servo generates torque
			
			/*hand pose control*/
			CAN2_RX_Hand_Pose_Ctrl(5,0,&ARM_D); // without left plam	
		}		
	}
	
	
	/*Touch senson control right arm*/
	if (arm_index==2)	// right arm
	{
		if(arm->gb_right_touchsensor_switch_cmd == 0)
		{
			// P3
			r_teach_pos[0]=305+50; 	r_teach_spd[0]=50;
			r_teach_pos[1]=205; 		r_teach_spd[1]=50;
			r_teach_pos[2]=512; 		r_teach_spd[2]=50;
			r_teach_pos[3]=512+150; r_teach_spd[3]=50; // New configure for TeleRobII arm (three), 05/20/2014 added
			r_teach_pos[4]=512; 		r_teach_spd[4]=50;	
			
			AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,r_teach_pos,r_teach_spd);				
			Delay_ms(1000); // exactly delay, ms, v2 version, 01/04/2014
		}

		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7) == 1) // Right hand is touched
		{
			arm->gb_right_touchsensor_switch_cmd = arm->gb_right_touchsensor_switch_cmd + 1; // break P3
			
			AX_12_Torque_Enable(254, 0); // servo releases torque 
			
			Delay_ms(50); // exactly delay, ms
			/*hand pose control*/
			CAN2_RX_Hand_Pose_Ctrl(4,0,&ARM_D); // right fist					
		}
		else if((GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7) != 1) && (arm->gb_right_touchsensor_switch_cmd != 0))
		{
			AX_12_Torque_Enable(254, 1); // servo generates torque 
			
			/*hand pose control*/
			CAN2_RX_Hand_Pose_Ctrl(6,0,&ARM_D); // right fist				
		}		
	}

	arm->breakForLoop = 0; // release break, 06052013
}




/*Joint_Teach_DEMO_4, 05312013*/
/* all fingers demo*/
void Joint_Teach_DEMO_4(uint8_t arm_index,Arm_Data *arm)
{
	uint8_t i;
	int32_t l_teach_pos[5],r_teach_pos[5];
	int32_t l_teach_spd[5],r_teach_spd[5];
	 	
	if(arm_index ==1) // left arm
	{
		// P2
		l_teach_pos[0]=1023-355; 	l_teach_spd[0]=50;
		l_teach_pos[1]=1023-205; 	l_teach_spd[1]=50;
		l_teach_pos[2]=1023-512; 	l_teach_spd[2]=50;
		l_teach_pos[3]=320; 			l_teach_spd[3]=50;
		l_teach_pos[4]=320; 			l_teach_spd[4]=50;	 
	   
		AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,l_teach_pos,l_teach_spd);			
		Delay(0x2affff); 

		// repeat
		for(i=1;i<38;i++)
		{
			arm->gb_hand_pose_enable = 1; arm->gb_hand_index = 1;	arm->gb_hand_pose_index = i; // 12/19/2013 update 
			Delay(0xffffff); 
		}

		arm->gb_hand_pose_enable = 0; 	arm->gb_hand_index = 0; arm->gb_hand_pose_index = 0; // 12/19/2013 update
		Delay(0xffffff);  		
		
		// P1, Home pos				
		Homing(&ARM_D);
		Delay(0xffffff); 
	}
	if(arm_index ==2) // right arm
	{
		// P2
		r_teach_pos[0]=355; r_teach_spd[0]=50;
		r_teach_pos[1]=205; r_teach_spd[1]=50;
		r_teach_pos[2]=512; r_teach_spd[2]=50;
		r_teach_pos[3]=320; r_teach_spd[3]=50;
		r_teach_pos[4]=320; r_teach_spd[4]=50;	 
	   
		AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,r_teach_pos,r_teach_spd);			
		Delay(0x2affff); 

		// repeat
		for(i=1;i<38;i++)
		{
			arm->gb_hand_pose_enable = 1; arm->gb_hand_index = 2; arm->gb_hand_pose_index = i; // 12/19/2013 update
			Delay(0xffffff);  
		}
		
	    arm->gb_hand_pose_enable = 0; arm->gb_hand_index = 0; arm->gb_hand_pose_index = 0; // 12/19/2013 update
		Delay(0xffffff);  		
		
		// P1, Home pos				
		Homing(&ARM_D);
		Delay(0xffffff); 		
	}	

	arm->breakForLoop = 0; // release break, 06052013
}


/*Joint_Teach_DEMO_5, 05312013*/
/* power*/
void Joint_Teach_DEMO_5(uint8_t arm_index,Arm_Data *arm)
{	 
	arm->breakForLoop = 0; // release break, 06052013
}


/*Joint_Teach_DEMO_6, 05312013*/
/* dancing*/
void Joint_Teach_DEMO_6(uint8_t arm_index,Arm_Data *arm)
{ 		
	arm->breakForLoop = 0; // release break, 06052013													
}






/*fixed theta3 inverse kinematics for 4dof rigid arm, by rhqi 08072013*/
uint8_t ikine_4dof_rigid_arm_theta3f(uint8_t arm_index, int32_t px, int32_t py, int32_t pz, float theta3f, uint8_t s_index, Arm_Data *arm)		
{	
	// 4dof, new added
	float theta1,theta3,theta3_1,theta3_2;
	float d,a,beta,psi; 
	float px_2,pz_2,le_;
	float temp_p1,temp_p2,temp_p3,temp_r;
	// old 2dof used
	float theta2_1,theta2_2,theta4_1,theta4_2;	
	float A,B;
	float v_temp1,v_temp2,v_temp3,v_temp4;
	float v_temp5;
	float v_temp6;
	float fi,pai_1,pai_2;
	float v_temp7_1,v_temp7_2,v_temp8_1,v_temp8_2; 	
	// turning constraint
	float theta1_max, theta2_max, theta3_max, theta4_max;
	float theta1_min, theta2_min, theta3_min, theta4_min; 
	float theta_max[4];
	float theta_min[4]; 
	float solution[2][4];
	uint8_t i,j;
	uint8_t Fsolution_check[2];	
	uint8_t exist_s; // judge solution exist
	float c3,s3,c4_1,c4_2,s4_1,s4_2; 
	float t_x,t_y,t_z,t_z_;
	float t_le,cfi,sfi,t_fi;

	float l1=170.0; 	
	float l2=210.0;	  
	float l3=0.0; // tool
	float le=l2+l3;
	float theta_error = 0.01; // important vs matlab 	
	float theta3_init_offset = 0;


	//right arm region, update 09/06/2013, latest used, test
	theta1_max = (float)(4*pi/3) + theta_error;  //max 240 degree, NOTE: RX64 may diff with ax12, update latter<=> motor 1 inital pos: 205 
	theta1_min = -(float)(pi/3) - theta_error;   //min -60 degree, NOTE: RX64 may diff with ax12, update latter<=> motor 1 inital pos: 205
	theta2_max = (float)pi + theta_error;        //max 180 degree, <=> motor 2 inital pos: 512
	theta2_min = ZERO_f - theta_error;	      	 //min 0 degree(-45 may be better, but unsafty), <=> motor 2 inital pos: 512

	// 09172013 test 2
	theta3_max = (float)(pi/3) + theta_error;  //max 150+(-90) degree, <=> motor 3 inital pos: 512
	theta3_min = -(float)(5*pi/6) - theta_error; //min -150+(-90) degree, <=> motor 3 inital pos: 512	
	theta4_max = (float)(11*pi/18) + theta_error;  //max 110 degree, <=> motor 4 inital pos: 512
	theta4_min = ZERO_f - theta_error;	       //min 0 degree, <=> motor 4 inital pos: 512

	theta_max[0]=theta1_max; theta_max[1]=theta2_max; theta_max[2]=theta3_max; theta_max[3]=theta4_max;
  theta_min[0]=theta1_min; theta_min[1]=theta2_min; theta_min[2]=theta3_min; theta_min[3]=theta4_min;  
		
	/*Initialize*/
	exist_s = 0;  
	Fsolution_check[0]=1; 
	Fsolution_check[1]=1;  
 
	/*Inverse kinematics*/
	//STEP 0: get  theta3
	theta3 = theta3f;    

 	//STEP 1: get  theta1    
	d = (float)sinf(theta3f); 
	d = d*le; //d = le*sin(theta3f);
	temp_p1 = px*px; 
	temp_p2 = py*py;
	temp_p3 = temp_p1+temp_p2;
 
	temp_r = d*d;
	temp_r = temp_p3 - temp_r; 
	if(temp_r<ZERO_f) //singularity 
	{
	   return exist_s;
	}
	
	a = (float)my_sqrt(temp_r); //a = sqrt(px*px + py*py - d*d); 
	beta = (float)atan2f(d,a); //beta = atan(d/a); //old
	psi = (float)atan2f(py,px);
	theta1 = psi-beta;   

  // when more than pi, update 08/09/2013, this should be modified ref motor region [-pi/3 4*pi/3]
	if((theta1>=-(float)pi)&&(theta1<=-(float)(2*pi/3)))
	{
	  theta1 = theta1 + (float)(2*pi);	
	}


	//STEP 2: get  theta4_1,theta4_2
	px_2 = a; //px_2 = sqrt(px_*px_ + py_*py_);
	pz_2 = pz;
	le_ = (float)cosf(theta3f); 
	le_ = le_*le;//le_ = le*cos(theta3f); 
   
	/******************************************************/
  /* the following are the same this the old 2dof ikine */
	/******************************************************/
	v_temp1 = l1+le_;
	v_temp1 = v_temp1*v_temp1; 
	v_temp2 = px_2*px_2;
	v_temp3 = pz_2*pz_2;	 
	v_temp4 = v_temp2+v_temp3;  
	A = (float)(v_temp1 - v_temp4); 	
	v_temp5 = l1-le_;
	v_temp5 = v_temp5*v_temp5;
	B = (float)(v_temp4-v_temp5);	
	if(my_abs(B)<=EPSILON) //singularity
	{
	   return exist_s;
	}
	v_temp6 = (float)(A/B);	 // attention to this, may le_ad error, 04302013
	if(v_temp6<ZERO_f) //singularity
	{
	   return exist_s;
	}		
	v_temp6 = (float)my_sqrt(v_temp6);	
	// solution 1
	theta4_1 = (float)atanf(v_temp6);
	theta4_1 = theta4_1*2; 
	// solution 2
	theta4_2 = -(float)atanf(v_temp6); 
	theta4_2 = theta4_2*2;
	
	// STEP 2: get  theta2_1,theta2_2
	fi=(float)atan2f(pz_2,px_2);
	// solution 1	 
	v_temp7_1 = (float)sinf(theta4_1);
	v_temp7_1 = v_temp7_1*le_;
	v_temp8_1 = (float)cosf(theta4_1);
    v_temp8_1 =  v_temp8_1*le_;
	v_temp8_1 = v_temp8_1 + l1;

	pai_1 = (float)atan2f(v_temp7_1,v_temp8_1);
	theta2_1 = 	fi-	pai_1;	
	// solution 2
	v_temp7_2 = (float)sinf(theta4_2);
	v_temp7_2 = v_temp7_2*le_;
	v_temp8_2= (float)cosf(theta4_2);
    v_temp8_2 =  v_temp8_2*le_;
	v_temp8_2 = v_temp8_2 + l1;

	pai_2 = (float)atan2f(v_temp7_2,v_temp8_2);
	theta2_2 = 	fi-	pai_2; 

	//STEP 3: update  theta3,theta4 for the new configure
	/*get rotation new version, 08012013*/ 
	c3=(float)cosf(theta3);c4_1=(float)cosf(theta4_1);c4_2=(float)cosf(theta4_2);
	s3=(float)sinf(theta3);s4_1=(float)sinf(theta4_1);s4_2=(float)sinf(theta4_2);
  t_y = s3;
  t_le = c3;	
	// update solution theta4_1
	t_x =  t_le*s4_1;
	t_z =  t_le*c4_1;
	t_fi = (float)atan2f(t_y,t_x);
	sfi = (float)sinf(t_fi);
	cfi = (float)cosf(t_fi);
	t_z_ = t_x*cfi + t_y*sfi;
	theta4_1 = (float)atan2f(t_z_,t_z);   
	theta3_1 = -t_fi; //update as new solution, note: negtive here	
	// update solution theta4_2
	t_x =  t_le*s4_2;
	t_z =  t_le*c4_2;
	t_fi = (float)atan2f(t_y,t_x);
	sfi = (float)sinf(t_fi);
	cfi = (float)cosf(t_fi);
	t_z_ = t_x*cfi + t_y*sfi;
	theta4_2 = (float)atan2f(t_z_,t_z);   
	theta3_2 = -t_fi; //update as new solution, note: negtive here  

 
	//all solutions 
  solution[0][0]=theta1; solution[0][1]=theta2_1; solution[0][2]=theta3_1 + theta3_init_offset; solution[0][3]=theta4_1;
	solution[1][0]=theta1; solution[1][1]=theta2_2; solution[1][2]=theta3_2 + theta3_init_offset; solution[1][3]=theta4_2;
   
	
	// check if turning over constraint and solution exist
	for(j=0;j<2;j++)
	{
	  for(i=0;i<4;i++)
		{
	    if ((solution[j][i]>theta_max[i])||(solution[j][i]<theta_min[i]))
			{
				Fsolution_check[j]=0;	
	      break;
	    }
	  }		
		if (Fsolution_check[j]==1)
		{
		   exist_s = exist_s + 1;		
		}
	}


	/* return global values*/
	switch (exist_s)
	{
		case 1: // exist one solution
			if(Fsolution_check[0]==1)
			{
				arm->theta3f_solution[0]= solution[0][0];
				arm->theta3f_solution[1]= solution[0][1];
				arm->theta3f_solution[2]= solution[0][2];
				arm->theta3f_solution[3]= solution[0][3]; 		
			}
			else
			{
				arm->theta3f_solution[0]= solution[1][0];
				arm->theta3f_solution[1]= solution[1][1];
				arm->theta3f_solution[2]= solution[1][2];
				arm->theta3f_solution[3]= solution[1][3];		
			}
			break;		
		case 2: // exist two solutions 	
			arm->theta3f_solution[0]= solution[s_index][0];
			arm->theta3f_solution[1]= solution[s_index][1];
			arm->theta3f_solution[2]= solution[s_index][2];
			arm->theta3f_solution[3]= solution[s_index][3];
			break;	
		default: 				
			break;
	}

	/*foward kinematics, and get t01 position, to slection best solution, 08072013 update*/
	arm->gb_fwd_pos[0] = l1*(float)cosf(arm->theta3f_solution[1])*(float)cosf(arm->theta3f_solution[0]); //x, in new 4dof arm coordinate, only use this to judge

	return exist_s;
}

/*hand desired angle control, update 12/05/2013*/
void hand_desired_angle_crl(uint8_t enable_angle_crl, uint8_t desired_rx_angle, Arm_Data *arm)
{ 
	/*calc hand rotation angle, 02/18/2013 new version*/ 
	arm->theta_solution[4] =  - arm->theta_solution[0] - arm->theta_solution[2]; // 02/18/2014,  general ok
	
}

/*(4dof rigid arm sulution)angles to motor postions, update 08/07/2013*/
void angle_2_motor_pos(uint8_t arm_index, float angle[5],Arm_Data *arm)
{
	int32_t right_motor_pos[5]; 
	int32_t left_motor_pos[5]; 	
	uint8_t i; 

	if (arm_index==1) // left arm
	{
		// 02/18/2014 v2, ok
		left_motor_pos[0]=arm->left_arm_init_motor_pos[0]-(int32_t)(angle[0]*(float)ANGLE_2_MOTOR_POS_RADIO); //note here
		left_motor_pos[1]=arm->left_arm_init_motor_pos[1]-(int32_t)(angle[1]*(float)ANGLE_2_MOTOR_POS_RADIO);
		left_motor_pos[2]=arm->left_arm_init_motor_pos[2]-(int32_t)(angle[2]*(float)ANGLE_2_MOTOR_POS_RADIO);	//note here	
		left_motor_pos[3]=arm->left_arm_init_motor_pos[3]-(int32_t)(angle[3]*(float)ANGLE_2_MOTOR_POS_RADIO);// New configure for TeleRobII arm (three), 05/20/2014 added	
		left_motor_pos[4]=arm->left_arm_init_motor_pos[4]-(int32_t)(angle[4]*(float)ANGLE_2_MOTOR_POS_RADIO); //11/06/2013 waiting for verify
		
		for(i=0;i<5;i++)
		{
			// move to motor max or min pos
			if(left_motor_pos[i]>=1023)
			{
			  left_motor_pos[i]=1023;
			}
			else if(left_motor_pos[i]<=0)		
			{
			  left_motor_pos[i]=0;
			} 		
		   	// Dec,v3
			arm->gb_left_m_pos[i] = left_motor_pos[i];
		}
	}

	if (arm_index==2) // right arm
	{
		right_motor_pos[0] = arm->rigid_arm_init_motor_pos[0]+(int32_t)(angle[0]*(float)ANGLE_2_MOTOR_POS_RADIO);
		right_motor_pos[1] = arm->rigid_arm_init_motor_pos[1]+(int32_t)(angle[1]*(float)ANGLE_2_MOTOR_POS_RADIO);
		right_motor_pos[2] = arm->rigid_arm_init_motor_pos[2]+(int32_t)(angle[2]*(float)ANGLE_2_MOTOR_POS_RADIO);
		//right_motor_pos[3] = arm->rigid_arm_init_motor_pos[3]-(int32_t)(angle[3]*(float)ANGLE_2_MOTOR_POS_RADIO);
		right_motor_pos[3] = arm->rigid_arm_init_motor_pos[3]+(int32_t)(angle[3]*(float)ANGLE_2_MOTOR_POS_RADIO); // New configure for TeleRobII arm (three), 05/20/2014 added
		right_motor_pos[4] = arm->rigid_arm_init_motor_pos[4]+(int32_t)(angle[4]*(float)ANGLE_2_MOTOR_POS_RADIO);//11/06/2013 waiting for verify

	  for(i=0;i<5;i++)
		{
			// move to motor max or min pos
			if(right_motor_pos[i]>=1023)
			{
			  right_motor_pos[i]=1023;
			}
			else if(right_motor_pos[i]<=0)		
			{
			  right_motor_pos[i]=0;
			}
	
		   	// Dec,v3
			arm->gb_right_m_pos[i] = right_motor_pos[i];
		}
	}

}

/* joint position syn move for 5dof rigid arm, update 1106203 */
void AX_12_Syn_Ctrl_5DOF_Rigid_Arm(uint8_t arm_index, int32_t position_in[5], int32_t speed_in[5])	//[6] is suitable muti situations
{
	int ii,j;
	
	uint8_t head1, head2, id, length, instruction, position, each_length, sub_id_sum, positionLow_sum, positionHigh_sum, speedLow_sum, speedHigh_sum, checksum;
	uint8_t ax12_command[syn_joint_ctrl_len_5dof];
	uint8_t sub_id_N[5],positionLow_N[5],positionHigh_N[5], speedLow_N[5],speedHigh_N[5];

	// initial
	sub_id_sum=0;
	positionLow_sum=0;
	positionHigh_sum=0;
	speedLow_sum=0;
	speedHigh_sum=0;
	
	// command
	head1 = 255; ax12_command[0] = head1;
	head2 = 255; ax12_command[1] = head2;			  
	id = 254;	ax12_command[2] = id;
	length = 29; ax12_command[3] = length;	// 4 motors move at the same time	
	instruction = 131; ax12_command[4] = instruction;
	position = 30; ax12_command[5] = position; 	
	each_length=4;ax12_command[6] = each_length;  	 	
	
	for(j=0;j<5;j++)
	{
	 	// transfer
		positionLow_N[j] = XDec2Low(position_in[j]);
		positionHigh_N[j] = XDec2High(position_in[j]);
		speedLow_N[j] = XDec2Low(speed_in[j]);
		speedHigh_N[j] = XDec2High(speed_in[j]);

		if (arm_index==2) // right arm
		{
			sub_id_N[j]=j+1;                   ax12_command[7+j*5]=sub_id_N[j];
			/*positionLow_N[j]=posLow[j];  */  ax12_command[8+j*5]=positionLow_N[j];
			/*positionHigh_N[j]=posHigh[j];*/  ax12_command[9+j*5]=positionHigh_N[j];
			/*speedLow_N[j]=speedLow[j];   */  ax12_command[10+j*5]=speedLow_N[j];
			/*speedHigh_N[j]=speedHigh[j]; */  ax12_command[11+j*5]=speedHigh_N[j];
		}
		if (arm_index==1) // left arm
		{
			sub_id_N[j]=j+6;                   ax12_command[7+j*5]=sub_id_N[j];
			/*positionLow_N[j]=posLow[j];  */  ax12_command[8+j*5]=positionLow_N[j];
			/*positionHigh_N[j]=posHigh[j];*/  ax12_command[9+j*5]=positionHigh_N[j];
			/*speedLow_N[j]=speedLow[j];   */  ax12_command[10+j*5]=speedLow_N[j];
			/*speedHigh_N[j]=speedHigh[j]; */  ax12_command[11+j*5]=speedHigh_N[j];
		}

		sub_id_sum=sub_id_sum + sub_id_N[j];
		positionLow_sum=positionLow_sum + positionLow_N[j];
		positionHigh_sum=positionHigh_sum + positionHigh_N[j];
		speedLow_sum=speedLow_sum + speedLow_N[j];
		speedHigh_sum=speedHigh_sum + speedHigh_N[j];
	}
	
	checksum = testNot1(id + length + instruction + position + each_length + sub_id_sum + positionLow_sum + positionHigh_sum + speedLow_sum + speedHigh_sum);

	ax12_command[32] = checksum;

	for(ii=0;ii<syn_joint_ctrl_len_5dof;ii++)
	{
		USART_SendData(USART1, ax12_command[ii]);
	 	/* Loop until the end of transmission */
	 	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); //??璣ゅ?σ521??TXE砆竚癬???誹??ЧΘ
	}	
}


void delay_t(unsigned int dl)
{
	unsigned int i,y;
	for(i = 0; i < 1000; i++)
	{
		for(y = 0; y < dl; y++);
	}
}


/*exactly count ms, 01/01/2014 update*/
void Delay_ms(uint32_t nTime)
{
	TimingDelay = nTime;
	while((TimingDelay != 0x00)&&(ARM_D.breakForLoop != 1));//new version with break 
}



/*Homing, 06052013*/
void Homing(Arm_Data *arm)
{	
	int32_t l_teach_pos[5],l_teach_spd[5];
	int32_t r_teach_pos[5],r_teach_spd[5];	 	
	
	// P0, Home pos
	l_teach_pos[0]=arm->left_arm_home_motor_pos[0]; l_teach_spd[0]=50;
	l_teach_pos[1]=arm->left_arm_home_motor_pos[1]; l_teach_spd[1]=50;
	l_teach_pos[2]=arm->left_arm_home_motor_pos[2]; l_teach_spd[2]=50;
	l_teach_pos[3]=arm->left_arm_home_motor_pos[3]; l_teach_spd[3]=50;
	l_teach_pos[4]=arm->left_arm_home_motor_pos[4]; l_teach_spd[4]=50;
  AX_12_Syn_Ctrl_5DOF_Rigid_Arm(1,l_teach_pos,l_teach_spd); 

	r_teach_pos[0]=arm->right_arm_home_motor_pos[0]; r_teach_spd[0]=50;
	r_teach_pos[1]=arm->right_arm_home_motor_pos[1]; r_teach_spd[1]=50;
	r_teach_pos[2]=arm->right_arm_home_motor_pos[2]; r_teach_spd[2]=50;
	r_teach_pos[3]=arm->right_arm_home_motor_pos[3]; r_teach_spd[3]=50;
	r_teach_pos[4]=arm->right_arm_home_motor_pos[4]; r_teach_spd[4]=50; 
	AX_12_Syn_Ctrl_5DOF_Rigid_Arm(2,r_teach_pos,r_teach_spd);		
}





/*system interput, 01/01/2014*/
// void SysTick_Handler(void)
// {
// 	//if (TimingDelay != 0x00) //original version 
// 	if ((TimingDelay != 0x00)&&(ARM_D.breakForLoop != 1)) //new version with break
// 	TimingDelay --;
// }



/* Dynamixel_Moving_Mode_Swtich*/
/* mode=0 is endless, mode=1 is joint control, 05222013*/
void AX_12_Moving_Mode_Swtich(uint8_t id_index, uint8_t mode)
{
	int ii;
	
	uint8_t head1, head2, id, length, instruction, CWAngleLimit, CWAngleLimitLow, CWAngleLimitHigh, CCWAngleLimitLow, CCWAngleLimitHigh, checksum;
	uint8_t ax12_command[ctrl_mode_swtich_len];

	head1 = 255; ax12_command[0] = head1;
	head2 = 255; ax12_command[1] = head2;			  
	id = id_index;	ax12_command[2] = id;
	length = 7; ax12_command[3] = length;
	instruction = 3; ax12_command[4] = instruction;
	CWAngleLimit = 6; ax12_command[5] = CWAngleLimit;

	// switch control mode
	if (mode==0) // Wheel endless moving
	{	
		CWAngleLimitLow = 0; ax12_command[6] = CWAngleLimitLow;
		CWAngleLimitHigh = 0; ax12_command[7] = CWAngleLimitHigh;
		CCWAngleLimitLow = 0; ax12_command[8] = CCWAngleLimitLow;
		CCWAngleLimitHigh = 0; ax12_command[9] = CCWAngleLimitHigh;	
	}
	else   // joint CWAngleLimit moving
	{
	  CWAngleLimitLow = 0; ax12_command[6] = CWAngleLimitLow;	  // important here
		CWAngleLimitHigh = 0; ax12_command[7] = CWAngleLimitHigh; // important here
		CCWAngleLimitLow = 255; ax12_command[8] = CCWAngleLimitLow;
		CCWAngleLimitHigh = 3; ax12_command[9] = CCWAngleLimitHigh;	
	}
	
	checksum = testNot1(id + length + instruction + CWAngleLimit + CWAngleLimitLow + CWAngleLimitHigh + CCWAngleLimitLow + CCWAngleLimitHigh);
	ax12_command[10] = checksum;

	for(ii=0;ii<ctrl_mode_swtich_len;ii++)
	{
		USART_SendData(USART1, ax12_command[ii]);
	 	/* Loop until the end of transmission */
	 	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); //??璣ゅ?σ521??TXE砆竚癬???誹??ЧΘ
	}	
}


/* Read AX_12 current positions, and speed, 05222013*/
//void AX_12_Read(uint8_t id_index)
uint8_t AX_12_Read(uint8_t id_index)
{
	int ii;
	
	uint8_t head1, head2, id, length, instruction, positionLow, positionHigh, speedLow, speedHigh, readLength, checksum;
	uint8_t ax12_command[read_motor_len];

	head1 = 255; ax12_command[0] = head1;
	head2 = 255; ax12_command[1] = head2;			  
	id = id_index;	ax12_command[2] = id;
	length = 7; ax12_command[3] = length;
	instruction = 2; ax12_command[4] = instruction;		
	positionLow = 36; ax12_command[5] = positionLow;
	positionHigh = 37; ax12_command[6] = positionHigh;
	speedLow = 38; ax12_command[7] = speedLow;
	speedHigh = 39; ax12_command[8] = speedHigh;	
	readLength = 4; ax12_command[9] = readLength;
	
	checksum = testNot1(id + length + instruction + positionLow + positionHigh + speedLow + speedHigh + readLength);

	ax12_command[10] = checksum;

	for(ii=0;ii<read_motor_len;ii++)
	{
		USART_SendData(USART1, ax12_command[ii]);
	 	/* Loop until the end of transmission */
	 	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); //??璣ゅ?σ521??TXE砆竚癬???誹??ЧΘ
	}	
	
	// judge have sent data here, 05222013
	return id;		
}


/* single motor or all motors torque on/off, 07/24/2014*/
void AX_12_Torque_Enable(uint8_t id_index, uint8_t torque_enable)
{
	int ii;
	
	uint8_t head1, head2, id, length, instruction, tTorque, checksum;
	uint8_t ax12_command[single_maxtorque_ctrl_len];
	
	// commond
	head1 = 255; ax12_command[0] = head1;
	head2 = 255; ax12_command[1] = head2;			  
	id = id_index;	ax12_command[2] = id;	
	length = 4; ax12_command[3] = length;	
	instruction = 3; ax12_command[4] = instruction;	
	tTorque = 24; ax12_command[5] = tTorque;	
	ax12_command[6] = torque_enable;
	
	checksum = testNot1(id + length + instruction + tTorque + torque_enable);
	ax12_command[7] = checksum;

	for(ii=0;ii<single_torque_enable_len;ii++)
	{
		USART_SendData(USART1, ax12_command[ii]);
	 	/* Loop until the end of transmission */
	 	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); //?????s?521???TXE????????????T
	}	
}


/* 5dof motors torque on/off, 07/24/2014*/
void AX_12_5DOF_Syn_Torque_Enable(uint8_t arm_index, uint8_t torque_enable)
{
	int ii,j;
	
	uint8_t head1, head2, id, length, instruction, tTorque, each_length, sub_id_sum, torque_sum, checksum;
	uint8_t ax12_command[syn_5dof_torque_enable_len];
	uint8_t sub_id_N[5],sub_torque_N[5];	
	

	// initial
	sub_id_sum=0;
	torque_sum=0;	
	
	// command
	head1 = 255; ax12_command[0] = head1;
	head2 = 255; ax12_command[1] = head2;			  
	id = 254;	ax12_command[2] = id;
	length = 14; ax12_command[3] = length;	
	instruction = 131; ax12_command[4] = instruction;
	tTorque = 24; ax12_command[5] = tTorque; 	
	each_length=1;ax12_command[6] = each_length;  	 	
	
	for(j=0;j<5;j++)
	{
		sub_torque_N[j] = 0;
		
		if (arm_index==2) // right arm
		{
			sub_id_N[j]=j+1;  ax12_command[7+j*5]=sub_id_N[j];
												ax12_command[8+j*5]=sub_torque_N[j];
		}
		if (arm_index==1) // left arm
		{
			sub_id_N[j]=j+6;  ax12_command[7+j*5]=sub_id_N[j];
												ax12_command[8+j*5]=sub_torque_N[j];
		}

		sub_id_sum=sub_id_sum + sub_id_N[j];
		torque_sum=torque_sum + sub_torque_N[j];
	}
	
	checksum = testNot1(id + length + instruction + tTorque + each_length + sub_id_sum + torque_sum);

	ax12_command[29] = checksum;

	for(ii=0;ii<syn_5dof_torque_enable_len;ii++)
	{
		USART_SendData(USART1, ax12_command[ii]);
	 	/* Loop until the end of transmission */
	 	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); //?????s?521???TXE????????????T
	}	
}




/* single motor or all motors torque, must be mofied when swtiches from turnless to joint mode, 05272013*/
void AX_12_MaxTorque_Ctrl(uint8_t id_index, int max_torque)
{
	int ii;
	
	uint8_t head1, head2, id, length, instruction, maxTorque, maxTorqueLow, maxTorqueHigh, checksum;
	uint8_t ax12_command[single_maxtorque_ctrl_len];
	
	// transfer
	maxTorqueLow = XDec2Low(max_torque); // 0-1023 
	maxTorqueHigh = XDec2High(max_torque);  // 0-1023 
	
	// commond
	head1 = 255; ax12_command[0] = head1;
	head2 = 255; ax12_command[1] = head2;			  
	id = id_index;	ax12_command[2] = id;	
	length = 5; ax12_command[3] = length;	
	instruction = 3; ax12_command[4] = instruction;	
	maxTorque = 34; ax12_command[5] = maxTorque;
	ax12_command[6] = maxTorqueLow;
	ax12_command[7] = maxTorqueHigh;
	
	
	checksum = testNot1(id + length + instruction + maxTorque + maxTorqueLow + maxTorqueHigh);
	ax12_command[8] = checksum;

	for(ii=0;ii<single_maxtorque_ctrl_len;ii++)
	{
		USART_SendData(USART1, ax12_command[ii]);
	 	/* Loop until the end of transmission */
	 	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); //??璣ゅ?σ521??TXE砆竚癬???誹??ЧΘ
	}	
}

/* single motor or all motors move(input position and fixed speed), 05222013*/
void AX_12_Ctrl(uint8_t id_index,uint8_t posLow, uint8_t posHigh)
{
	int ii;
	
	uint8_t head1, head2, id, length, instruction, position, positionLow, positionHigh, speedLow, speedHigh, checksum;
	uint8_t ax12_command[single_joint_ctrl_len];

	head1 = 255; 							ax12_command[0] = head1;
	head2 = 255; 							ax12_command[1] = head2;			  
	id = id_index;						ax12_command[2] = id;
	length = 7; 							ax12_command[3] = length;
	instruction = 3; 					ax12_command[4] = instruction;
	position = 30; 						ax12_command[5] = position;
	positionLow = posLow; 		ax12_command[6] = positionLow;
	positionHigh = posHigh; 	ax12_command[7] = positionHigh;
	speedLow = 30; 						ax12_command[8] = speedLow;
	speedHigh = 0; 						ax12_command[9] = speedHigh;
	
	checksum = testNot1(id + length + instruction + position + positionLow + positionHigh + speedLow + speedHigh);

	ax12_command[10] = checksum;

	for(ii=0;ii<single_joint_ctrl_len;ii++)
	{
		USART_SendData(USART1, ax12_command[ii]);
	 	/* Loop until the end of transmission */
	 	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); //??璣ゅ?σ521??TXE砆竚癬???誹??ЧΘ
	}	
}


/* single motor or all motors move(input position and speed), 05222013*/
void AX_12_Ctrl_Pos_and_Speed(uint8_t id_index, int position_in, int speed_in)
{
	int ii;
	
	uint8_t head1, head2, id, length, instruction, position, positionLow, positionHigh, speedLow, speedHigh, checksum;
	uint8_t ax12_command[single_joint_ctrl_len];

	// transfer
	positionLow = XDec2Low(position_in);
	positionHigh = XDec2High(position_in);
	speedLow = XDec2Low(speed_in);
	speedHigh = XDec2High(speed_in);
	
	// command
	head1 = 255; ax12_command[0] = head1;
	head2 = 255; ax12_command[1] = head2;			  
	id = id_index;	ax12_command[2] = id;
	length = 7; ax12_command[3] = length;
	instruction = 3; ax12_command[4] = instruction;
	position = 30; ax12_command[5] = position;
	positionLow = positionLow; ax12_command[6] = positionLow;
	positionHigh = positionHigh; ax12_command[7] = positionHigh;
	speedLow = speedLow; ax12_command[8] = speedLow;
	speedHigh = speedHigh; ax12_command[9] = speedHigh;	
	
	
	checksum = testNot1(id + length + instruction + position + positionLow + positionHigh + speedLow + speedHigh);

	ax12_command[10] = checksum;

	for(ii=0;ii<single_joint_ctrl_len;ii++)
	{
		USART_SendData(USART1, ax12_command[ii]);
	 	/* Loop until the end of transmission */
	 	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); //??璣ゅ?σ521??TXE砆竚癬???誹??ЧΘ
	}	
}

/* joint position syn move */
void AX_12_Syn_Ctrl(uint8_t arm_index, int32_t position_in[6], int32_t speed_in[6])
{
	int ii,j;
	
	uint8_t head1, head2, id, length, instruction, position, each_length, sub_id_sum, positionLow_sum, positionHigh_sum, speedLow_sum, speedHigh_sum, checksum;
	uint8_t ax12_command[syn_joint_ctrl_len];
	uint8_t sub_id_N[6],positionLow_N[6],positionHigh_N[6], speedLow_N[6],speedHigh_N[6];

	// initial
	sub_id_sum=0;
	positionLow_sum=0;
	positionHigh_sum=0;
	speedLow_sum=0;
	speedHigh_sum=0;
	
	// command
	head1 = 255; ax12_command[0] = head1;
	head2 = 255; ax12_command[1] = head2;			  
	id = 254;	ax12_command[2] = id;
	length = 34; ax12_command[3] = length;	// 6 motors move at the same time	
	instruction = 131; ax12_command[4] = instruction;
	position = 30; ax12_command[5] = position; 	
	each_length=4;ax12_command[6] = each_length;  	 	
	
	for(j=0;j<6;j++)
	{
	 	// transfer
		positionLow_N[j] = XDec2Low(position_in[j]);
		positionHigh_N[j] = XDec2High(position_in[j]);
		speedLow_N[j] = XDec2Low(speed_in[j]);
		speedHigh_N[j] = XDec2High(speed_in[j]);

		if (arm_index==2) // right arm
		{
			sub_id_N[j]=j+1;                   ax12_command[7+j*5]=sub_id_N[j];
			/*positionLow_N[j]=posLow[j];  */  ax12_command[8+j*5]=positionLow_N[j];
			/*positionHigh_N[j]=posHigh[j];*/  ax12_command[9+j*5]=positionHigh_N[j];
			/*speedLow_N[j]=speedLow[j];   */  ax12_command[10+j*5]=speedLow_N[j];
			/*speedHigh_N[j]=speedHigh[j]; */  ax12_command[11+j*5]=speedHigh_N[j];
		}
		if (arm_index==1) // left arm
		{
			sub_id_N[j]=j+7;                   ax12_command[7+j*5]=sub_id_N[j];
			/*positionLow_N[j]=posLow[j];  */  ax12_command[8+j*5]=positionLow_N[j];
			/*positionHigh_N[j]=posHigh[j];*/  ax12_command[9+j*5]=positionHigh_N[j];
			/*speedLow_N[j]=speedLow[j];   */  ax12_command[10+j*5]=speedLow_N[j];
			/*speedHigh_N[j]=speedHigh[j]; */  ax12_command[11+j*5]=speedHigh_N[j];
		}

		sub_id_sum=sub_id_sum + sub_id_N[j];
		positionLow_sum=positionLow_sum + positionLow_N[j];
		positionHigh_sum=positionHigh_sum + positionHigh_N[j];
		speedLow_sum=speedLow_sum + speedLow_N[j];
		speedHigh_sum=speedHigh_sum + speedHigh_N[j];
	}
	
	checksum = testNot1(id + length + instruction + position + each_length + sub_id_sum + positionLow_sum + positionHigh_sum + speedLow_sum + speedHigh_sum);

	ax12_command[37] = checksum;

	for(ii=0;ii<syn_joint_ctrl_len;ii++)
	{
		USART_SendData(USART1, ax12_command[ii]);
	 	/* Loop until the end of transmission */
	 	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); //??璣ゅ?σ521??TXE砆竚癬???誹??ЧΘ
	}	
} 




/* clear all AX_12 errors, 06052013*/
void AX_12_CLEAR_ERROR(uint8_t id_index)
{
	int ii;
	
	uint8_t head1, head2, id, length, instruction, alarmShutdown, clearErrorCmd, checksum;
	uint8_t ax12_command[clear_ax12_error_len];
	
	// commond
	head1 = 255; 					ax12_command[0] = head1;
	head2 = 255; 					ax12_command[1] = head2;			  
	id = id_index;				ax12_command[2] = id;	
	length = 4; 					ax12_command[3] = length;	
	instruction = 3;			ax12_command[4] = instruction;	
	alarmShutdown = 18; 	ax12_command[5] = alarmShutdown;
	clearErrorCmd = 127; 	ax12_command[6] = clearErrorCmd;	// clear all motor and all errors 	
	
	checksum = testNot1(id + length + instruction + alarmShutdown + clearErrorCmd);
	ax12_command[7] = checksum;

	for(ii=0;ii<clear_ax12_error_len;ii++)
	{
		USART_SendData(USART1, ax12_command[ii]);
	 	/* Loop until the end of transmission */
	 	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); //??璣ゅ?σ521??TXE砆竚癬???誹??ЧΘ
	}	
}

/* AX_12 single motor or all motors speed control mode, 05222013*/
void AX_12_Speed_Ctrl(uint8_t id_index, int goal_speed_in)
{
	int ii;
	
	uint8_t head1, head2, id, length, instruction, goalspeed, goalspeedLow, goalspeedHigh, checksum;
	uint8_t ax12_command[single_speed_ctrl_len];
	
	// transfer
	goalspeedLow = XDec2Low(goal_speed_in); // 0-1023 is CCW(0 stop), 1024-2047 is CW(1024 stop)
	goalspeedHigh = XDec2High(goal_speed_in);  // 0-1023 is CCW(0 stop), 1024-2047 is CW(1024 stop)	
	
	// commond
	head1 = 255; ax12_command[0] = head1;
	head2 = 255; ax12_command[1] = head2;			  
	id = id_index;	ax12_command[2] = id;	
	length = 5; ax12_command[3] = length;	
	instruction = 3; ax12_command[4] = instruction;	
	goalspeed = 32; ax12_command[5] = goalspeed;
	ax12_command[6] = goalspeedLow;
	ax12_command[7] = goalspeedHigh;
	
	
	checksum = testNot1(id + length + instruction + goalspeed + goalspeedLow + goalspeedHigh);
	ax12_command[8] = checksum;

	for(ii=0;ii<single_speed_ctrl_len;ii++)
	{
		USART_SendData(USART1, ax12_command[ii]);
	 	/* Loop until the end of transmission */
	 	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); //??璣ゅ?σ521??TXE砆竚癬???誹??ЧΘ
	}	
}


/* syn wheel turnless speed move, 05222013*/
void AX_12_Speed_Syn_Ctrl(unsigned char arm_index, int speed_in[6])
{
	int ii,j;
	
	uint8_t head1, head2, id, length, instruction, goalspeed, each_length, sub_id_sum, speedLow_sum, speedHigh_sum, checksum;
	uint8_t ax12_command[syn_speed_ctrl_len];
	uint8_t sub_id_N[6],speedLow_N[6],speedHigh_N[6];

	// initial
	sub_id_sum=0;
	speedLow_sum=0;
	speedHigh_sum=0;

	head1 = 255; ax12_command[0] = head1;
	head2 = 255; ax12_command[1] = head2;			  
	id = 254;	ax12_command[2] = id;	
	length = 22; ax12_command[3] = length;	// 6 motors move at the same time	
	instruction = 131; ax12_command[4] = instruction;
	goalspeed = 32; ax12_command[5] = goalspeed; 	
	each_length=2;ax12_command[6] = each_length;  	 	
	
	for(j=0;j<6;j++)
	{
		// transfer	
		speedLow_N[j] = XDec2Low(speed_in[j]);
		speedHigh_N[j] = XDec2High(speed_in[j]);

		if (arm_index==2) // right arm
		{
			sub_id_N[j]=j+1;                   ax12_command[7+j*5]=sub_id_N[j];
			/*speedLow_N[j]=speedLow[j];   */  ax12_command[8+j*5]=speedLow_N[j];
			/*speedHigh_N[j]=speedHigh[j]; */  ax12_command[9+j*5]=speedHigh_N[j];
		}
		if (arm_index==1)  // left arm
		{
			sub_id_N[j]=j+7;                   ax12_command[7+j*5]=sub_id_N[j];
			/*speedLow_N[j]=speedLow[j];   */  ax12_command[8+j*5]=speedLow_N[j];
			/*speedHigh_N[j]=speedHigh[j]; */  ax12_command[9+j*5]=speedHigh_N[j];
		}
		 
		sub_id_sum=sub_id_sum + sub_id_N[j];
		speedLow_sum=speedLow_sum + speedLow_N[j];
		speedHigh_sum=speedHigh_sum + speedHigh_N[j];
	}
	
	checksum = testNot1(id + length + instruction + goalspeed + each_length + sub_id_sum +  speedLow_sum + speedHigh_sum);

	ax12_command[25] = checksum;

	for(ii=0;ii<syn_speed_ctrl_len;ii++)
	{
		USART_SendData(USART1, ax12_command[ii]);
	 	/* Loop until the end of transmission */
	 	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); //??璣ゅ?σ521??TXE砆竚癬???誹??ЧΘ
	}	
}







 /*Initial postion, 06052013*/
void InitialPosTest(Arm_Data *arm)
{	
	int32_t l_teach_pos[5],l_teach_spd[5];
	int32_t r_teach_pos[5],r_teach_spd[5];	 	
	
	// P0, Home pos
	l_teach_pos[0]=arm->left_arm_init_motor_pos[0]; l_teach_spd[0]=50;
	l_teach_pos[1]=arm->left_arm_init_motor_pos[1]; l_teach_spd[1]=50;
	l_teach_pos[2]=arm->left_arm_init_motor_pos[2]; l_teach_spd[2]=50;
	l_teach_pos[3]=arm->left_arm_init_motor_pos[3]; l_teach_spd[3]=50;
	l_teach_pos[4]=arm->left_arm_init_motor_pos[4]; l_teach_spd[4]=50;
    AX_12_Syn_Ctrl(0,l_teach_pos,l_teach_spd); 

	r_teach_pos[0]=arm->rigid_arm_init_motor_pos[0]; r_teach_spd[0]=50;
	r_teach_pos[1]=arm->rigid_arm_init_motor_pos[1]; r_teach_spd[1]=50;
	r_teach_pos[2]=arm->rigid_arm_init_motor_pos[2]; r_teach_spd[2]=50;
	r_teach_pos[3]=arm->rigid_arm_init_motor_pos[3]; r_teach_spd[3]=50; 
	r_teach_pos[4]=arm->rigid_arm_init_motor_pos[4]; r_teach_spd[4]=50; 
	AX_12_Syn_Ctrl(1,r_teach_pos,r_teach_spd);		
}

/*Initial postion, 06052013*/
void InitialPos(Arm_Data *arm)
{	
	int32_t l_teach_pos[5],l_teach_spd[5];
	int32_t r_teach_pos[5],r_teach_spd[5];	 	
	
	// P0, Home pos
	l_teach_pos[0]=arm->left_arm_init_motor_pos[0]; l_teach_spd[0]=50;
	l_teach_pos[1]=arm->left_arm_init_motor_pos[1]; l_teach_spd[1]=50;
	l_teach_pos[2]=arm->left_arm_init_motor_pos[2]; l_teach_spd[2]=50;
	l_teach_pos[3]=arm->left_arm_init_motor_pos[3]; l_teach_spd[3]=50;
	l_teach_pos[4]=arm->left_arm_init_motor_pos[4]; l_teach_spd[4]=50;
  AX_12_Syn_Ctrl_5DOF_Rigid_Arm(1,l_teach_pos,l_teach_spd); 

	r_teach_pos[0]=arm->rigid_arm_init_motor_pos[0]; r_teach_spd[0]=50;
	r_teach_pos[1]=arm->rigid_arm_init_motor_pos[1]; r_teach_spd[1]=50;
	r_teach_pos[2]=arm->rigid_arm_init_motor_pos[2]; r_teach_spd[2]=50;
	r_teach_pos[3]=arm->rigid_arm_init_motor_pos[3]; r_teach_spd[3]=50;
	r_teach_pos[4]=arm->rigid_arm_init_motor_pos[4]; r_teach_spd[4]=50; 
	AX_12_Syn_Ctrl_5DOF_Rigid_Arm(2,r_teach_pos,r_teach_spd);		
}


/*Homing, 06052013*/
void SingleArmHoming(uint8_t arm_index,Arm_Data *arm)
{	
	int32_t l_teach_pos[5],l_teach_spd[5];
	int32_t r_teach_pos[5],r_teach_spd[5];	 	
	
	// left arm homing
	if(arm_index == 1)
	{
		// P0, Home pos
		l_teach_pos[0]=arm->left_arm_home_motor_pos[0]; l_teach_spd[0]=50;
		l_teach_pos[1]=arm->left_arm_home_motor_pos[1]; l_teach_spd[1]=50;
		l_teach_pos[2]=arm->left_arm_home_motor_pos[2]; l_teach_spd[2]=50;
		l_teach_pos[3]=arm->left_arm_home_motor_pos[3]; l_teach_spd[3]=50;
		l_teach_pos[4]=arm->left_arm_home_motor_pos[4]; l_teach_spd[4]=50;
		AX_12_Syn_Ctrl_5DOF_Rigid_Arm(1,l_teach_pos,l_teach_spd);	
	}	
	
	// right arm homing
	if(arm_index == 2)
	{	
		// P0, Home pos
		r_teach_pos[0]=arm->right_arm_home_motor_pos[0]; r_teach_spd[0]=50;
		r_teach_pos[1]=arm->right_arm_home_motor_pos[1]; r_teach_spd[1]=50;
		r_teach_pos[2]=arm->right_arm_home_motor_pos[2]; r_teach_spd[2]=50;
		r_teach_pos[3]=arm->right_arm_home_motor_pos[3]; r_teach_spd[3]=50;
		r_teach_pos[4]=arm->right_arm_home_motor_pos[4]; r_teach_spd[4]=50; 
		AX_12_Syn_Ctrl_5DOF_Rigid_Arm(2,r_teach_pos,r_teach_spd);			
	}	
}


/*Homing, 06052013*/
void HomingFast(Arm_Data *arm)
{	
	int32_t l_teach_pos[5],l_teach_spd[5];
	int32_t r_teach_pos[5],r_teach_spd[5];	 	
	
	// P0, Home pos
	l_teach_pos[0]=arm->left_arm_home_motor_pos[0]; l_teach_spd[0]=100;
	l_teach_pos[1]=arm->left_arm_home_motor_pos[1]; l_teach_spd[1]=100;
	l_teach_pos[2]=arm->left_arm_home_motor_pos[2]; l_teach_spd[2]=100;
	l_teach_pos[3]=arm->left_arm_home_motor_pos[3]; l_teach_spd[3]=100;
	l_teach_pos[4]=arm->left_arm_home_motor_pos[4]; l_teach_spd[4]=100;
    AX_12_Syn_Ctrl_5DOF_Rigid_Arm(1,l_teach_pos,l_teach_spd); 

	r_teach_pos[0]=arm->right_arm_home_motor_pos[0]; r_teach_spd[0]=100;
	r_teach_pos[1]=arm->right_arm_home_motor_pos[1]; r_teach_spd[1]=100;
	r_teach_pos[2]=arm->right_arm_home_motor_pos[2]; r_teach_spd[2]=100;
	r_teach_pos[3]=arm->right_arm_home_motor_pos[3]; r_teach_spd[3]=100;
	r_teach_pos[4]=arm->right_arm_home_motor_pos[4]; r_teach_spd[4]=100; 
	AX_12_Syn_Ctrl_5DOF_Rigid_Arm(2,r_teach_pos,r_teach_spd);		
}





/*infltableHandMotion, 11/06/2013*/
void infltableFingerMotionCtrl(uint8_t hand_index, uint8_t finger_index, int8_t motion_direction,Arm_Data *arm)
{ 

	/*PID pump control*/
   if (hand_index==1) //left hand
   {
   		/* commands*/
		switch (finger_index)
		{
			case 1:       // 左大拇指
				if (motion_direction==1) // fist 
				{
					GPIO_SetBits(GPIOG, GPIO_Pin_2);  //左大拇指气泵 ON
					GPIO_SetBits(GPIOG, GPIO_Pin_5);  //左大拇指气阀 ON
					left_hand_pump_pwm_duty[finger_index-1] = pid_update(arm->left_fist_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else if (motion_direction==-1) //thumb
				{
					GPIO_SetBits(GPIOG, GPIO_Pin_2);    //左大拇指气泵 ON
					GPIO_ResetBits(GPIOG, GPIO_Pin_5);  //左大拇指气阀 OFF 
					left_hand_pump_pwm_duty[finger_index-1] = pid_update(arm->left_plam_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else // without 
				{
					GPIO_ResetBits(GPIOG, GPIO_Pin_2);  //左大拇指气泵 OFF
					GPIO_ResetBits(GPIOG, GPIO_Pin_5);  //左大拇指气阀 OFF	
					left_hand_pump_pwm_duty[finger_index-1]=0;// stop 	 				
				}
				break;
			case 2:	      // 左食指	
				if (motion_direction==1) // fist 
				{
					GPIO_SetBits(GPIOG, GPIO_Pin_15);  //左食指气泵	ON
					GPIO_SetBits(GPIOB, GPIO_Pin_5);  //左食指气阀	ON
					left_hand_pump_pwm_duty[finger_index-1] = pid_update(arm->left_fist_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else if (motion_direction==-1) //thumb
				{
					GPIO_SetBits(GPIOG, GPIO_Pin_15);  //左食指气泵	ON
					GPIO_ResetBits(GPIOB, GPIO_Pin_5);  //左食指气阀 OFF   
					left_hand_pump_pwm_duty[finger_index-1] = pid_update(arm->left_plam_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else // without 
				{
					GPIO_ResetBits(GPIOG, GPIO_Pin_15);  //左食指气泵	ON
					GPIO_ResetBits(GPIOB, GPIO_Pin_5);  //左食指气阀 OFF  	
					left_hand_pump_pwm_duty[finger_index-1]=0;// stop 				
				}
				break;
			case 3: 	  // 左中指
				if (motion_direction==1) // fist 
				{
					GPIO_SetBits(GPIOE, GPIO_Pin_0);  //左中指气泵	ON
					GPIO_SetBits(GPIOC, GPIO_Pin_13);  //左中指气阀	ON
					left_hand_pump_pwm_duty[finger_index-1] = pid_update(arm->left_fist_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
					//left_hand_pump_pwm_duty[finger_index-1] = pid_update(left_fist_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); // 01/24/2014 test
				}
				else if (motion_direction==-1) //thumb
				{
					GPIO_SetBits(GPIOE, GPIO_Pin_0);  //左中指气泵	ON
					GPIO_ResetBits(GPIOC, GPIO_Pin_13);  //左中指气阀	OFF 
					left_hand_pump_pwm_duty[finger_index-1] = pid_update(arm->left_plam_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else // without 
				{
					GPIO_ResetBits(GPIOE, GPIO_Pin_0);  //左中指气泵	OFF
					GPIO_ResetBits(GPIOC, GPIO_Pin_13);  //左中指气阀	OFF 	
					left_hand_pump_pwm_duty[finger_index-1]=0;// stop 				
				}
				break;
			case 4: 	  // 左无名指
				if (motion_direction==1) // fist 
				{
					GPIO_SetBits(GPIOF, GPIO_Pin_0);  //左无名指气泵	ON
					GPIO_SetBits(GPIOF, GPIO_Pin_3);  //左无名指气阀	ON
					left_hand_pump_pwm_duty[finger_index-1] = pid_update(arm->left_fist_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else if (motion_direction==-1) //thumb
				{
					GPIO_SetBits(GPIOF, GPIO_Pin_0);  //左无名指气泵	ON
					GPIO_ResetBits(GPIOF, GPIO_Pin_3);  //左无名指气阀	OFF
					left_hand_pump_pwm_duty[finger_index-1] = pid_update(arm->left_plam_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else // without 
				{
					GPIO_ResetBits(GPIOF, GPIO_Pin_0);  //左无名指气泵	ON
					GPIO_ResetBits(GPIOF, GPIO_Pin_3);  //左无名指气阀	OFF	
					left_hand_pump_pwm_duty[finger_index-1]=0;// stop 				
				}		
				break;
			case 5: 	  // 左小指	
				if (motion_direction==1) // fist 
				{
					GPIO_SetBits(GPIOF, GPIO_Pin_11);  //左小指气泵	ON
					GPIO_SetBits(GPIOF, GPIO_Pin_14);  //左小指气阀	ON
					left_hand_pump_pwm_duty[finger_index-1] = pid_update(arm->left_fist_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else if (motion_direction==-1) //thumb
				{
					GPIO_SetBits(GPIOF, GPIO_Pin_11);  //左小指气泵	ON
					GPIO_ResetBits(GPIOF, GPIO_Pin_14);  //左小指气阀	OFF
					left_hand_pump_pwm_duty[finger_index-1] = pid_update(arm->left_plam_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else // without 
				{
					GPIO_ResetBits(GPIOF, GPIO_Pin_11);  //左小指气泵	OFF
					GPIO_ResetBits(GPIOF, GPIO_Pin_14);  //左小指气阀	OFF
					left_hand_pump_pwm_duty[finger_index-1]=0;// stop 				
				}
				break; 			
			default: 				
				break;
		}	   	   
   }
   if (hand_index==2)	// right hand
   {
		/* commands*/
		switch (finger_index)
		{
			case 1:       // 右大拇指
				if (motion_direction==1) // fist 
				{
					GPIO_SetBits(GPIOG, GPIO_Pin_3); //右大拇指气泵 ON
					GPIO_SetBits(GPIOG, GPIO_Pin_4);  //右大拇指气阀 ON
					right_hand_pump_pwm_duty[finger_index-1] = pid_update(arm->right_fist_desired_press[finger_index-1],right_hand_actual_AD_value[finger_index-1], 100,&pid_per); //right thumb
				}
				else if (motion_direction==-1) //thumb
				{
					GPIO_SetBits(GPIOG, GPIO_Pin_3); //右大拇指气泵 ON
					GPIO_ResetBits(GPIOG, GPIO_Pin_4);  //右大拇指气阀 OFF 
					right_hand_pump_pwm_duty[finger_index-1] = pid_update(arm->right_plam_desired_press[finger_index-1],right_hand_actual_AD_value[finger_index-1], 100,&pid_per); //right thumb
				}
				else // without 
				{
					GPIO_ResetBits(GPIOG, GPIO_Pin_3); //右大拇指气泵 OFF
					GPIO_ResetBits(GPIOG, GPIO_Pin_4);  //右大拇指气阀 OFF 						
					right_hand_pump_pwm_duty[finger_index-1]=0;// stop 				
				}
				break;
			case 2:	      // 右食指
				if (motion_direction==1) // fist 
				{
					GPIO_SetBits(GPIOB, GPIO_Pin_3); //右食指气泵 ON
					GPIO_SetBits(GPIOB, GPIO_Pin_4);  //右食指气阀 ON
					right_hand_pump_pwm_duty[finger_index-1] = pid_update(arm->right_fist_desired_press[finger_index-1],right_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else if (motion_direction==-1) //thumb
				{
					GPIO_SetBits(GPIOB, GPIO_Pin_3); //右食指气泵 ON
					GPIO_ResetBits(GPIOB, GPIO_Pin_4);  //右食指气阀 OFF
					right_hand_pump_pwm_duty[finger_index-1] = pid_update(arm->right_plam_desired_press[finger_index-1],right_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else // without 
				{
					GPIO_ResetBits(GPIOB, GPIO_Pin_3); //右食指气泵 OFF
					GPIO_ResetBits(GPIOB, GPIO_Pin_4);  //右食指气阀 OFF					
					right_hand_pump_pwm_duty[finger_index-1]=0;// stop 				
				} 
				break;
			case 3: 	  // 右中指
				if (motion_direction==1) // fist 
				{
					GPIO_SetBits(GPIOE, GPIO_Pin_1); //右中指气泵 ON
					GPIO_SetBits(GPIOE, GPIO_Pin_6);  //右中指气阀 ON
					right_hand_pump_pwm_duty[finger_index-1] = pid_update(arm->right_fist_desired_press[finger_index-1],right_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else if (motion_direction==-1) //thumb
				{
					GPIO_SetBits(GPIOE, GPIO_Pin_1); 		//右中指气泵 ON
					GPIO_ResetBits(GPIOE, GPIO_Pin_6);  //右中指气阀 OFF
					right_hand_pump_pwm_duty[finger_index-1] = pid_update(arm->right_plam_desired_press[finger_index-1],right_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else // without 
				{
					GPIO_ResetBits(GPIOE, GPIO_Pin_1); //右中指气泵 OFF
					GPIO_ResetBits(GPIOE, GPIO_Pin_6);  //右中指气阀 OFF					
					right_hand_pump_pwm_duty[finger_index-1]=0;// stop 				
				}
				break;
			case 4: 	  // 右无名指
				if (motion_direction==1) // fist 
				{
					GPIO_SetBits(GPIOF, GPIO_Pin_1); //右无名指气泵 ON
					GPIO_SetBits(GPIOF, GPIO_Pin_2);  //右无名指气阀 ON
					right_hand_pump_pwm_duty[finger_index-1] = pid_update(arm->right_fist_desired_press[finger_index-1],right_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else if (motion_direction==-1) //thumb
				{
					GPIO_SetBits(GPIOF, GPIO_Pin_1); //右无名指气泵 ON
					GPIO_ResetBits(GPIOF, GPIO_Pin_2);  //右无名指气阀 OFF
					right_hand_pump_pwm_duty[finger_index-1] = pid_update(arm->right_plam_desired_press[finger_index-1],right_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else // without 
				{
					GPIO_ResetBits(GPIOF, GPIO_Pin_1); //右无名指气泵 OFF
					GPIO_ResetBits(GPIOF, GPIO_Pin_2);  //右无名指气阀 OFF					
					right_hand_pump_pwm_duty[finger_index-1]=0;// stop 				
				}		
				break;
			case 5: 	  // 右小指	
				if (motion_direction==1) // fist 
				{
					GPIO_SetBits(GPIOF, GPIO_Pin_12); //右小指气泵 ON
					GPIO_SetBits(GPIOF, GPIO_Pin_13);  //右小指气阀 ON
					right_hand_pump_pwm_duty[finger_index-1] = pid_update(arm->right_fist_desired_press[finger_index-1],right_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else if (motion_direction==-1) //thumb
				{
					GPIO_SetBits(GPIOF, GPIO_Pin_12); //右小指气泵 ON
					GPIO_ResetBits(GPIOF, GPIO_Pin_13);  //右小指气阀 OFF
					right_hand_pump_pwm_duty[finger_index-1] = pid_update(arm->right_plam_desired_press[finger_index-1],right_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else // without 
				{
					GPIO_ResetBits(GPIOF, GPIO_Pin_12); //右小指气泵 OFF
					GPIO_ResetBits(GPIOF, GPIO_Pin_13);  //右小指气阀 OFF				
					right_hand_pump_pwm_duty[finger_index-1]=0;// stop 				
				}
				break; 			
			default: 				
				break;
		}
   }
}


/*hand pose, 12/18/2013*/
void infltableHandPose(uint8_t pneumatic_enable, uint8_t hand_index, uint8_t pose_index)
{
	/*PID pump control*/
	if (pneumatic_enable==1) 
	{	  		
		/* commands */
		switch (pose_index)   
		{
			case 1: // plam
				infltableFingerMotionCtrl(hand_index, 1, -1, &ARM_D); //12/18/2103, ok			
				infltableFingerMotionCtrl(hand_index, 2, -1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1, &ARM_D); //12/18/2103, ok				
				break; 
			case 2: // only thumb, but not control others
				infltableFingerMotionCtrl(hand_index, 1, 1, &ARM_D); //12/18/2103, ok
				break;	
			case 3: // only forefinger, but not control others			
				infltableFingerMotionCtrl(hand_index, 2, 1, &ARM_D); //12/18/2103, ok				
				break;	
			case 4: // only middle finger, but not control others
				infltableFingerMotionCtrl(hand_index, 3, 1, &ARM_D); //12/18/2103, ok
				break;	
			case 5: // only ring finger, but not control others
				infltableFingerMotionCtrl(hand_index, 4, 1, &ARM_D); //12/18/2103, ok
				break;
			case 6: // only ring finger, but not control others
				infltableFingerMotionCtrl(hand_index, 5, 1, &ARM_D); //12/18/2103, ok
				break; 
			case 7: // only thumb
				infltableFingerMotionCtrl(hand_index, 1, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1,&ARM_D); //12/18/2103, ok
				break;	
			case 8: // only forefinger
				infltableFingerMotionCtrl(hand_index, 1, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1,&ARM_D); //12/18/2103, ok
				break;	
			case 9: // only middle finger
				infltableFingerMotionCtrl(hand_index, 1, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1,&ARM_D); //12/18/2103, ok
				break;	
			case 10: // only ring finger
				infltableFingerMotionCtrl(hand_index, 1, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1,&ARM_D); //12/18/2103, ok
				break;
			case 11: // only ring finger
				infltableFingerMotionCtrl(hand_index, 1, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1, &ARM_D); //12/18/2103, ok
				break;
			case 12: // finger 1,2 enable, means "ok"
				infltableFingerMotionCtrl(hand_index, 1, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1,&ARM_D); //12/18/2103, ok
				break;
			case 13: // finger 1,3 enable
				infltableFingerMotionCtrl(hand_index, 1, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1,&ARM_D); //12/18/2103, ok
				break;
			case 14: // finger 1,4 enable
				infltableFingerMotionCtrl(hand_index, 1, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1,&ARM_D); //12/18/2103, ok
				break;
			case 15: // finger 1,5 enable
				infltableFingerMotionCtrl(hand_index, 1, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1, &ARM_D); //12/18/2103, ok
				break;
			case 16: // finger 2,3 enable
				infltableFingerMotionCtrl(hand_index, 1, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1,&ARM_D); //12/18/2103, ok
				break;
			case 17: // finger 2,4 enable
				infltableFingerMotionCtrl(hand_index, 1, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1,&ARM_D); //12/18/2103, ok*/
				infltableFingerMotionCtrl(hand_index, 4, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1,&ARM_D); //12/18/2103, ok
				break;
			case 18: // finger 2,5 enable
				infltableFingerMotionCtrl(hand_index, 1, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1, &ARM_D); //12/18/2103, ok
				break;
			case 19: // finger 3,4 enable
				infltableFingerMotionCtrl(hand_index, 1, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1,&ARM_D); //12/18/2103, ok
				break;
			case 20: // finger 3,5 enable
				infltableFingerMotionCtrl(hand_index, 1, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1, &ARM_D); //12/18/2103, ok
				break;
			case 21: // finger 4,5 enable
				infltableFingerMotionCtrl(hand_index, 1, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1, &ARM_D); //12/18/2103, ok
				break;
			case 22: // finger 1,2,3 enable
				infltableFingerMotionCtrl(hand_index, 1, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1,&ARM_D); //12/18/2103, ok
				break; 
			case 23: // finger 1,2,4 enable
				infltableFingerMotionCtrl(hand_index, 1, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1,&ARM_D); //12/18/2103, ok
				break;	
			case 24: // finger 1,2,5 enable
				infltableFingerMotionCtrl(hand_index, 1, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1, &ARM_D); //12/18/2103, ok
				break;
			case 25: // finger 1,3,4 enable
				infltableFingerMotionCtrl(hand_index, 1, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1,&ARM_D); //12/18/2103, ok
				break;
			case 26: // finger 1,3,5 enable
				infltableFingerMotionCtrl(hand_index, 1, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1, &ARM_D); //12/18/2103, ok
				break;
			case 27: // finger 1,4,5 enable, means "victory"
				infltableFingerMotionCtrl(hand_index, 1, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1, &ARM_D); //12/18/2103, ok
				break;
			case 28: // finger 2,3,4 enable
				infltableFingerMotionCtrl(hand_index, 1, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1,&ARM_D); //12/18/2103, ok
				break;
			case 29: // finger 2,3,5 enable
				infltableFingerMotionCtrl(hand_index, 1, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1, &ARM_D); //12/18/2103, ok
				break;
			case 30: // finger 2,4,5 enable
				infltableFingerMotionCtrl(hand_index, 1, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1, &ARM_D); //12/18/2103, ok
				break;
			case 31: // finger 3,4,5 enable
				infltableFingerMotionCtrl(hand_index, 1, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1, &ARM_D); //12/18/2103, ok
				break;
			case 32: // finger 1,2,3,4 enable
				infltableFingerMotionCtrl(hand_index, 1, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1,&ARM_D); //12/18/2103, ok
				break;
			case 33: // finger 1,2,3,5 enable
				infltableFingerMotionCtrl(hand_index, 1, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1, &ARM_D); //12/18/2103, ok
				break;			
			case 34: // finger 1,2,4,5 enable, means "this is an impolite pose"
				infltableFingerMotionCtrl(hand_index, 1, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1, &ARM_D); //12/18/2103, ok
				break;			
			case 35: // finger 1,3,4,5 enable, means "win"
				infltableFingerMotionCtrl(hand_index, 1, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1, &ARM_D); //12/18/2103, ok
				break;			
			case 36: // finger 2,3,4,5 enable, means "good"
				infltableFingerMotionCtrl(hand_index, 1, -1,&ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1, &ARM_D); //12/18/2103, ok
				break;			
		  case 37: // finger 1, 2,3,4,5 all enable, means "fist"
				infltableFingerMotionCtrl(hand_index, 1, 1, &ARM_D); //12/18/2103, ok			
				infltableFingerMotionCtrl(hand_index, 2, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1, &ARM_D); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1, &ARM_D); //12/18/2103, ok
				break; 	 							
			default: 				
				break;
		} 
	}
	else
	{		
		if ((hand_index==1)||(hand_index==2)) // stop current hand pose
		{
			infltableFingerMotionCtrl(hand_index, 1, 0, &ARM_D); //12/18/2103, ok
			infltableFingerMotionCtrl(hand_index, 2, 0, &ARM_D); //12/18/2103, ok
			infltableFingerMotionCtrl(hand_index, 3, 0, &ARM_D); //12/18/2103, ok
			infltableFingerMotionCtrl(hand_index, 4, 0, &ARM_D); //12/18/2103, ok
			infltableFingerMotionCtrl(hand_index, 5, 0, &ARM_D); //12/18/2103, ok
		}
		else  // stop all hand poses
		{
		  // left hand
			infltableFingerMotionCtrl(1, 1, 0, &ARM_D); //12/18/2103, ok
			infltableFingerMotionCtrl(1, 2, 0, &ARM_D); //12/18/2103, ok
			infltableFingerMotionCtrl(1, 3, 0, &ARM_D); //12/18/2103, ok
			infltableFingerMotionCtrl(1, 4, 0, &ARM_D); //12/18/2103, ok
			infltableFingerMotionCtrl(1, 5, 0, &ARM_D); //12/18/2103, ok
			// right hand
			infltableFingerMotionCtrl(2, 1, 0, &ARM_D); //12/18/2103, ok
			infltableFingerMotionCtrl(2, 2, 0, &ARM_D); //12/18/2103, ok
			infltableFingerMotionCtrl(2, 3, 0, &ARM_D); //12/18/2103, ok
			infltableFingerMotionCtrl(2, 4, 0, &ARM_D); //12/18/2103, ok
			infltableFingerMotionCtrl(2, 5, 0, &ARM_D); //12/18/2103, ok		
		}		
	}
}





/*TouchSensorBasedArmTorqueCtrl, 08/12/2014*/
void TouchSensorBasedArmTorqueCtrl(uint8_t hand_index)
{ 
		/*Touch senson control left arm*/
		if (hand_index==1) //left hand
		{
			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) == 1) // Left hand is touched
			{
				AX_12_Torque_Enable(254, 0); // servo releases torque  

				Delay_ms(500); // exactly delay, ms
				/*hand pose control*/
				CAN2_RX_Hand_Pose_Ctrl(2,0,&ARM_D); // left fist	
			}
			else 
			{
				AX_12_Torque_Enable(254, 1); // servo generates torque 	

				/*hand pose control*/
				CAN2_RX_Hand_Pose_Ctrl(5,0,&ARM_D); // without left plam	
				Delay_ms(1000); // exactly delay, ms	
				//SingleArmHoming(1); // left arm homing				
			}
		}		
		
		
		/*Touch senson control right arm*/
		if (hand_index==2)	// right hand
		{
			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7) == 1) // Right hand is touched
			{
				AX_12_Torque_Enable(254, 0); // servo releases torque  
				
				Delay_ms(500); // exactly delay, ms
				/*hand pose control*/
				CAN2_RX_Hand_Pose_Ctrl(4,0,&ARM_D); // right fist		
			}
			else 
			{
				AX_12_Torque_Enable(254, 1); // servo generates torque 
				/*hand pose control*/
				CAN2_RX_Hand_Pose_Ctrl(6,0,&ARM_D); // right fist
				Delay_ms(1000); // exactly delay, ms	
		
			}
		}	
}


uint8_t testNot1(int32_t XDec)
{
	uint8_t i,result;
	// new version
	for(i=0;i<20;i++)
	{
		if (XDec>(255+(i-1)*256) && XDec<(512+(i-1)*256))
		{
		    XDec=XDec-i*256;
		}
	}
	
	result = 255-XDec;

	return result;
}



uint8_t XDec2Low(int32_t XDec)  
{		
	uint8_t i,result;

	// new version
	if (XDec>255)
	{
		for(i=0;i<6;i++)
		{
			if (XDec>(255+(i-1)*256) && XDec<(512+(i-1)*256))
			{
			    XDec=XDec-i*256;
			}
		}
	}

	result = XDec; 
		
	return result;	

}

//XDec2High, 05222013
uint8_t XDec2High(int32_t XDec)  
{		
	uint8_t result;	 	
		
	// get high value
	result=(int)(XDec/256); // important here, 05222013
	
	return result;
}

 




/**
  * @brief  This function handles TIM4 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{

  if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
		/* LED4 toggling with frequency = 4.57 Hz */
		
		
		/*This channel is used to monitor routine to check whether back to home and disable servo torque, 10/24/2014 added*/		
		switch (ARM_D.gb_current_moving_mode)
		{
			case 0: /* Homing */	
				if(ARM_D.gb_monitor_counter <13*6) // monitor 6 seconds  
				{
					ARM_D.gb_monitor_counter = ARM_D.gb_monitor_counter + 1;	
				}		
				else if(ARM_D.gb_monitor_counter >=13*6) // after 6 seconds, disable all servo torque to save energy
				{
					ARM_D.gb_previous_moving_mode = ARM_D.gb_current_moving_mode;  //update command, 11/06/2014
					AX_12_Torque_Enable(254, 0);  // disable all servo torque			
				}				
				break;
			case 1: /* Joint space teach mode*/				
				break;
			case 2:	  /* vision tracking mode*/
				if(ARM_D.gb_monitor_counter <13*3) // monitor 3 seconds  
				{
					/*monitor single arm tracking*/
					if(ARM_D.gb_monitor_counter >=1) // monitor 0.5 seconds  
					{
						if(ARM_D.gb_arm_index==1) // left arm tracking
						{
							SingleArmHoming(2,&ARM_D); // right arm homing
						}
						if(ARM_D.gb_arm_index==2) // right arm	tracking
						{
							SingleArmHoming(2,&ARM_D); // right arm homing
						}
					}
					ARM_D.gb_monitor_counter = ARM_D.gb_monitor_counter + 1;	
				}		
				else if((ARM_D.gb_monitor_counter >=13*3) && (ARM_D.gb_monitor_counter <13*8))// after 3 seconds, homing, lasts 5s
				{
					Homing(&ARM_D); // back to home positon
					ARM_D.gb_monitor_counter = ARM_D.gb_monitor_counter + 1;
				}
				else if(ARM_D.gb_monitor_counter >=13*8) // after 5 seconds, disable all servo torque to save energy
				{
					AX_12_Torque_Enable(254, 0);  // disable all servo torque			
				}	
				break;			
			default: 				
				break;
		}
			
				
    /* LED4 toggling with frequency = 4.57 Hz */
    //STM_EVAL_LEDToggle(LED4);
	  capture = TIM_GetCapture1(TIM2);
    TIM_SetCompare1(TIM2, capture + CCR1_Val);

  }
  else if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);

    /* LED3 toggling with frequency = 9.15 Hz */
    capture = TIM_GetCapture2(TIM2);
    TIM_SetCompare2(TIM2, capture + CCR2_Val);
  }
  else if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC3); 
    /* LED5 toggling with frequency = 18.31 Hz */   
    capture = TIM_GetCapture3(TIM2);
    TIM_SetCompare3(TIM2, capture + CCR3_Val);
  }
  else
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);	
	
		/*Main control prog*/
		/*initial value*/
		ARM_D.delta_arm_px=0; // delta_arm_px
		ARM_D.delta_arm_py=0; // delta_arm_py
		ARM_D.delta_arm_pz=0; // delta_arm_pz
		ARM_D.delta_arm_pos=0; // delta_arm pos	 
		


		/* get ADC values, 1000x*/
		// 11282013 based on new pcb config	
		left_hand_actual_AD_value[0]= ADC1_ConvertedValue[0]*3300/0xFFF;	// 1000X, left thumb 左大拇指
			//printf("left_hand_actual_AD_value[0] = %dV \r\n", left_hand_actual_AD_value[0]);
		right_hand_actual_AD_value[0]= ADC1_ConvertedValue[1]*3300/0xFFF;	// 1000X, right thumb 右大拇指
			//printf("right_hand_actual_AD_value[0] = %dV \r\n", right_hand_actual_AD_value[0]);
		
		// 12162013 based on new pcb config	
		left_hand_actual_AD_value[1]= ADC3_ConvertedValue[0]*3300/0xFFF;	// 1000X, left thumb 左食指
			//printf("left_hand_actual_AD_value[1] = %dV \r\n", left_hand_actual_AD_value[1]);
		right_hand_actual_AD_value[1]= ADC3_ConvertedValue[1]*3300/0xFFF;	// 1000X, right thumb 右食指
			//printf("right_hand_actual_AD_value[1] = %dV \r\n", right_hand_actual_AD_value[1]); 	   
		left_hand_actual_AD_value[2]= ADC3_ConvertedValue[2]*3300/0xFFF;	// 1000X, left thumb 左中指
			//printf("left_hand_actual_AD_value[2] = %dV \r\n", left_hand_actual_AD_value[2]);
		right_hand_actual_AD_value[2]= ADC3_ConvertedValue[3]*3300/0xFFF;	// 1000X, right thumb 右中指
			//printf("right_hand_actual_AD_value[2] = %dV \r\n", right_hand_actual_AD_value[2]);	
		left_hand_actual_AD_value[3]= ADC3_ConvertedValue[4]*3300/0xFFF;	// 1000X, left thumb 左无名指
			//printf("left_hand_actual_AD_value[3] = %dV \r\n", left_hand_actual_AD_value[3]);	 

		// 11282013 based on new pcb config	
		right_hand_actual_AD_value[3]= ADC1_ConvertedValue[2]*3300/0xFFF;	// 1000X, 右无名指
			//printf("left_hand_actual_AD_value[3] = %dV \r\n", right_hand_actual_AD_value[3]);
		left_hand_actual_AD_value[4]= ADC1_ConvertedValue[3]*3300/0xFFF;	// 1000X, 左小指
			//printf("left_hand_actual_AD_value[4] = %dV \r\n", left_hand_actual_AD_value[4]); 
		right_hand_actual_AD_value[4]= ADC1_ConvertedValue[4]*3300/0xFFF;	// 1000X, 右小指
			//printf("right_hand_actual_AD_value[4] = %dV \r\n", right_hand_actual_AD_value[4]);  

		/*infltableHandMotionCtrl, 12/19/2013*/		
		infltableHandPose(ARM_D.gb_hand_pose_enable, ARM_D.gb_hand_index, ARM_D.gb_hand_pose_index); // 12/19/2013 added

		/* UART1 receive data, defalut Hex*/
		if(uart1_rx_index==uart1_rx_len)
		{ 
			//command		
			ARM_D.command_hand =(uint8_t)uart1_rx_buffer[0]; 		
			// px
			ARM_D.gb_rx_px =(uint8_t)uart1_rx_buffer[1]; 		
			// py
			ARM_D.gb_rx_py =(uint8_t)uart1_rx_buffer[2];	
			// pz
			ARM_D.gb_rx_pz =(uint8_t)uart1_rx_buffer[3];		
			
			/*4dof rigid arm region tranfer: ARM :-380~380 => p=(rx-95)*4	<=> tx=p/4+95 , 08072013*/
			ARM_D.gb_arm_px = (ARM_D.gb_rx_px-95)*4; 
			ARM_D.gb_arm_py = (ARM_D.gb_rx_py-95)*4; 
			ARM_D.gb_arm_pz = (ARM_D.gb_rx_pz-95)*4; 

			/* commands */
			switch (ARM_D.command_hand)
			{
				case 1:       // left plam
					ARM_D.gb_hand_pose_enable = 1; ARM_D.gb_hand_index = 1;	ARM_D.gb_hand_pose_index = 1; // 12/19/2013 update
					break;
				case 2:	      // left fist
					ARM_D.gb_hand_pose_enable = 1; ARM_D.gb_hand_index = 1; ARM_D.gb_hand_pose_index = 37; // 12/19/2013 update
					break;
				case 5: 	  // without left plam	
					ARM_D.gb_hand_pose_enable = 0; ARM_D.gb_hand_index = 1; ARM_D.gb_hand_pose_index = 0; // 12/19/2013 update		
					break;
				case 3: 	  // right plam	
					ARM_D.gb_hand_pose_enable = 1; ARM_D.gb_hand_index = 2; ARM_D.gb_hand_pose_index = 1; // 12/19/2013 update		
					break;
				case 4: 	  // right fist
					ARM_D.gb_hand_pose_enable = 1; ARM_D.gb_hand_index = 2; ARM_D.gb_hand_pose_index = 37; // 12/19/2013 update
					break;
				case 6:       // without right plam
					ARM_D.gb_hand_pose_enable = 0; ARM_D.gb_hand_index = 2; ARM_D.gb_hand_pose_index = 0; // 12/19/2013 update
					break;
				case 7: 	  // enable teach moving mode, default initial system 
					ARM_D.pneumatic_control_enable=1;
					ARM_D.gb_current_moving_mode = 1;  //	teach moving mode 
					ARM_D.breakForLoop = 0; // release break, 06052013				
					GPIO_SetBits(GPIOC, GPIO_Pin_15);  //pump on						
					break;
				case 8: 	  // all initial, may lead error	
					ARM_D.pneumatic_control_enable=0; 
					ARM_D.gb_hand_pose_enable = 0; ARM_D.gb_hand_index = 0; ARM_D.gb_hand_pose_index = 0; // 12/19/2013 update
					ARM_D.gb_current_moving_mode = 0; //	release moving mode 
					Homing(&ARM_D); //Home pos
					GPIO_ResetBits(GPIOC, GPIO_Pin_15); // pump off
					ARM_D.gb_left_touchsensor_switch_cmd = 0; // 08/08/2014 added	
					ARM_D.gb_right_touchsensor_switch_cmd = 0; // 08/08/2014 added	
					break; 
				case 9: 	  // back to home pos 					 
					Homing(&ARM_D); //Home pos
					ARM_D.gb_hand_pose_enable = 0; ARM_D.gb_hand_index = 0; ARM_D.gb_hand_pose_index = 0; // 12/19/2013 update
					ARM_D.gb_left_touchsensor_switch_cmd = 0; // 08/08/2014 added	
					ARM_D.gb_right_touchsensor_switch_cmd = 0; // 08/08/2014 added	
					break;
				case 10: 	  // add vectority input and control, 03152013
					motor_speed_input = (int)uart_speed_map_radio*(int)uart1_rx_buffer[1]; //speed control
					ARM_D.motor_fixed_speed[0] = motor_speed_input;		
					ARM_D.motor_fixed_speed[1] = motor_speed_input;		
					ARM_D.motor_fixed_speed[2] = motor_speed_input;		
					ARM_D.motor_fixed_speed[3] = motor_speed_input;		
					ARM_D.motor_fixed_speed[4] = motor_speed_input;		
					ARM_D.motor_fixed_speed[5] = motor_speed_input;
					break;
				case 11: 	  // clear motor errors, 06052013
					AX_12_CLEAR_ERROR(254);
					ARM_D.gb_hand_pose_enable = 0; ARM_D.gb_hand_index = 0; ARM_D.gb_hand_pose_index = 0; // 12/19/2013 update
					break;
				case 12: 	  //	enable vision tracking moving mode 
					ARM_D.pneumatic_control_enable=1;
					ARM_D.gb_current_moving_mode = 2; //	vision tracking moving mode 
					ARM_D.breakForLoop = 0; // release break, 06052013
					ARM_D.gb_left_touchsensor_switch_cmd = 0; // 08/08/2014 added	
					ARM_D.gb_right_touchsensor_switch_cmd = 0; // 08/08/2014 added	
					// motion follows ->7 -> 12..->8->7->8->9->7 (important, 05282013)
					GPIO_SetBits(GPIOC, GPIO_Pin_15);  //pump on						
					break;			
				case 13: 	  // hand pose control, 01/22/2014 update
					ARM_D.gb_hand_pose_enable = (uint8_t)uart1_rx_buffer[1]; 
					ARM_D.gb_hand_index = (uint8_t)uart1_rx_buffer[2]; 
					ARM_D.gb_hand_pose_index = (uint8_t)uart1_rx_buffer[3];
					break;			
				case 253: 	  // to kinematica intial postion 09172013, for debugging
					InitialPos(&ARM_D);
					ARM_D.gb_hand_pose_enable = 0; ARM_D.gb_hand_index = 0; ARM_D.gb_hand_pose_index = 0; // 12/19/2013 update
					break;
				case 254: 	  // debug	 
					AX_12_Ctrl((uint8_t)uart1_rx_buffer[1],(uint8_t)uart1_rx_buffer[2],(uint8_t)uart1_rx_buffer[3]); // ID: command_hand*/
					ARM_D.gb_hand_pose_enable = 0; ARM_D.gb_hand_index = 0; ARM_D.gb_hand_pose_index = 0; // 12/19/2013 update
					break;				
				default: 				
					break;
			}	

		
		
				/*PID pump control*/
			if (ARM_D.pneumatic_control_enable==1)  // <=1mm	means arriving the target pos
			{
					pump_pwm_duty = pid_update(arm_suitable_voltage,AD_value_1, 100,&pid_per);
			}
			else
			{
				pump_pwm_duty=0;// stop
			} 

			/*jump to new movement, 06052013*/
			if((ARM_D.command_hand==1)||(ARM_D.command_hand==2)||(ARM_D.command_hand==3)||(ARM_D.command_hand==4)
			||(ARM_D.command_hand==5)||(ARM_D.command_hand==6)||(ARM_D.command_hand==8)||(ARM_D.command_hand==9))
			{
				 if((ARM_D.preMoveCommand!=ARM_D.command_hand)&&(ARM_D.preMoveCommand!=7))//diff hand
				 {
						ARM_D.breakForLoop = 1; // enable break
				 }
				 else if((ARM_D.preMoveCommand==ARM_D.command_hand)&&((ARM_D.gb_pre_rx_px!=ARM_D.gb_rx_px)||(ARM_D.gb_pre_rx_py!=ARM_D.gb_rx_py)||(ARM_D.gb_pre_rx_pz!=ARM_D.gb_rx_pz)))	//same hand but diff command
				 {
						ARM_D.breakForLoop = 1; // enable break
				 }		
			}
				
			// update command
			ARM_D.preMoveCommand = ARM_D.command_hand;
			ARM_D.gb_pre_rx_px = ARM_D.gb_rx_px;
			ARM_D.gb_pre_rx_py = ARM_D.gb_rx_py;
			ARM_D.gb_pre_rx_pz = ARM_D.gb_rx_pz;
			/* control command over*/
				

			//delay_t(100);
			uart1_rx_index=0;
			//plan2send_index = 0; //important here, 05222013		//---jim test 01222015
	}	
		
	

	/* LED6 toggling with frequency = 36.62 Hz */
	capture = TIM_GetCapture4(TIM2);
	TIM_SetCompare4(TIM2, capture + CCR4_Val);
  }
}





/**
  * @brief  This function handles CAN2 RX0 request.
  * @param  None
  * @retval None
  */
void CAN2_RX0(void)
{
	CanTxMsg TXMSG;	
	CanRxMsg RxMessage_FIFO0;	
	
	CAN_ITConfig(CAN2,CAN_IT_FMP0, DISABLE);
	CAN_Receive(CAN2, CAN_FIFO0, &RxMessage_FIFO0);
	GPIO_ResetBits(GPIOE, GPIO_Pin_5);//LED "ON",on pcb is used for CAN2 communictation display
		
	//TXMSG.StdId = ARM_TO_PC_CAN_ID;
	TXMSG.StdId = 0x021; // arm->pc	//TXMSG.StdId = 0x061; //ref lifter ->pc	
	TXMSG.ExtId = 0;
	TXMSG.IDE = CAN_ID_STD;                                    
	TXMSG.RTR = CAN_RTR_DATA;
	TXMSG.DLC = 4;  	

	
	switch (RxMessage_FIFO0.Data[0])
	{
		case 0x10:       // 请求状态 

			break;
		case 0x11:       // 设置状态			

			break;
		case 0x12:       // 启动示教动作模式			
			ARM_D.pneumatic_control_enable=1;
			ARM_D.gb_current_moving_mode = 1;  //	teach moving mode 
			ARM_D.breakForLoop = 0; // release break, 06052013	
			ARM_D.gb_monitor_counter = 0; // enable gb_monitor_counter, 10/24/2014
		
		
			/* RX commands, and TX statements of arm and hand, 01/14/2014 new version*/	
			ARM_D.command_hand =(uint8_t)RxMessage_FIFO0.Data[1]; //command
			ARM_D.gb_rx_px =(uint8_t)RxMessage_FIFO0.Data[2];	// px
			ARM_D.gb_rx_py =(uint8_t)RxMessage_FIFO0.Data[3];	// py
			ARM_D.gb_rx_pz =(uint8_t)RxMessage_FIFO0.Data[4]; // pz
				
			/*hand pose control*/
			CAN2_RX_Hand_Pose_Ctrl((uint8_t)RxMessage_FIFO0.Data[1],(uint8_t)RxMessage_FIFO0.Data[2],&ARM_D); 

			ARM_D.gb_previous_moving_mode = ARM_D.gb_current_moving_mode;  //update command, 11/06/2014
			break;
		case 0x13:       // 启动手势跟踪模式			
			ARM_D.pneumatic_control_enable=1;
			ARM_D.gb_current_moving_mode = 2; //	vision tracking moving mode 
			ARM_D.breakForLoop = 0; // release break, 06052013  
			ARM_D.gb_left_touchsensor_switch_cmd = 0; // 08/08/2014 added	
			ARM_D.gb_right_touchsensor_switch_cmd = 0; // 08/08/2014 added	
			ARM_D.gb_monitor_counter = 0; // enable gb_monitor_counter, 10/24/2014		

		
			/* RX commands, and TX statements of arm and hand, 01/14/2014 new version*/	
			ARM_D.command_hand =(uint8_t)RxMessage_FIFO0.Data[1]; //command
			ARM_D.gb_rx_px =(uint8_t)RxMessage_FIFO0.Data[2];	// px
			ARM_D.gb_rx_py =(uint8_t)RxMessage_FIFO0.Data[3];	// py
			ARM_D.gb_rx_pz =(uint8_t)RxMessage_FIFO0.Data[4]; // pz
			ARM_D.gb_rx_az =(uint8_t)RxMessage_FIFO0.Data[5]; // hand rotation			

		
			/*transfer data*/
			/*4dof rigid arm region tranfer: ARM :-380~380 => p=(rx-95)*4	<=> tx=p/4+95 , 08072013*/
			ARM_D.gb_arm_px = (ARM_D.gb_rx_px-95)*4; 
			ARM_D.gb_arm_py = (ARM_D.gb_rx_py-95)*4; 
			ARM_D.gb_arm_pz = (ARM_D.gb_rx_pz-95)*4;	

			/*hand pose control*/
			CAN2_RX_Hand_Pose_Ctrl(ARM_D.command_hand,0,&ARM_D); 			

			ARM_D.gb_previous_moving_mode = ARM_D.gb_current_moving_mode;  //update command, 11/06/2014
			break;
		case 0x14:       // 手臂复位			 
			ARM_D.pneumatic_control_enable=0; 
			ARM_D.gb_hand_pose_enable = 0; ARM_D.gb_hand_index = 0; ARM_D.gb_hand_pose_index = 0; // 12/19/2013 update
			ARM_D.gb_current_moving_mode = 0; //	release moving mode 					
			ARM_D.gb_left_touchsensor_switch_cmd = 0; // 08/08/2014 added	
			ARM_D.gb_right_touchsensor_switch_cmd = 0; // 08/08/2014 added				
				
			ARM_D.breakForLoop = 1; // enable break, 11/27/2014 added

			// Homing with disable torque function, 11/06/2014
			if(ARM_D.gb_previous_moving_mode != ARM_D.gb_current_moving_mode)
			{			
				Homing(&ARM_D);								
			}				
		
			/* RX commands, and TX statements of arm and hand, 01/14/2014 new version*/	
			ARM_D.command_hand =(uint8_t)RxMessage_FIFO0.Data[1]; //command
			ARM_D.gb_rx_px =(uint8_t)RxMessage_FIFO0.Data[2];	// px
			ARM_D.gb_rx_py =(uint8_t)RxMessage_FIFO0.Data[3];	// py
			ARM_D.gb_rx_pz =(uint8_t)RxMessage_FIFO0.Data[4]; // pz
			
			break; 
		case 0x15:       // parameters setting, added 06/27/2014	 								
			break; 		
		default: 
			break; 	
			
	}	
	
	// update command
	ARM_D.preMoveCommand = ARM_D.command_hand;
	ARM_D.gb_pre_rx_px = ARM_D.gb_rx_px;
	ARM_D.gb_pre_rx_py = ARM_D.gb_rx_py;
	ARM_D.gb_pre_rx_pz = ARM_D.gb_rx_pz;	
	/* control command over*/
	

	CAN_Transmit(CAN2,&TXMSG);	
	//GPIO_SetBits(GPIOE,GPIO_Pin_5); //LED "OFF",on pcb is used for CAN2 communictation display	
	CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
	CAN_ITConfig(CAN2,CAN_IT_FMP0, ENABLE);
}



void CAN2_RX_Hand_Pose_Ctrl(uint8_t hand_pose, uint8_t arm_motion, Arm_Data *arm)
{
	/* hand pose commands */
	arm->command_hand = hand_pose;
	
	if(arm_motion!=0x02) // not available to shake hand, 08/12/2014 added test
	{
		switch (hand_pose)
		{
			case 0x01:       // left plam
				//gb_hand_pose_enable = 1; gb_hand_index = 1;	gb_hand_pose_index = 1; // 12/19/2013 update
			  arm->gb_hand_pose_enable = 0; arm->gb_hand_index = 1; arm->gb_hand_pose_index = 0; // more stable, 10/24/2014 update	
				break;
			case 0x02:	      // left fist
				arm->gb_hand_pose_enable = 1; arm->gb_hand_index = 1; arm->gb_hand_pose_index = 37; // 12/19/2013 update
				break;
			case 0x05: 	  // without left plam	
				arm->gb_hand_pose_enable = 0; arm->gb_hand_index = 1; arm->gb_hand_pose_index = 0; // 12/19/2013 update		
				break;
			case 0x03: 	  // right plam	
				//gb_hand_pose_enable = 1; gb_hand_index = 2; gb_hand_pose_index = 1; // 12/19/2013 update	
				arm->gb_hand_pose_enable = 0; arm->gb_hand_index = 2; arm->gb_hand_pose_index = 0; // more stable, 10/24/2014 update			
				break;
			case 0x04: 	  // right fist
				arm->gb_hand_pose_enable = 1; arm->gb_hand_index = 2; arm->gb_hand_pose_index = 37; // 12/19/2013 update
				break;
			case 0x06:       // without right plam
				arm->gb_hand_pose_enable = 0; arm->gb_hand_index = 2; arm->gb_hand_pose_index = 0; // 12/19/2013 update
				break; 			
			default: 				
				break;
		}
	}

  /* arm motion commands */
	switch (arm_motion)
	{
		case 0x01:  // wave hand
		  arm->gb_rx_px = 0; arm->gb_rx_py = 0;arm->gb_rx_pz = 1; // wave hand 
			break;
		case 0x02:	      // shake hand
			arm->gb_rx_px = 0; arm->gb_rx_py = 1; arm->gb_rx_pz = 0; // shake hand
			break;	 	
		default: 
			arm->gb_rx_px = 0; arm->gb_rx_py = 0; arm->gb_rx_pz = 0;  		
			break;
	}
}




UNS32 OnArmCommandWordUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	printf("Chassis Control Word Update @2000|00...\r\n");
	
	//ARM_D.motion_command = motion_command;
	
  return 0;
}


void Delay(u32 n) 
{ 
  while(n!=0){n--;} 
} 


void CAN1Master_heartbeatError(CO_Data* d,  UNS8 heartbeatID)
{
	d->NMTable[heartbeatID]=Unknown_state;
    printf(" HeartBeat not received from node : %d \r\n",heartbeatID);
}

void CAN1Master_initialisation(CO_Data* d)
{
  	printf(" CAN1 Entering in INIT \r\n");
}
void CAN1Master_preOperational(CO_Data* d)
{
	printf(" CAN1 Entering in PRE-OPERATIONAL \r\n");
	(void)masterSendNMTstateChange(d, 0, NMT_Reset_Node);
	//BodyMaster_boottimeInit(d);  //start a timer f or boot time
}
void CAN1Master_operational(CO_Data* d)
{
 	printf(" CAN1 Entering in OPERATIONAL \r\n");
	 
}
void CAN1Master_stopped(CO_Data* d)
{
    printf(" CAN1 Entering in STOPPED \r\n");
}
void CAN1Master_SlaveBootup(CO_Data* d, UNS8 SlaveID)
{
    printf(" receive Bootup SlaveId:%d \r\n",SlaveID);
	(void)masterSendNMTstateChange (d, SlaveID, NMT_Start_Node);
}
void CAN1Master_emcy(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg)
{
    printf(" there is node in emcy state\r\n");
}

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
