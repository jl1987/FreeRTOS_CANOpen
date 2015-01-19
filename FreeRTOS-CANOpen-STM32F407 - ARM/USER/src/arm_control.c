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

/* Private variables */
xQueueHandle xQ_ARM_MSG = NULL;
xTaskHandle	 xT_ARM 	  = NULL;

Arm_Data ARM_D;

struct pid_t pid_per;
int pid_mv = 0;

volatile uint8_t left_hand_pump_pwm_duty[5] 		= {0,0,0,0,0};	//[0 100]
volatile uint8_t right_hand_pump_pwm_duty[5] 		= {0,0,0,0,0};	//[0 100]


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
	
	arm->gb_left_touchsensor_switch_cmd	 =	0;  // 08/12/2014 added
	arm->gb_right_touchsensor_switch_cmd =	0; 	// 08/12/2014 added
	
	arm->gb_current_moving_mode	 = -1; 		 	// 0 is homing mode, 1 is teach mode, 2 is tracking mode
	arm->gb_previous_moving_mode = -2;  			// 0 is homing mode, 1 is teach mode, 2 is tracking mode, added 11/06/2014
	
	
	arm->gb_hand_index 			= 0; 
	arm->gb_hand_pose_index 	= 0; 
	arm->gb_hand_pose_enable = 0;
	
	arm->command_hand=0; 
	
	
	arm->gb_rx_px=0; //reveived positions
	arm->gb_rx_py=0;
	arm->gb_rx_pz=0; 
	arm->gb_rx_az=0; 

	arm->gb_arm_px=0; // target px 
	arm->gb_arm_py=0; // target py 
	arm->gb_arm_pz=0; // target pz 

	arm->pre_arm_px=440; // previous px, or initial pos
	arm->pre_arm_py=0; // previous py
	arm->pre_arm_pz=0; // previous pz 

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
		arm->motor_pos_low[i] 		= 255; // fixed;
		arm->motor_pos_high[i]	  = 3; 						// fixed; 
		arm->motor_speed_low[i]   =30; 			// fixed
		arm->motor_speed_high[i]  = 0; 						// fixed
		arm->motor_fixed_speed[i] = 50; 					// fixed tracking speed better
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
							Joint_Teach_DEMO_2(1,&ARM_D);// wave hand, ok
						}
						else if((arm->gb_rx_px==0)&&(arm->gb_rx_py==1)&&(arm->gb_rx_pz==0))
						{
							 Joint_Teach_DEMO_3(1,&ARM_D);	// shake hand, ok
						}
						else if((arm->gb_rx_px==1)&&(arm->gb_rx_py==0)&&(arm->gb_rx_pz==0))
						{
							 Joint_Teach_DEMO_4(1,&ARM_D); // go style, ok
						}
						else if((arm->gb_rx_px==0)&&(arm->gb_rx_py==1)&&(arm->gb_rx_pz==1))
						{
							 Joint_Teach_DEMO_6(1,&ARM_D); // dancing, ok
						}
					}
					else if ((arm->command_hand==3)||(arm->command_hand==4)||(arm->command_hand==6))
					{
						if((arm->gb_rx_px==0)&&(arm->gb_rx_py==0)&&(arm->gb_rx_pz==1))
						{
							Joint_Teach_DEMO_2(2,&ARM_D);// wave hand, ok
						}
						else if((arm->gb_rx_px==0)&&(arm->gb_rx_py==1)&&(arm->gb_rx_pz==0))
						{
							 Joint_Teach_DEMO_3(2,&ARM_D);	// shake hand, ok
						}
						else if((arm->gb_rx_px==1)&&(arm->gb_rx_py==0)&&(arm->gb_rx_pz==0))
						{
							 Joint_Teach_DEMO_4(2,&ARM_D); // go style, ok
						}
						else if((arm->gb_rx_px==0)&&(arm->gb_rx_py==1)&&(arm->gb_rx_pz==1))
						{
							 Joint_Teach_DEMO_6(2,&ARM_D); // dancing, ok
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
						arm->gb_theta3f_exist = 0; //initialize
						arm->gb_theta_s_index = 0; // solution index						
						arm->gb_theta3f_calc_times = 0; //debug
						arm->gb_theta3f_var = ZERO_f; //initialize
						/* theta3f is variable*/
						for(arm->gb_theta3f_t=-30;arm->gb_theta3f_t<31;arm->gb_theta3f_t++) // calc 29*2 times, scale 5 degree,08092013 update
						{
							arm->gb_theta3f_calc_times = arm->gb_theta3f_calc_times+1; //debug	
							arm->gb_theta3f_var = (float)(arm->gb_theta3f_t*PI_DIV_36); //pi/30, region [-150 150] degree,08092013 update								
							arm->gb_theta3f_exist = ikine_4dof_rigid_arm_theta3f(arm->gb_arm_index, arm->gb_arm_px, arm->gb_arm_py, arm->gb_arm_pz,arm->gb_theta3f_var,0,&ARM_D);		
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
							angle_2_motor_pos(arm->gb_arm_index,arm->theta_solution);
							
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
		Homing();		
		Delay_ms(5000); /// exactly delay, ms*/
	}
	if(arm_index ==2) // right arm
	{		
		// P1
		r_teach_pos[0]=205+357; r_teach_spd[0]=50;
		r_teach_pos[1]=205+10; r_teach_spd[1]=40;
		r_teach_pos[2]=512-50; r_teach_spd[2]=50;
		//r_teach_pos[3]=512-250; r_teach_spd[3]=50;// for 1006
		r_teach_pos[3]=512+250; r_teach_spd[3]=50;// for 1007		
		r_teach_pos[4]=512-235; r_teach_spd[4]=50; 
		AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,r_teach_pos,r_teach_spd);	
		Delay_ms(3000);

		// repeat
		for(i=0;i<10;i++)
		{
			// P1			
			r_teach_pos[0]=205+357; r_teach_spd[0]=50;
			r_teach_pos[1]=205+10; r_teach_spd[1]=40;
			r_teach_pos[2]=512-50; r_teach_spd[2]=50;
			//r_teach_pos[3]=512-250; r_teach_spd[3]=50;// for 1006
			r_teach_pos[3]=512+250; r_teach_spd[3]=50;// for 1007		
			r_teach_pos[4]=512-235; r_teach_spd[4]=50; 
			AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,r_teach_pos,r_teach_spd);	
			Delay_ms(900);

			// P2
			r_teach_pos[0]=205+357; r_teach_spd[0]=50;
			r_teach_pos[1]=205+90; r_teach_spd[1]=40;
			r_teach_pos[2]=512+30; r_teach_spd[2]=50;
			//r_teach_pos[3]=512-250; r_teach_spd[3]=50;// for 1006
			r_teach_pos[3]=512+250; r_teach_spd[3]=50;// for 1007		
			r_teach_pos[4]=512-235; r_teach_spd[4]=50; 
			AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,r_teach_pos,r_teach_spd);	
			Delay_ms(900);			
		}

		// P0, Home pos				
		Homing();		
		Delay_ms(5000); /// exactly delay, ms*/
	}

	arm->breakForLoop = 0; // release break, 06052013
}




/*Joint_Teach_DEMO_2, 07/29/2014 new version*/
/* wave hand*/
/*void Joint_Teach_DEMO_2(uint8_t arm_index)
{
	uint8_t i;
	int32_t l_teach_pos[5],r_teach_pos[5];
	int32_t l_teach_spd[5],r_teach_spd[5];
	 	
	if(arm_index ==1) // left arm
	{
		// P3
		l_teach_pos[0]=1023-105; l_teach_spd[0]=100;
		l_teach_pos[1]=1023-700; l_teach_spd[1]=100;
		l_teach_pos[2]=1023-800; l_teach_spd[2]=100;
		l_teach_pos[3]=512-100; l_teach_spd[3]=50;	
		l_teach_pos[4]=512; l_teach_spd[4]=100;	
		AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,l_teach_pos,l_teach_spd); 
	  //Delay(0x5fffff); 
		Delay_ms(2000); // exactly delay, ms, v2 version, 01/04/2014

		// repeat
		for(i=0;i<10;i++)
		{
			// P3
			l_teach_pos[0]=1023-105; l_teach_spd[0]=50;
			l_teach_pos[1]=1023-700; l_teach_spd[1]=50;
			l_teach_pos[2]=1023-800; l_teach_spd[2]=50;
			l_teach_pos[3]=512-100; l_teach_spd[3]=30;	
			l_teach_pos[4]=512; l_teach_spd[4]=50;	
		  AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,l_teach_pos,l_teach_spd);				
			//Delay(0x3fffff); 
			Delay_ms(800); // exactly delay, ms

			// P2
			l_teach_pos[0]=1023-105; l_teach_spd[0]=50;
			l_teach_pos[1]=1023-800; l_teach_spd[1]=50;
			l_teach_pos[2]=1023-818; l_teach_spd[2]=50;
			//l_teach_pos[3]=1023-205; l_teach_spd[3]=30;	
			l_teach_pos[3]=512-306; l_teach_spd[3]=30; // New configure for TeleRobII arm (three), 05/20/2014 added	
			l_teach_pos[4]=300+200; l_teach_spd[4]=50;	 
		  AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,l_teach_pos,l_teach_spd);			
			//Delay(0x3fffff); 
			Delay_ms(800); // exactly delay, ms
		} 		
		
		// P1, Home pos				
		HomingFast();
		//Delay(0x1ffffff);	
		Delay_ms(3000); // exactly delay, ms
	
	}
	if(arm_index ==2) // right arm
	{
		// P3
		r_teach_pos[0]=105; r_teach_spd[0]=100;
		r_teach_pos[1]=700; r_teach_spd[1]=100;
		r_teach_pos[2]=800; r_teach_spd[2]=100;
		r_teach_pos[3]=512+100; r_teach_spd[3]=50;	
		r_teach_pos[4]=512; r_teach_spd[4]=100;	
		AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,r_teach_pos,r_teach_spd); 
	  //Delay(0x5fffff); 		
		Delay_ms(2000); // exactly delay, ms, v2 version, 01/04/2014

		// repeat
		for(i=0;i<10;i++)
		{
			// P3
			r_teach_pos[0]=105; r_teach_spd[0]=50;
			r_teach_pos[1]=700; r_teach_spd[1]=50;
			r_teach_pos[2]=800; r_teach_spd[2]=50;
			r_teach_pos[3]=512+100; r_teach_spd[3]=30;
			r_teach_pos[4]=512; r_teach_spd[4]=50;	
		  AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,r_teach_pos,r_teach_spd);				
			//Delay(0x3fffff); 			
			Delay_ms(800); // exactly delay, ms

			// P2
			r_teach_pos[0]=105; r_teach_spd[0]=50;
			r_teach_pos[1]=800; r_teach_spd[1]=50;
			r_teach_pos[2]=818; r_teach_spd[2]=50;			
			r_teach_pos[3]=512+306; r_teach_spd[3]=30; // New configure for TeleRobII arm (three), 05/20/2014 added
			r_teach_pos[4]=300+200; r_teach_spd[4]=50;	 
		  AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,r_teach_pos,r_teach_spd);			
			//Delay(0x3fffff); 
			Delay_ms(800); // exactly delay, ms
		} 		
		
		// P1, Home pos				
		HomingFast();
		//Delay(0x1ffffff);	
		Delay_ms(3000); // exactly delay, ms
	}

	breakForLoop = 0; // release break, 06052013
}*/



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
			l_teach_pos[0]=1023-305-50; l_teach_spd[0]=50;
			l_teach_pos[1]=1023-205; l_teach_spd[1]=50;
			l_teach_pos[2]=1023-512; l_teach_spd[2]=50;
			//l_teach_pos[3]=1023-420; l_teach_spd[3]=50;	
			l_teach_pos[3]=512-150; l_teach_spd[3]=50; // New configure for TeleRobII arm (three), 05/20/2014 added	
			l_teach_pos[4]=512; l_teach_spd[4]=50;	
			AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,l_teach_pos,l_teach_spd);								
			
			Delay_ms(1000); // exactly delay, ms, v2 version, 01/04/2014
		}

		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) == 1) // Left hand is touched
		{
			arm->gb_left_touchsensor_switch_cmd = arm->gb_left_touchsensor_switch_cmd + 1; // break P3
			
			AX_12_Torque_Enable(254, 0); // servo releases torque 

			Delay_ms(50); // exactly delay, ms
			/*hand pose control*/
			CAN2_RX_Hand_Pose_Ctrl(2,0); // left fist		
		}
		else if((GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) != 1) && (arm->gb_left_touchsensor_switch_cmd != 0))
		{
			AX_12_Torque_Enable(254, 1); // servo generates torque
			
			/*hand pose control*/
			CAN2_RX_Hand_Pose_Ctrl(5,0); // without left plam	
			//Delay_ms(2000); // exactly delay, ms	
			//Homing(); // homing		
		}		
	}
	
	
	/*Touch senson control right arm*/
	if (arm_index==2)	// right arm
	{
		if(arm->gb_right_touchsensor_switch_cmd == 0)
		{
			// P3
			r_teach_pos[0]=305+50; r_teach_spd[0]=50;
			r_teach_pos[1]=205; r_teach_spd[1]=50;
			r_teach_pos[2]=512; r_teach_spd[2]=50;
			//r_teach_pos[3]=420; r_teach_spd[3]=50;
			r_teach_pos[3]=512+150; r_teach_spd[3]=50; // New configure for TeleRobII arm (three), 05/20/2014 added
			r_teach_pos[4]=512; r_teach_spd[4]=50;	
			AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,r_teach_pos,r_teach_spd);				
			
			Delay_ms(1000); // exactly delay, ms, v2 version, 01/04/2014
		}

		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7) == 1) // Right hand is touched
		{
			arm->gb_right_touchsensor_switch_cmd = arm->gb_right_touchsensor_switch_cmd + 1; // break P3
			
			AX_12_Torque_Enable(254, 0); // servo releases torque 
			
			Delay_ms(50); // exactly delay, ms
			/*hand pose control*/
			CAN2_RX_Hand_Pose_Ctrl(4,0); // right fist					
		}
		else if((GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7) != 1) && (arm->gb_right_touchsensor_switch_cmd != 0))
		{
			AX_12_Torque_Enable(254, 1); // servo generates torque 
			
			/*hand pose control*/
			CAN2_RX_Hand_Pose_Ctrl(6,0); // right fist
			//Delay_ms(2000); // exactly delay, ms	
			//Homing(); // homing						
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
		l_teach_pos[0]=1023-355; l_teach_spd[0]=50;
		l_teach_pos[1]=1023-205; l_teach_spd[1]=50;
		l_teach_pos[2]=1023-512; l_teach_spd[2]=50;
		l_teach_pos[3]=320; l_teach_spd[3]=50;
		l_teach_pos[4]=320; l_teach_spd[4]=50;	 
	    AX_12_Syn_Ctrl_5DOF_Rigid_Arm(arm_index,l_teach_pos,l_teach_spd);			
		Delay(0x2affff); 

		// repeat
		for(i=1;i<38;i++)
		{
			arm->gb_hand_pose_enable = 1; arm->gb_hand_index = 1;	arm->gb_hand_pose_index = i; // 12/19/2013 update 
			Delay(0xffffff); 
		}

		arm->gb_hand_pose_enable = 0; arm->gb_hand_index = 0; arm->gb_hand_pose_index = 0; // 12/19/2013 update
		Delay(0xffffff);  		
		
		// P1, Home pos				
		Homing();
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
		Homing();
		Delay(0xffffff); 		
	}	

	arm->breakForLoop = 0; // release break, 06052013
}


/*Joint_Teach_DEMO_5, 05312013*/
/* power*/
void Joint_Teach_DEMO_5(uint8_t arm_index,Arm_Data *arm)
{
	//uint8_t i;
	//int32_t l_teach_pos[6],r_teach_pos[6];
	//int32_t l_teach_spd[6],r_teach_spd[6];	 

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
	theta1_max=(float)(4*pi/3) + theta_error;  //max 240 degree, NOTE: RX64 may diff with ax12, update latter<=> motor 1 inital pos: 205 
	theta1_min=-(float)(pi/3) - theta_error;   //min -60 degree, NOTE: RX64 may diff with ax12, update latter<=> motor 1 inital pos: 205
	theta2_max=(float)pi + theta_error;        //max 180 degree, <=> motor 2 inital pos: 512
	theta2_min=ZERO_f - theta_error;	       //min 0 degree(-45 may be better, but unsafty), <=> motor 2 inital pos: 512
	//theta3_max=(float)(5*pi/6) + theta3_init_offset + theta_error;  //max 150+(-90) degree, <=> motor 3 inital pos: 512
	//theta3_min=-(float)(5*pi/6) + theta3_init_offset - theta_error; //min -150+(-90) degree, <=> motor 3 inital pos: 512
	// 09172013 test 1
	//theta3_max=(float)(5*pi/6) + theta_error;  //max 150+(-90) degree, <=> motor 3 inital pos: 512
	//theta3_min=-(float)(5*pi/6) - theta_error; //min -150+(-90) degree, <=> motor 3 inital pos: 512

	// 09172013 test 2
	theta3_max=(float)(pi/3) + theta_error;  //max 150+(-90) degree, <=> motor 3 inital pos: 512
	theta3_min=-(float)(5*pi/6) - theta_error; //min -150+(-90) degree, <=> motor 3 inital pos: 512	
	theta4_max=(float)(11*pi/18) + theta_error;  //max 110 degree, <=> motor 4 inital pos: 512
	theta4_min=ZERO_f - theta_error;	       //min 0 degree, <=> motor 4 inital pos: 512


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
    //r = sqrt(temp_p3); //r = sqrt(px*px + py*py); 
    temp_r = d*d;
    temp_r = temp_p3 - temp_r; 
	if(temp_r<ZERO_f) //singularity 
	{
	   return exist_s;
	}
    a = (float)my_sqrt(temp_r); //a = sqrt(px*px + py*py - d*d); 
	/*if((my_abs(d)<=EPSILON)&&(my_abs(a)<=EPSILON)) //singularity,//07-11-2013 new note: this is unnecessary, as system has defined atan2f(0,0)=0
	{
	   return exist_s;
	}*/ 	
    beta = (float)atan2f(d,a); //beta = atan(d/a); //old
	/*if((my_abs(py)<=EPSILON)&&(my_abs(px)<=EPSILON)) //singularity,//07-11-2013 new note: this is unnecessary, as system has defined atan2f(0,0)=0
	{
	   return exist_s;
	}*/
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
	/*if((my_abs(pz_2)<=EPSILON)&&(my_abs(px_2)<=EPSILON)) //singularity,//07-11-2013 new note: this is unnecessary, as system has defined atan2f(0,0)=0
	{
		return exist_s;
	}*/
	fi=(float)atan2f(pz_2,px_2);
	// solution 1	 
	v_temp7_1 = (float)sinf(theta4_1);
	v_temp7_1 = v_temp7_1*le_;
	v_temp8_1 = (float)cosf(theta4_1);
    v_temp8_1 =  v_temp8_1*le_;
	v_temp8_1 = v_temp8_1 + l1;
	/*if((my_abs(v_temp7_1)<=EPSILON)&&(my_abs(v_temp8_1)<=EPSILON)) //singularity,//07-11-2013 new note: this is unnecessary, as system has defined atan2f(0,0)=0
	{
		return exist_s;
	}*/
	pai_1 = (float)atan2f(v_temp7_1,v_temp8_1);
	theta2_1 = 	fi-	pai_1;	
	// solution 2
	v_temp7_2 = (float)sinf(theta4_2);
	v_temp7_2 = v_temp7_2*le_;
	v_temp8_2= (float)cosf(theta4_2);
    v_temp8_2 =  v_temp8_2*le_;
	v_temp8_2 = v_temp8_2 + l1;
	/*if((my_abs(v_temp7_2)<=EPSILON)&&(my_abs(v_temp8_2)<=EPSILON)) //singularity,//07-11-2013 new note: this is unnecessary, as system has defined atan2f(0,0)=0
	{
		return exist_s;
	}*/
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
	//gb_fwd_pos[1] = l1*(float)cosf(theta3f_solution[1])*(float)sinf(theta3f_solution[0]); //y, in new 4dof arm coordinate 
	//gb_fwd_pos[2] = l1*(float)sinf(theta3f_solution[1]);             //z, in new 4dof arm coordinate, needn't solution(2)-pi/2


	/*debug*//*
	printf("***theta3f***\r\n"); 
	printf("px = %d\r\n", px);
	printf("py = %d\r\n", py);
	printf("pz = %d\r\n", pz);							
	printf("theta1 = %4.4f \r\n", theta3f_solution[0]);
	printf("theta2 = %4.4f \r\n", theta3f_solution[1]);
	printf("theta3 = %4.4f \r\n", theta3f_solution[2]);
	printf("theta4 = %4.4f \r\n", theta3f_solution[3]);
	printf("exist_theta3f_s = %d\r\n", exist_s);*/


	return exist_s;
}

/*hand desired angle control, update 12/05/2013*/
void hand_desired_angle_crl(uint8_t enable_angle_crl, uint8_t desired_rx_angle, Arm_Data *arm)
{ 
	/*calc hand rotation angle, 02/18/2013 new version*/ 
	arm->theta_solution[4] =  - arm->theta_solution[0] - arm->theta_solution[2]; // 02/18/2014,  general ok
	
}

/*(4dof rigid arm sulution)angles to motor postions, update 08/07/2013*/
void angle_2_motor_pos(uint8_t arm_index, float angle[5])
{
	int32_t right_motor_pos[5]; 
	int32_t left_motor_pos[5]; 	
	uint8_t i; 
	
	
	if (arm_index==1) // left arm
	{
		// 02/18/2014 v2, ok
		left_motor_pos[0]=left_arm_init_motor_pos[0]-(int32_t)(angle[0]*(float)ANGLE_2_MOTOR_POS_RADIO); //note here
		left_motor_pos[1]=left_arm_init_motor_pos[1]-(int32_t)(angle[1]*(float)ANGLE_2_MOTOR_POS_RADIO);
		left_motor_pos[2]=left_arm_init_motor_pos[2]-(int32_t)(angle[2]*(float)ANGLE_2_MOTOR_POS_RADIO);	//note here
		//left_motor_pos[3]=left_arm_init_motor_pos[3]+(int32_t)(angle[3]*(float)ANGLE_2_MOTOR_POS_RADIO);	
		left_motor_pos[3]=left_arm_init_motor_pos[3]-(int32_t)(angle[3]*(float)ANGLE_2_MOTOR_POS_RADIO);// New configure for TeleRobII arm (three), 05/20/2014 added	
		left_motor_pos[4]=left_arm_init_motor_pos[4]-(int32_t)(angle[4]*(float)ANGLE_2_MOTOR_POS_RADIO); //11/06/2013 waiting for verify
		
		
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
			gb_left_m_pos[i] = left_motor_pos[i];
		}
	}


	if (arm_index==2) // right arm
	{
		right_motor_pos[0] = rigid_arm_init_motor_pos[0]+(int32_t)(angle[0]*(float)ANGLE_2_MOTOR_POS_RADIO);
		right_motor_pos[1] = rigid_arm_init_motor_pos[1]+(int32_t)(angle[1]*(float)ANGLE_2_MOTOR_POS_RADIO);
		right_motor_pos[2] = rigid_arm_init_motor_pos[2]+(int32_t)(angle[2]*(float)ANGLE_2_MOTOR_POS_RADIO);
		//right_motor_pos[3] = rigid_arm_init_motor_pos[3]-(int32_t)(angle[3]*(float)ANGLE_2_MOTOR_POS_RADIO);
		right_motor_pos[3] = rigid_arm_init_motor_pos[3]+(int32_t)(angle[3]*(float)ANGLE_2_MOTOR_POS_RADIO); // New configure for TeleRobII arm (three), 05/20/2014 added
		right_motor_pos[4] = rigid_arm_init_motor_pos[4]+(int32_t)(angle[4]*(float)ANGLE_2_MOTOR_POS_RADIO);//11/06/2013 waiting for verify

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
			gb_right_m_pos[i] = right_motor_pos[i];
		}
	}
		
	/*printf("motor_pos[0] = %d \r\n", motor_pos[0]);
	printf("motor_pos[1] = %d \r\n", motor_pos[1]);
	printf("motor_pos[2] = %d \r\n", motor_pos[2]);
	printf("motor_pos[3] = %d \r\n", motor_pos[3]);*/
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
	//while(TimingDelay != 0);//original version 
	while((TimingDelay != 0x00)&&(breakForLoop != 1));//new version with break 
}



/*Homing, 06052013*/
void Homing(void)
{	
	int32_t l_teach_pos[5],l_teach_spd[5];
	int32_t r_teach_pos[5],r_teach_spd[5];	 	
	
	// P0, Home pos
	l_teach_pos[0]=left_arm_home_motor_pos[0]; l_teach_spd[0]=50;
	l_teach_pos[1]=left_arm_home_motor_pos[1]; l_teach_spd[1]=50;
	l_teach_pos[2]=left_arm_home_motor_pos[2]; l_teach_spd[2]=50;
	l_teach_pos[3]=left_arm_home_motor_pos[3]; l_teach_spd[3]=50;
	l_teach_pos[4]=left_arm_home_motor_pos[4]; l_teach_spd[4]=50;
  AX_12_Syn_Ctrl_5DOF_Rigid_Arm(1,l_teach_pos,l_teach_spd); 

	r_teach_pos[0]=right_arm_home_motor_pos[0]; r_teach_spd[0]=50;
	r_teach_pos[1]=right_arm_home_motor_pos[1]; r_teach_spd[1]=50;
	r_teach_pos[2]=right_arm_home_motor_pos[2]; r_teach_spd[2]=50;
	r_teach_pos[3]=right_arm_home_motor_pos[3]; r_teach_spd[3]=50;
	r_teach_pos[4]=right_arm_home_motor_pos[4]; r_teach_spd[4]=50; 
	AX_12_Syn_Ctrl_5DOF_Rigid_Arm(2,r_teach_pos,r_teach_spd);		
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
	//while(TimingDelay != 0);//original version 
	while((TimingDelay != 0x00)&&(breakForLoop != 1));//new version with break 
}



/*system interput, 01/01/2014*/
void SysTick_Handler(void)
{
	//if (TimingDelay != 0x00) //original version 
	if ((TimingDelay != 0x00)&&(breakForLoop != 1)) //new version with break
	TimingDelay --;
}







/*used for debugging, 01/21/2014 update*/
//static void GPIO_Configuration_Debug(void)	   

//{
//	GPIO_InitTypeDef  GPIO_InitStructure;
	/*GPIOA Periph clock enable*/ 
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* Configure PA4(LED1) PA5(LED2) PA6(LED3) PA7(LED4) in output pushpull mode */
	/*Note: GPIO_Pin_4 | GPIO_Pin_5 is used for ADC1, so it can not be used for debugging, 01/22/2014 update*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//}  









/**
  * @brief  USART1 Reveive Interrupt Function
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)                           
{	
	uint8_t RX_dat;


	if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)   
	{
		USART_ClearITPendingBit(USART1,   USART_IT_RXNE);       
		RX_dat=USART_ReceiveData(USART1);// & 0x7F
		
		if (uart1_rx_index<uart1_rx_len)
		{
			uart1_rx_buffer[uart1_rx_index] = RX_dat;
			uart1_rx_index++;
		}
		 
		//USART_SendData(USART1, RX_dat); // ok	
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}//单?癳?
	}
} 





int Hex2Dec(unsigned char Hex)
{
	uint8_t a;

	a=(Hex/16)*10+ Hex%16;

	return a;
}


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
		//sub_torque_N[j] = torque_enable[j];
		//sub_torque_N[j] = torque_enable;
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

	head1 = 255; ax12_command[0] = head1;
	head2 = 255; ax12_command[1] = head2;			  
	id = id_index;	ax12_command[2] = id;
	length = 7; ax12_command[3] = length;
	instruction = 3; ax12_command[4] = instruction;
	position = 30; ax12_command[5] = position;
	positionLow = posLow; ax12_command[6] = positionLow;
	positionHigh = posHigh; ax12_command[7] = positionHigh;
	speedLow = 30; ax12_command[8] = speedLow;
	speedHigh = 0; ax12_command[9] = speedHigh;
	
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


/* joint position syn move for 4dof rigid arm, update 0809203 */
void AX_12_Syn_Ctrl_4DOF_Rigid_Arm(uint8_t arm_index, int32_t position_in[4], int32_t speed_in[4])	//[6] is suitable muti situations
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
	length = 24; ax12_command[3] = length;	// 4 motors move at the same time	
	instruction = 131; ax12_command[4] = instruction;
	position = 30; ax12_command[5] = position; 	
	each_length=4;ax12_command[6] = each_length;  	 	
	
	for(j=0;j<4;j++)
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
		if (arm_index==2) // left arm
		{
			sub_id_N[j]=j+5;                   ax12_command[7+j*5]=sub_id_N[j];
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

	ax12_command[27] = checksum;

	for(ii=0;ii<syn_joint_ctrl_len_4dof;ii++)
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
	head1 = 255; ax12_command[0] = head1;
	head2 = 255; ax12_command[1] = head2;			  
	id = id_index;	ax12_command[2] = id;	
	length = 4; ax12_command[3] = length;	
	instruction = 3; ax12_command[4] = instruction;	
	alarmShutdown = 18; ax12_command[5] = alarmShutdown;
	clearErrorCmd = 127; ax12_command[6] = clearErrorCmd;	// clear all motor and all errors 	
	
	
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


/* trajectory_planning_stm32 for 1 dof, 05/22/2013 */
void trajectory_planning_stm32(uint8_t id, int32_t q0, int32_t q1, int32_t qd0, int32_t qd1, int32_t tv, int32_t t_period, uint8_t movingMode)
{    
	uint8_t  i,t_index,t_max;
	float t,t_step;	
	float qt,qdt;   
	float t2,t3,t4,t5;
	float a1,a2,a3,a4;
	float b1,b2,b3;

	// get whole times
    t_max = (int)(tv/t_period);  
	t_step = tv-t_period;
	t_step = (float)(t_period/t_step); // normalized time from 0 -> 1  

	// clear all old id data at first
	current_traj_len=0;
	for(i=0;i<traj_buffer_len;i++)
	{ 	
	  //t_traj_buffer[i] = 0;
	  //qt_traj_buffer[i][id] = 0;
	  //qdt_traj_buffer[i][id] = 0;
	  qt_traj_buffer[i] = 0;
	  qdt_traj_buffer[i] = 0;
	}

    //for t_index=1:length(t_all) 
	for(t_index=0;t_index<t_max;t_index++)
	{       
		// //t_all=0:t_period/(tv-t_period):1;	// normalized time from 0 -> 1
		t = t_step*t_index;	// normalized time from 0 -> 1 

        // planning trajectory
        if (q0 == q1)
		{
            qt = q0;
            qdt = 0;      
            //return
        }
		else
		{
            // //ARM programs, 05222013, ok
            // // qt=(6*q1 - 6*q0 - (3*qd0 + 3*qd1))*t^5 + (15*q0 - 15*q1 + (8*qd0 +
            // // 7*qd1))*t^4 + (10*q1 - 10*q0 - (6*qd0 + 4*qd1))*t^3 + qd0*t + q0;
            t2 = t*t;
            t3 = t2*t;
            t4 = t3*t;
            t5 = t4*t;
            // a1=(6*q1 - 6*q0 - (3*qd0 + 3*qd1))*t^5
            a1 = 3*qd0;
            a1 = a1 + 3*qd1;
            a1 = -a1;
            a1 = a1 + 6*q1;
            a1 = a1 - 6*q0;
            a1 = a1*t5;
            // a2=(15*q0 - 15*q1 + (8*qd0 + 7*qd1))*t^4
            a2 = 8*qd0;
            a2 = a2 + 7*qd1;
            a2 = a2 + 15*q0;
            a2 = a2 - 15*q1;
            a2 = a2*t4;
            // a3=(10*q1 - 10*q0 - (6*qd0 + 4*qd1))*t^3
            a3 = 6*qd0;
            a3 = a3 + 4*qd1;
            a3 = -a3;
            a3 = a3 + 10*q1;
            a3 = a3 - 10*q0;
            a3 = a3*t3;
            // a4=qd0*t
            a4 = qd0*t;
            // qt = a1+a2+a3+a4+q0
            qt = a1 + a2;
            qt = qt + a3;
            qt = qt + a4;
            qt = qt + q0;
            // // qdt=(30*q1 - 30*q0 - 15*qd0 - 15*qd1)*t^4 + (60*q0 - 60*q1 + 
            // // 32*qd0 + 28*qd1)*t^3 + (30*q1 - 30*q0 - 18*qd0 - 12*qd1)*t^2 + qd0;   
            // b1=(30*q1 - 30*q0 - 15*qd0 - 15*qd1)*t^4
            b1 = 30*q1;
            b1 = b1 - 30*q0;
            b1 = b1 - 15*qd0;
            b1 = b1 - 15*qd1;
            b1 = b1*t4; 
            // b2=(60*q0 - 60*q1 + 32*qd0 + 28*qd1)*t^3
            b2 = 60*q0;
            b2 = b2 - 60*q1;
            b2 = b2 + 32*qd0;
            b2 = b2 + 28*qd1;
            b2 = b2*t3;
            // b3=(30*q1 - 30*q0 - 18*qd0 - 12*qd1)*t^2
            b3 = 30*q1;
            b3 = b3 - 30*q0;
            b3 = b3 - 18*qd0;
            b3 = b3 - 12*qd1;
            b3 = b3*t2;
            // qdt=b1+b2+b3+qd0;
            qdt = b1 + b2;
            qdt = qdt + b3;
            qdt = qdt + qd0;    
        }    

        // region limited  
        if (qt>1023)
        {
		    qt=1023;
        }
		else if (qt<0)
        {
		    qt=0;
        }
		
        // if (qdt>1023) &&(movingMode~=0) // note: // 0-1023 is CCW(0 stop), 1024-2047 is CW(1024 stop) for wheel mode
        //     qdt=1023;
        // elseif (qdt<0)
        //     qdt=0;
        // end
        
        if (qdt>1023)
		{
            qdt = 1023;
        }
		else if ((qdt<0)&&(movingMode==0)) // wheel mode note: // 0-1023 is CCW(0 stop), 1024-2047 is CW(1024 stop) for wheel mode
        {
		    qdt = (int)(1024 - qdt); // abs(qdt)           
        }
		else if ((qdt<0)&&(movingMode!=0)) // joint mode
        {
		    qdt = 0;
        }

		// store trajectory to buffer
		if(t_index<traj_buffer_len)	
		{
			//t_traj_buffer[t_index] = (int)t;
			//qt_traj_buffer[t_index][id] = qt;
			//qdt_traj_buffer[t_index][id] = qdt;	
			qt_traj_buffer[t_index] = (int)qt;
			qdt_traj_buffer[t_index] = (int)qdt;	
		}
		else
		{
			return;
		}
		/*
		printf("t_index= %d \r ", t_index); 	
		printf("qdt= %d \r\n", qdt); */
	} 	
	current_traj_len = t_max; 
	/*printf("t_max= %d \r\n", t_max);*/  
}

/* trajectory_planning_stm32 for 6dof, 05/22/2013 */
//void trajectory_planning_stm32_6dof(uint8_t arm_index, int32_t q0_6d[6], int32_t q1_6d[6], int32_t qd0_6d[6], int32_t qd1_6d[6], int32_t tv, int32_t t_period, uint8_t movingMode)
void trajectory_planning_stm32_6dof(int32_t q0_6d[6], int32_t q1_6d[6], int32_t qd0_6d[6], int32_t qd1_6d[6], int32_t tv, int32_t t_period, uint8_t movingMode)
{    
	int32_t q0,q1,qd0,qd1;
	uint8_t  i,j,t_index,t_max;
	float t,t_step;	
	float qt,qdt;   
	float t2,t3,t4,t5;
	float a1,a2,a3,a4;
	float b1,b2,b3;

	// get whole times
    t_max = (int)(tv/t_period);  
	t_step = tv-t_period;
	t_step = (float)(t_period/t_step); // normalized time from 0 -> 1  

	// clear all old id data at first
	current_traj_len=0;
	for(i=0;i<traj_buffer_len;i++)
	{ 	
	  	//t_traj_buffer[i] = 0;	
	  	//apply to 6 joint(motor)
		for(j=0;j<6;j++)
		{
			qt_traj_buf6d[i][j] = 0;
			qdt_traj_buf6d[i][j] = 0;			
		}
	}


	for(t_index=0;t_index<t_max;t_index++)
	{       
		// //t_all=0:t_period/(tv-t_period):1;	// normalized time from 0 -> 1
		t = t_step*t_index;	// normalized time from 0 -> 1 

		//apply to 6 joint(motor)
		for(j=0;j<6;j++)
		{
			q0 = q0_6d[j];
			q1 = q1_6d[j];
			qd0 = qd0_6d[j];
			qd1	= qd1_6d[j];

			// planning trajectory
	        if (q0 == q1)
			{
	            qt = q0;
	            //qdt = 0;  // important here, 05242013
				qdt = 1;  //set 1 is the lowest speed, motor limited(ref data sheet), 05242013 				
	            //return
	        }
			else
			{
	            // //ARM programs, 05222013, ok
	            // // qt=(6*q1 - 6*q0 - (3*qd0 + 3*qd1))*t^5 + (15*q0 - 15*q1 + (8*qd0 +
	            // // 7*qd1))*t^4 + (10*q1 - 10*q0 - (6*qd0 + 4*qd1))*t^3 + qd0*t + q0;
	            t2 = t*t;
	            t3 = t2*t;
	            t4 = t3*t;
	            t5 = t4*t;
	            // a1=(6*q1 - 6*q0 - (3*qd0 + 3*qd1))*t^5
	            a1 = 3*qd0;
	            a1 = a1 + 3*qd1;
	            a1 = -a1;
	            a1 = a1 + 6*q1;
	            a1 = a1 - 6*q0;
	            a1 = a1*t5;
	            // a2=(15*q0 - 15*q1 + (8*qd0 + 7*qd1))*t^4
	            a2 = 8*qd0;
	            a2 = a2 + 7*qd1;
	            a2 = a2 + 15*q0;
	            a2 = a2 - 15*q1;
	            a2 = a2*t4;
	            // a3=(10*q1 - 10*q0 - (6*qd0 + 4*qd1))*t^3
	            a3 = 6*qd0;
	            a3 = a3 + 4*qd1;
	            a3 = -a3;
	            a3 = a3 + 10*q1;
	            a3 = a3 - 10*q0;
	            a3 = a3*t3;
	            // a4=qd0*t
	            a4 = qd0*t;
	            // qt = a1+a2+a3+a4+q0
	            qt = a1 + a2;
	            qt = qt + a3;
	            qt = qt + a4;
	            qt = qt + q0;
	            // // qdt=(30*q1 - 30*q0 - 15*qd0 - 15*qd1)*t^4 + (60*q0 - 60*q1 + 
	            // // 32*qd0 + 28*qd1)*t^3 + (30*q1 - 30*q0 - 18*qd0 - 12*qd1)*t^2 + qd0;   
	            // b1=(30*q1 - 30*q0 - 15*qd0 - 15*qd1)*t^4
	            b1 = 30*q1;
	            b1 = b1 - 30*q0;
	            b1 = b1 - 15*qd0;
	            b1 = b1 - 15*qd1;
	            b1 = b1*t4; 
	            // b2=(60*q0 - 60*q1 + 32*qd0 + 28*qd1)*t^3
	            b2 = 60*q0;
	            b2 = b2 - 60*q1;
	            b2 = b2 + 32*qd0;
	            b2 = b2 + 28*qd1;
	            b2 = b2*t3;
	            // b3=(30*q1 - 30*q0 - 18*qd0 - 12*qd1)*t^2
	            b3 = 30*q1;
	            b3 = b3 - 30*q0;
	            b3 = b3 - 18*qd0;
	            b3 = b3 - 12*qd1;
	            b3 = b3*t2;
	            // qdt=b1+b2+b3+qd0;
	            qdt = b1 + b2;
	            qdt = qdt + b3;
	            qdt = qdt + qd0;    
	        }
			
		   
			qdt = (float)SPEED_PERCENT*qdt; // add speed percent, important here, 05/29/2013    
	

	        // region limited  
	        if (qt>1023)
	        {
			    qt=1023;
	        }
			else if (qt<=0)
	        {
			    qt=0;
	        }		
	        
			//wheel mode
			/*if (qdt>2047&&(movingMode==0))
			{
	            qdt = 2047;
	        }
			if ((qdt<0)&&(movingMode==0)) // wheel mode note: // 0-1023 is CCW(0 stop), 1024-2047 is CW(1024 stop) for wheel mode
	        {
			    qdt = (int)(1024 - qdt); // abs(qdt)           
	        }
			if (((int)qdt==0)&&(movingMode==0)) // for mode modes
	        {
				qdt = 0;  //set 0 is stop 
	        }*/

			// joint mode
			/*if (qdt>1023&&(movingMode!=0))
			{
	            qdt = 1023;
	        }
			if ((qdt<0)&&(movingMode!=0)) // joint mode
	        {
			    qdt = my_abs_int(qdt);
				//qdt = 1;  //set 1 is the lowest speed, motor limited(ref data sheet), 05242013 
	        }			
			if (((int)qdt==0)&&(movingMode!=0)) // for joint modes
	        {
				qdt = 1;  //set 1 is the lowest speed, motor limited(ref data sheet), 05242013 
	        }*/
			
			// joint mode
			if (qdt<0) // joint mode
	        {
			    qdt = my_abs_int(qdt);
				//qdt = 1;  //set 1 is the lowest speed, motor limited(ref data sheet), 05242013 
	        }
			if (qdt>1023)  // order is important
			{
	            qdt = 1023;
	        }			
			if ((int)qdt==0) // for joint modes
	        {
				qdt = 1;  //set 1 is the lowest speed, motor limited(ref data sheet), 05242013 
	        }  

	
			// store trajectory to buffer
			if(t_index<traj_buffer_len)	
			{
				//t_traj_buffer[t_index] = (int)t;
				qt_traj_buf6d[t_index][j] = (int)qt;
				qdt_traj_buf6d[t_index][j] = (int)qdt;				
			}
			else
			{
				return;
			}
		}
	} 	
	current_traj_len = t_max; 	
}

/* trajectory scale, 05/28/2013 */
uint16_t trajectory_scale_6d(uint16_t Min_Scale, int32_t pre_q_6d[6], int32_t target_q_6d[6])
{
	uint32_t delta_pos;	
	uint16_t Step_Scale;
	// get scale
	delta_pos = my_max_int_abs_6d(pre_q_6d,target_q_6d);	// max delta distance error

    Step_Scale = delta_pos/Min_Scale;   // initial set 50

	if(Step_Scale<Min_Scale)
	{
	   Step_Scale = 3;	// min interapation points
	}
	else if(Step_Scale>traj_buffer_len)
	{
		 Step_Scale=traj_buffer_len; // max interapation points	
	}

	return Step_Scale;
}


/* trajectory planning new simple method, 05/28/2013 */
void trajectory_simple_planning_6dof(uint8_t arm_index, int32_t target_q_6d[6])
{
  	int32_t left_current_q_6d[6];
	int32_t right_current_q_6d[6];
	int32_t left_max_delta_pos, right_max_delta_pos;	


	if (arm_index ==1)// left arm
	{
	 
		left_max_delta_pos = my_max_int_abs_6d(left_current_q_6d,target_q_6d);	// max delta distance error	
		
		if (left_max_delta_pos<=MAX_DELTA_POS)
		{
			AX_12_Syn_Ctrl(0,target_q_6d, qdt_current_buf6d);	  
		}
		else
		{
		 	AX_12_Syn_Ctrl(0,left_current_q_6d, qdt_current_buf6d);	  
		}

		// update pos	  
		left_current_q_6d[0] = target_q_6d[0];  
		left_current_q_6d[1] = target_q_6d[1];  
		left_current_q_6d[2] = target_q_6d[2];  
		left_current_q_6d[3] = target_q_6d[3];  
		left_current_q_6d[4] = target_q_6d[4];  
		left_current_q_6d[5] = target_q_6d[5];  
	}
	if (arm_index ==2)   // right arm
	{
		right_max_delta_pos = my_max_int_abs_6d(right_current_q_6d,target_q_6d);	// max delta distance error	
		
		if (right_max_delta_pos<=MAX_DELTA_POS)
		{
			AX_12_Syn_Ctrl(1,target_q_6d, qdt_current_buf6d);	  
		}
		else
		{
			AX_12_Syn_Ctrl(1,right_current_q_6d, qdt_current_buf6d);	  
		}
		
		// update pos	  
		right_current_q_6d[0] = target_q_6d[0]; 
		right_current_q_6d[1] = target_q_6d[1]; 
		right_current_q_6d[2] = target_q_6d[2]; 
		right_current_q_6d[3] = target_q_6d[3]; 
		right_current_q_6d[4] = target_q_6d[4]; 
		right_current_q_6d[5] = target_q_6d[5]; 
	} 
}





 /*Initial postion, 06052013*/
void InitialPosTest(void)
{	
	int32_t l_teach_pos[5],l_teach_spd[5];
	int32_t r_teach_pos[5],r_teach_spd[5];	 	
	
	// P0, Home pos
	l_teach_pos[0]=left_arm_init_motor_pos[0]; l_teach_spd[0]=50;
	l_teach_pos[1]=left_arm_init_motor_pos[1]; l_teach_spd[1]=50;
	l_teach_pos[2]=left_arm_init_motor_pos[2]; l_teach_spd[2]=50;
	l_teach_pos[3]=left_arm_init_motor_pos[3]; l_teach_spd[3]=50;
	l_teach_pos[4]=left_arm_init_motor_pos[4]; l_teach_spd[4]=50;
    AX_12_Syn_Ctrl(0,l_teach_pos,l_teach_spd); 

	r_teach_pos[0]=rigid_arm_init_motor_pos[0]; r_teach_spd[0]=50;
	r_teach_pos[1]=rigid_arm_init_motor_pos[1]; r_teach_spd[1]=50;
	r_teach_pos[2]=rigid_arm_init_motor_pos[2]; r_teach_spd[2]=50;
	r_teach_pos[3]=rigid_arm_init_motor_pos[3]; r_teach_spd[3]=50; 
	r_teach_pos[4]=rigid_arm_init_motor_pos[4]; r_teach_spd[4]=50; 
	AX_12_Syn_Ctrl(1,r_teach_pos,r_teach_spd);		
}

/*Initial postion, 06052013*/
void InitialPos(void)
{	
	int32_t l_teach_pos[5],l_teach_spd[5];
	int32_t r_teach_pos[5],r_teach_spd[5];	 	
	
	// P0, Home pos
	l_teach_pos[0]=left_arm_init_motor_pos[0]; l_teach_spd[0]=50;
	l_teach_pos[1]=left_arm_init_motor_pos[1]; l_teach_spd[1]=50;
	l_teach_pos[2]=left_arm_init_motor_pos[2]; l_teach_spd[2]=50;
	l_teach_pos[3]=left_arm_init_motor_pos[3]; l_teach_spd[3]=50;
	l_teach_pos[4]=left_arm_init_motor_pos[4]; l_teach_spd[4]=50;
  AX_12_Syn_Ctrl_5DOF_Rigid_Arm(1,l_teach_pos,l_teach_spd); 

	r_teach_pos[0]=rigid_arm_init_motor_pos[0]; r_teach_spd[0]=50;
	r_teach_pos[1]=rigid_arm_init_motor_pos[1]; r_teach_spd[1]=50;
	r_teach_pos[2]=rigid_arm_init_motor_pos[2]; r_teach_spd[2]=50;
	r_teach_pos[3]=rigid_arm_init_motor_pos[3]; r_teach_spd[3]=50;
	r_teach_pos[4]=rigid_arm_init_motor_pos[4]; r_teach_spd[4]=50; 
	AX_12_Syn_Ctrl_5DOF_Rigid_Arm(2,r_teach_pos,r_teach_spd);		
}







/*Homing, 06052013*/
void SingleArmHoming(uint8_t arm_index)
{	
	int32_t l_teach_pos[5],l_teach_spd[5];
	int32_t r_teach_pos[5],r_teach_spd[5];	 	
	
	// left arm homing
	if(arm_index == 1)
	{
		// P0, Home pos
		l_teach_pos[0]=left_arm_home_motor_pos[0]; l_teach_spd[0]=50;
		l_teach_pos[1]=left_arm_home_motor_pos[1]; l_teach_spd[1]=50;
		l_teach_pos[2]=left_arm_home_motor_pos[2]; l_teach_spd[2]=50;
		l_teach_pos[3]=left_arm_home_motor_pos[3]; l_teach_spd[3]=50;
		l_teach_pos[4]=left_arm_home_motor_pos[4]; l_teach_spd[4]=50;
		AX_12_Syn_Ctrl_5DOF_Rigid_Arm(1,l_teach_pos,l_teach_spd);	
	}	
	
	// right arm homing
	if(arm_index == 2)
	{	
		// P0, Home pos
		r_teach_pos[0]=right_arm_home_motor_pos[0]; r_teach_spd[0]=50;
		r_teach_pos[1]=right_arm_home_motor_pos[1]; r_teach_spd[1]=50;
		r_teach_pos[2]=right_arm_home_motor_pos[2]; r_teach_spd[2]=50;
		r_teach_pos[3]=right_arm_home_motor_pos[3]; r_teach_spd[3]=50;
		r_teach_pos[4]=right_arm_home_motor_pos[4]; r_teach_spd[4]=50; 
		AX_12_Syn_Ctrl_5DOF_Rigid_Arm(2,r_teach_pos,r_teach_spd);			
	}	
}




/*Homing, 06052013*/
void HomingFast(void)
{	
	int32_t l_teach_pos[5],l_teach_spd[5];
	int32_t r_teach_pos[5],r_teach_spd[5];	 	
	
	// P0, Home pos
	l_teach_pos[0]=left_arm_home_motor_pos[0]; l_teach_spd[0]=100;
	l_teach_pos[1]=left_arm_home_motor_pos[1]; l_teach_spd[1]=100;
	l_teach_pos[2]=left_arm_home_motor_pos[2]; l_teach_spd[2]=100;
	l_teach_pos[3]=left_arm_home_motor_pos[3]; l_teach_spd[3]=100;
	l_teach_pos[4]=left_arm_home_motor_pos[4]; l_teach_spd[4]=100;
    AX_12_Syn_Ctrl_5DOF_Rigid_Arm(1,l_teach_pos,l_teach_spd); 

	r_teach_pos[0]=right_arm_home_motor_pos[0]; r_teach_spd[0]=100;
	r_teach_pos[1]=right_arm_home_motor_pos[1]; r_teach_spd[1]=100;
	r_teach_pos[2]=right_arm_home_motor_pos[2]; r_teach_spd[2]=100;
	r_teach_pos[3]=right_arm_home_motor_pos[3]; r_teach_spd[3]=100;
	r_teach_pos[4]=right_arm_home_motor_pos[4]; r_teach_spd[4]=100; 
	AX_12_Syn_Ctrl_5DOF_Rigid_Arm(2,r_teach_pos,r_teach_spd);		
}


/*infltableHandMotion, 11/06/2013*/
void infltableHandMotionCtrl(uint8_t pneumatic_enable, uint8_t arm_index)
{
	uint8_t i;	

	/*PID pump control*/
	if (pneumatic_enable==1) 
	{
		/* commands */
		switch (arm_index)
		{
			case 1:       // left plam
				/*currently add thumb and forefinger control, 11072013*/
				/*GPIO_ResetBits(GPIOD, GPIO_Pin_4);  //left thumb pump off
				GPIO_ResetBits(GPIOD, GPIO_Pin_6);  //left thumb valve off	
				GPIO_ResetBits(GPIOD, GPIO_Pin_8);  //left forefinger valve off
				GPIO_ResetBits(GPIOD, GPIO_Pin_10);  //left forefinger valve off*/

				// 11282013 based on new pcb config
				GPIO_SetBits(GPIOG, GPIO_Pin_2);  //left thumb pump on
				GPIO_ResetBits(GPIOG, GPIO_Pin_5);  //left thumb valve off 


				// version1
				for(i=0;i<5;i++)
				{
					left_hand_pump_pwm_duty[i] = pid_update(left_plam_desired_press[i],left_hand_actual_AD_value[i], 100,&pid_per); //left thumb
				}
				
				// version2, 11072013
				/*for(i=0;i<5;i++)
				{
					left_hand_pump_pwm_duty[i]=0;// stop
				}*/					
				break;
			case 2:	      // left fist
				/*currently add thumb and forefinger control, 11072013*/
				/*GPIO_SetBits(GPIOD, GPIO_Pin_4);  //left thumb pump on
				GPIO_SetBits(GPIOD, GPIO_Pin_6);  //left thumb valve on  
				GPIO_SetBits(GPIOD, GPIO_Pin_8);  //left forefinger valve on
				GPIO_SetBits(GPIOD, GPIO_Pin_10);  //left forefinger valve on*/ 

				// 11282013 based on new pcb config 
			   	GPIO_SetBits(GPIOG, GPIO_Pin_2);  //left thumb pump on
				GPIO_SetBits(GPIOG, GPIO_Pin_5);  //left thumb valve on 


				for(i=0;i<5;i++)
				{
					left_hand_pump_pwm_duty[i] = pid_update(left_fist_desired_press[i],left_hand_actual_AD_value[i], 100,&pid_per); //left thumb
				}
				break;
			case 5: 	  // without left plam	
				/*currently add thumb and forefinger control, 11072013*/
				/*GPIO_ResetBits(GPIOD, GPIO_Pin_4);  //left thumb pump off
				GPIO_ResetBits(GPIOD, GPIO_Pin_6);  //left thumb valve off
				GPIO_ResetBits(GPIOD, GPIO_Pin_8);  //left forefinger valve off
				GPIO_ResetBits(GPIOD, GPIO_Pin_10);  //left forefinger valve off*/

				// 11282013 based on new pcb config
				GPIO_ResetBits(GPIOG, GPIO_Pin_2);  //left thumb pump off
				GPIO_ResetBits(GPIOG, GPIO_Pin_5);  //left thumb valve off
				

				for(i=0;i<5;i++)
				{
					left_hand_pump_pwm_duty[i]=0;// stop
				}		
				break;
			case 3: 	  // right plam
				/*currently add thumb and forefinger control, 11072013*/
				/*GPIO_ResetBits(GPIOD, GPIO_Pin_5);  //right thumb pump off
				GPIO_ResetBits(GPIOD, GPIO_Pin_7);  //right thumb valve off
				GPIO_ResetBits(GPIOD, GPIO_Pin_9);  //right forefinger valve off
				GPIO_ResetBits(GPIOD, GPIO_Pin_11);  //right forefinger valve off*/

				// 11282013 based on new pcb config
			    //GPIO_ResetBits(GPIOG, GPIO_Pin_3);  //right thumb pump off, old version
			    GPIO_SetBits(GPIOG, GPIO_Pin_3);  //right thumb pump on
				GPIO_ResetBits(GPIOG, GPIO_Pin_4);  //right thumb valve off
			


				// version1
				for(i=0;i<5;i++)
				{
					right_hand_pump_pwm_duty[i] = pid_update(right_plam_desired_press[i],right_hand_actual_AD_value[i], 100,&pid_per); //left thumb
				}
				// version2	
				/*for(i=0;i<5;i++)
				{
					right_hand_pump_pwm_duty[i]=0;// stop
				}*/				
				break;
			case 4: 	  // right fist
				/*currently add thumb and forefinger control, 11072013*/
				/*GPIO_SetBits(GPIOD, GPIO_Pin_5);  //right thumb pump on
				GPIO_SetBits(GPIOD, GPIO_Pin_7);  //right thumb valve on
				GPIO_SetBits(GPIOD, GPIO_Pin_9);  //right forefinger valve on
				GPIO_SetBits(GPIOD, GPIO_Pin_11);  //right forefinger valve on*/
				
				// 11282013 based on new pcb config
				GPIO_SetBits(GPIOG, GPIO_Pin_3);  //right thumb pump on
				GPIO_SetBits(GPIOG, GPIO_Pin_4);  //right thumb valve on					


				for(i=0;i<5;i++)
				{
					right_hand_pump_pwm_duty[i] = pid_update(right_fist_desired_press[i],right_hand_actual_AD_value[i], 100,&pid_per); //left thumb
				}
				break;
			case 6:       // without right plam
				/*currently add thumb and forefinger control, 11072013*/
				/*8GPIO_ResetBits(GPIOD, GPIO_Pin_5);  //right thumb pump off
				GPIO_ResetBits(GPIOD, GPIO_Pin_7);  //right thumb valve off
				GPIO_ResetBits(GPIOD, GPIO_Pin_9);  //right forefinger valve off
				GPIO_ResetBits(GPIOD, GPIO_Pin_11);  //right forefinger valve off*/

				// 11282013 based on new pcb config
				GPIO_ResetBits(GPIOG, GPIO_Pin_3);  //right thumb pump off
				GPIO_ResetBits(GPIOG, GPIO_Pin_4);  //right thumb valve off


				for(i=0;i<5;i++)
				{
					right_hand_pump_pwm_duty[i]=0;// stop
				}
				break;				
			default: 				
				break;
		} 
	}
	else
	{
		/*currently add thumb and forefinger control, 11072013*/
		/*GPIO_ResetBits(GPIOD, GPIO_Pin_4);  //left thumb pump off
		GPIO_ResetBits(GPIOD, GPIO_Pin_6);  //left thumb valve off	
		GPIO_ResetBits(GPIOD, GPIO_Pin_8);  //left forefinger valve off
		GPIO_ResetBits(GPIOD, GPIO_Pin_10);  //left forefinger valve off
		
		GPIO_ResetBits(GPIOD, GPIO_Pin_5);  //right thumb pump off
		GPIO_ResetBits(GPIOD, GPIO_Pin_7);  //right thumb valve off
		GPIO_ResetBits(GPIOD, GPIO_Pin_9);  //right forefinger valve off
		GPIO_ResetBits(GPIOD, GPIO_Pin_11);  //right forefinger valve off*/

		// 11282013 based on new pcb config
		GPIO_ResetBits(GPIOG, GPIO_Pin_2);  //left thumb pump off
		GPIO_ResetBits(GPIOG, GPIO_Pin_5);  //left thumb valve off

		GPIO_ResetBits(GPIOG, GPIO_Pin_3);  //right thumb pump off
		GPIO_ResetBits(GPIOG, GPIO_Pin_4);  //right thumb valve off


		for(i=0;i<5;i++)
		{
			left_hand_pump_pwm_duty[i]=0;// stop
			right_hand_pump_pwm_duty[i]=0;// stop
		}
	}
}


/*infltableHandMotion, 11/06/2013*/
void infltableFingerMotionCtrl(uint8_t hand_index, uint8_t finger_index, int8_t motion_direction)
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
					left_hand_pump_pwm_duty[finger_index-1] = pid_update(left_fist_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else if (motion_direction==-1) //thumb
				{
					GPIO_SetBits(GPIOG, GPIO_Pin_2);    //左大拇指气泵 ON
					GPIO_ResetBits(GPIOG, GPIO_Pin_5);  //左大拇指气阀 OFF 
					left_hand_pump_pwm_duty[finger_index-1] = pid_update(left_plam_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
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
					left_hand_pump_pwm_duty[finger_index-1] = pid_update(left_fist_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else if (motion_direction==-1) //thumb
				{
					GPIO_SetBits(GPIOG, GPIO_Pin_15);  //左食指气泵	ON
					GPIO_ResetBits(GPIOB, GPIO_Pin_5);  //左食指气阀 OFF   
					left_hand_pump_pwm_duty[finger_index-1] = pid_update(left_plam_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
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
					left_hand_pump_pwm_duty[finger_index-1] = pid_update(left_fist_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
					//left_hand_pump_pwm_duty[finger_index-1] = pid_update(left_fist_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); // 01/24/2014 test
				}
				else if (motion_direction==-1) //thumb
				{
					GPIO_SetBits(GPIOE, GPIO_Pin_0);  //左中指气泵	ON
					GPIO_ResetBits(GPIOC, GPIO_Pin_13);  //左中指气阀	OFF 
					left_hand_pump_pwm_duty[finger_index-1] = pid_update(left_plam_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
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
					left_hand_pump_pwm_duty[finger_index-1] = pid_update(left_fist_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else if (motion_direction==-1) //thumb
				{
					GPIO_SetBits(GPIOF, GPIO_Pin_0);  //左无名指气泵	ON
					GPIO_ResetBits(GPIOF, GPIO_Pin_3);  //左无名指气阀	OFF
					left_hand_pump_pwm_duty[finger_index-1] = pid_update(left_plam_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
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
					left_hand_pump_pwm_duty[finger_index-1] = pid_update(left_fist_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else if (motion_direction==-1) //thumb
				{
					GPIO_SetBits(GPIOF, GPIO_Pin_11);  //左小指气泵	ON
					GPIO_ResetBits(GPIOF, GPIO_Pin_14);  //左小指气阀	OFF
					left_hand_pump_pwm_duty[finger_index-1] = pid_update(left_plam_desired_press[finger_index-1],left_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
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
					right_hand_pump_pwm_duty[finger_index-1] = pid_update(right_fist_desired_press[finger_index-1],right_hand_actual_AD_value[finger_index-1], 100,&pid_per); //right thumb
				}
				else if (motion_direction==-1) //thumb
				{
					GPIO_SetBits(GPIOG, GPIO_Pin_3); //右大拇指气泵 ON
					GPIO_ResetBits(GPIOG, GPIO_Pin_4);  //右大拇指气阀 OFF 
					right_hand_pump_pwm_duty[finger_index-1] = pid_update(right_plam_desired_press[finger_index-1],right_hand_actual_AD_value[finger_index-1], 100,&pid_per); //right thumb
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
					right_hand_pump_pwm_duty[finger_index-1] = pid_update(right_fist_desired_press[finger_index-1],right_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else if (motion_direction==-1) //thumb
				{
					GPIO_SetBits(GPIOB, GPIO_Pin_3); //右食指气泵 ON
					GPIO_ResetBits(GPIOB, GPIO_Pin_4);  //右食指气阀 OFF
					right_hand_pump_pwm_duty[finger_index-1] = pid_update(right_plam_desired_press[finger_index-1],right_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
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
					right_hand_pump_pwm_duty[finger_index-1] = pid_update(right_fist_desired_press[finger_index-1],right_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else if (motion_direction==-1) //thumb
				{
					GPIO_SetBits(GPIOE, GPIO_Pin_1); //右中指气泵 ON
					GPIO_ResetBits(GPIOE, GPIO_Pin_6);  //右中指气阀 OFF
					right_hand_pump_pwm_duty[finger_index-1] = pid_update(right_plam_desired_press[finger_index-1],right_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
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
					right_hand_pump_pwm_duty[finger_index-1] = pid_update(right_fist_desired_press[finger_index-1],right_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else if (motion_direction==-1) //thumb
				{
					GPIO_SetBits(GPIOF, GPIO_Pin_1); //右无名指气泵 ON
					GPIO_ResetBits(GPIOF, GPIO_Pin_2);  //右无名指气阀 OFF
					right_hand_pump_pwm_duty[finger_index-1] = pid_update(right_plam_desired_press[finger_index-1],right_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
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
					right_hand_pump_pwm_duty[finger_index-1] = pid_update(right_fist_desired_press[finger_index-1],right_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
				}
				else if (motion_direction==-1) //thumb
				{
					GPIO_SetBits(GPIOF, GPIO_Pin_12); //右小指气泵 ON
					GPIO_ResetBits(GPIOF, GPIO_Pin_13);  //右小指气阀 OFF
					right_hand_pump_pwm_duty[finger_index-1] = pid_update(right_plam_desired_press[finger_index-1],right_hand_actual_AD_value[finger_index-1], 100,&pid_per); //left thumb
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
				infltableFingerMotionCtrl(hand_index, 1, -1); //12/18/2103, ok			
				infltableFingerMotionCtrl(hand_index, 2, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1); //12/18/2103, ok				
				break; 
			case 2: // only thumb, but not control others
				infltableFingerMotionCtrl(hand_index, 1, 1); //12/18/2103, ok
				break;	
			case 3: // only forefinger, but not control others			
				infltableFingerMotionCtrl(hand_index, 2, 1); //12/18/2103, ok				
				break;	
			case 4: // only middle finger, but not control others
				infltableFingerMotionCtrl(hand_index, 3, 1); //12/18/2103, ok
				break;	
			case 5: // only ring finger, but not control others
				infltableFingerMotionCtrl(hand_index, 4, 1); //12/18/2103, ok
				break;
			case 6: // only ring finger, but not control others
				infltableFingerMotionCtrl(hand_index, 5, 1); //12/18/2103, ok
				break; 
			case 7: // only thumb
				infltableFingerMotionCtrl(hand_index, 1, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1); //12/18/2103, ok
				break;	
			case 8: // only forefinger
				infltableFingerMotionCtrl(hand_index, 1, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1); //12/18/2103, ok
				break;	
			case 9: // only middle finger
				infltableFingerMotionCtrl(hand_index, 1, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1); //12/18/2103, ok
				break;	
			case 10: // only ring finger
				infltableFingerMotionCtrl(hand_index, 1, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1); //12/18/2103, ok
				break;
			case 11: // only ring finger
				infltableFingerMotionCtrl(hand_index, 1, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1); //12/18/2103, ok
				break;
			case 12: // finger 1,2 enable, means "ok"
				infltableFingerMotionCtrl(hand_index, 1, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1); //12/18/2103, ok
				break;
			case 13: // finger 1,3 enable
				infltableFingerMotionCtrl(hand_index, 1, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1); //12/18/2103, ok
				break;
			case 14: // finger 1,4 enable
				infltableFingerMotionCtrl(hand_index, 1, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1); //12/18/2103, ok
				break;
			case 15: // finger 1,5 enable
				infltableFingerMotionCtrl(hand_index, 1, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1); //12/18/2103, ok
				break;
			case 16: // finger 2,3 enable
				infltableFingerMotionCtrl(hand_index, 1, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1); //12/18/2103, ok
				break;
			case 17: // finger 2,4 enable
				infltableFingerMotionCtrl(hand_index, 1, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1); //12/18/2103, ok*/
				infltableFingerMotionCtrl(hand_index, 4, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1); //12/18/2103, ok
				break;
			case 18: // finger 2,5 enable
				infltableFingerMotionCtrl(hand_index, 1, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1); //12/18/2103, ok
				break;
			case 19: // finger 3,4 enable
				infltableFingerMotionCtrl(hand_index, 1, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1); //12/18/2103, ok
				break;
			case 20: // finger 3,5 enable
				infltableFingerMotionCtrl(hand_index, 1, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1); //12/18/2103, ok
				break;
			case 21: // finger 4,5 enable
				infltableFingerMotionCtrl(hand_index, 1, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1); //12/18/2103, ok
				break;
			case 22: // finger 1,2,3 enable
				infltableFingerMotionCtrl(hand_index, 1, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1); //12/18/2103, ok
				break; 
			case 23: // finger 1,2,4 enable
				infltableFingerMotionCtrl(hand_index, 1, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1); //12/18/2103, ok
				break;	
			case 24: // finger 1,2,5 enable
				infltableFingerMotionCtrl(hand_index, 1, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1); //12/18/2103, ok
				break;
			case 25: // finger 1,3,4 enable
				infltableFingerMotionCtrl(hand_index, 1, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1); //12/18/2103, ok
				break;
			case 26: // finger 1,3,5 enable
				infltableFingerMotionCtrl(hand_index, 1, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1); //12/18/2103, ok
				break;
			case 27: // finger 1,4,5 enable, means "victory"
				infltableFingerMotionCtrl(hand_index, 1, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1); //12/18/2103, ok
				break;
			case 28: // finger 2,3,4 enable
				infltableFingerMotionCtrl(hand_index, 1, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1); //12/18/2103, ok
				break;
			case 29: // finger 2,3,5 enable
				infltableFingerMotionCtrl(hand_index, 1, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1); //12/18/2103, ok
				break;
			case 30: // finger 2,4,5 enable
				infltableFingerMotionCtrl(hand_index, 1, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1); //12/18/2103, ok
				break;
			case 31: // finger 3,4,5 enable
				infltableFingerMotionCtrl(hand_index, 1, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1); //12/18/2103, ok
				break;
			case 32: // finger 1,2,3,4 enable
				infltableFingerMotionCtrl(hand_index, 1, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, -1); //12/18/2103, ok
				break;
			case 33: // finger 1,2,3,5 enable
				infltableFingerMotionCtrl(hand_index, 1, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1); //12/18/2103, ok
				break;			
			case 34: // finger 1,2,4,5 enable, means "this is an impolite pose"
				infltableFingerMotionCtrl(hand_index, 1, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1); //12/18/2103, ok
				break;			
			case 35: // finger 1,3,4,5 enable, means "win"
				infltableFingerMotionCtrl(hand_index, 1, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1); //12/18/2103, ok
				break;			
			case 36: // finger 2,3,4,5 enable, means "good"
				infltableFingerMotionCtrl(hand_index, 1, -1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 2, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1); //12/18/2103, ok
				break;			
		  case 37: // finger 1, 2,3,4,5 all enable, means "fist"
				infltableFingerMotionCtrl(hand_index, 1, 1); //12/18/2103, ok			
				infltableFingerMotionCtrl(hand_index, 2, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 3, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 4, 1); //12/18/2103, ok
				infltableFingerMotionCtrl(hand_index, 5, 1); //12/18/2103, ok
				break; 	 							
			default: 				
				break;
		} 
	}
	else
	{		
		if ((hand_index==1)||(hand_index==2)) // stop current hand pose
		{
			infltableFingerMotionCtrl(hand_index, 1, 0); //12/18/2103, ok
			infltableFingerMotionCtrl(hand_index, 2, 0); //12/18/2103, ok
			infltableFingerMotionCtrl(hand_index, 3, 0); //12/18/2103, ok
			infltableFingerMotionCtrl(hand_index, 4, 0); //12/18/2103, ok
			infltableFingerMotionCtrl(hand_index, 5, 0); //12/18/2103, ok
		}
		else  // stop all hand poses
		{
		  // left hand
			infltableFingerMotionCtrl(1, 1, 0); //12/18/2103, ok
			infltableFingerMotionCtrl(1, 2, 0); //12/18/2103, ok
			infltableFingerMotionCtrl(1, 3, 0); //12/18/2103, ok
			infltableFingerMotionCtrl(1, 4, 0); //12/18/2103, ok
			infltableFingerMotionCtrl(1, 5, 0); //12/18/2103, ok
			// right hand
			infltableFingerMotionCtrl(2, 1, 0); //12/18/2103, ok
			infltableFingerMotionCtrl(2, 2, 0); //12/18/2103, ok
			infltableFingerMotionCtrl(2, 3, 0); //12/18/2103, ok
			infltableFingerMotionCtrl(2, 4, 0); //12/18/2103, ok
			infltableFingerMotionCtrl(2, 5, 0); //12/18/2103, ok		
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
				//AX_12_Torque_Enable(7, 0);  
				//AX_12_Torque_Enable(8, 0);  
				//AX_12_Torque_Enable(9, 0);  
				//AX_12_Torque_Enable(10, 0); 

				Delay_ms(500); // exactly delay, ms
				/*hand pose control*/
				CAN2_RX_Hand_Pose_Ctrl(2,0); // left fist	
			}
			else 
			{
				AX_12_Torque_Enable(254, 1); // servo generates torque 
				//AX_12_Torque_Enable(7, 1);  
				//AX_12_Torque_Enable(8, 1);  
				//AX_12_Torque_Enable(9, 1);  
				//AX_12_Torque_Enable(10, 1); 	

				//Delay_ms(500); // exactly delay, ms
				/*hand pose control*/
				CAN2_RX_Hand_Pose_Ctrl(5,0); // without left plam	
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
				//AX_12_Torque_Enable(2, 0);  
				//AX_12_Torque_Enable(3, 0);  
				//AX_12_Torque_Enable(4, 0);  
				//AX_12_Torque_Enable(5, 0);  
				
				Delay_ms(500); // exactly delay, ms
				/*hand pose control*/
				CAN2_RX_Hand_Pose_Ctrl(4,0); // right fist		
			}
			else 
			{
				AX_12_Torque_Enable(254, 1); // servo generates torque 
				//AX_12_Torque_Enable(2, 1);  
				//AX_12_Torque_Enable(3, 1);  
				//AX_12_Torque_Enable(4, 1);  
				//AX_12_Torque_Enable(5, 1); 	

				//Delay_ms(500); // exactly delay, ms
				/*hand pose control*/
				CAN2_RX_Hand_Pose_Ctrl(6,0); // right fist
				Delay_ms(1000); // exactly delay, ms	
				//SingleArmHoming(2); // right arm homing				
			}
		}	
}




/*hand pose, 12/18/2013*/
/*void infltableHandPoseMotionPlanning(uint8_t pneumatic_enable, uint8_t hand_index, uint8_t pose_index, uint32_t delay_ms)
{	
	infltableHandPose(pneumatic_enable, hand_index, pose_index);	
	
}*/

uint8_t testNot1(int32_t XDec)
{
	uint8_t i,result;

	// old version
	/*if (XDec>255 && XDec<512)
	{
	    XDec=XDec-256;
	}
	else if (XDec>511 && XDec<768)
	{
	    XDec=XDec-512;
	}
	else if (XDec>767 && XDec<1024)
	{
	    XDec=XDec-768;
	}*/

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

	// old version
	/*// get low value
 	if (XDec<=255)
	{
	    //XDec=XDec-256;	  //before 052012013
		XDec=XDec;	  //052012013
	}	
	else if (XDec>255 && XDec<512)
	{
	    XDec=XDec-256;
	}
	else if (XDec>511 && XDec<768)
	{
	    XDec=XDec-512;
	}
	else if (XDec>767 && XDec<1024)
	{
	    XDec=XDec-768;
	}
	result=XDec; */	
}

//XDec2High, 05222013
uint8_t XDec2High(int32_t XDec)  
{		
	uint8_t result;	 	
		
	// get high value
	result=(int)(XDec/256); // important here, 05222013
	
	return result;
}

 
// two links verify 04112013, v2
void kinematics_2dof(uint8_t arm_index, int32_t px, int32_t pz, uint8_t s_index) //1x input, resulotion 1mm		
{
	float theta2_1,theta2_2,theta4_1,theta4_2; 	
	float A,B;
	uint32_t v_temp1,v_temp2,v_temp3,v_temp4;
	int32_t v_temp5;
	float v_temp6;
	float fi,pai_1,pai_2;
	float v_temp7_1,v_temp7_2,v_temp8_1,v_temp8_2;	
	

	// parameters
	uint16_t l1=220; 	
	uint16_t l2=220;
	uint16_t l3=0; // tool
	uint16_t le=l2+l3;
	

	// STEP 1: get  theta4_1,theta4_2
	v_temp1 = l1+le;
	v_temp1 = v_temp1*v_temp1; 
	v_temp2 = px*px;
	v_temp3 = pz*pz;	 
	v_temp4 = v_temp2+v_temp3;  
	A = (float)(v_temp1 - v_temp4); 	
	v_temp5 = l1-le;
	v_temp5 = v_temp5*v_temp5;
	B = (float)(v_temp4-v_temp5);	
	v_temp6 = (float)(A/B);	 // attention to this, may lead error, 04302013 
	v_temp6 = (float)my_sqrt(v_temp6);	
	// solution 1
	theta4_1 = (float)atanf(v_temp6);
	theta4_1 = theta4_1*2; 
	// solution 2
	theta4_2 = -(float)atanf(v_temp6); 
	theta4_2 = theta4_2*2;
	

	// STEP 2: get  theta2_1,theta2_2
	fi=(float)atan2f(pz,px);
	// solution 1	 
	v_temp7_1 = (float)sinf(theta4_1);
	v_temp7_1 = v_temp7_1*le;
	v_temp8_1 = (float)cosf(theta4_1);
    v_temp8_1 =  v_temp8_1*le;
	v_temp8_1 = v_temp8_1 + l1;
	pai_1 = (float)atan2f(v_temp7_1,v_temp8_1);
	theta2_1 = 	fi-	pai_1;

	// solution 2
	v_temp7_2 = (float)sinf(theta4_2);
	v_temp7_2 = v_temp7_2*le;
	v_temp8_2= (float)cosf(theta4_2);
    v_temp8_2 =  v_temp8_2*le;
	v_temp8_2 = v_temp8_2 + l1;
	pai_2 = (float)atan2f(v_temp7_2,v_temp8_2);
	theta2_2 = 	fi-	pai_2; 


	/* select solutions */
	switch (s_index)
	{
		case 1: //solution 1, better
			theta_solution[0]=0; 
			theta_solution[1]=theta2_1; 
			theta_solution[2]=0; 
			theta_solution[3]=theta4_1;  
			break;
		case 2: //solution 2
			theta_solution[0]=0; 
			theta_solution[1]=theta2_2; 
			theta_solution[2]=0; 
			theta_solution[3]=theta4_2; 
			break;
		default: 				
			break;
	} 
  

	/*printf("*************\r\n");
	printf("px = %d\r\n", px);
	printf("pz = %d\r\n", pz); */	
	/*theta_2_deg_test = theta_solution[1]*57;	
	theta_4_deg_test = 	theta_solution[3]*57;			
	printf("theta_2_deg_test = %4.2f \r\n", theta_2_deg_test);	
	printf("theta_4_deg_test = %4.2f \r\n", theta_4_deg_test);	
	printf("v_temp1 = %d \r\n", v_temp1);
	printf("v_temp2 = %d \r\n", v_temp2);
	printf("v_temp3 = %d \r\n", v_temp3);
	printf("v_temp4 = %d \r\n", v_temp4);	
	printf("v_temp5 = %d \r\n", v_temp5);
	printf("A = %4.2f \r\n", A);
	printf("B = %4.2f \r\n", B); 
	printf("v_temp6 = %4.4f \r\n", v_temp6);
	printf("theta[1] = %4.2f \r\n", theta_solution[1]);	
	printf("theta[3] = %4.2f \r\n", theta_solution[3]);*/
}





/*fixed theta3 inverse kinematics, by rhqi 06272013*/
uint8_t ikine_theta3f(uint8_t arm_index, int32_t px, int32_t py, int32_t pz, float theta3f, uint8_t s_index)		
{	
	// 4dof, new added
	float theta1,theta3;
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


	// parameters
	//float l1=220.0; 	
	//float l2=220.0;
	float l1=170.0; 	
	float l2=210.0;	  
	float l3=0.0; // tool
	float le=l2+l3;
	float theta_error = 0.01; // important vs matlab

	// Max and Min limited
	//finally used
	theta1_max=(float)(pi/3) + theta_error;
	theta1_min=ZERO_f - theta_error;  
	theta2_max=ZERO_f + theta_error;
	theta2_min=(float)(-pi/2) - theta_error;
	theta3_max=(float)(pi/3) + theta_error;
	theta3_min=(float)(-pi/6) - theta_error;
	theta4_max=(float)(7*pi/18) + theta_error;	
	theta4_min=ZERO_f - theta_error; 
	 
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
    //r = sqrt(temp_p3); //r = sqrt(px*px + py*py); 
    temp_r = d*d;
    temp_r = temp_p3 - temp_r; 
	if(temp_r<ZERO_f) //singularity 
	{
	   return exist_s;
	}
    a = (float)my_sqrt(temp_r); //a = sqrt(px*px + py*py - d*d); 
	/*if((my_abs(d)<=EPSILON)&&(my_abs(a)<=EPSILON)) //singularity,//07-11-2013 new note: this is unnecessary, as system has defined atan2f(0,0)=0
	{
	   return exist_s;
	}*/ 	
    beta = (float)atan2f(d,a); //beta = atan(d/a); //old
	/*if((my_abs(py)<=EPSILON)&&(my_abs(px)<=EPSILON)) //singularity,//07-11-2013 new note: this is unnecessary, as system has defined atan2f(0,0)=0
	{
	   return exist_s;
	}*/
    psi = (float)atan2f(py,px);
    theta1 = psi-beta;   
    

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
	/*if((my_abs(pz_2)<=EPSILON)&&(my_abs(px_2)<=EPSILON)) //singularity,//07-11-2013 new note: this is unnecessary, as system has defined atan2f(0,0)=0
	{
		return exist_s;
	}*/
	fi=(float)atan2f(pz_2,px_2);
	// solution 1	 
	v_temp7_1 = (float)sinf(theta4_1);
	v_temp7_1 = v_temp7_1*le_;
	v_temp8_1 = (float)cosf(theta4_1);
    v_temp8_1 =  v_temp8_1*le_;
	v_temp8_1 = v_temp8_1 + l1;
	/*if((my_abs(v_temp7_1)<=EPSILON)&&(my_abs(v_temp8_1)<=EPSILON)) //singularity,//07-11-2013 new note: this is unnecessary, as system has defined atan2f(0,0)=0
	{
		return exist_s;
	}*/
	pai_1 = (float)atan2f(v_temp7_1,v_temp8_1);
	theta2_1 = 	fi-	pai_1;	
	// solution 2
	v_temp7_2 = (float)sinf(theta4_2);
	v_temp7_2 = v_temp7_2*le_;
	v_temp8_2= (float)cosf(theta4_2);
    v_temp8_2 =  v_temp8_2*le_;
	v_temp8_2 = v_temp8_2 + l1;
	/*if((my_abs(v_temp7_2)<=EPSILON)&&(my_abs(v_temp8_2)<=EPSILON)) //singularity,//07-11-2013 new note: this is unnecessary, as system has defined atan2f(0,0)=0
	{
		return exist_s;
	}*/
	pai_2 = (float)atan2f(v_temp7_2,v_temp8_2);
	theta2_2 = 	fi-	pai_2; 

 
	//all solutions 
    solution[0][0]=theta1; solution[0][1]=theta2_1; solution[0][2]=theta3; solution[0][3]=theta4_1;
	solution[1][0]=theta1; solution[1][1]=theta2_2; solution[1][2]=theta3; solution[1][3]=theta4_2;
   
	
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
				theta3f_solution[0]= solution[0][0];
				theta3f_solution[1]= solution[0][1];
				theta3f_solution[2]= solution[0][2];
				theta3f_solution[3]= solution[0][3]; 		
			}
			else
			{
				theta3f_solution[0]= solution[1][0];
				theta3f_solution[1]= solution[1][1];
				theta3f_solution[2]= solution[1][2];
				theta3f_solution[3]= solution[1][3];		
			}
			break;		
		case 2: // exist two solutions 	
			theta3f_solution[0]= solution[s_index][0];
			theta3f_solution[1]= solution[s_index][1];
			theta3f_solution[2]= solution[s_index][2];
			theta3f_solution[3]= solution[s_index][3];
			break;	
		default: 				
			break;
	}



	/*debug*//*
	printf("***theta3f***\r\n"); 
	printf("px = %d\r\n", px);
	printf("py = %d\r\n", py);
	printf("pz = %d\r\n", pz);							
	printf("theta1 = %4.4f \r\n", theta3f_solution[0]);
	printf("theta2 = %4.4f \r\n", theta3f_solution[1]);
	printf("theta3 = %4.4f \r\n", theta3f_solution[2]);
	printf("theta4 = %4.4f \r\n", theta3f_solution[3]);
	printf("exist_theta3f_s = %d\r\n", exist_s);*/


	return exist_s;
}


/*fixed theta4 inverse kinematics, by rhqi 06272013*/
uint8_t ikine_theta4f(uint8_t arm_index, int32_t px_in, int32_t py_in, int32_t pz_in, float theta4f, uint8_t s_index)		
{	
	// 4dof, new added
	int32_t px,py,pz;
	float theta2, theta4;
	float d,a,beta,psi;
	float px_2,pz_2,le_;
	float temp_p1,temp_p2,temp_p3,temp_r;
	// old 2dof used
	float theta1_1,theta1_2,theta3_1,theta3_2;	
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
	
	
	// parameters
	//float l1=220.0; 	
	//float l2=220.0;
	float l1=170.0; 	
	float l2=210.0;	 
	float l3=0.0; // tool
	float le=l2+l3;
	float theta_error = 0.01; // important vs matlab

	// Max and Min limited
	//finally used
	theta1_max=(float)(pi/3) + theta_error;
	theta1_min=ZERO_f - theta_error;  
	theta2_max=ZERO_f + theta_error;
	theta2_min=(float)(-pi/2) - theta_error;
	theta3_max=(float)(pi/3) + theta_error;
	theta3_min=(float)(-pi/6) - theta_error;
	theta4_max=(float)(7*pi/18) + theta_error;	
	theta4_min=ZERO_f - theta_error;  
	 
	theta_max[0]=theta1_max; theta_max[1]=theta2_max; theta_max[2]=theta3_max; theta_max[3]=theta4_max;
    theta_min[0]=theta1_min; theta_min[1]=theta2_min; theta_min[2]=theta3_min; theta_min[3]=theta4_min;  


	/*Initialize*/
	exist_s = 0;  
	Fsolution_check[0]=1; 
	Fsolution_check[1]=1; 

	
	/*transfer at the beginning*/
	px=px_in;py=pz_in;pz=-py_in;  // in x1,y1,z1 coordinate system; px_in,py_in,pz_in are in x0,y0,z0 coordinate system

	
    /*inverse kinematics*/
    //STEP 0: get  theta4
    theta4 = theta4f;    
    

 	//STEP 1: get  theta2    
    d = (float)sinf(theta4f); 
    d = d*le; //d = le*sin(theta4f);
    temp_p1 = px*px; 
    temp_p2 = py*py;
    temp_p3 = temp_p1+temp_p2;
    //r = sqrt(temp_p3); //r = sqrt(px*px + py*py); 
    temp_r = d*d;
    temp_r = temp_p3 - temp_r;	
	if(temp_r<ZERO_f) //singularity 
	{
	   return exist_s;
	}	
    a = (float)my_sqrt(temp_r); //a = sqrt(px*px + py*py - d*d);
	/*if((my_abs(d)<=EPSILON)&&(my_abs(a)<=EPSILON)) //singularity,//07-11-2013 new note: this is unnecessary, as system has defined atan2f(0,0)=0
	{
	   return exist_s;
	}*/    
    beta = (float)atan2f(d,a); //beta = atan(d/a); //old
	/*if((my_abs(py)<=EPSILON)&&(my_abs(px)<=EPSILON)) //singularity,//07-11-2013 new note: this is unnecessary, as system has defined atan2f(0,0)=0
	{
	   return exist_s;
	}*/
    psi = (float)atan2f(py,px);
    theta2 = psi-beta;   
    

    //STEP 2: get  theta3_1,theta3_2
    px_2 = a; //px_2 = sqrt(px_*px_ + py_*py_);
    pz_2 = pz;
    le_ = (float)cosf(theta4f); 
    le_ = le_*le;//le_ = le*cos(theta4f); 


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
	theta3_1 = (float)atanf(v_temp6);
	theta3_1 = theta3_1*2; 
	// solution 2
	theta3_2 = -(float)atanf(v_temp6); 
	theta3_2 = theta3_2*2;
	

	// STEP 2: get  theta1_1,theta1_2
	/*if((my_abs(pz_2)<=EPSILON)&&(my_abs(px_2)<=EPSILON)) //singularity,//07-11-2013 new note: this is unnecessary, as system has defined atan2f(0,0)=0
	{
		return exist_s;
	}*/	
	fi=(float)atan2f(pz_2,px_2);
	// solution 1	 
	v_temp7_1 = (float)sinf(theta3_1);
	v_temp7_1 = v_temp7_1*le_;
	v_temp8_1 = (float)cosf(theta3_1);
    v_temp8_1 =  v_temp8_1*le_;
	v_temp8_1 = v_temp8_1 + l1;	
	/*if((my_abs(v_temp7_1)<=EPSILON)&&(my_abs(v_temp8_1)<=EPSILON)) //singularity,//07-11-2013 new note: this is unnecessary, as system has defined atan2f(0,0)=0
	{
		return exist_s;
	}*/
	pai_1 = (float)atan2f(v_temp7_1,v_temp8_1);
	theta1_1 = 	fi-	pai_1;	
	// solution 2
	v_temp7_2 = (float)sinf(theta3_2);
	v_temp7_2 = v_temp7_2*le_;
	v_temp8_2= (float)cosf(theta3_2);
    v_temp8_2 =  v_temp8_2*le_;
	v_temp8_2 = v_temp8_2 + l1;	
	/*if((my_abs(v_temp7_2)<=EPSILON)&&(my_abs(v_temp8_2)<=EPSILON)) //singularity,//07-11-2013 new note: this is unnecessary, as system has defined atan2f(0,0)=0
	{
		return exist_s;
	}*/
	pai_2 = (float)atan2f(v_temp7_2,v_temp8_2);
	theta1_2 = 	fi-	pai_2; 
	
 
	//all solutions 
    solution[0][0]=-theta1_1; solution[0][1]=theta2; solution[0][2]=-theta3_1; solution[0][3]=theta4; //note negtive, better here
	solution[1][0]=-theta1_2; solution[1][1]=theta2; solution[1][2]=-theta3_2; solution[1][3]=theta4; //note negtive, better here

	
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
				theta4f_solution[0]= solution[0][0]; // note here
				theta4f_solution[1]= solution[0][1];
				theta4f_solution[2]= solution[0][2]; // note here
				theta4f_solution[3]= solution[0][3]; 		
			}
			else
			{
				theta4f_solution[0]= solution[1][0]; // note here
				theta4f_solution[1]= solution[1][1];
				theta4f_solution[2]= solution[1][2]; // note here
				theta4f_solution[3]= solution[1][3];		
			}
			break;		
		case 2: // exist two solutions 	
			theta4f_solution[0]= solution[s_index][0];	// note here
			theta4f_solution[1]= solution[s_index][1];
			theta4f_solution[2]= solution[s_index][2];	// note here
			theta4f_solution[3]= solution[s_index][3];
			break;	
		default: 				
			break;
	}



	/*debug*/
	/*printf("***theta4f***\r\n"); 
	printf("px = %d\r\n", px);
	printf("py = %d\r\n", py);
	printf("pz = %d\r\n", pz);							
	printf("theta1 = %4.4f \r\n", theta4f_solution[0]);
	printf("theta2 = %4.4f \r\n", theta4f_solution[1]);
	printf("theta3 = %4.4f \r\n", theta4f_solution[2]);
	printf("theta4 = %4.4f \r\n", theta4f_solution[3]);
	printf("exist_theta4f_s = %d\r\n", exist_s);*/

	return exist_s;
}


/* cable_456_kine_m, and is suitable for 123*/
void cable_456_kine_m(uint16_t cable_comd, uint8_t enable_compensation, float permter[5], float a3, float a4)	
{
   	float pa_4_a[3],pa_5_a[3],pa_6_a[3];
	//float pb_4_b[3],pb_5_b[3],pb_6_b[3];
	float pb_4_a[3],pb_5_a[3],pb_6_a[3];
	//float pc_4_a0[3],pc_5_a0[3],pc_6_a0[3];
	//float p0[3];
	float c3,c4,s3,s4;
	float c3_2,c4_2,s3_2,s4_2;
	float d4_v[3],d5_v[3],d6_v[3];	 
	float d4,d5,d6;
	float pc_456_vn[3];	  
	float Da,Db,Ra,Rb,Rc;

	Da = permter[0];
	Db = permter[0]; //current used
	Ra = permter[2];
	Rb = permter[2]; //current used	
	Rc = permter[4];  	
	
	
	// fixed platform
	pa_4_a[0] = Ra;
	pa_4_a[1] = 0;
	pa_4_a[2] = -Da;  
    pa_5_a[0] = -Ra/2;
	pa_5_a[1] = (float)SQRT3_DIV2*Ra; //sqrt(3)*Ra/2;
	pa_5_a[2] = -Da;   
	pa_6_a[0] = -Ra/2;
	pa_6_a[1] = -(float)SQRT3_DIV2*Ra; //-sqrt(3)*Ra/2;
	pa_6_a[2] = -Da;   	

   	// moved platform
	/*pb_4_b[0] = Rb;
	pb_4_b[1] = 0;
	pb_4_b[2] = Db;  
    pb_5_b[0] = -Rb/2;
	pb_5_b[1] = (float)SQRT3_DIV2*Rb;  //sqrt(3)*Rb/2;
	pb_5_b[2] = Db;   
	pb_6_b[0] = -Rb/2;
	pb_6_b[1] = -(float)SQRT3_DIV2*Rb; //-sqrt(3)*Rb/2;
	pb_6_b[2] = Db;*/	  

	// center platform
	/*pc_4_a0[0] = Rc;
	pc_4_a0[1] = 0;
	pc_4_a0[2] = 0;	
	pc_5_a0[0] = -Rc/2;
	pc_5_a0[1] = (float)SQRT3_DIV2*Rc; //sqrt(3)*Rc/2;
	pc_5_a0[2] = 0;	  
	pc_6_a0[0] = -Rc/2;
	pc_6_a0[1] = -(float)SQRT3_DIV2*Rc; //-sqrt(3)*Rc/2;
	pc_6_a0[2] = 0;*/	  

	// zero position
	/*p0[0] = 0;
	p0[1] = 0;
	p0[2] = 0;*/	 



	/*calc*/
	c3=(float)cosf(a3);c4=(float)cosf(a4);
	s3=(float)sinf(a3);s4=(float)sinf(a4); 
	c3_2=(float)cosf(a3/2);c4_2=(float)cosf(a4/2);
	s3_2=(float)sinf(a3/2);s4_2=(float)sinf(a4/2);    

   	//get pb in a coordinate system
	pb_4_a[0] = Rb*c4 + Db*c3*s4;
	pb_4_a[1] = -Db*s3;
	pb_4_a[2] = Db*c3*c4 - Rb*s4;
	pb_5_a[0] = (2*Db*c3*s4 - Rb*c4 + (float)SQRT3*Rb*s3*s4)/2;
	pb_5_a[1] = -(2*Db*s3 - (float)SQRT3*Rb*c3)/2;
	pb_5_a[2] = (Rb*s4 + 2*Db*c3*c4 + (float)SQRT3*Rb*c4*s3)/2;
	pb_6_a[0] = (2*Db*c3*s4 - Rb*c4 - (float)SQRT3*Rb*s3*s4)/2;
	pb_6_a[1] = -(2*Db*s3 + (float)SQRT3*Rb*c3)/2;
	pb_6_a[2] = (Rb*s4 + 2*Db*c3*c4 - (float)SQRT3*Rb*c4*s3)/2;   

	//get d4,d5,d6 vectors
	d4_v[0] =  pb_4_a[0]-pa_4_a[0];
	d4_v[1] =  pb_4_a[1]-pa_4_a[1];
	d4_v[2] =  pb_4_a[2]-pa_4_a[2];
	d5_v[0] =  pb_5_a[0]-pa_5_a[0];
	d5_v[1] =  pb_5_a[1]-pa_5_a[1];
	d5_v[2] =  pb_5_a[2]-pa_5_a[2];
	d6_v[0] =  pb_6_a[0]-pa_6_a[0];
	d6_v[1] =  pb_6_a[1]-pa_6_a[1];
	d6_v[2] =  pb_6_a[2]-pa_6_a[2];	  		

	// get cables length(without compensation)
	d4 = d4_v[0]*d4_v[0] + d4_v[1]*d4_v[1] + d4_v[2]*d4_v[2]; 	
	d5 = d5_v[0]*d5_v[0] + d5_v[1]*d5_v[1] + d5_v[2]*d5_v[2]; 	
	d6 = d6_v[0]*d6_v[0] + d6_v[1]*d6_v[1] + d6_v[2]*d6_v[2];  
	d4 = (float)my_sqrt(d4);
	d5 = (float)my_sqrt(d5);
	d6 = (float)my_sqrt(d6); 

	// get plane n vector
	pc_456_vn[0] = (3*SQRT3*Rc*Rc*c3_2*s4_2)/2;
	pc_456_vn[1] = -(3*SQRT3*Rc*Rc*s3_2)/2;
	pc_456_vn[2] = (3*SQRT3*Rc*Rc*c3_2*c4_2)/2; 
 

	
	/* return to global*/
	switch (cable_comd)
	{
		case 123: //cable 123
			gb_pa_1_a[0] = pa_4_a[0];
			gb_pa_1_a[1] = pa_4_a[1];
			gb_pa_1_a[2] = pa_4_a[2];  
			gb_pa_2_a[0] = pa_5_a[0];
			gb_pa_2_a[1] = pa_5_a[1];
			gb_pa_2_a[2] = pa_5_a[2];   
			gb_pa_3_a[0] = pa_6_a[0];
			gb_pa_3_a[1] = pa_6_a[1];
			gb_pa_3_a[2] = pa_6_a[2];

			// return to gloabl	
			gb_pb_1_a[0] = pb_4_a[0];
			gb_pb_1_a[1] = pb_4_a[1];
			gb_pb_1_a[2] = pb_4_a[2];
			gb_pb_2_a[0] = pb_5_a[0];
			gb_pb_2_a[1] = pb_5_a[1];
			gb_pb_2_a[2] = pb_5_a[2];
			gb_pb_3_a[0] = pb_6_a[0];
			gb_pb_3_a[1] = pb_6_a[1];
			gb_pb_3_a[2] = pb_6_a[2]; 

			// reburn to global variable
			gb_d1_v[0] =  d4_v[0];
			gb_d1_v[1] =  d4_v[1];
			gb_d1_v[2] =  d4_v[2];
			gb_d2_v[0] =  d5_v[0];
			gb_d2_v[1] =  d5_v[1];
			gb_d2_v[2] =  d5_v[2];
			gb_d3_v[0] =  d6_v[0];
			gb_d3_v[1] =  d6_v[1];
			gb_d3_v[2] =  d6_v[2];

			// return golbal
			gb_d1 = d4;
			gb_d2 = d5;
			gb_d3 = d6;

			// reburn to golbal variable
			gb_pc_123_vn[0] = pc_456_vn[0];
			gb_pc_123_vn[1] = pc_456_vn[1];
			gb_pc_123_vn[2] = pc_456_vn[2];	
			
			break;
		case 456: //cable 456
			gb_pa_4_a[0] = pa_4_a[0];
			gb_pa_4_a[1] = pa_4_a[1];
			gb_pa_4_a[2] = pa_4_a[2];  
			gb_pa_5_a[0] = pa_5_a[0];
			gb_pa_5_a[1] = pa_5_a[1];
			gb_pa_5_a[2] = pa_5_a[2];   
			gb_pa_6_a[0] = pa_6_a[0];
			gb_pa_6_a[1] = pa_6_a[1];
			gb_pa_6_a[2] = pa_6_a[2];

			// return to gloabl	
			gb_pb_4_a[0] = pb_4_a[0];
			gb_pb_4_a[1] = pb_4_a[1];
			gb_pb_4_a[2] = pb_4_a[2];
			gb_pb_5_a[0] = pb_5_a[0];
			gb_pb_5_a[1] = pb_5_a[1];
			gb_pb_5_a[2] = pb_5_a[2];
			gb_pb_6_a[0] = pb_6_a[0];
			gb_pb_6_a[1] = pb_6_a[1];
			gb_pb_6_a[2] = pb_6_a[2]; 

			// reburn to global variable
			gb_d4_v[0] =  d4_v[0];
			gb_d4_v[1] =  d4_v[1];
			gb_d4_v[2] =  d4_v[2];
			gb_d5_v[0] =  d5_v[0];
			gb_d5_v[1] =  d5_v[1];
			gb_d5_v[2] =  d5_v[2];
			gb_d6_v[0] =  d6_v[0];
			gb_d6_v[1] =  d6_v[1];
			gb_d6_v[2] =  d6_v[2];

			// return golbal
			gb_d4 = d4;
			gb_d5 = d5;
			gb_d6 = d6;

			// reburn to golbal variable
			gb_pc_456_vn[0] = pc_456_vn[0];
			gb_pc_456_vn[1] = pc_456_vn[1];
			gb_pc_456_vn[2] = pc_456_vn[2];	

			break;	
		default: 				
			break;
	}

	
	/*debug*/
	/*printf("***cable length***\r\n");
	 
	printf("cable_comd = %4.4f \r\n", cable_comd);

	printf("d4 = %4.4f \r\n", d4);
	printf("d5 = %4.4f \r\n", d5);
	printf("d6 = %4.4f \r\n", d6);
	printf("pc_456_vn[0] = %4.4f \r\n", pc_456_vn[0]);   
	printf("pc_456_vn[1] = %4.4f \r\n", pc_456_vn[1]); 
	printf("pc_456_vn[2] = %4.4f \r\n", pc_456_vn[2]); */
}



/* cable_123_kine_m3, calc rotation offset for cable 456, 07052013 new version*/
float cable_123_kine_m3(uint16_t cable_comd, uint8_t enable_compensation, float permter[5], float a3, float a4)	
{
   	float pa_4_a[3],pa_5_a[3],pa_6_a[3];
	//float pb_4_b[3],pb_5_b[3],pb_6_b[3];
	float pb_4_b[2];
	float pb_4_a[3],pb_5_a[3],pb_6_a[3];
	//float pc_4_a0[3],pc_5_a0[3],pc_6_a0[3];
	//float p0[3];
	float c3,c4,s3,s4; 	
	float d4_v[3],d5_v[3],d6_v[3];	 
	float d4,d5,d6;
	float pc_456_vn[3];	  
	float Da,Db,Ra,Rb,Rc;
	float t_theta,t_fi,cta,sta,ctf,stf,sta_2,cta_2;
	float t_x,t_y,t_z,t_z_;
	float x1_,y1_,x2,y2,z2,x2_,y2_,rot_d,l12,delta_fi;
	float t_le,cfi,sfi;


	Da = permter[0];
	Db = permter[0]; //current used
	Ra = permter[2];
	Rb = permter[2]; //current used	
	Rc = permter[4];  	
	
	
	// fixed platform
	pa_4_a[0] = Ra;
	pa_4_a[1] = 0;
	pa_4_a[2] = -Da;  
    pa_5_a[0] = -Ra/2;
	pa_5_a[1] = (float)SQRT3_DIV2*Ra; //sqrt(3)*Ra/2;
	pa_5_a[2] = -Da;   
	pa_6_a[0] = -Ra/2;
	pa_6_a[1] = -(float)SQRT3_DIV2*Ra; //-sqrt(3)*Ra/2;
	pa_6_a[2] = -Da;   	

   	// moved platform
	pb_4_b[0] = Rb;
	pb_4_b[1] = 0;
	/*pb_4_b[2] = Db;  
    pb_5_b[0] = -Rb/2;
	pb_5_b[1] = (float)SQRT3_DIV2*Rb;  //sqrt(3)*Rb/2;
	pb_5_b[2] = Db;   
	pb_6_b[0] = -Rb/2;
	pb_6_b[1] = -(float)SQRT3_DIV2*Rb; //-sqrt(3)*Rb/2;
	pb_6_b[2] = Db;*/	  

	// center platform
	/*pc_4_a0[0] = Rc;
	pc_4_a0[1] = 0;
	pc_4_a0[2] = 0;	
	pc_5_a0[0] = -Rc/2;
	pc_5_a0[1] = (float)SQRT3_DIV2*Rc; //sqrt(3)*Rc/2;
	pc_5_a0[2] = 0;	  
	pc_6_a0[0] = -Rc/2;
	pc_6_a0[1] = -(float)SQRT3_DIV2*Rc; //-sqrt(3)*Rc/2;
	pc_6_a0[2] = 0;*/	  

	// zero position
	/*p0[0] = 0;
	p0[1] = 0;
	p0[2] = 0;*/	 



	/*calc*/
	c3=(float)cosf(a3);c4=(float)cosf(a4);
	s3=(float)sinf(a3);s4=(float)sinf(a4); 		
													
	// Rot XY to Rot ZY transfer method
	/*t_x=s4;
	t_y = -c4*s3; //t_y = c4*sin(-a3);
	t_z = c4*c3; //t_z = c4*cos(-a3); 
	t_z_ = t_x*t_x;	//t_z_ = sqrt(t_x*t_x + t_y*t_y); 
	t_z_ = t_z_+t_y*t_y;
	t_z_ = (float)my_sqrt(t_z_); 
	t_theta = (float)atan2f(t_z_,t_z);
	t_fi = (float)atan2f(t_y,t_x);*/

	/*get rotation new version, 08012013*/ 
    t_y = s3;
    t_le = c3;
    t_x =  t_le*s4;
    t_z =  t_le*c4;
    t_fi = (float)atan2f(t_y,t_x);
	sfi = (float)sinf(t_fi);
	cfi = (float)cosf(t_fi);
    t_z_ = t_x*cfi + t_y*sfi;
    t_theta = (float)atan2f(t_z_,t_z);   
    t_fi = -t_fi; //update as new solution, note: negtive here   

	 

   	//get pb in a coordinate system
	cta=(float)cosf(t_theta);ctf=(float)cosf(t_fi);
	sta=(float)sinf(t_theta);stf=(float)sinf(t_fi);	
	sta_2 = (float)sinf(t_theta/2);cta_2 = (float)cosf(t_theta/2);	 

	pb_4_a[0] = Rb*(ctf*ctf*cta - ctf*ctf + 1) + Db*ctf*sta;
	pb_4_a[1] = stf*(Db*sta - Rb*ctf + Rb*ctf*cta);
	pb_4_a[2] = Db*cta - Rb*ctf*sta;
	pb_5_a[0] = Db*ctf*sta - (Rb*(cta*ctf*ctf + stf*stf))/2 - (float)SQRT3_DIV2*Rb*(ctf*stf - ctf*cta*stf);
	pb_5_a[1] = (Rb*(ctf*stf - ctf*cta*stf))/2 + (float)SQRT3_DIV2*Rb*(ctf*ctf + cta*stf*stf) + Db*stf*sta;
	pb_5_a[2] = Db*cta + (Rb*ctf*sta)/2 - (float)SQRT3_DIV2*Rb*stf*sta;
	pb_6_a[0] = Db*ctf*sta - (Rb*(cta*ctf*ctf + stf*stf))/2 + (float)SQRT3_DIV2*Rb*(ctf*stf - ctf*cta*stf);
	pb_6_a[1] = (Rb*(ctf*stf - ctf*cta*stf))/2 - (float)SQRT3_DIV2*Rb*(ctf*ctf + cta*stf*stf) + Db*stf*sta;
	pb_6_a[2] = Db*cta + (Rb*ctf*sta)/2 + (float)SQRT3_DIV2*Rb*stf*sta;

	//get d4,d5,d6 vectors
	d4_v[0] =  pb_4_a[0]-pa_4_a[0];
	d4_v[1] =  pb_4_a[1]-pa_4_a[1];
	d4_v[2] =  pb_4_a[2]-pa_4_a[2];
	d5_v[0] =  pb_5_a[0]-pa_5_a[0];
	d5_v[1] =  pb_5_a[1]-pa_5_a[1];
	d5_v[2] =  pb_5_a[2]-pa_5_a[2];
	d6_v[0] =  pb_6_a[0]-pa_6_a[0];
	d6_v[1] =  pb_6_a[1]-pa_6_a[1];
	d6_v[2] =  pb_6_a[2]-pa_6_a[2];	  		

	// get cables length(without compensation)
	d4 = d4_v[0]*d4_v[0] + d4_v[1]*d4_v[1] + d4_v[2]*d4_v[2]; 	
	d5 = d5_v[0]*d5_v[0] + d5_v[1]*d5_v[1] + d5_v[2]*d5_v[2]; 	
	d6 = d6_v[0]*d6_v[0] + d6_v[1]*d6_v[1] + d6_v[2]*d6_v[2];  
	d4 = (float)my_sqrt(d4);
	d5 = (float)my_sqrt(d5);
	d6 = (float)my_sqrt(d6); 

	// get plane n vector
	pc_456_vn[0] = 3*(float)SQRT3_DIV2*Rc*Rc*sta_2*ctf;
	pc_456_vn[1] = 3*(float)SQRT3_DIV2*Rc*Rc*sta_2*stf;
	pc_456_vn[2] = 3*(float)SQRT3_DIV2*Rc*Rc*cta_2;
 

	// calc rotation offset t_fi for cable 456
	x1_ = pb_4_b[0]; //original cable123 Rot XY to P1, or in b coordinate
	y1_ = pb_4_b[1]; 
	x2 = pb_4_a[0]; //New cable123 Rot ZYZ to p2,in a coordinate
	y2 = pb_4_a[1];
	z2 = pb_4_a[2];		
	x2_ = x2*c4 - z2*c3*s4 + y2*s3*s4; //P2 in Rot XY plane position, or in b coordinate
	y2_ = y2*c3 + z2*s3;
	//z2_ = x2*s4 + z2*c3*c4 - y2*c4*s3;  % z2_ = +-57, for veriry
	//x2_y2_sqt = sqrt(x2_*x2_ + y2_*y2_); % test_xy2_ = 35 , for veriry
	l12 = (x1_-x2_)*(x1_-x2_) + (y1_-y2_)*(y1_-y2_);  
	l12 = (float)my_sqrt(l12);
	l12 = l12/2;   //delta_fi = 2*asin(l12/(2*Rb));
	l12 = l12/Rb;  
	delta_fi = (float)asinf(l12); 
	delta_fi = 2*delta_fi;	   
	rot_d = x2_*y1_ - x1_*y2_;	// p2->p1, use cross to judge direction, may better, 07082013

	//rot_d = -x2_*y1_ + x1_*y2_;	// p2->p1, use cross to judge direction
	if (rot_d < 0)
	{
	    delta_fi = -delta_fi;  // p2->p1 rot offset, neigtive if the end effector is wrong
	}

	
	/* return to global*/
	switch (cable_comd)
	{
		case 123: //cable 123
			gb_pa_1_a[0] = pa_4_a[0];
			gb_pa_1_a[1] = pa_4_a[1];
			gb_pa_1_a[2] = pa_4_a[2];  
			gb_pa_2_a[0] = pa_5_a[0];
			gb_pa_2_a[1] = pa_5_a[1];
			gb_pa_2_a[2] = pa_5_a[2];   
			gb_pa_3_a[0] = pa_6_a[0];
			gb_pa_3_a[1] = pa_6_a[1];
			gb_pa_3_a[2] = pa_6_a[2];

			// return to gloabl	
			gb_pb_1_a[0] = pb_4_a[0];
			gb_pb_1_a[1] = pb_4_a[1];
			gb_pb_1_a[2] = pb_4_a[2];
			gb_pb_2_a[0] = pb_5_a[0];
			gb_pb_2_a[1] = pb_5_a[1];
			gb_pb_2_a[2] = pb_5_a[2];
			gb_pb_3_a[0] = pb_6_a[0];
			gb_pb_3_a[1] = pb_6_a[1];
			gb_pb_3_a[2] = pb_6_a[2]; 

			// reburn to global variable
			gb_d1_v[0] =  d4_v[0];
			gb_d1_v[1] =  d4_v[1];
			gb_d1_v[2] =  d4_v[2];
			gb_d2_v[0] =  d5_v[0];
			gb_d2_v[1] =  d5_v[1];
			gb_d2_v[2] =  d5_v[2];
			gb_d3_v[0] =  d6_v[0];
			gb_d3_v[1] =  d6_v[1];
			gb_d3_v[2] =  d6_v[2];

			// return golbal
			gb_d1 = d4;
			gb_d2 = d5;
			gb_d3 = d6;

			// reburn to golbal variable
			gb_pc_123_vn[0] = pc_456_vn[0];
			gb_pc_123_vn[1] = pc_456_vn[1];
			gb_pc_123_vn[2] = pc_456_vn[2];	
			
			break;
		case 456: //cable 456
			gb_pa_4_a[0] = pa_4_a[0];
			gb_pa_4_a[1] = pa_4_a[1];
			gb_pa_4_a[2] = pa_4_a[2];  
			gb_pa_5_a[0] = pa_5_a[0];
			gb_pa_5_a[1] = pa_5_a[1];
			gb_pa_5_a[2] = pa_5_a[2];   
			gb_pa_6_a[0] = pa_6_a[0];
			gb_pa_6_a[1] = pa_6_a[1];
			gb_pa_6_a[2] = pa_6_a[2];

			// return to gloabl	
			gb_pb_4_a[0] = pb_4_a[0];
			gb_pb_4_a[1] = pb_4_a[1];
			gb_pb_4_a[2] = pb_4_a[2];
			gb_pb_5_a[0] = pb_5_a[0];
			gb_pb_5_a[1] = pb_5_a[1];
			gb_pb_5_a[2] = pb_5_a[2];
			gb_pb_6_a[0] = pb_6_a[0];
			gb_pb_6_a[1] = pb_6_a[1];
			gb_pb_6_a[2] = pb_6_a[2]; 

			// reburn to global variable
			gb_d4_v[0] =  d4_v[0];
			gb_d4_v[1] =  d4_v[1];
			gb_d4_v[2] =  d4_v[2];
			gb_d5_v[0] =  d5_v[0];
			gb_d5_v[1] =  d5_v[1];
			gb_d5_v[2] =  d5_v[2];
			gb_d6_v[0] =  d6_v[0];
			gb_d6_v[1] =  d6_v[1];
			gb_d6_v[2] =  d6_v[2];

			// return golbal
			gb_d4 = d4;
			gb_d5 = d5;
			gb_d6 = d6;

			// reburn to golbal variable
			gb_pc_456_vn[0] = pc_456_vn[0];
			gb_pc_456_vn[1] = pc_456_vn[1];
			gb_pc_456_vn[2] = pc_456_vn[2];	

			break;	
		default: 				
			break;
	}

	
	/*debug*/
	/*printf("***cable length***\r\n");	 
	printf("cable_comd = %4.4f \r\n", cable_comd);
	printf("d4 = %4.4f \r\n", d4);
	printf("d5 = %4.4f \r\n", d5);
	printf("d6 = %4.4f \r\n", d6);
	printf("pc_456_vn[0] = %4.4f \r\n", pc_456_vn[0]);   
	printf("pc_456_vn[1] = %4.4f \r\n", pc_456_vn[1]); 
	printf("pc_456_vn[2] = %4.4f \r\n", pc_456_vn[2]);
    printf("delta_fi = %4.4f \r\n", delta_fi);	*/    

	return delta_fi;
}



/* cable_456_kine_m2, and is suitable for 123, 07032013 new version*/
void cable_456_kine_m2(uint16_t cable_comd, uint8_t enable_compensation, float permter[5], float a3, float a4)	
{
   	float pa_4_a[3],pa_5_a[3],pa_6_a[3];
	//float pb_4_b[3],pb_5_b[3],pb_6_b[3];
	float pb_4_a[3],pb_5_a[3],pb_6_a[3];
	//float pc_4_a0[3],pc_5_a0[3],pc_6_a0[3];
	//float p0[3];
	float c3,c4,s3,s4; 	
	float d4_v[3],d5_v[3],d6_v[3];	 
	float d4,d5,d6;
	float pc_456_vn[3];	  
	float Da,Db,Ra,Rb,Rc;
	float t_theta,t_fi,cta,sta,ctf,stf,sta_2,cta_2;
	float t_x,t_y,t_z,t_z_;


	Da = permter[0];
	Db = permter[0]; //current used
	Ra = permter[2];
	Rb = permter[2]; //current used	
	Rc = permter[4];  	
	
	
	// fixed platform
	pa_4_a[0] = Ra;
	pa_4_a[1] = 0;
	pa_4_a[2] = -Da;  
    pa_5_a[0] = -Ra/2;
	pa_5_a[1] = (float)SQRT3_DIV2*Ra; //sqrt(3)*Ra/2;
	pa_5_a[2] = -Da;   
	pa_6_a[0] = -Ra/2;
	pa_6_a[1] = -(float)SQRT3_DIV2*Ra; //-sqrt(3)*Ra/2;
	pa_6_a[2] = -Da;   	

   	// moved platform
	/*pb_4_b[0] = Rb;
	pb_4_b[1] = 0;
	pb_4_b[2] = Db;  
    pb_5_b[0] = -Rb/2;
	pb_5_b[1] = (float)SQRT3_DIV2*Rb;  //sqrt(3)*Rb/2;
	pb_5_b[2] = Db;   
	pb_6_b[0] = -Rb/2;
	pb_6_b[1] = -(float)SQRT3_DIV2*Rb; //-sqrt(3)*Rb/2;
	pb_6_b[2] = Db;*/	  

	// center platform
	/*pc_4_a0[0] = Rc;
	pc_4_a0[1] = 0;
	pc_4_a0[2] = 0;	
	pc_5_a0[0] = -Rc/2;
	pc_5_a0[1] = (float)SQRT3_DIV2*Rc; //sqrt(3)*Rc/2;
	pc_5_a0[2] = 0;	  
	pc_6_a0[0] = -Rc/2;
	pc_6_a0[1] = -(float)SQRT3_DIV2*Rc; //-sqrt(3)*Rc/2;
	pc_6_a0[2] = 0;*/	  

	// zero position
	/*p0[0] = 0;
	p0[1] = 0;
	p0[2] = 0;*/	 



	/*calc*/
	c3=(float)cosf(a3);c4=(float)cosf(a4);
	s3=(float)sinf(a3);s4=(float)sinf(a4); 
	
													
	// Rot XY to Rot ZY transfer method
	t_x=s4;
	t_y = -c4*s3; //t_y = c4*sin(-a3);
	t_z = c4*c3; //t_z = c4*cos(-a3); 
	t_z_ = t_x*t_x;	//t_z_ = sqrt(t_x*t_x + t_y*t_y); 
	t_z_ = t_z_+t_y*t_y;
	t_z_ = (float)my_sqrt(t_z_); 
	t_theta = (float)atan2f(t_z_,t_z);
	t_fi = (float)atan2f(t_y,t_x);
	 

   	//get pb in a coordinate system
	cta=(float)cosf(t_theta);ctf=(float)cosf(t_fi);
	sta=(float)sinf(t_theta);stf=(float)sinf(t_fi);	
	sta_2 = (float)sinf(t_theta/2);cta_2 = (float)cosf(t_theta/2);	 

	pb_4_a[0] = Rb*(ctf*ctf*cta - ctf*ctf + 1) + Db*ctf*sta;
	pb_4_a[1] = stf*(Db*sta - Rb*ctf + Rb*ctf*cta);
	pb_4_a[2] = Db*cta - Rb*ctf*sta;
	pb_5_a[0] = Db*ctf*sta - (Rb*(cta*ctf*ctf + stf*stf))/2 - (float)SQRT3_DIV2*Rb*(ctf*stf - ctf*cta*stf);
	pb_5_a[1] = (Rb*(ctf*stf - ctf*cta*stf))/2 + (float)SQRT3_DIV2*Rb*(ctf*ctf + cta*stf*stf) + Db*stf*sta;
	pb_5_a[2] = Db*cta + (Rb*ctf*sta)/2 - (float)SQRT3_DIV2*Rb*stf*sta;
	pb_6_a[0] = Db*ctf*sta - (Rb*(cta*ctf*ctf + stf*stf))/2 + (float)SQRT3_DIV2*Rb*(ctf*stf - ctf*cta*stf);
	pb_6_a[1] = (Rb*(ctf*stf - ctf*cta*stf))/2 - (float)SQRT3_DIV2*Rb*(ctf*ctf + cta*stf*stf) + Db*stf*sta;
	pb_6_a[2] = Db*cta + (Rb*ctf*sta)/2 + (float)SQRT3_DIV2*Rb*stf*sta;

	//get d4,d5,d6 vectors
	d4_v[0] =  pb_4_a[0]-pa_4_a[0];
	d4_v[1] =  pb_4_a[1]-pa_4_a[1];
	d4_v[2] =  pb_4_a[2]-pa_4_a[2];
	d5_v[0] =  pb_5_a[0]-pa_5_a[0];
	d5_v[1] =  pb_5_a[1]-pa_5_a[1];
	d5_v[2] =  pb_5_a[2]-pa_5_a[2];
	d6_v[0] =  pb_6_a[0]-pa_6_a[0];
	d6_v[1] =  pb_6_a[1]-pa_6_a[1];
	d6_v[2] =  pb_6_a[2]-pa_6_a[2];	  		

	// get cables length(without compensation)
	d4 = d4_v[0]*d4_v[0] + d4_v[1]*d4_v[1] + d4_v[2]*d4_v[2]; 	
	d5 = d5_v[0]*d5_v[0] + d5_v[1]*d5_v[1] + d5_v[2]*d5_v[2]; 	
	d6 = d6_v[0]*d6_v[0] + d6_v[1]*d6_v[1] + d6_v[2]*d6_v[2];  
	d4 = (float)my_sqrt(d4);
	d5 = (float)my_sqrt(d5);
	d6 = (float)my_sqrt(d6); 

	// get plane n vector
	pc_456_vn[0] = 3*(float)SQRT3_DIV2*Rc*Rc*sta_2*ctf;
	pc_456_vn[1] = 3*(float)SQRT3_DIV2*Rc*Rc*sta_2*stf;
	pc_456_vn[2] = 3*(float)SQRT3_DIV2*Rc*Rc*cta_2;
 

	
	/* return to global*/
	switch (cable_comd)
	{
		case 123: //cable 123
			gb_pa_1_a[0] = pa_4_a[0];
			gb_pa_1_a[1] = pa_4_a[1];
			gb_pa_1_a[2] = pa_4_a[2];  
			gb_pa_2_a[0] = pa_5_a[0];
			gb_pa_2_a[1] = pa_5_a[1];
			gb_pa_2_a[2] = pa_5_a[2];   
			gb_pa_3_a[0] = pa_6_a[0];
			gb_pa_3_a[1] = pa_6_a[1];
			gb_pa_3_a[2] = pa_6_a[2];

			// return to gloabl	
			gb_pb_1_a[0] = pb_4_a[0];
			gb_pb_1_a[1] = pb_4_a[1];
			gb_pb_1_a[2] = pb_4_a[2];
			gb_pb_2_a[0] = pb_5_a[0];
			gb_pb_2_a[1] = pb_5_a[1];
			gb_pb_2_a[2] = pb_5_a[2];
			gb_pb_3_a[0] = pb_6_a[0];
			gb_pb_3_a[1] = pb_6_a[1];
			gb_pb_3_a[2] = pb_6_a[2]; 

			// reburn to global variable
			gb_d1_v[0] =  d4_v[0];
			gb_d1_v[1] =  d4_v[1];
			gb_d1_v[2] =  d4_v[2];
			gb_d2_v[0] =  d5_v[0];
			gb_d2_v[1] =  d5_v[1];
			gb_d2_v[2] =  d5_v[2];
			gb_d3_v[0] =  d6_v[0];
			gb_d3_v[1] =  d6_v[1];
			gb_d3_v[2] =  d6_v[2];

			// return golbal
			gb_d1 = d4;
			gb_d2 = d5;
			gb_d3 = d6;

			// reburn to golbal variable
			gb_pc_123_vn[0] = pc_456_vn[0];
			gb_pc_123_vn[1] = pc_456_vn[1];
			gb_pc_123_vn[2] = pc_456_vn[2];	
			
			break;
		case 456: //cable 456
			gb_pa_4_a[0] = pa_4_a[0];
			gb_pa_4_a[1] = pa_4_a[1];
			gb_pa_4_a[2] = pa_4_a[2];  
			gb_pa_5_a[0] = pa_5_a[0];
			gb_pa_5_a[1] = pa_5_a[1];
			gb_pa_5_a[2] = pa_5_a[2];   
			gb_pa_6_a[0] = pa_6_a[0];
			gb_pa_6_a[1] = pa_6_a[1];
			gb_pa_6_a[2] = pa_6_a[2];

			// return to gloabl	
			gb_pb_4_a[0] = pb_4_a[0];
			gb_pb_4_a[1] = pb_4_a[1];
			gb_pb_4_a[2] = pb_4_a[2];
			gb_pb_5_a[0] = pb_5_a[0];
			gb_pb_5_a[1] = pb_5_a[1];
			gb_pb_5_a[2] = pb_5_a[2];
			gb_pb_6_a[0] = pb_6_a[0];
			gb_pb_6_a[1] = pb_6_a[1];
			gb_pb_6_a[2] = pb_6_a[2]; 

			// reburn to global variable
			gb_d4_v[0] =  d4_v[0];
			gb_d4_v[1] =  d4_v[1];
			gb_d4_v[2] =  d4_v[2];
			gb_d5_v[0] =  d5_v[0];
			gb_d5_v[1] =  d5_v[1];
			gb_d5_v[2] =  d5_v[2];
			gb_d6_v[0] =  d6_v[0];
			gb_d6_v[1] =  d6_v[1];
			gb_d6_v[2] =  d6_v[2];

			// return golbal
			gb_d4 = d4;
			gb_d5 = d5;
			gb_d6 = d6;

			// reburn to golbal variable
			gb_pc_456_vn[0] = pc_456_vn[0];
			gb_pc_456_vn[1] = pc_456_vn[1];
			gb_pc_456_vn[2] = pc_456_vn[2];	

			break;	
		default: 				
			break;
	}

	
	/*debug*/
	/*printf("***cable length***\r\n");
	 
	printf("cable_comd = %4.4f \r\n", cable_comd);

	printf("d4 = %4.4f \r\n", d4);
	printf("d5 = %4.4f \r\n", d5);
	printf("d6 = %4.4f \r\n", d6);
	printf("pc_456_vn[0] = %4.4f \r\n", pc_456_vn[0]);   
	printf("pc_456_vn[1] = %4.4f \r\n", pc_456_vn[1]); 
	printf("pc_456_vn[2] = %4.4f \r\n", pc_456_vn[2]); */
}


/* cable_456_kine_m3, add rotation offset t_fi, 07052013 new version*/
void cable_456_kine_m3(uint16_t cable_comd, uint8_t enable_compensation, float permter[5], float a3, float a4, float offset_t_fi)	
{
   	float pa_4_a[3],pa_5_a[3],pa_6_a[3];
	//float pb_4_b[3],pb_5_b[3],pb_6_b[3];
	float pb_4_a[3],pb_5_a[3],pb_6_a[3];
	//float pc_4_a0[3],pc_5_a0[3],pc_6_a0[3];
	//float p0[3];
	float c3,c4,s3,s4; 	
	float d4_v[3],d5_v[3],d6_v[3];	 
	float d4,d5,d6;
	float pc_456_vn[3];	  
	float Da,Db,Ra,Rb,Rc;
	float t_theta,t_fi,cta,sta,ctf,stf,sta_2,cta_2;
	float t_x,t_y,t_z,t_z_;
	float t_le,cfi,sfi;


	Da = permter[0];
	Db = permter[0]; //current used
	Ra = permter[2];
	Rb = permter[2]; //current used	
	Rc = permter[4];  	
	
	
	// fixed platform
	pa_4_a[0] = Ra;
	pa_4_a[1] = 0;
	pa_4_a[2] = -Da;  
    pa_5_a[0] = -Ra/2;
	pa_5_a[1] = (float)SQRT3_DIV2*Ra; //sqrt(3)*Ra/2;
	pa_5_a[2] = -Da;   
	pa_6_a[0] = -Ra/2;
	pa_6_a[1] = -(float)SQRT3_DIV2*Ra; //-sqrt(3)*Ra/2;
	pa_6_a[2] = -Da;   	

   	// moved platform
	/*pb_4_b[0] = Rb;
	pb_4_b[1] = 0;
	pb_4_b[2] = Db;  
    pb_5_b[0] = -Rb/2;
	pb_5_b[1] = (float)SQRT3_DIV2*Rb;  //sqrt(3)*Rb/2;
	pb_5_b[2] = Db;   
	pb_6_b[0] = -Rb/2;
	pb_6_b[1] = -(float)SQRT3_DIV2*Rb; //-sqrt(3)*Rb/2;
	pb_6_b[2] = Db;*/	  

	// center platform
	/*pc_4_a0[0] = Rc;
	pc_4_a0[1] = 0;
	pc_4_a0[2] = 0;	
	pc_5_a0[0] = -Rc/2;
	pc_5_a0[1] = (float)SQRT3_DIV2*Rc; //sqrt(3)*Rc/2;
	pc_5_a0[2] = 0;	  
	pc_6_a0[0] = -Rc/2;
	pc_6_a0[1] = -(float)SQRT3_DIV2*Rc; //-sqrt(3)*Rc/2;
	pc_6_a0[2] = 0;*/	  

	// zero position
	/*p0[0] = 0;
	p0[1] = 0;
	p0[2] = 0;*/


	/*calc*/
	c3=(float)cosf(a3);c4=(float)cosf(a4);
	s3=(float)sinf(a3);s4=(float)sinf(a4); 
	
													
	// Rot XY to Rot ZY transfer method
	/*t_x=s4;
	t_y = -c4*s3; //t_y = c4*sin(-a3);
	t_z = c4*c3; //t_z = c4*cos(-a3); 
	t_z_ = t_x*t_x;	//t_z_ = sqrt(t_x*t_x + t_y*t_y); 
	t_z_ = t_z_+t_y*t_y;
	t_z_ = (float)my_sqrt(t_z_); 
	t_theta = (float)atan2f(t_z_,t_z);
	t_fi = (float)atan2f(t_y,t_x);
	t_fi = t_fi + offset_t_fi; // add rotation offset from cable 123*/

	/*get rotation new version, 08012013*/ 
    t_y = s3;
    t_le = c3;
    t_x =  t_le*s4;
    t_z =  t_le*c4;
    t_fi = (float)atan2f(t_y,t_x);
	sfi = (float)sinf(t_fi);
	cfi = (float)cosf(t_fi);
    t_z_ = t_x*cfi + t_y*sfi;
    t_theta = (float)atan2f(t_z_,t_z);   
    t_fi = -t_fi; //update as new solution, note: negtive here
	t_fi = t_fi + offset_t_fi; // add rotation offset from cable 123 


   	//get pb in a coordinate system
	cta=(float)cosf(t_theta);ctf=(float)cosf(t_fi);
	sta=(float)sinf(t_theta);stf=(float)sinf(t_fi);	
	sta_2 = (float)sinf(t_theta/2);cta_2 = (float)cosf(t_theta/2);	 

	pb_4_a[0] = Rb*(ctf*ctf*cta - ctf*ctf + 1) + Db*ctf*sta;
	pb_4_a[1] = stf*(Db*sta - Rb*ctf + Rb*ctf*cta);
	pb_4_a[2] = Db*cta - Rb*ctf*sta;
	pb_5_a[0] = Db*ctf*sta - (Rb*(cta*ctf*ctf + stf*stf))/2 - (float)SQRT3_DIV2*Rb*(ctf*stf - ctf*cta*stf);
	pb_5_a[1] = (Rb*(ctf*stf - ctf*cta*stf))/2 + (float)SQRT3_DIV2*Rb*(ctf*ctf + cta*stf*stf) + Db*stf*sta;
	pb_5_a[2] = Db*cta + (Rb*ctf*sta)/2 - (float)SQRT3_DIV2*Rb*stf*sta;
	pb_6_a[0] = Db*ctf*sta - (Rb*(cta*ctf*ctf + stf*stf))/2 + (float)SQRT3_DIV2*Rb*(ctf*stf - ctf*cta*stf);
	pb_6_a[1] = (Rb*(ctf*stf - ctf*cta*stf))/2 - (float)SQRT3_DIV2*Rb*(ctf*ctf + cta*stf*stf) + Db*stf*sta;
	pb_6_a[2] = Db*cta + (Rb*ctf*sta)/2 + (float)SQRT3_DIV2*Rb*stf*sta;

	//get d4,d5,d6 vectors
	d4_v[0] =  pb_4_a[0]-pa_4_a[0];
	d4_v[1] =  pb_4_a[1]-pa_4_a[1];
	d4_v[2] =  pb_4_a[2]-pa_4_a[2];
	d5_v[0] =  pb_5_a[0]-pa_5_a[0];
	d5_v[1] =  pb_5_a[1]-pa_5_a[1];
	d5_v[2] =  pb_5_a[2]-pa_5_a[2];
	d6_v[0] =  pb_6_a[0]-pa_6_a[0];
	d6_v[1] =  pb_6_a[1]-pa_6_a[1];
	d6_v[2] =  pb_6_a[2]-pa_6_a[2];	  		

	// get cables length(without compensation)
	d4 = d4_v[0]*d4_v[0] + d4_v[1]*d4_v[1] + d4_v[2]*d4_v[2]; 	
	d5 = d5_v[0]*d5_v[0] + d5_v[1]*d5_v[1] + d5_v[2]*d5_v[2]; 	
	d6 = d6_v[0]*d6_v[0] + d6_v[1]*d6_v[1] + d6_v[2]*d6_v[2];  
	d4 = (float)my_sqrt(d4);
	d5 = (float)my_sqrt(d5);
	d6 = (float)my_sqrt(d6); 

	// get plane n vector
	pc_456_vn[0] = 3*(float)SQRT3_DIV2*Rc*Rc*sta_2*ctf;
	pc_456_vn[1] = 3*(float)SQRT3_DIV2*Rc*Rc*sta_2*stf;
	pc_456_vn[2] = 3*(float)SQRT3_DIV2*Rc*Rc*cta_2;
 

	
	/* return to global*/
	switch (cable_comd)
	{
		case 123: //cable 123
			gb_pa_1_a[0] = pa_4_a[0];
			gb_pa_1_a[1] = pa_4_a[1];
			gb_pa_1_a[2] = pa_4_a[2];  
			gb_pa_2_a[0] = pa_5_a[0];
			gb_pa_2_a[1] = pa_5_a[1];
			gb_pa_2_a[2] = pa_5_a[2];   
			gb_pa_3_a[0] = pa_6_a[0];
			gb_pa_3_a[1] = pa_6_a[1];
			gb_pa_3_a[2] = pa_6_a[2];

			// return to gloabl	
			gb_pb_1_a[0] = pb_4_a[0];
			gb_pb_1_a[1] = pb_4_a[1];
			gb_pb_1_a[2] = pb_4_a[2];
			gb_pb_2_a[0] = pb_5_a[0];
			gb_pb_2_a[1] = pb_5_a[1];
			gb_pb_2_a[2] = pb_5_a[2];
			gb_pb_3_a[0] = pb_6_a[0];
			gb_pb_3_a[1] = pb_6_a[1];
			gb_pb_3_a[2] = pb_6_a[2]; 

			// reburn to global variable
			gb_d1_v[0] =  d4_v[0];
			gb_d1_v[1] =  d4_v[1];
			gb_d1_v[2] =  d4_v[2];
			gb_d2_v[0] =  d5_v[0];
			gb_d2_v[1] =  d5_v[1];
			gb_d2_v[2] =  d5_v[2];
			gb_d3_v[0] =  d6_v[0];
			gb_d3_v[1] =  d6_v[1];
			gb_d3_v[2] =  d6_v[2];

			// return golbal
			gb_d1 = d4;
			gb_d2 = d5;
			gb_d3 = d6;

			// reburn to golbal variable
			gb_pc_123_vn[0] = pc_456_vn[0];
			gb_pc_123_vn[1] = pc_456_vn[1];
			gb_pc_123_vn[2] = pc_456_vn[2];	
			
			break;
		case 456: //cable 456
			gb_pa_4_a[0] = pa_4_a[0];
			gb_pa_4_a[1] = pa_4_a[1];
			gb_pa_4_a[2] = pa_4_a[2];  
			gb_pa_5_a[0] = pa_5_a[0];
			gb_pa_5_a[1] = pa_5_a[1];
			gb_pa_5_a[2] = pa_5_a[2];   
			gb_pa_6_a[0] = pa_6_a[0];
			gb_pa_6_a[1] = pa_6_a[1];
			gb_pa_6_a[2] = pa_6_a[2];

			// return to gloabl	
			gb_pb_4_a[0] = pb_4_a[0];
			gb_pb_4_a[1] = pb_4_a[1];
			gb_pb_4_a[2] = pb_4_a[2];
			gb_pb_5_a[0] = pb_5_a[0];
			gb_pb_5_a[1] = pb_5_a[1];
			gb_pb_5_a[2] = pb_5_a[2];
			gb_pb_6_a[0] = pb_6_a[0];
			gb_pb_6_a[1] = pb_6_a[1];
			gb_pb_6_a[2] = pb_6_a[2]; 

			// reburn to global variable
			gb_d4_v[0] =  d4_v[0];
			gb_d4_v[1] =  d4_v[1];
			gb_d4_v[2] =  d4_v[2];
			gb_d5_v[0] =  d5_v[0];
			gb_d5_v[1] =  d5_v[1];
			gb_d5_v[2] =  d5_v[2];
			gb_d6_v[0] =  d6_v[0];
			gb_d6_v[1] =  d6_v[1];
			gb_d6_v[2] =  d6_v[2];

			// return golbal
			gb_d4 = d4;
			gb_d5 = d5;
			gb_d6 = d6;

			// reburn to golbal variable
			gb_pc_456_vn[0] = pc_456_vn[0];
			gb_pc_456_vn[1] = pc_456_vn[1];
			gb_pc_456_vn[2] = pc_456_vn[2];	

			break;	
		default: 				
			break;
	}

	
	/*debug*//*
	printf("***cable length***\r\n");  	 
	printf("cable_comd = %4.4f \r\n", cable_comd); 
	printf("d4 = %4.4f \r\n", d4);
	printf("d5 = %4.4f \r\n", d5);
	printf("d6 = %4.4f \r\n", d6);
	printf("pc_456_vn[0] = %4.4f \r\n", pc_456_vn[0]);   
	printf("pc_456_vn[1] = %4.4f \r\n", pc_456_vn[1]); 
	printf("pc_456_vn[2] = %4.4f \r\n", pc_456_vn[2]); 
	printf("offset_t_fi = %4.4f \r\n", offset_t_fi);
	printf("t_fi = %4.4f \r\n", t_fi);*/	 
	
}


/* getPlaneLineIntersection, rhqi 06272013*/
void getPlaneLineIntersection(uint8_t cable_index,float planeVector[3], float planePoint[3], float lineVector[3], float linePoint[3])
{
	float intersec_p[4];
	float vp1, vp2, vp3, n1, n2, n3, v1, v2, v3, m1, m2, m3, t,vpt;  


	/*calc*/
    vp1 = planeVector[0];
    vp2 = planeVector[1];
    vp3 = planeVector[2];
    n1 = planePoint[0];
    n2 = planePoint[1];
    n3 = planePoint[2];
    v1 = lineVector[0];
    v2 = lineVector[1];
    v3 = lineVector[2];
    m1 = linePoint[0];
    m2 = linePoint[1];
    m3 = linePoint[2];
    vpt = v1 * vp1 + v2 * vp2 + v3 * vp3;

    
	/*if the line parallel with the plane*/
    if (vpt == 0)
	{ 	
		intersec_p[0] = (float)1.0e38; // max value
		intersec_p[1] = (float)1.0e38; // max value
		intersec_p[2] = (float)1.0e38; // max value 
    }
	else
	{
        t = ((n1 - m1) * vp1 + (n2 - m2) * vp2 + (n3 - m3) * vp3) / vpt;
        intersec_p[0] = m1 + v1 * t;
        intersec_p[1] = m2 + v2 * t;
        intersec_p[2] = m3 + v3 * t;

		/* calc radium to compare with rotation center(radium) to judge exist intersection*/
		intersec_p[3] =  (intersec_p[0]-n1)*(intersec_p[0]-n1);	   		
		intersec_p[3] =  intersec_p[3] + (intersec_p[1]-n2)*(intersec_p[1]-n2);	
		intersec_p[3] =  intersec_p[3] + (intersec_p[2]-n3)*(intersec_p[2]-n3);
		intersec_p[3] =  (float)my_sqrt(intersec_p[3]);
    }
	

	/* return to global intersec_p */
	switch (cable_index)
	{
		case 1: //cable 1
			gb_intersec_p1[0]=intersec_p[0];
			gb_intersec_p1[1]=intersec_p[1];
			gb_intersec_p1[2]=intersec_p[2];							
			break;
		case 2: //cable 2
			gb_intersec_p2[0]=intersec_p[0];
			gb_intersec_p2[1]=intersec_p[1];
			gb_intersec_p2[2]=intersec_p[2];				
			break;
		case 3: //cable 3
			gb_intersec_p3[0]=intersec_p[0];
			gb_intersec_p3[1]=intersec_p[1];
			gb_intersec_p3[2]=intersec_p[2];				
			break;
		case 4: //cable 4
			gb_intersec_p4[0]=intersec_p[0];
			gb_intersec_p4[1]=intersec_p[1];
			gb_intersec_p4[2]=intersec_p[2];				
			break;
		case 5: //cable 5
			gb_intersec_p5[0]=intersec_p[0];
			gb_intersec_p5[1]=intersec_p[1];
			gb_intersec_p5[2]=intersec_p[2];				
			break;
		case 6: //cable 6
			gb_intersec_p6[0]=intersec_p[0];
			gb_intersec_p6[1]=intersec_p[1];
			gb_intersec_p6[2]=intersec_p[2];				
			break;	
		default: 				
			break;
	}

	// return to global d_pn to jusdge existing intersection
	gb_d_pn[cable_index-1] = intersec_p[3]; 

	

	//return intersec_p; 

	/*debug2*/	
	/*printf("***intersec_p***\r\n");	  
	printf("cable_index  = %d\r\n", cable_index );
	printf("intersec_p[0] = %4.4f \r\n", intersec_p[0]); 
	printf("intersec_p[1] = %4.4f \r\n", intersec_p[1]); 
	printf("intersec_p[2] = %4.4f \r\n", intersec_p[2]); 
	printf("intersec_p[3] = %4.4f \r\n", intersec_p[3]); */

	/*debug1*//* 
	printf("***intersec_p***\r\n");	
	printf("t = %4.4f \r\n", t); 

	printf("planeVector[0] = %4.4f \r\n", planeVector[0]); 
	printf("planeVector[1] = %4.4f \r\n", planeVector[1]); 
	printf("planeVector[2] = %4.4f \r\n", planeVector[2]); 
	
	printf("planePoint[0] = %4.4f \r\n", planePoint[0]); 
	printf("planePoint[1] = %4.4f \r\n", planePoint[1]); 
	printf("planePoint[2] = %4.4f \r\n", planePoint[2]);
	
	printf("lineVector[0] = %4.4f \r\n", lineVector[0]); 
	printf("lineVector[1] = %4.4f \r\n", lineVector[1]); 
	printf("lineVector[2] = %4.4f \r\n", lineVector[2]); 
	
	printf("linePoint[0] = %4.4f \r\n", linePoint[0]);
	printf("linePoint[1] = %4.4f \r\n", linePoint[1]);
	printf("linePoint[2] = %4.4f \r\n", linePoint[2]); 	
	
	printf("intersec_p[0] = %4.4f \r\n", intersec_p[0]); 
	printf("intersec_p[1] = %4.4f \r\n", intersec_p[1]); 
	printf("intersec_p[2] = %4.4f \r\n", intersec_p[2]); 
	printf("intersec_p[3] = %4.4f \r\n", intersec_p[3]);*/   	
}




/* getPlaneLineIntersection, rhqi 06272013*/
void getIntersectionDirection(uint8_t cable_index,float p0[3], float pn[3], float pm[3],float pb[3],float Rc) 
{		 
	float pm_arc[3];
	float x0,y0,z0,x1,y1,z1,x2,y2,z2,xm,ym,zm;
	float xa,ya,za;
	float p0m_v[3],p0m_n;
	float p12_m[3],p012_v[3],p012_n;
	float pa[3];
	float l_1a,l_2a,l_am,l_12,l_1m,l_2m;
	float angle_1m2,angle_1ma,angle_2ma;
	int32_t t_1m2,t_1ma,t_2ma,t_diff;
	int8_t N;

    x0 = p0[0];
    y0 = p0[1];
    z0 = p0[2];
    x1 = pn[0];
    y1 = pn[1];
    z1 = pn[2];   
    x2 = pb[0];
    y2 = pb[1];
    z2 = pb[2];    
    xm = pm[0];
    ym = pm[1];
    zm = pm[2];    
 
	
	/*calc*/
    // get intersection on the arc
	p0m_v[0] = xm-x0;
	p0m_v[1] = ym-y0;
	p0m_v[2] = zm-z0;
	p0m_n = p0m_v[0]*p0m_v[0] + p0m_v[1]*p0m_v[1] + p0m_v[2]*p0m_v[2];
    p0m_n = (float)my_sqrt(p0m_n); 
	   
    
    if (p0m_n==0)  // inifite solution
	{
		p12_m[0] = (x1+x2)/2;
		p12_m[1] = (y1+y2)/2;
		p12_m[2] = (z1+z2)/2;
		
		p012_v[0] = p12_m[0]-x0;
		p012_v[1] = p12_m[1]-y0;
		p012_v[2] = p12_m[2]-z0;
		
		p012_n = p012_v[0]*p012_v[0] + p012_v[1]*p012_v[1] + p012_v[2]*p012_v[2];
		p012_n = (float)my_sqrt(p012_n); 

		pm_arc[0] = p012_v[0]*Rc/p012_n + x0;
		pm_arc[1] = p012_v[1]*Rc/p012_n + y0;
		pm_arc[2] = p012_v[2]*Rc/p012_n + z0;	      
    }
	else
	{   
		pa[0] = p0m_v[0]*Rc/p0m_n + x0;
		pa[1] = p0m_v[1]*Rc/p0m_n + y0;
		pa[2] = p0m_v[2]*Rc/p0m_n + z0;		 
        xa = pa[0];
        ya = pa[1];
        za = pa[2];

		
        // get distaace
        l_1a = (x1-xa)*(x1-xa) + (y1-ya)*(y1-ya) + (z1-za)*(z1-za);
        l_2a = (x2-xa)*(x2-xa) + (y2-ya)*(y2-ya) + (z2-za)*(z2-za);    
        l_am = (xm-xa)*(xm-xa) + (ym-ya)*(ym-ya) + (zm-za)*(zm-za);   
        l_12 = (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) + (z2-z1)*(z2-z1);
        l_1m = (xm-x1)*(xm-x1) + (ym-y1)*(ym-y1) + (zm-z1)*(zm-z1);
        l_2m = (xm-x2)*(xm-x2) + (ym-y2)*(ym-y2) + (zm-z2)*(zm-z2);  

		l_1a = (float)my_sqrt(l_1a);
        l_2a = (float)my_sqrt(l_2a); 
        l_am = (float)my_sqrt(l_am);
        l_12 = (float)my_sqrt(l_12);
        l_1m = (float)my_sqrt(l_1m);
        l_2m = (float)my_sqrt(l_2m);

        // get angles
        angle_1m2 = (l_1m*l_1m + l_2m*l_2m - l_12*l_12)/(2*l_1m*l_2m);
        angle_1ma = (l_1m*l_1m + l_am*l_am - l_1a*l_1a)/(2*l_1m*l_am);
        angle_2ma = (l_2m*l_2m + l_am*l_am - l_2a*l_2a)/(2*l_2m*l_am);
		angle_1m2 = (float)acosf(angle_1m2);
        angle_1ma = (float)acosf(angle_1ma);
        angle_2ma = (float)acosf(angle_2ma);  

        // get directioa
        t_1m2 = (int32_t)(1000*angle_1m2);
        t_1ma = (int32_t)(1000*angle_1ma);
        t_2ma = (int32_t)(1000*angle_2ma);  

		t_diff = (int32_t)my_abs_int(t_1ma + t_2ma - t_1m2);
        if (t_diff<3)  // 3 is resulution,used(if hasn't better method)
		{
           N = -1;
        }
		else
        {
		   N = 1; 
        }
        
        pm_arc[0] = N*p0m_v[0]*Rc/p0m_n + x0;
		pm_arc[1] = N*p0m_v[1]*Rc/p0m_n + y0;
		pm_arc[2] = N*p0m_v[2]*Rc/p0m_n + z0;
		
		
		
		/*Debug 2*/
		//printf("N = %d \r\n", N);  	
    }


	/* return to global pm_arc */
	switch (cable_index)
	{
		case 1: //cable 1
			gb_pm_arc1[0]=pm_arc[0];
			gb_pm_arc1[1]=pm_arc[1];
			gb_pm_arc1[2]=pm_arc[2];							
			break;
		case 2: //cable 2
			gb_pm_arc2[0]=pm_arc[0];
			gb_pm_arc2[1]=pm_arc[1];
			gb_pm_arc2[2]=pm_arc[2];				
			break;
		case 3: //cable 3
			gb_pm_arc3[0]=pm_arc[0];
			gb_pm_arc3[1]=pm_arc[1];
			gb_pm_arc3[2]=pm_arc[2];				
			break;
		case 4: //cable 4
			gb_pm_arc4[0]=pm_arc[0];
			gb_pm_arc4[1]=pm_arc[1];
			gb_pm_arc4[2]=pm_arc[2];				
			break;
		case 5: //cable 5
			gb_pm_arc5[0]=pm_arc[0];
			gb_pm_arc5[1]=pm_arc[1];
			gb_pm_arc5[2]=pm_arc[2];				
			break;
		case 6: //cable 6
			gb_pm_arc6[0]=pm_arc[0];
			gb_pm_arc6[1]=pm_arc[1];
			gb_pm_arc6[2]=pm_arc[2];				
			break;	
		default: 				
			break;
	} 

	//return pm_arc; 




	/*Debug*/		
	/*
	printf("***pm_arc***\r\n");			
	printf("p0[0] = %4.4f \r\n", p0[0]);
	printf("p0[1] = %4.4f \r\n", p0[1]);
	printf("p0[2] = %4.4f \r\n", p0[2]);
	
	printf("pn[0] = %4.4f \r\n", pn[0]);
	printf("pn[1] = %4.4f \r\n", pn[1]);
	printf("pn[2] = %4.4f \r\n", pn[2]);
	
	printf("pm[0] = %4.4f \r\n", pm[0]);
	printf("pm[1] = %4.4f \r\n", pm[1]);
	printf("pm[2] = %4.4f \r\n", pm[2]);
	
	printf("pb[0] = %4.4f \r\n", pb[0]);
	printf("pb[1] = %4.4f \r\n", pb[1]);
	printf("pb[2] = %4.4f \r\n", pb[2]);
	
	printf("Rc = %4.4f \r\n",Rc);*/ 
	/*		
	printf("angle_1m2 = %4.4f \r\n", angle_1m2);
	printf("angle_1ma = %4.4f \r\n", angle_1ma);
	printf("angle_2ma = %4.4f \r\n", angle_2ma);
	 		 
	printf("t_1m2 = %d \r\n", t_1m2);
	printf("t_1ma = %d \r\n", t_1ma);
	printf("t_2ma = %d \r\n", t_2ma);
	printf("t_diff = %d \r\n", t_diff);*/ 		
	
	/*printf("pm_arc[0] = %4.4f \r\n", pm_arc[0]);
	printf("pm_arc[1] = %4.4f \r\n", pm_arc[1]);
	printf("pm_arc[2] = %4.4f \r\n", pm_arc[2]);	
			
	printf("N = %d \r\n", N);*/ 
}



/* getArcLength, rhqi 06272013*/
float getArcLength(float pa[3], float pm[3],float pb[3],uint8_t command) 
{	
	float pr[4],R_m,R,l13_m,s13,angle_103;	 
	float x1,y1,z1,x2,y2,z2,x3,y3,z3;
	float l12,l23;
	float A1,B1,C1,D1,A2,B2,C2,D2,A3,B3,C3,D3,G,H,I,J;
	float l13,l13_2;
	
    x1 = pa[0];
    y1 = pa[1];
    z1 = pa[2];
    x2 = pm[0];
    y2 = pm[1];
    z2 = pm[2];
    x3 = pb[0];
    y3 = pb[1];
    z3 = pb[2];


    if(command ==0)	// calc as two lines
	{
        // calc as two lines
        l12 = (float)my_sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));  
        l23 = (float)my_sqrt((x2-x3)*(x2-x3) + (y2-y3)*(y2-y3) + (z2-z3)*(z2-z3)); 
        s13 = l12 + l23;         
        //p0 = [NaN NaN NaN];
        //R = NaN;        
    }
	else // calc as arc
	{ 
        // calc as arc
        A1 = y1*z2 - y1*z3 - z1*y2 + z1*y3 + y2*z3 - y3*z2;
        B1 = -x1*z2 + x1*z3 + z1*x2 - z1*x3 - x2*z3 + x3*z2;
        C1 = x1*y2 - x1*y3 - y1*x2 + y1*x3 + x2*y3 - x3*y2;
        D1 = -x1*y2*z3 + x1*y3*z2 + x2*y1*z3 - x3*y1*z2 - x2*y3*z1 + x3*y2*z1;

        A2 = 2*(x2 - x1);
        B2 = 2*(y2 - y1);
        C2 = 2*(z2 - z1);
        D2 = x1*x1 + y1*y1 + z1*z1 - x2*x2 - y2*y2 -z2*z2;

        A3 = 2*(x3 - x1);
        B3 = 2*(y3 - y1);
        C3 = 2*(z3 - z1);
        D3 = x1*x1 + y1*y1 + z1*z1 - x3*x3 - y3*y3 -z3*z3;

        G = (A1*B2*C3 - A1*B3*C2 - A2*B1*C3 + A2*B3*C1 + A3*B1*C2 - A3*B2*C1);
        H = -(B1*C2*D3 - B1*C3*D2 - B2*C1*D3 + B2*C3*D1 + B3*C1*D2 - B3*C2*D1);
        I = (A1*C2*D3 - A1*C3*D2 - A2*C1*D3 + A2*C3*D1 + A3*C1*D2 - A3*C2*D1);
        J = -(A1*B2*D3 - A1*B3*D2 - A2*B1*D3 + A2*B3*D1 + A3*B1*D2 - A3*B2*D1);
        
		pr[0] = H/G;
		pr[1] = I/G;
		pr[2] = J/G;


        // get arc radium
		R_m = (x1-pr[0])*(x1-pr[0]) + (y1-pr[1])*(y1-pr[1]) + (z1-pr[2])*(z1-pr[2]); 
		R = (float)my_sqrt(R_m);     

        
		// get arc length
		// v1, old method, some times error, bad
		/*l13_m = (x1-x3)*(x1-x3) + (y1-y3)*(y1-y3) + (z1-z3)*(z1-z3);
		angle_103 = (float)acosf(1- l13_m/(2*R_m));*/

	    // v2, new method, good, verified ok, update 07/04/2013, 07/08/2013 find calc (float)atan2f(l13_2,R) error,
		/*l13_m = (x1-x3)*(x1-x3) + (y1-y3)*(y1-y3) + (z1-z3)*(z1-z3); 	
		l13 = (float)my_sqrt(l13_m);  
		l13_2 = l13/2;	
		angle_103 = (float)atan2f(l13_2,R);
		angle_103 = angle_103*2; 
        s13 = R*angle_103;*/

		//v3, angle_103 region [0, pi], update 07/10/2013		
		l13_m = (x1-x3)*(x1-x3) + (y1-y3)*(y1-y3) + (z1-z3)*(z1-z3); 	
		l13 = (float)my_sqrt(l13_m);  
		l13_2 = l13/2;
		if(R>0)
		{
			angle_103 = (float)asinf(l13_2/R); //important here, may lead errors, 07/10/2013
			angle_103 = angle_103*2; 
			s13 = R*angle_103;
		}
		else
		{
		   s13 = ZERO_f;
		}						   		


		/*Debug*//*
		printf("***s13***\r\n"); 
		printf("R = %4.8f \r\n", R); 
		printf("l13 = %4.8f \r\n", l13);
		printf("l13_2 = %4.8f \r\n", l13_2); 
		printf("angle_103 = %4.8f \r\n", angle_103); 
		printf("s13 = %4.4f \r\n", s13);*/ 	  
    }

	return s13;
}



/*cable_compensation, rhqi 06282013*/
float cable_compensation(float theta1,float theta2, float sa_half[2], float ca_half[2], float D0, float R0)
{
	float D_m;
	//float theta;
	float sa2,ca2;
	float a,b,c,delta,fi; 
	float abs_a1,abs_a2; 
	int32_t exist_s;

	// parameters
	float r=15.0; // important here
	float A = 0.16663525;
	float B =-1.69013382; 
	
	//only calc the max angle
	abs_a1 = my_abs(theta1);
	abs_a2 = my_abs(theta2);
    if (abs_a1>abs_a2)
	{
        //theta = abs_a1;	
		sa2 = sa_half[0];
		ca2 = ca_half[0];
    }
	else
	{
        //theta = abs_a2;
		sa2 = sa_half[1];
		ca2 = ca_half[1];
    } 	

	
	/*judge solution exist*/
	exist_s = (int32_t)(1000*ca2);
	if (exist_s==0)	 // no solution
	{
	   	D_m = D0;	
	}
	else
	{
		a = -A*ca2;
		b = -B*ca2-(float)(r/2);
		c = D0 - D0*ca2 + R0*sa2;
		
		delta = b*b - 4*a*c;
		delta = (float)my_sqrt(delta);
		
		fi = -b-delta;
		fi = (float)(fi/a);
		fi = (float)(fi/2);	
		
		//get new D for 3d model, 06/25/2013
		D_m = D0 + (A*fi*fi + B*fi);	
	}	

	
	
	/*Debug*/
	//printf("***cable compensation***\r\n");	
	//printf("Da = %4.4f \r\n", D_m);	
		

	return D_m;
}


/*cable_compensation modified version, rhqi 07/04/2013*/
float cable_compensation_m(float theta1, float theta2, float D0, float R0)
{
	float D_m;
	float theta;  
	float a,b,c,delta,fi; 
	int32_t exist_s;
	float ca,sa,c1,c2,s1,s2;
	float t_x,t_y,t_z,t_z_;	
	float t_fi,t_le,cfi,sfi;


	// parameters
	float r=15.0; // important here
	float A = 0.16663525;
	float B =-1.69013382; 
	
	/* calc angle of rotation*/
	c1=(float)cosf(theta1);c2=(float)cosf(theta2);
	s1=(float)sinf(theta1);s2=(float)sinf(theta2); 	
	/*// Rot XY to Rot ZY transfer method
	t_x=s2;
	t_y = -c2*s1; //t_y = c2*sin(-theta1);
	t_z = c2*c1; //t_z = c2*cos(-theta1); 
	t_z_ = t_x*t_x;	//t_z_ = sqrt(t_x*t_x + t_y*t_y); 
	t_z_ = t_z_+t_y*t_y;
	t_z_ = (float)my_sqrt(t_z_); 
	theta = (float)atan2f(t_z_,t_z);*/
	
	/*get rotation new version, 08012013*/ 
    t_y = s1;
    t_le = c1;
    t_x =  t_le*s2;
    t_z =  t_le*c2;
    t_fi = (float)atan2f(t_y,t_x);
	sfi = (float)sinf(t_fi);
	cfi = (float)cosf(t_fi);
    t_z_ = t_x*cfi + t_y*sfi;
    theta = (float)atan2f(t_z_,t_z); //update as new solution
		
	theta = theta/2;
	theta = my_abs(theta);	
	ca=(float)cosf(theta);	
	sa=(float)sinf(theta);	

	
	/*judge solution exist*/  
	exist_s = (int32_t)(1000*ca);
	if (exist_s==0)	 // no solution
	{
	   	D_m = D0;	
	}
	else
	{
		a = -A*ca;
		b = -B*ca-(float)(r/2);
		c = D0 - D0*ca + R0*sa;
		
		delta = b*b - 4*a*c;
		delta = (float)my_sqrt(delta);
		
		fi = -b-delta;
		fi = (float)(fi/a);
		fi = (float)(fi/2);	
		
		//get new D for 3d model, 06/25/2013
		D_m = D0 + (A*fi*fi + B*fi);	
	}	

	
	
	/*Debug*/
	//printf("***cable compensation***\r\n");
	/*printf("t_x = %4.4f \r\n", t_x);
	printf("t_y = %4.4f \r\n", t_y);
	printf("t_z = %4.4f \r\n", t_z);
	printf("t_z_ = %4.4f \r\n", t_z_);
	printf("theta = %4.4f \r\n", theta);		
	printf("Da = %4.4f \r\n", D_m);	*/		

	return D_m;
}



/* new version test, 04292013, verified by matlab*/
void motor_position_calc_2dof(uint8_t arm_index)
{
	float theta_temp[2];
	float sa[2],ca[2];
	float ai[2],bi[2],ci[2],delta[2],fi[2];
	float fai[6];
	int32_t motor_pos[6]; 	
	uint8_t s,i;	
	//int32_t delta_m_pos[6]; 	
	//int32_t abs_delta_m_pos[6];
	//int32_t min_abs_delta_m_pos,max_abs_delta_m_pos;
	//float speed_radio[6]; 
	//int32_t motor_speed[6];

		
	// parameters		 
	uint8_t L0=130; // v2, 04152013, link2
	uint8_t L0_1=114; // v2, 05022013,link1
	/*uint8_t l1=L0/2;
	uint8_t l2=L0/2;
	uint8_t h1=35;
	uint8_t h2=35;*/
	uint8_t l0=L0/2;	  // link2
	uint8_t l0_1=L0_1/2;  // link1
	uint8_t h=35;
	float r=15;
	
	/*target theta -> fi, important here, 05292013*/	
	if((theta_solution[1]<0)||(theta_solution[3]<0))
	{
	  return;
	}

	theta_temp[0] = theta_solution[1]/2;
	theta_temp[1] = theta_solution[3]/2; 	
  	for(s=0;s<2;s++)
	{				
		sa[s] = (float)sinf(theta_temp[s]);
		ca[s] = (float)cosf(theta_temp[s]);	
		ai[s] = -PRESS_COEFFICIENT_1*ca[s];	
		bi[s] = -PRESS_COEFFICIENT_2*ca[s];
		bi[s] = bi[s]-r/2; 	
			
		ci[s] = h*sa[s]; 
		if (s==0)// link1
		{
			ci[s] = ci[s] - l0_1*ca[s];
			ci[s] = ci[s] + l0_1;
		}
		else // link2
		{
			ci[s] = ci[s] - l0*ca[s];
			ci[s] = ci[s] + l0;			
		}				   
		
		delta[s] = bi[s]*bi[s];
		delta[s] = delta[s] - 4*ai[s]*ci[s];   
		delta[s] =(float)my_sqrt(delta[s]); 

		fi[s] =	-bi[s]-delta[s];
		fi[s] = (float)(fi[s]/ai[s]);
		fi[s] = (float)(fi[s]/2);	
	}

	// two links
	fai[0] = 0;	fai[2] = 0; fai[3] = 0; fai[5] = 0;
	fai[1] = fi[0];  fai[4] = fi[1];
		

	/*printf("theta_solution[1] = %4.4f \r\n", theta_solution[1]);
	printf("theta_solution[3] = %4.4f \r\n", theta_solution[3]);
	printf("theta_temp[0] = %4.4f \r\n", theta_temp[0]);
	printf("theta_temp[1] = %4.4f \r\n", theta_temp[1]);
	printf("sa[0] = %4.4f \r\n", sa[0]);
	printf("sa[1] = %4.4f \r\n", sa[1]);
	printf("ca[0] = %4.4f \r\n", ca[0]);
	printf("ca[1] = %4.4f \r\n", ca[1]); 
	printf("ai[0] = %4.4f \r\n", ai[0]);
	printf("ai[1] = %4.4f \r\n", ai[1]);
	printf("bi[0] = %4.4f \r\n", bi[0]);
	printf("bi[1] = %4.4f \r\n", bi[1]);
	printf("ci[0] = %4.4f \r\n", ci[0]);
	printf("ci[1] = %4.4f \r\n", ci[1]);
	printf("delta[0] = %4.4f \r\n", delta[0]);
	printf("delta[1] = %4.4f \r\n", delta[1]);
	printf("fi[0] = %4.4f \r\n", fi[0]);
	printf("fi[4] = %4.4f \r\n", fi[4]); 	*/

   	for(i=0;i<6;i++)
	{
	   switch (i)
		{
			case 0:	// note
				motor_pos[i]=(int32_t)(fai[i]*(float)MOTOR_TRANSFER_1X); // 3*fai*1023/(5*pi)e
				break; 
			case 1: 	  
				motor_pos[i]=1023-(int32_t)(fai[i]*(float)MOTOR_TRANSFER_1X); // 3*fai*1023/(5*pi)e
				break; 	
			case 2: 	  
				motor_pos[i]=(int32_t)(fai[i]*(float)MOTOR_TRANSFER_1X); // 3*fai*1023/(5*pi)e
				break; 				
			case 3:	 // note
				motor_pos[i]=1023-(int32_t)(fai[i]*(float)MOTOR_TRANSFER_1X); // 3*fai*1023/(5*pi)e 
				break; 				
			case 4: 	  
				motor_pos[i]=1023-(int32_t)(fai[i]*(float)MOTOR_TRANSFER_1X); // 3*fai*1023/(5*pi)e
				break; 				
			case 5: 	  
				motor_pos[i]=511-(int32_t)(fai[i]*(float)MOTOR_TRANSFER_1X); // 3*fai*1023/(5*pi)e
				break; 									
					
			default: 				
				break;
		} 	
	 	

		// move to motor max or min pos
		if(motor_pos[i]>=1023)
		{
		  motor_pos[i]=1023;
		}
		else if(motor_pos[i]<=0)		
		{
		  motor_pos[i]=0;
		}


		// move to motor max or min pos
		if(motor_pos[i]>=1023)
		{
		  motor_pos[i]=1023;
		}
		else if(motor_pos[i]<=0)		
		{
		  motor_pos[i]=0;
		}
	
		/*// Dec
		// v2
		motor_pos_low[i]=XDec2Low(motor_pos[i]);
		motor_pos_high[i]=XDec2High(motor_pos[i]); */

	   	// Dec,v3
	//	gb_m_pos[i] = motor_pos[i];
	}


   	// vectority interpolation by simple method	
	/*max_abs_delta_m_pos = abs_delta_m_pos[0];
	for(j=0;j<6;j++)
	{
		if(abs_delta_m_pos[j]>max_abs_delta_m_pos)
		{
			max_abs_delta_m_pos = abs_delta_m_pos[j]; // get max value
		}	
	}
	for(k=0;k<6;k++)
	{
		if(max_abs_delta_m_pos>0)
		{			
			speed_radio[k] = abs_delta_m_pos[k];
			speed_radio[k] = (float)(speed_radio[k]/max_abs_delta_m_pos);			
			speed_radio[k] =  motor_speed_input*speed_radio[k];  // 03152013, new version
			motor_speed[k] = (int32_t)speed_radio[k]; 

			// move to motor max or min pos
			if(motor_speed[k]>=1023)
			{
			  motor_speed[k]=1023;
			}
			else if(motor_speed[k]<=0)		
			{
			  motor_speed[k]=0;
			}		
		}
		else
		{
			motor_speed[k]=0;
		}
		motor_speed_low[k]=XDec2Low(motor_speed[k]);
		motor_speed_high[k]=XDec2High(motor_speed[k]); 
	}*/




	/* //05152013
	printf("*************\r\n");
	printf("theta_solution[1] = %4.4f \r\n", theta_solution[1]);
	printf("theta_solution[3] = %4.4f \r\n", theta_solution[3]);
	printf("fai[1] = %4.4f \r\n", fai[1]);
	printf("fai[4] = %4.4f \r\n", fai[4]);	 		
   	printf("motor_pos[1] = %d \r\n", motor_pos[1]);
	printf("motor_pos[4] = %d \r\n", motor_pos[4]);*/

	/*
	printf("fi[0] = %4.4f \r\n", fi[0]);
	printf("fi[0] = %4.4f \r\n", fi[1]);
	*/
}






/* input cable length test, 06212013*/
void motor_position_by_cable(uint8_t arm_index, float d1, float d2, float d3, float d4, float d5, float d6)
{	
	float fai[6];
	int32_t motor_pos[6]; 	
	uint8_t i;
		
	  
	/*parameters*/
	float r=15.0; 
 

	/*//07/11/2013 new, consider intial offset
	float L0_1=123.2163; 
	float L0_2=40.8956; 
	float L0_3=40.89564;
	float L0_4=112.7568; 
	float L0_5=120.7714; 
	float L0_6=120.7714;*/

	/*//07/25/2013 new, consider intial offset
	float L0_1=123.2163; 
	float L0_2=40.8956; 
	float L0_3=40.89564;
	float L0_4=120.0; 
	float L0_5=132.2; 
	float L0_6=132.2;*/ 

	/*//07/25/2013 new 2, consider intial offset
	float L0_1=123.2163; 
	float L0_2=50.0; 
	float L0_3=40.0;
	float L0_4=124.0; 
	float L0_5=132.0; 
	float L0_6=132.0;*/ 

	//07/26/2013 new 3, initial length of cables by actual measure, consider intial offset
	float L0_1=120.0; 
	float L0_2=50.0; 
	float L0_3=37.0;
	float L0_4=124.0; 
	float L0_5=132.0; 
	float L0_6=132.0; 


	// v3, used, 06212013
	fai[0] = d3-L0_3;	 fai[0] = fai[0]/r;
	fai[1] = L0_1-d1;	 fai[1] = fai[1]/r;
	fai[2] = d2-L0_2;	 fai[2] = fai[2]/r;

	fai[3] = L0_6-d6;	 fai[3] = fai[3]/r;
	fai[4] = L0_4-d4;	 fai[4] = fai[4]/r;
	fai[5] = L0_5-d5;	 fai[5] = fai[5]/r;

	/*fai[3] = L0-d6;	 fai[3] = fai[3]/r;
	fai[4] = L0-d4;	 fai[4] = fai[4]/r;
	fai[5] = L0-d5;	 fai[5] = fai[5]/r;*/


   	for(i=0;i<6;i++)
	{
	   switch (i)
		{
			case 0:	// note
				motor_pos[i]=(int32_t)(fai[i]*(float)MOTOR_TRANSFER_1X); // 3*fai*1023/(5*pi)e
				break; 
			case 1: 	  
				motor_pos[i]=1023-(int32_t)(fai[i]*(float)MOTOR_TRANSFER_1X); // 3*fai*1023/(5*pi)e
				break; 	
			case 2: 	  
				motor_pos[i]=(int32_t)(fai[i]*(float)MOTOR_TRANSFER_1X); // 3*fai*1023/(5*pi)e
				break; 				
			case 3:	 // note
				motor_pos[i]=1023-(int32_t)(fai[i]*(float)MOTOR_TRANSFER_1X); // 3*fai*1023/(5*pi)e 
				break; 				
			case 4: 	  
				motor_pos[i]=1023-(int32_t)(fai[i]*(float)MOTOR_TRANSFER_1X); // 3*fai*1023/(5*pi)e
				break; 				
			case 5: 	  
				motor_pos[i]=511-(int32_t)(fai[i]*(float)MOTOR_TRANSFER_1X); // 3*fai*1023/(5*pi)e
				break; 									
					
			default: 				
				break;
		}

		// move to motor max or min pos
		if(motor_pos[i]>=1023)
		{
		  motor_pos[i]=1023;
		}
		else if(motor_pos[i]<=0)		
		{
		  motor_pos[i]=0;
		}

	   	// Dec,v3
//		gb_m_pos[i] = motor_pos[i];
	}

	/*Debug, 07/25/2013*/
	/*printf("motor_pos[0] = %d \r\n", motor_pos[0]);
	printf("motor_pos[1] = %d \r\n", motor_pos[1]);
	printf("motor_pos[2] = %d \r\n", motor_pos[2]);
	printf("motor_pos[3] = %d \r\n", motor_pos[3]);
	printf("motor_pos[4] = %d \r\n", motor_pos[4]);
	printf("motor_pos[5] = %d \r\n", motor_pos[5]);*/
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
		switch (gb_current_moving_mode)
		{
			case 0: /*Homing*/	
				if(gb_monitor_counter <13*6) // monitor 6 seconds  
				{
					gb_monitor_counter = gb_monitor_counter + 1;	
				}		
				else if(gb_monitor_counter >=13*6) // after 6 seconds, disable all servo torque to save energy
				{
					gb_previous_moving_mode = gb_current_moving_mode;  //update command, 11/06/2014
					AX_12_Torque_Enable(254, 0);  // disable all servo torque			
				}				
				break;
			case 1: /* Joint space teach mode*/				
				break;
			case 2:	  /* vision tracking mode*/
				if(gb_monitor_counter <13*3) // monitor 3 seconds  
				{
					/*monitor single arm tracking*/
					if(gb_monitor_counter >=1) // monitor 0.5 seconds  
					{
						if(gb_arm_index==1) // left arm tracking
						{
							SingleArmHoming(2); // right arm homing
						}
						if(gb_arm_index==2) // right arm	tracking
						{
							SingleArmHoming(2); // right arm homing
						}
					}
					gb_monitor_counter = gb_monitor_counter + 1;	
				}		
				else if((gb_monitor_counter >=13*3) && (gb_monitor_counter <13*8))// after 3 seconds, homing, lasts 5s
				{
					Homing(); // back to home positon
					gb_monitor_counter = gb_monitor_counter + 1;
				}
				else if(gb_monitor_counter >=13*8) // after 5 seconds, disable all servo torque to save energy
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
    //STM_EVAL_LEDToggle(LED3);
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
		delta_arm_px=0; // delta_arm_px
		delta_arm_py=0; // delta_arm_py
		delta_arm_pz=0; // delta_arm_pz
		delta_arm_pos=0; // delta_arm pos	 
		


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



		/* single figner test, ok, 12/19/2013*/
		/*infltableFingerMotionCtrl(1, 1, 1);  //12/19/2103,ok
		infltableFingerMotionCtrl(2, 1, 1); //12/19/2103,ok
		
		infltableFingerMotionCtrl(1, 2, 1); //12/18/2103, ok
		infltableFingerMotionCtrl(2, 2, 1); //12/17/2103, ok 
		
		infltableFingerMotionCtrl(1, 3, 1); //12/18/2103, ok
		infltableFingerMotionCtrl(2, 3, 1); //12/19/2103, ok
		
		infltableFingerMotionCtrl(1, 4, 1); //12/18/2103, ok
		infltableFingerMotionCtrl(2, 4, 1); //12/18/2103, ok
		
		infltableFingerMotionCtrl(1, 5, 1); //12/18/2103, ok
		infltableFingerMotionCtrl(2, 5, 1); //12/18/2103, ok */


				
		/*infltableHandMotionCtrl, 12/19/2013*/		
		infltableHandPose(gb_hand_pose_enable, gb_hand_index, gb_hand_pose_index); // 12/19/2013 added


		/* UART1 receive data, defalut Hex*/
		if(uart1_rx_index==uart1_rx_len)
		{ 
			//command		
			command_hand =(uint8_t)uart1_rx_buffer[0]; 		
			// px
			gb_rx_px =(uint8_t)uart1_rx_buffer[1]; 		
			// py
			gb_rx_py =(uint8_t)uart1_rx_buffer[2];	
			// pz
			gb_rx_pz =(uint8_t)uart1_rx_buffer[3];		
			

			/*region tranfer: ARM :-440~440 => p=(rx-110)*4	<=> tx=p/4+110 , 05152013 v3*/
			/*gb_arm_px = (gb_rx_px-110)*4; 
			gb_arm_py = (gb_rx_py-110)*4; 
			gb_arm_pz = (gb_rx_pz-110)*4;*/ 

			/*07/25/2013, for tracing, 0-380, latest*/
			/*gb_arm_px = gb_rx_px*2; 
			gb_arm_py = gb_rx_py*2; 
			gb_arm_pz = gb_rx_pz*2;*/

			/*07/26/2013, for tracing, 0-380, latest*/ //tx=p/2+50			
			/*gb_arm_px = (gb_rx_px-50)*2; 
			gb_arm_py = gb_rx_py*2; 
			gb_arm_pz = gb_rx_pz*2;*/

			/*4dof rigid arm region tranfer: ARM :-380~380 => p=(rx-95)*4	<=> tx=p/4+95 , 08072013*/
			gb_arm_px = (gb_rx_px-95)*4; 
			gb_arm_py = (gb_rx_py-95)*4; 
			gb_arm_pz = (gb_rx_pz-95)*4; 
			
			/*4dof rigid arm, 08/09/2013, for tracing test, 0-380, latest*/ //tx=p/2+50			
			/*gb_arm_px = -gb_rx_pz*2;
			gb_arm_py = gb_rx_py*2; 
			gb_arm_pz = (gb_rx_px-50)*2;*/



			/* commands */
			switch (command_hand)
			{
				case 1:       // left plam
					gb_hand_pose_enable = 1; gb_hand_index = 1;	gb_hand_pose_index = 1; // 12/19/2013 update
					break;
				case 2:	      // left fist
					gb_hand_pose_enable = 1; gb_hand_index = 1; gb_hand_pose_index = 37; // 12/19/2013 update
					break;
				case 5: 	  // without left plam	
					gb_hand_pose_enable = 0; gb_hand_index = 1; gb_hand_pose_index = 0; // 12/19/2013 update		
					break;
				case 3: 	  // right plam	
					gb_hand_pose_enable = 1; gb_hand_index = 2; gb_hand_pose_index = 1; // 12/19/2013 update		
					break;
				case 4: 	  // right fist
					gb_hand_pose_enable = 1; gb_hand_index = 2; gb_hand_pose_index = 37; // 12/19/2013 update
					break;
				case 6:       // without right plam
					gb_hand_pose_enable = 0; gb_hand_index = 2; gb_hand_pose_index = 0; // 12/19/2013 update
					break;
				case 7: 	  // enable teach moving mode, default initial system 
					pneumatic_control_enable=1;
					gb_current_moving_mode = 1;  //	teach moving mode 
					breakForLoop = 0; // release break, 06052013				
					GPIO_SetBits(GPIOC, GPIO_Pin_15);  //pump on						
					break;
				case 8: 	  // all initial, may lead error	
					pneumatic_control_enable=0; 
					gb_hand_pose_enable = 0; gb_hand_index = 0; gb_hand_pose_index = 0; // 12/19/2013 update
					gb_current_moving_mode = 0; //	release moving mode 
					Homing(); //Home pos
					GPIO_ResetBits(GPIOC, GPIO_Pin_15); // pump off
					gb_left_touchsensor_switch_cmd = 0; // 08/08/2014 added	
					gb_right_touchsensor_switch_cmd = 0; // 08/08/2014 added	
					break; 
				case 9: 	  // back to home pos 					 
					Homing(); //Home pos
					gb_hand_pose_enable = 0; gb_hand_index = 0; gb_hand_pose_index = 0; // 12/19/2013 update
					gb_left_touchsensor_switch_cmd = 0; // 08/08/2014 added	
					gb_right_touchsensor_switch_cmd = 0; // 08/08/2014 added	
					break;
				case 10: 	  // add vectority input and control, 03152013
					motor_speed_input = (int)uart_speed_map_radio*(int)uart1_rx_buffer[1]; //speed control
					motor_fixed_speed[0] = motor_speed_input;		
					motor_fixed_speed[1] = motor_speed_input;		
					motor_fixed_speed[2] = motor_speed_input;		
					motor_fixed_speed[3] = motor_speed_input;		
					motor_fixed_speed[4] = motor_speed_input;		
					motor_fixed_speed[5] = motor_speed_input;
					break;
				case 11: 	  // clear motor errors, 06052013
					AX_12_CLEAR_ERROR(254);
					gb_hand_pose_enable = 0; gb_hand_index = 0; gb_hand_pose_index = 0; // 12/19/2013 update
					break;
				case 12: 	  //	enable vision tracking moving mode 
					pneumatic_control_enable=1;
					gb_current_moving_mode = 2; //	vision tracking moving mode 
					breakForLoop = 0; // release break, 06052013
					gb_left_touchsensor_switch_cmd = 0; // 08/08/2014 added	
					gb_right_touchsensor_switch_cmd = 0; // 08/08/2014 added	
					// motion follows ->7 -> 12..->8->7->8->9->7 (important, 05282013)
					GPIO_SetBits(GPIOC, GPIO_Pin_15);  //pump on						
					break;			
				case 13: 	  // hand pose control, 01/22/2014 update
					gb_hand_pose_enable = (uint8_t)uart1_rx_buffer[1]; 
					gb_hand_index = (uint8_t)uart1_rx_buffer[2]; 
					gb_hand_pose_index = (uint8_t)uart1_rx_buffer[3];
					break;			
				case 253: 	  // to kinematica intial postion 09172013, for debugging
					InitialPos();
					gb_hand_pose_enable = 0; gb_hand_index = 0; gb_hand_pose_index = 0; // 12/19/2013 update
					break;
				case 254: 	  // debug	 
					AX_12_Ctrl((uint8_t)uart1_rx_buffer[1],(uint8_t)uart1_rx_buffer[2],(uint8_t)uart1_rx_buffer[3]); // ID: command_hand*/
					gb_hand_pose_enable = 0; gb_hand_index = 0; gb_hand_pose_index = 0; // 12/19/2013 update
					break;				
				default: 				
					break;
			}	

		
		
				/*PID pump control*/
			if (pneumatic_control_enable==1)  // <=1mm	means arriving the target pos
			{
					pump_pwm_duty = pid_update(arm_suitable_voltage,AD_value_1, 100,&pid_per);
			}
			else
			{
				pump_pwm_duty=0;// stop
			} 
			
			
			/*infltableHandMotionCtrl, 11062013*/
			//infltableHandMotionCtrl(pneumatic_control_enable, command_hand);
			//infltableHandPose(gb_hand_pose_enable, gb_hand_index, gb_hand_pose_index); // 12/19/2013 added, error here, why?

			
			/*jump to new movement, 06052013*/
			if((command_hand==1)||(command_hand==2)||(command_hand==3)||(command_hand==4)
			||(command_hand==5)||(command_hand==6)||(command_hand==8)||(command_hand==9))
			{
				 if((preMoveCommand!=command_hand)&&(preMoveCommand!=7))//diff hand
				 {
					breakForLoop = 1; // enable break
				 }
				 else if((preMoveCommand==command_hand)&&((gb_pre_rx_px!=gb_rx_px)||(gb_pre_rx_py!=gb_rx_py)||(gb_pre_rx_pz!=gb_rx_pz)))	//same hand but diff command
				 {
					breakForLoop = 1; // enable break
				 }		
			}
				
			// update command
			preMoveCommand = command_hand;
			gb_pre_rx_px = gb_rx_px;
			gb_pre_rx_py = gb_rx_py;
			gb_pre_rx_pz = gb_rx_pz;
			/* control command over*/
				

			//delay_t(100);
			uart1_rx_index=0;
			plan2send_index = 0; //important here, 05222013		
	}	
		
	

	/* LED6 toggling with frequency = 36.62 Hz */
	//STM_EVAL_LEDToggle(LED6);
	capture = TIM_GetCapture4(TIM2);
	TIM_SetCompare4(TIM2, capture + CCR4_Val);
  }
}





/**
  * @brief  This function handles CAN2 RX0 request.
  * @param  None
  * @retval None
  */
void CAN2_RX0_IRQHandler(void)
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
	//TXMSG.DLC = RxMessage_FIFO0.DLC;
	TXMSG.DLC = 4;  	

	
	switch (RxMessage_FIFO0.Data[0])
	{
		case 0x10:       // 请求状态 
			/*return statement*/
			/*TXMSG.Data[0] = RxMessage_FIFO0.Data[0];
		  TXMSG.Data[1] = 0x01; // received 0x10
		  GPIO_SetBits(GPIOE, GPIO_Pin_5);//LED "OFF",on pcb is used for CAN2 communictation display*/	
			break;
		case 0x11:       // 设置状态			
			/*return statement*//*
			TXMSG.Data[0] = RxMessage_FIFO0.Data[0];
			TXMSG.Data[1] = 0x01; // received 0x11
			//GPIO_SetBits(GPIOE, GPIO_Pin_5);//LED "OFF",on pcb is used for CAN2 communictation display*/
			break;
		case 0x12:       // 启动示教动作模式			
			pneumatic_control_enable=1;
			gb_current_moving_mode = 1;  //	teach moving mode 
			breakForLoop = 0; // release break, 06052013	
			gb_monitor_counter = 0; // enable gb_monitor_counter, 10/24/2014
		
		
			/* RX commands, and TX statements of arm and hand, 01/14/2014 new version*/	
			command_hand =(uint8_t)RxMessage_FIFO0.Data[1]; //command
			gb_rx_px =(uint8_t)RxMessage_FIFO0.Data[2];	// px
			gb_rx_py =(uint8_t)RxMessage_FIFO0.Data[3];	// py
			gb_rx_pz =(uint8_t)RxMessage_FIFO0.Data[4]; // pz
				
			/*hand pose control*/
			CAN2_RX_Hand_Pose_Ctrl((uint8_t)RxMessage_FIFO0.Data[1],(uint8_t)RxMessage_FIFO0.Data[2]); 
					
		
			/*return statement*//*		
			TXMSG.Data[0] = RxMessage_FIFO0.Data[0];
			TXMSG.Data[1] = 0x01; // received 0x12
			TXMSG.Data[2] = RxMessage_FIFO0.Data[1];
		  TXMSG.Data[3] = RxMessage_FIFO0.Data[2];
			//GPIO_SetBits(GPIOE, GPIO_Pin_5);//LED "OFF",on pcb is used for CAN2 communictation display*/	

			gb_previous_moving_mode = gb_current_moving_mode;  //update command, 11/06/2014
			break;
		case 0x13:       // 启动手势跟踪模式			
			pneumatic_control_enable=1;
			gb_current_moving_mode = 2; //	vision tracking moving mode 
			breakForLoop = 0; // release break, 06052013  
			gb_left_touchsensor_switch_cmd = 0; // 08/08/2014 added	
			gb_right_touchsensor_switch_cmd = 0; // 08/08/2014 added	
			gb_monitor_counter = 0; // enable gb_monitor_counter, 10/24/2014		

		
			/* RX commands, and TX statements of arm and hand, 01/14/2014 new version*/	
			command_hand =(uint8_t)RxMessage_FIFO0.Data[1]; //command
			gb_rx_px =(uint8_t)RxMessage_FIFO0.Data[2];	// px
			gb_rx_py =(uint8_t)RxMessage_FIFO0.Data[3];	// py
			gb_rx_pz =(uint8_t)RxMessage_FIFO0.Data[4]; // pz
			gb_rx_az =(uint8_t)RxMessage_FIFO0.Data[5]; // hand rotation			

		
			/*transfer data*/
			/*4dof rigid arm region tranfer: ARM :-380~380 => p=(rx-95)*4	<=> tx=p/4+95 , 08072013*/
			gb_arm_px = (gb_rx_px-95)*4; 
			gb_arm_py = (gb_rx_py-95)*4; 
			gb_arm_pz = (gb_rx_pz-95)*4;	

			/*hand pose control*/
			CAN2_RX_Hand_Pose_Ctrl(command_hand,0); 			
			
		
			/*return statement*//*
			TXMSG.Data[0] = RxMessage_FIFO0.Data[0];
			TXMSG.Data[1] = 0x01;  // received 0x13	
		  TXMSG.Data[2] = RxMessage_FIFO0.Data[1];	
			//GPIO_SetBits(GPIOE, GPIO_Pin_5);//LED "OFF",on pcb is used for CAN2 communictation display*/

			gb_previous_moving_mode = gb_current_moving_mode;  //update command, 11/06/2014
			break;
		case 0x14:       // 手臂复位			 
			pneumatic_control_enable=0; 
			gb_hand_pose_enable = 0; gb_hand_index = 0; gb_hand_pose_index = 0; // 12/19/2013 update
			gb_current_moving_mode = 0; //	release moving mode 					
			gb_left_touchsensor_switch_cmd = 0; // 08/08/2014 added	
			gb_right_touchsensor_switch_cmd = 0; // 08/08/2014 added				
				
			breakForLoop = 1; // enable break, 11/27/2014 added
		
			//Homing(); //Home pos, old version	
			// Homing with disable torque function, 11/06/2014
			if(gb_previous_moving_mode != gb_current_moving_mode)
			{			
				Homing();								
			}				
		
			/* RX commands, and TX statements of arm and hand, 01/14/2014 new version*/	
			command_hand =(uint8_t)RxMessage_FIFO0.Data[1]; //command
			gb_rx_px =(uint8_t)RxMessage_FIFO0.Data[2];	// px
			gb_rx_py =(uint8_t)RxMessage_FIFO0.Data[3];	// py
			gb_rx_pz =(uint8_t)RxMessage_FIFO0.Data[4]; // pz

		
			/*return statement*//*
			TXMSG.Data[0] = RxMessage_FIFO0.Data[0];
			TXMSG.Data[1] = 0x01; // received 0x14
			//GPIO_SetBits(GPIOE, GPIO_Pin_5);//LED "OFF",on pcb is used for CAN2 communictation display*/			
			break; 
		case 0x15:       // parameters setting, added 06/27/2014	 								
			/*if ((uint8_t)RxMessage_FIFO0.Data[1] == 0x01)  // setting arm tracking speed, 06/27/2014
			{
				motor_speed_input = (uint8_t)uart_speed_map_radio*(uint8_t)RxMessage_FIFO0.Data[2]; //speed control
				motor_fixed_speed[0] = motor_speed_input;		
				motor_fixed_speed[1] = motor_speed_input;		
				motor_fixed_speed[2] = motor_speed_input;		
				motor_fixed_speed[3] = motor_speed_input;		
				motor_fixed_speed[4] = motor_speed_input;		
				motor_fixed_speed[5] = motor_speed_input;
			}*/		
			
			/*return statement*/
			/*TXMSG.Data[0] = RxMessage_FIFO0.Data[0];
			TXMSG.Data[1] = RxMessage_FIFO0.Data[1];
			TXMSG.Data[2] = RxMessage_FIFO0.Data[2];
			//GPIO_SetBits(GPIOE, GPIO_Pin_5);//LED "OFF",on pcb is used for CAN2 communictation display*/			
			break; 		
		default: /*
			TXMSG.Data[0] = RxMessage_FIFO0.Data[0];
			TXMSG.Data[1] = 0x02; // not received RxMessage_FIFO0.Data[0] command*/
			break;
	}	
	
	
	/*jump to new movement, 06052013*/
	/*if((command_hand==1)||(command_hand==2)||(command_hand==3)||(command_hand==4)
	||(command_hand==5)||(command_hand==6)||(command_hand==8)||(command_hand==9))
	{
		 if((preMoveCommand!=command_hand)&&(preMoveCommand!=7))//diff hand
		 {
			breakForLoop = 1; // enable break
		 }
		 else if((preMoveCommand==command_hand)&&((gb_pre_rx_px!=gb_rx_px)||(gb_pre_rx_py!=gb_rx_py)||(gb_pre_rx_pz!=gb_rx_pz)))	//same hand but diff command
		 {
			breakForLoop = 1; // enable break
		 }		
	}*/
		
	// update command
	preMoveCommand = command_hand;
	gb_pre_rx_px = gb_rx_px;
	gb_pre_rx_py = gb_rx_py;
	gb_pre_rx_pz = gb_rx_pz;	
	/* control command over*/
	

	CAN_Transmit(CAN2,&TXMSG);	
	//GPIO_SetBits(GPIOE,GPIO_Pin_5); //LED "OFF",on pcb is used for CAN2 communictation display	
	CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
	CAN_ITConfig(CAN2,CAN_IT_FMP0, ENABLE);
}



void CAN2_RX_Hand_Pose_Ctrl(uint8_t hand_pose, uint8_t arm_motion)
{
	/* hand pose commands */
	command_hand = hand_pose;
	/*switch (hand_pose)
	{
		case 0x01:       // left plam
		    gb_hand_pose_enable = 1; gb_hand_index = 1;	gb_hand_pose_index = 1; // 12/19/2013 update
			break;
		case 0x02:	      // left fist
			gb_hand_pose_enable = 1; gb_hand_index = 1; gb_hand_pose_index = 37; // 12/19/2013 update
			break;
		case 0x05: 	  // without left plam	
			gb_hand_pose_enable = 0; gb_hand_index = 1; gb_hand_pose_index = 0; // 12/19/2013 update		
			break;
		case 0x03: 	  // right plam	
			gb_hand_pose_enable = 1; gb_hand_index = 2; gb_hand_pose_index = 1; // 12/19/2013 update		
			break;
		case 0x04: 	  // right fist
			gb_hand_pose_enable = 1; gb_hand_index = 2; gb_hand_pose_index = 37; // 12/19/2013 update
			break;
		case 0x06:       // without right plam
			gb_hand_pose_enable = 0; gb_hand_index = 2; gb_hand_pose_index = 0; // 12/19/2013 update
			break; 			
		default: 				
			break;
	}*/
	
	if(arm_motion!=0x02) // not available to shake hand, 08/12/2014 added test
	{
		switch (hand_pose)
		{
			case 0x01:       // left plam
				//gb_hand_pose_enable = 1; gb_hand_index = 1;	gb_hand_pose_index = 1; // 12/19/2013 update
			  gb_hand_pose_enable = 0; gb_hand_index = 1; gb_hand_pose_index = 0; // more stable, 10/24/2014 update	
				break;
			case 0x02:	      // left fist
				gb_hand_pose_enable = 1; gb_hand_index = 1; gb_hand_pose_index = 37; // 12/19/2013 update
				break;
			case 0x05: 	  // without left plam	
				gb_hand_pose_enable = 0; gb_hand_index = 1; gb_hand_pose_index = 0; // 12/19/2013 update		
				break;
			case 0x03: 	  // right plam	
				//gb_hand_pose_enable = 1; gb_hand_index = 2; gb_hand_pose_index = 1; // 12/19/2013 update	
				gb_hand_pose_enable = 0; gb_hand_index = 2; gb_hand_pose_index = 0; // more stable, 10/24/2014 update			
				break;
			case 0x04: 	  // right fist
				gb_hand_pose_enable = 1; gb_hand_index = 2; gb_hand_pose_index = 37; // 12/19/2013 update
				break;
			case 0x06:       // without right plam
				gb_hand_pose_enable = 0; gb_hand_index = 2; gb_hand_pose_index = 0; // 12/19/2013 update
				break; 			
			default: 				
				break;
		}
	}

	

  /* arm motion commands */
	switch (arm_motion)
	{
		case 0x01:  // wave hand
		  gb_rx_px = 0; gb_rx_py = 0; gb_rx_pz = 1; // wave hand 
			break;
		case 0x02:	      // shake hand
			gb_rx_px = 0; gb_rx_py = 1; gb_rx_pz = 0; // shake hand
			break;	 	
		default: 
			gb_rx_px = 0; gb_rx_py = 0; gb_rx_pz = 0;  		
			break;
	}
}




UNS32 OnArmCommandWordUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	printf("Chassis Control Word Update @2000|00...\r\n");
	
	//ARM_D.motion_command = motion_command;
	
  return 0;
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
