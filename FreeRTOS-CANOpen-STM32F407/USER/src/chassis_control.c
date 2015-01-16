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

#include "chassis_control.h"
#include "math.h"

/* Object Dictionary */
#include "CHASSIS_OD.h"

/* Private variables */
xQueueHandle xQ_CHASSIS_MSG = NULL;
xTaskHandle	 xT_CHASSIS 	  = NULL;

Chassis_Data CHASSIS_D;

DRIVE DRIVE1,DRIVE2,DRIVE3;


volatile int lidar_offset_count = 0;
volatile int lidar_message[10] = {0,0,0,0,0,0,0,0,0,0};


volatile int RP_Lidar_Buf[360][1];
volatile int RP_Lidar_Duf_Index = 0;
volatile double repul_force_x[360];
volatile double repul_force_y[360];




void chassis_control_thread(void * pvParameters)
{
	printf("start the chassis control\r\n");
	
	Chassis_Init(&CHASSIS_D);
	
	RegisterSetODentryCallBack(CO_D.CO_CAN2, 0x2000, 0x00, &OnChassisControlWordUpdate);	//Control word 
	
	while(1)
	{
		
		
		/* Check Motor Fault */
		MotorFault();
		
		/* Lidar ??? */
// 		if(CHASSIS_D.Lidar_delay<100)	
// 		{
// 			CHASSIS_D.Lidar_delay++;
// 		}
// 		if(CHASSIS_D.lidar_init_P == FALSE && CHASSIS_D.Lidar_delay>=100)
// 		{
// 			Lidar_Stop();
// 			vTaskDelay(20);	
// 			Lidar_Init();
// 			CHASSIS_D.lidar_init_P = true;
// 		}

// 		/* The stepwise acceleration/deceleration function */
 		StepwiseFunction(&CHASSIS_D);
// 		
// 		/* Chassis Motion Control */
 		ChassisMotionCtrl(&CHASSIS_D);

		
		
		//printf("chassis control is running\r\n");
		printf("ODODODODO:0x2000|00: %x \r\n",motion_command);
		bsp_LedToggle(2,100);
		vTaskDelay(CHASSIS_CONTROL_THREAD_DELAY_TIMER);
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




void start_chassis_control(void)
{
	xTaskCreate(chassis_control_thread, "lifter_control", CHASSIS_CONTROL_THREAD_STACK, NULL,CHASSIS_CONTROL_THREAD_PRIO, &xT_CHASSIS);
	if(NULL == xT_CHASSIS)
	{
	 	printf("chassis control thread creat failed!\r\n");
	}else
	{
		printf("chassis control thread creat successfully!\r\n");
	}
}




void Chassis_Init(Chassis_Data *ch)
{
	Motor_Init();
	
	ch->Lidar_delay = 0;  
	ch->lidar_init_P = false;
	ch->lidar_init_ok_P = false;
	
	ch->target_speed1 = 0;					// Angular speed of wheel 1 [target]
	ch->target_speed2 = 0;
	ch->target_speed3 = 0;
	ch->drive_speed1 = 0;					// Angular speed of wheel 1 [driving]
	ch->drive_speed2 = 0;
	ch->drive_speed3 = 0;
	ch->motion_command = 30;						// the motion command of the chassis
	
	
	
	ch->angle = 0;
	ch->angle_diff = 0;
	ch->prev_angle = 0;
	ch->prev_angle_diff = 0;
	ch->magnitude = 0;
	ch->magnitude_diff = 0;
	ch->prev_magnitude = 0;
	ch->prev_magnitude_diff = 0;
	ch->Kp = 60;
	ch->Kd = 1;
}


void ChassisMotionCtrl(Chassis_Data *ch)
{
	int ii;	
	switch(ch->motion_command)
	{
		case 0:		// No keys is pressed --> stop the chassis
			ch->target_speed1  = 0; ch->target_speed2  = 0; ch->target_speed3  = 0;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			break;
		case 1:		// Key D is pressed	--> right turning
			ch->target_speed1  = 0; ch->target_speed2  = 0; ch->target_speed3  = SPEED_CONSTANT/2;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			break;
		case 2:		// Key S is pressed	--> move backward
			// compute the resultant speed
			ch->result_speed_x = 0;
			ch->result_speed_y = 0;
			for(ii=0;ii<360;ii++)
			{
				if(ii>89 && ii<271)
				{
					ch->result_speed_x += repul_force_x[ii];
					ch->result_speed_y += repul_force_y[ii];
				}
			}
			ch->result_speed_y += -SPEED_CONSTANT;
				
			// reverse motion filtering
			if(ch->result_speed_y>0)
			{
				ch->result_speed_y = 0;
			}
			
			ch->result_speed = sqrt(ch->result_speed_x * ch->result_speed_x + ch->result_speed_y * ch->result_speed_y);
			if(ch->result_speed > SPEED_CONSTANT)
			{
					ch->result_speed_x = ch->result_speed_x*SPEED_CONSTANT/ch->result_speed;
					ch->result_speed_y = ch->result_speed_y*SPEED_CONSTANT/ch->result_speed;
			}
			else if(ch->result_speed<SPEED_CONSTANT_MIN)
			{
				ch->result_speed_x = 0;
				ch->result_speed_y = 0;
			}
					
			ch->target_speed1  = ch->result_speed_x; 
			ch->target_speed2  = ch->result_speed_y; 
			ch->target_speed3  = 0;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);
			
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			break;
		case 4:		// Key A is pressed	--> left turning
			ch->target_speed1  = 0; ch->target_speed2  = 0; ch->target_speed3  = -SPEED_CONSTANT/2;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			break;
		case 8:		// Key E is pressed	--> move to right
			
			// compute the resultant speed
			ch->result_speed_x = 0;
			ch->result_speed_y = 0;
			for(ii=0;ii<360;ii++)
			{
				if(ii>-1 && ii<181)
				{
					ch->result_speed_x += repul_force_x[ii];
					ch->result_speed_y += repul_force_y[ii];
				}
			}
			ch->result_speed_x += SPEED_CONSTANT;

			// reverse motion filtering
			if(ch->result_speed_x<0)
			{
				ch->result_speed_x = 0;
			}		
			ch->result_speed = sqrt(ch->result_speed_x*ch->result_speed_x + ch->result_speed_y*ch->result_speed_y);
			if(ch->result_speed > SPEED_CONSTANT)
			{
					ch->result_speed_x = ch->result_speed_x*SPEED_CONSTANT/ch->result_speed;
					ch->result_speed_y = ch->result_speed_y*SPEED_CONSTANT/ch->result_speed;
			}
			else if(ch->result_speed<SPEED_CONSTANT_MIN)
			{
				ch->result_speed_x = 0;
				ch->result_speed_y = 0;
			}

			
			ch->target_speed1  = ch->result_speed_x; 
			ch->target_speed2  = ch->result_speed_y; 
			ch->target_speed3  = 0;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);
			
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			break;
		case 16:		// Key W is pressed	--> move forward
			// compute the resultant speed
			ch->result_speed_x = 0;
			ch->result_speed_y = 0;
			for(ii=0;ii<360;ii++)
			{
				if((ii>-1&&ii<91)||(ii>269&&ii<360))
				{
					ch->result_speed_x += repul_force_x[ii];
					ch->result_speed_y += repul_force_y[ii];
				}
			}
			ch->result_speed_y += SPEED_CONSTANT;
			
			// reverse motion filtering
			if(ch->result_speed_y<0)
			{
				ch->result_speed_y = 0;	
			}
		
			ch->result_speed = sqrt(ch->result_speed_x*ch->result_speed_x + ch->result_speed_y*ch->result_speed_y);
			if(ch->result_speed > SPEED_CONSTANT)
			{
					ch->result_speed_x = ch->result_speed_x*SPEED_CONSTANT/ch->result_speed;
					ch->result_speed_y = ch->result_speed_y*SPEED_CONSTANT/ch->result_speed;
			}
			else if(ch->result_speed<SPEED_CONSTANT_MIN)
			{
				ch->result_speed_x = 0;
				ch->result_speed_y = 0;
			}
			
			
			ch->target_speed1  = ch->result_speed_x; ch->target_speed2  = ch->result_speed_y; ch->target_speed3  = 0;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);
						
			 
// 			sprintf(message,"%d %d %d \r\n",target_speed1, target_speed2,123);
// 			USART1_Puts(message);
			
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);		
			break;
		case 17:		// Keys W + D are pressed --> turn right [forward]
			
			// compute the resultant speed
			ch->result_speed_x = 0;
			ch->result_speed_y = 0;
			for(ii=0;ii<360;ii++)
			{
				if((ii>-1&&ii<136)||(ii>314&&ii<360))
				{
					ch->result_speed_x += repul_force_x[ii];
					ch->result_speed_y += repul_force_y[ii];
				}
			}
			ch->result_speed_y += SPEED_CONSTANT;

			// reverse motion filtering
			if(ch->result_speed_y<0)
			{
				ch->result_speed_y = 0;	
			}
			if(ch->result_speed > SPEED_CONSTANT)
			{
					ch->result_speed_x = ch->result_speed_x*SPEED_CONSTANT/ch->result_speed;
					ch->result_speed_y = ch->result_speed_y*SPEED_CONSTANT/ch->result_speed;
			}
			else if(ch->result_speed<SPEED_CONSTANT_MIN)
			{
				ch->result_speed_x = 0;
				ch->result_speed_y = 0;
			}

			ch->target_speed1  = ch->result_speed_x; ch->target_speed2  = ch->result_speed_y; ch->target_speed3  = 0;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);
			
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			break;
		case 20:		// Keys W + A are pressed --> turn left [forward]
			
			// compute the resultant speed
			ch->result_speed_x = 0;
			ch->result_speed_y = 0;
			for(ii=0;ii<360;ii++)
			{
				if((ii>-1 && ii<46)||(ii>224&&ii<360))
				{
					ch->result_speed_x += repul_force_x[ii];
					ch->result_speed_y += repul_force_y[ii];
				}
			}
			ch->result_speed_y += SPEED_CONSTANT;
			
			// reverse motion filtering
			if(ch->result_speed_y<0)
			{
				ch->result_speed_y = 0;	
			}

			ch->result_speed = sqrt(ch->result_speed_x*ch->result_speed_x + ch->result_speed_y*ch->result_speed_y);
			if(ch->result_speed > SPEED_CONSTANT)
			{
					ch->result_speed_x = ch->result_speed_x*SPEED_CONSTANT/ch->result_speed;
					ch->result_speed_y = ch->result_speed_y*SPEED_CONSTANT/ch->result_speed;
			}
			else if(ch->result_speed<SPEED_CONSTANT_MIN)
			{
				ch->result_speed_x = 0;
				ch->result_speed_y = 0;
			}

			ch->target_speed1  = ch->result_speed_x; ch->target_speed2  = ch->result_speed_y; ch->target_speed3  = 0;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);

			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			
			break;
		case 24:		  // Keys W + E are pressed	--> move to right [forward]
			
			// compute the resultant speed
			ch->result_speed_x = 0;
			ch->result_speed_y = 0;
			for(ii=0;ii<360;ii++)
			{
				if((ii>-1 && ii<136)||(ii>314&&ii<360))
				{
					ch->result_speed_x += repul_force_x[ii];
					ch->result_speed_y += repul_force_y[ii];
				}
			}
			ch->result_speed_y += SPEED_CONSTANT;
			
			// reverse motion filtering
			if(ch->result_speed_y<0)
			{
				ch->result_speed_y = 0;	
			}
			
			ch->result_speed = sqrt(ch->result_speed_x*ch->result_speed_x + ch->result_speed_y*ch->result_speed_y);
			if(ch->result_speed > SPEED_CONSTANT)
			{
					ch->result_speed_x = ch->result_speed_x*SPEED_CONSTANT/ch->result_speed;
					ch->result_speed_y = ch->result_speed_y*SPEED_CONSTANT/ch->result_speed;
			}
			else if(ch->result_speed<SPEED_CONSTANT_MIN)
			{
				ch->result_speed_x = 0;
				ch->result_speed_y = 0;
			}

			ch->target_speed1  = ch->result_speed_x; ch->target_speed2  = ch->result_speed_y; ch->target_speed3  = 0;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);
	
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			break;
		case 30:
		break;
		case 32:		// Key Q is pressed --> move to left
			
			// compute the resultant speed
			ch->result_speed_x = 0;
			ch->result_speed_y = 0;
			for(ii=0;ii<360;ii++)
			{
				if(ii>179&&ii<360)
				{
					ch->result_speed_x += repul_force_x[ii];
					ch->result_speed_y +=repul_force_y[ii];
				}
			}
			ch->result_speed_x += -SPEED_CONSTANT;
			
			// reverse motion filtering
			if(ch->result_speed_x>0)
			{
				ch->result_speed_x = 0;
			}

			ch->result_speed = sqrt(ch->result_speed_x*ch->result_speed_x + ch->result_speed_y*ch->result_speed_y);
			if(ch->result_speed > SPEED_CONSTANT)
			{
					ch->result_speed_x = ch->result_speed_x*SPEED_CONSTANT/ch->result_speed;
					ch->result_speed_y = ch->result_speed_y*SPEED_CONSTANT/ch->result_speed;
			}
			else if(ch->result_speed<SPEED_CONSTANT_MIN)
			{
				ch->result_speed_x = 0;
				ch->result_speed_y = 0;
			}

			ch->target_speed1  = ch->result_speed_x; ch->target_speed2  = ch->result_speed_y; ch->target_speed3  = 0;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);
		
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			break;
		case 48:		// Keys W + Q are pressed --> move to left [forward]
			
			// compute the resultant speed
			ch->result_speed_x = 0;
			ch->result_speed_y = 0;
			for(ii=0;ii<360;ii++)
			{
				if((ii>-1 && ii<46)||(ii>224 && ii<360))
				{
					ch->result_speed_x += repul_force_x[ii];
					ch->result_speed_y += repul_force_y[ii];
				}
			}
			ch->result_speed_y += SPEED_CONSTANT;
			
			// reverse motion filtering
			if(ch->result_speed_y<0)
			{
				ch->result_speed_y = 0;	
			}

			ch->result_speed = sqrt(ch->result_speed_x*ch->result_speed_x + ch->result_speed_y*ch->result_speed_y);
			if(ch->result_speed > SPEED_CONSTANT)
			{
					ch->result_speed_x = ch->result_speed_x*SPEED_CONSTANT/ch->result_speed;
					ch->result_speed_y = ch->result_speed_y*SPEED_CONSTANT/ch->result_speed;
			}
			else if(ch->result_speed<SPEED_CONSTANT_MIN)
			{
				ch->result_speed_x = 0;
				ch->result_speed_y = 0;
			}

			ch->target_speed1  = ch->result_speed_x; ch->target_speed2  = ch->result_speed_y; ch->target_speed3  = 0;
			DRIVE1.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,1);
			DRIVE2.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,2);
			DRIVE3.V_SET=chassis_drive(ch->drive_speed1,ch->drive_speed2,ch->drive_speed3,3);
		
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			break;
		default:
			DRIVE1.V_SET=chassis_drive(0,0,0,1);
			DRIVE2.V_SET=chassis_drive(0,0,0,2);
			DRIVE3.V_SET=chassis_drive(0,0,0,3);
			chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
			break;
	}
} 



UNS32 OnChassisControlWordUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
	printf("Chassis Control Word Update @2000|00...\r\n");
	
	CHASSIS_D.motion_command = motion_command;
	
  return 0;
}





double chassis_drive(double X_out,double Y_out,double Theta_out,u8 wheel)
{
 double VALUE[3];
	
	VALUE[0] = X_out*sin(Alpha1) + Y_out*cos(Alpha1) + L1O*Theta_out*Pi/180;
  VALUE[1] = X_out*sin(Alpha2) + Y_out*cos(Alpha2) + L2O*Theta_out*Pi/180;
  VALUE[2] = X_out*sin(Alpha3) + Y_out*cos(Alpha3) + L3O*Theta_out*Pi/180;
	
	
 if (wheel==1)
  return VALUE[0];
 else
 if (wheel==2)
	return VALUE[1];	
 else
 if (wheel==3)
	return VALUE[2];
 else
	return 0; 
}




void StepwiseFunction(Chassis_Data *chassis)
{
	chassis->drive_diff1 = chassis->drive_speed1 - chassis->target_speed1; 
	chassis->drive_diff2 = chassis->drive_speed2 - chassis->target_speed2; 
	chassis->drive_diff3 = chassis->drive_speed3 - chassis->target_speed3;
	
	if(chassis->drive_diff1>0)
	{
		chassis->drive_speed1 -= abs(chassis->drive_diff1)/SPEED_STEP;
	}
	else if(chassis->drive_diff1<0)
	{
		chassis->drive_speed1 += abs(chassis->drive_diff1)/SPEED_STEP;
	}

	if(abs(chassis->drive_speed1)<SPEED_CONSTANT_MIN)
	{
		chassis->drive_speed1 = 0;
	}

	if(chassis->drive_diff2>0)
	{
		chassis->drive_speed2 -= abs(chassis->drive_diff2)/SPEED_STEP;			
	}
	else if(chassis->drive_diff2<0)
	{
		chassis->drive_speed2 += abs(chassis->drive_diff2)/SPEED_STEP;
	}
	if(abs(chassis->drive_speed2)<SPEED_CONSTANT_MIN)
	{
		chassis->drive_speed2 = 0;
	}

	if(chassis->drive_diff3>0)
	{
		chassis->drive_speed3 -= abs(chassis->drive_diff3)/SPEED_STEP;
	}
	else if(chassis->drive_diff3<0)
	{
		chassis->drive_speed3 += abs(chassis->drive_diff3)/SPEED_STEP;
	}
	if(abs(chassis->drive_speed3)<SPEED_CONSTANT_MIN)
	{
		chassis->drive_speed3 = 0;
	}
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
