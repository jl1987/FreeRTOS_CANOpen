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
#include "CHASSIS_OD.h"

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
 	SENSOR_Force_Init();
 	SENSOR_Lidar_Init();
 	SENSOR_IR_Distance_Init();
 	SENSOR_IR_Autocharge_Init();
 	SENSOR_Anticollision();
}

// void GetDataFromForceSensor(void *force)
// {
// 	int ii;	 
// 	int x_value, y_value;
// 	double angle = 0;
// 	double angle_diff = 0;
// 	double prev_angle = 0;
// 	double prev_angle_diff = 0;
// 	double magnitude = 0;
// 	double magnitude_diff = 0;
// 	double prev_magnitude = 0;
// 	double prev_magnitude_diff = 0;
// 	double Kp = 60;
// 	double Kd = 1;
// 	while(1)
// 	{
// 		if(force_calibration_count<FORCE_SENSOR_CALIBRATION_LEN)
// 		{
// 			for(ii=0;ii<8;ii++)
// 			{
// 				force_sensor_offset[ii]+=ADC3ConvertedValue[ii];
// 			}
// 			force_calibration_count++;
// 		}									 
// 		else if(force_calibration_count == FORCE_SENSOR_CALIBRATION_LEN)
// 		{
// 			for(ii=0;ii<8;ii++)
// 			{
// 				force_sensor_offset[ii] = force_sensor_offset[ii]/FORCE_SENSOR_CALIBRATION_LEN;
// 			}
// 			force_calibration_count++;
// 			USART1_Puts("OK");
// 		}	
// 		else
// 		{
// 			for(ii=0;ii<8;ii++)
// 	
// 			{
// 				force_temp[ii] = ADC3ConvertedValue[ii]-force_sensor_offset[ii];
// 				if(abs(force_temp[ii])<=30)
// 				{
// 					 force_temp[ii] = 0;
// 				}
// 			}
// 			for(ii=0;ii<4;ii++)
// 			{
// 				if(abs(force_temp[ii*2+1]-force_temp[ii*2])>50)
// 				{
// 					force_value[ii] = -1*force_temp[ii*2+1];
// 				}
// 				else
// 				{
// 					force_value[ii] = force_temp[ii*2];
// 				}
// 			}
// 		}
// 		
// 		
// 		if(force_value[0]<force_value[2])
// 		{
// 			y_value = (abs(force_value[0])+force_value[2]);
// 		}
// 		else
// 		{
// 			y_value = -1*(force_value[0]+abs(force_value[2]));
// 		}
// 		if(force_value[1]<force_value[3])
// 		{
// 			x_value = (abs(force_value[1])+force_value[3]);
// 		}
// 		else
// 		{
// 			x_value = -1*(force_value[1]+abs(force_value[3]));
// 		}
// 		
// 		angle = atan2(y_value,x_value);
// 		angle_diff = angle - prev_angle;
// 		magnitude = FORCE_SENSOR_CENDIST*sqrt(pow(x_value,2)+pow(y_value,2))/Height;
// 		magnitude_diff = magnitude - prev_magnitude;
// 		result_speed = -Kp*magnitude - Kd*(magnitude_diff - prev_magnitude_diff);
// 		prev_angle = angle;
// 		prev_magnitude = magnitude;
// 		prev_angle_diff = angle_diff;
// 		prev_magnitude_diff = magnitude_diff;
// 		
// 		if(result_speed>SPEED_CONSTANT)
// 		{
// 			result_speed = SPEED_CONSTANT;
// 		}
// 		else if(result_speed<-SPEED_CONSTANT)
// 		{
// 			result_speed = -SPEED_CONSTANT;
// 		}
// 		result_speed_x = result_speed*sin(angle);
// 		result_speed_y = -result_speed*cos(angle);
// 		target_speed1 = result_speed_x;
// 		target_speed2 = result_speed_y;
// 	
// 		DRIVE1.V_SET=chassis_drive(drive_speed1,drive_speed2,0,1);
// 		DRIVE2.V_SET=chassis_drive(drive_speed1,drive_speed2,0,2);
// 		DRIVE3.V_SET=chassis_drive(drive_speed1,drive_speed2,0,3);
// 		chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
// 		OSTimeDly(25);
//  	}
// 	
// }	

// void Infrared_Distance_task (void *pdata)
// {
// 	int pp;
// 	double AD_value_temp;
// //	float AD_value_temp_actual;
// 	double V_5v;

// 	while(1)
// 	{	
// 		/*	 		 
// 		for(pp=0;pp<SHARP_DISTANCE_SENSOR_NUM;pp++)
// 		{
// 			AD_value_temp = ADC1ConvertedValue[pp];
// 	
// 			// transfer for GP2D120 IR sensor
// 			// original 1024 <->5V for PIC18C521
// 			// current 4096 <->3.3V for STM32F407 
// 			//V_5v=AD_value_1*3.3/20;	
// 			V_5v=AD_value_temp*0.165;		            
// 			sharp_ir_distance[pp] = 2914/(V_5v+5)-1; //for DP2D120
// 			sharp_ir_distance[pp] = sharp_ir_distance[pp]*10;
// 		//	printf("%4.2f mm\r\n", distance);    	 
// 		}
// 		*/

// 		if(sharp_calibration_count<SHARP_SENSOR_CALIBRATION_LEN)
// 		{
// 			for(pp=0;pp<SHARP_DISTANCE_SENSOR_NUM;pp++)
// 			{
// 				AD_value_temp = ADC1ConvertedValue[pp];	
// 				V_5v=AD_value_temp*0.165;		            
// 				sharp_ir_distance[pp] = 2914/(V_5v+5)-1; //for DP2D120
// 				sharp_ir_distance[pp] = sharp_ir_distance[pp]*10;
// 				sharp_sensor_offset[pp]+=sharp_ir_distance[pp];
// 			}
// 			sharp_calibration_count++;
// 		}									 
// 		else if(sharp_calibration_count == SHARP_SENSOR_CALIBRATION_LEN)
// 		{
// 			for(pp=0;pp<SHARP_DISTANCE_SENSOR_NUM;pp++)
// 			{
// 				sharp_sensor_offset[pp] = sharp_sensor_offset[pp]/SHARP_SENSOR_CALIBRATION_LEN;
// 			}
// 			sharp_calibration_count++;
// 			USART1_Puts("OK");
// 		}	
// 		else
// 		{
// 			for(pp=0;pp<SHARP_DISTANCE_SENSOR_NUM;pp++)
// 			{
// 				AD_value_temp = ADC1ConvertedValue[pp];	
// 				V_5v=AD_value_temp*0.165;		            
// 				sharp_ir_distance[pp] = 2914/(V_5v+5)-1; //for DP2D120
// 				sharp_ir_distance[pp] = sharp_ir_distance[pp]*10;
// 				sharp_value[pp] = sharp_ir_distance[pp]-sharp_sensor_offset[pp];
// 				if(abs(sharp_value[pp])<=50)
// 				{
// 					 sharp_fall_value[pp] = 0;
// 				}
// 				else
// 				{
// 					sharp_fall_value[pp] = 1;
// 				}
// 			}
// 		}

// 		switch(motion_command)
// 		{
// 			case 0:		// No keys is pressed --> stop the chassis
// 				break;
// 			case 1:		// Key D is pressed	--> right turning
// 				break;
// 			case 2:		// Key S is pressed	--> move backward
// 				if(IR_Switch_enable_P)
// 				{
// 					// if there exist at least one sensor on each size of the moving direction detected a falling risk
// 					if(sharp_fall_value[2]&&sharp_fall_value[3])
// 					{
// 						// stop the chassis
// 					   	DRIVE1.V_SET=chassis_drive(0,0,0,1); DRIVE2.V_SET=chassis_drive(0,0,0,2); DRIVE3.V_SET=chassis_drive(0,0,0,3);
// 					}
// 					else if	(sharp_fall_value[2])
// 					{
// 						//move to the other side
// 						DRIVE1.V_SET=chassis_drive(-SPEED_CONSTANT,0,0,1); DRIVE2.V_SET=chassis_drive(-SPEED_CONSTANT,0,0,2); DRIVE3.V_SET=chassis_drive(-SPEED_CONSTANT,0,0,3);
// 					}
// 					else if	(sharp_fall_value[3])
// 					{
// 						//move to the other side
// 						DRIVE1.V_SET=chassis_drive(SPEED_CONSTANT,0,0,1); DRIVE2.V_SET=chassis_drive(SPEED_CONSTANT,0,0,2); DRIVE3.V_SET=chassis_drive(SPEED_CONSTANT,0,0,3);
// 					}
// 				}
// 				break;
// 			case 4:		// Key A is pressed	--> left turning
// 				break;
// 			case 8:		// Key E is pressed	--> move to right
// 				if(IR_Switch_enable_P)
// 				{
// 					// if there exist at least one sensor on each size of the moving direction detected a falling risk
// 					if( sharp_fall_value[1])
// 					{
// 					   	// stop the chassis
// 					   	DRIVE1.V_SET=chassis_drive(0,0,0,1); DRIVE2.V_SET=chassis_drive(0,0,0,2); DRIVE3.V_SET=chassis_drive(0,0,0,3);
// 					}
// 					else if	(sharp_fall_value[0])
// 					{
// 						//move to the other side
// 						DRIVE1.V_SET=chassis_drive(0,-SPEED_CONSTANT,0,1); DRIVE2.V_SET=chassis_drive(0,-SPEED_CONSTANT,0,2); DRIVE3.V_SET=chassis_drive(0,-SPEED_CONSTANT,0,3);
// 					}
// 					else if	(sharp_fall_value[2])
// 					{
// 						//move to the other side
// 						DRIVE1.V_SET=chassis_drive(0,SPEED_CONSTANT,0,1); DRIVE2.V_SET=chassis_drive(0,SPEED_CONSTANT,0,2); DRIVE3.V_SET=chassis_drive(0,SPEED_CONSTANT,0,3);
// 					}
// 				}
// 				break;
// 			case 16:		// Key W is pressed	--> move forward
// 				if(IR_Switch_enable_P)
// 				{
// 					// if there exist at least one sensor on each size of the moving direction 
// 					if(sharp_fall_value[0]&&sharp_fall_value[5])
// 					{
// 					   	// stop the chassis
// 						DRIVE1.V_SET=chassis_drive(0,0,0,1); DRIVE2.V_SET=chassis_drive(0,0,0,2); DRIVE3.V_SET=chassis_drive(0,0,0,3);
// 					}
// 					else if	(sharp_fall_value[0])
// 					{
// 						//move to the other side
// 						DRIVE1.V_SET=chassis_drive(-SPEED_CONSTANT,0,0,1); DRIVE2.V_SET=chassis_drive(-SPEED_CONSTANT,0,0,2); DRIVE3.V_SET=chassis_drive(-SPEED_CONSTANT,0,0,3);
// 					}
// 					else if	(sharp_fall_value[5])
// 					{
// 						//move to the other side
// 						DRIVE1.V_SET=chassis_drive(SPEED_CONSTANT,0,0,1); DRIVE2.V_SET=chassis_drive(SPEED_CONSTANT,0,0,2); DRIVE3.V_SET=chassis_drive(SPEED_CONSTANT,0,0,3);
// 					}
// 				}
// 				break;
// 			case 17:		// Keys W + D are pressed --> turn right [forward]
// 				if(IR_Switch_enable_P)	// if the obstacle avoidance function is in used
// 				{
// 				}
// 				break;
// 			case 20:		// Keys W + A are pressed --> turn left [forward]
// 				if(IR_Switch_enable_P)	// if the obstacle avoidance function is in used
// 				{	
// 				}
// 				break;
// 			case 24:		  // Keys W + E are pressed	--> move to right [forward]
// 				if(IR_Switch_enable_P)
// 				{
// 					// if there exist at least one sensor on each size of the moving direction 
// 					if(sharp_fall_value[0]&&sharp_fall_value[1])
// 					{
// 					   	// stop the chassis
// 						DRIVE1.V_SET=chassis_drive(0,0,0,1); DRIVE2.V_SET=chassis_drive(0,0,0,2); DRIVE3.V_SET=chassis_drive(0,0,0,3);
// 					}
// 					else if	(sharp_fall_value[0])
// 					{
// 						//move to the other side
// 						DRIVE1.V_SET=chassis_drive(SPEED_CONSTANT*COS_45,-SPEED_CONSTANT*SIN_45,0,1); DRIVE2.V_SET=chassis_drive(SPEED_CONSTANT*COS_45,-SPEED_CONSTANT*SIN_45,0,2); DRIVE3.V_SET=chassis_drive(SPEED_CONSTANT*COS_45,-SPEED_CONSTANT*SIN_45,0,3);
// 					}
// 					else if	(sharp_fall_value[1])
// 					{
// 						//move to the other side
// 						DRIVE1.V_SET=chassis_drive(-SPEED_CONSTANT*COS_45,SPEED_CONSTANT*SIN_45,0,1); DRIVE2.V_SET=chassis_drive(-SPEED_CONSTANT*COS_45,SPEED_CONSTANT*SIN_45,0,2); DRIVE3.V_SET=chassis_drive(-SPEED_CONSTANT*COS_45,SPEED_CONSTANT*SIN_45,0,3);
// 					}
// 				}
// 				break;
// 			case 32:		// Key Q is pressed --> move to left
// 				if(IR_Switch_enable_P)
// 				{
// 					// if there exist at least one sensor on each size of the moving direction 
// 					if( sharp_fall_value[4])
// 					{
// 					   	// stop the chassis
// 						DRIVE1.V_SET=chassis_drive(0,0,0,1); DRIVE2.V_SET=chassis_drive(0,0,0,2); DRIVE3.V_SET=chassis_drive(0,0,0,3);
// 					}
// 					else if	(sharp_fall_value[3])
// 					{
// 						//move to the other side
// 						DRIVE1.V_SET=chassis_drive(0,SPEED_CONSTANT,0,1); DRIVE2.V_SET=chassis_drive(0,SPEED_CONSTANT,0,2); DRIVE3.V_SET=chassis_drive(0,SPEED_CONSTANT,0,3);
// 					}
// 					else if	(sharp_fall_value[5])
// 					{
// 						//move to the other side
// 						DRIVE1.V_SET=chassis_drive(0,-SPEED_CONSTANT,0,1); DRIVE2.V_SET=chassis_drive(0,-SPEED_CONSTANT,0,2); DRIVE3.V_SET=chassis_drive(0,-SPEED_CONSTANT,0,3);
// 					}
// 				}
// 				break;
// 			case 48:		// Keys W + Q are pressed --> move to left [forward]
// 				if(IR_Switch_enable_P)
// 				{
// 					// if there exist at least one sensor on each size of the moving direction 
// 					if(sharp_fall_value[4]&&sharp_fall_value[5])
// 					{
// 					   	// stop the chassis
// 						DRIVE1.V_SET=chassis_drive(0,0,0,1); DRIVE2.V_SET=chassis_drive(0,0,0,2); DRIVE3.V_SET=chassis_drive(0,0,0,3);
// 					}
// 					else if	(sharp_fall_value[4])
// 					{
// 						//move to the other side
// 						DRIVE1.V_SET=chassis_drive(SPEED_CONSTANT*COS_45,SPEED_CONSTANT*SIN_45,0,1); DRIVE2.V_SET=chassis_drive(SPEED_CONSTANT*COS_45,SPEED_CONSTANT*SIN_45,0,2); DRIVE3.V_SET=chassis_drive(SPEED_CONSTANT*COS_45,SPEED_CONSTANT*SIN_45,0,3);
// 					}
// 					else if	(sharp_fall_value[5])
// 					{
// 						//move to the other side
// 						DRIVE1.V_SET=chassis_drive(-SPEED_CONSTANT*COS_45,-SPEED_CONSTANT*SIN_45,0,1); DRIVE2.V_SET=chassis_drive(-SPEED_CONSTANT*COS_45,-SPEED_CONSTANT*SIN_45,0,2); DRIVE3.V_SET=chassis_drive(-SPEED_CONSTANT*COS_45,-SPEED_CONSTANT*SIN_45,0,3);
// 					}
// 				}
// 				break;
// 			default:
// 			break;
// 		}

// 		OSTimeDly(50);
// 	}
// }
// void AUTOCHARGE_task(void* pdata)
// {
// 	char message[100];
// 	while(1)
// 	 {		 
// 	   ///检测第1个传感器是否收到
// 		 if (Auto_charge.Infared0_time<100)Auto_charge.Infared0_time++;
// 		
// 		 if ((Auto_charge.Infared0_time_end>=Auto_charge_Infared0_time_MIN)&&(Auto_charge.Infared0_time_end<=Auto_charge_Infared0_time_MAX))
// 		 {
// 		 	if (Auto_charge.Infared0_collected<50)Auto_charge.Infared0_collected++;	 
// 	   	 }
// 		 else
// 		 {
// 		 	Auto_charge.Infared0_collected=0;
// 		 }
// 		 
// 		 ///检测第2个传感器是否收到
// 		 if (Auto_charge.Infared1_time<100)Auto_charge.Infared1_time++;
// 		
// 		 if ((Auto_charge.Infared1_time_end>=Auto_charge_Infared1_time_MIN)&&(Auto_charge.Infared1_time_end<=Auto_charge_Infared1_time_MAX))
// 		 {
// 	     	if (Auto_charge.Infared1_collected<50)Auto_charge.Infared1_collected++;	 
// 	   	 }
// 		 else
// 		 {
// 		 	Auto_charge.Infared1_collected=0; 
// 		 }
// 		 //防止误入中断
// 		 if(Auto_charge.Infared0_time>=10)Auto_charge.Infared0_collected=0;
// 		 if(Auto_charge.Infared1_time>=10)Auto_charge.Infared1_collected=0;	 
// 		 
// 		 //判断何去何从
// 		 if ((Auto_charge.Infared0_collected>Auto_charge_Infared_collected_valve)&&(Auto_charge.Infared1_collected==0))
// 		 {
// 		 	if(previous_charge_status!=101)
// 	     	{
// 				Auto_charge.receive_status=49;
// 			 	//MSG_U1_T[0]=0x01;  //向左转
// 			 	GPIO_WriteBit(LEDPORT, LED2, Bit_RESET);
// 			 	GPIO_WriteBit(LEDPORT, LED3, Bit_SET);
// 			 	GPIO_WriteBit(LEDPORT, LED4, Bit_SET);
// 			}
// 	   	}
// 		else
// 		{
// 			if ((Auto_charge.Infared1_collected>Auto_charge_Infared_collected_valve)&&(Auto_charge.Infared0_collected==0))
// 		 	{
// 	     		if(previous_charge_status!=101)
// 	     		{
// 					 Auto_charge.receive_status=51;
// 					 //MSG_U1_T[0]=0x04;  //向右转
// 					 GPIO_WriteBit(LEDPORT, LED4, Bit_RESET);
// 					 GPIO_WriteBit(LEDPORT, LED2, Bit_SET);
// 					 GPIO_WriteBit(LEDPORT, LED3, Bit_SET);
// 				}
// 	   		}
// 			else
// 			{
// 				if ((Auto_charge.Infared0_collected>Auto_charge_Infared_collected_valve)&&(Auto_charge.Infared1_collected>Auto_charge_Infared_collected_valve))
// 		 		{
// 	     		Auto_charge.receive_status=50;
// 				previous_charge_status=50;
// 				 //MSG_U1_T[0]=0x10;  //向前走
// 				 GPIO_WriteBit(LEDPORT, LED3, Bit_RESET);
// 				 GPIO_WriteBit(LEDPORT, LED2, Bit_SET);
// 				 GPIO_WriteBit(LEDPORT, LED4, Bit_SET);
// 	   			}
// 				else
// 		 		{
// 					if ((Auto_charge.Infared0_collected==0)&&(Auto_charge.Infared1_collected==0))
// 			 		{
// 					     Auto_charge.receive_status=101;
// 						 previous_charge_status=101;
// 						 GPIO_WriteBit(LEDPORT, LED2, Bit_SET);
// 						 GPIO_WriteBit(LEDPORT, LED3, Bit_SET);
// 						 GPIO_WriteBit(LEDPORT, LED4, Bit_SET);
// 		   			}
// 				}
// 			}
// 		}
// 		
// 		// The force sensors based anti-tilting algorithm
// 		if(auto_charge_P && motion_command!=0)	 
// 		{
// 		 ///状态机
// 		 	switch(Auto_charge.receive_status)
// 		 	{
// 				case 101: 
// 				{
// 					MSG_U1_T[0]=0x04;
// 					Auto_charge.nosignal_counting++;
// 					DRIVE1.V_SET=chassis_drive(0,0,1000,1);
// 					DRIVE2.V_SET=chassis_drive(0,0,1000,2);
// 					DRIVE3.V_SET=chassis_drive(0,0,1000,3);
// 					sprintf(message,"Searching %d %d \r\n", Auto_charge.Infared0_collected, Auto_charge.Infared1_collected);
// 					USART1_Puts(message);
// 					chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);	 
// 					break;  //向右转
// 				}
// 				case 49:  
// 				 {
// 				 	DRIVE1.V_SET=chassis_drive(0,1500,0,1);
// 					DRIVE2.V_SET=chassis_drive(0,300,0,2);
// 					DRIVE3.V_SET=chassis_drive(0,1500,0,3);
// 					sprintf(message,"Right turing\r\n");
// 					USART1_Puts(message);
// 					chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
// 					MSG_U1_T[0]=0x01;break;  //向右转
// 				 }
// 				case 51:  
// 				{
// 					DRIVE1.V_SET=chassis_drive(0,300,0,1);
// 					DRIVE2.V_SET=chassis_drive(0,1500,0,2);
// 					DRIVE3.V_SET=chassis_drive(0,1500,0,3);
// 					sprintf(message,"Left turing\r\n");
// 					USART1_Puts(message);
// 					chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
// 					MSG_U1_T[0]=0x04;break;  //向左转
// 				} 
// 				case 50:  
// 				{
// 					DRIVE1.V_SET=chassis_drive(0,1500,0,1);
// 					DRIVE2.V_SET=chassis_drive(0,1500,0,2);
// 					DRIVE3.V_SET=chassis_drive(0,1500,0,3);
// 					sprintf(message,"Forward\r\n");
// 					USART1_Puts(message);
// 					chassis_move(DRIVE1.V_SET, DRIVE2.V_SET, DRIVE3.V_SET);
// 					MSG_U1_T[0]=0x10;break;  //向前走
// 				}
// 			}
// 		}
// 		OSTimeDly(20);
// 	}
// }



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
