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
  * <h2><center>&copy; COPYRIGHT 2014 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */

#include "lifter_control.h"
#include "stdio.h"
#include "globalstruct.h"

#include "ObjDict_CAN1.h"
#include "ObjDict_CAN2.h"
#include "LIFTER_OD.h"

xQueueHandle xQ_LIFTER_MSG = NULL;
xTaskHandle	 xT_LIFTER 	   = NULL;

void lifter_control_thread(void * pvParameters)
{
	 
	CO_Data *d;
	d = (CO_Data *)pvParameters;

//  xQueueCreate(uxQueueLength, /* 队列能够存储的最大单元数目，即队列深度 */
//               uxItemSize)    /* 队列中数据单元的长度，以字节为单位 */
  	/*create a queue can store 20 data*/
	xQ_LIFTER_MSG = xQueueCreate(20,sizeof(CO_Data));/* 20个CAN信息？ */

	/* Success Or Fail */
	if(NULL == xQ_LIFTER_MSG)
	{
		/*failed to creat the queue*/
		while(1)
		{	
			printf("Creat the lifter Queue failed! \r\n");

			vTaskDelay(10);
		}
	}
	
	//canInit(CAN1,CAN_BAUD_1M);       /*Use   CAN1(Develop PCB)   1Mbps*/
	//canInit(CAN2,CAN_BAUD_1M);     /*Use   CAN2(ARM_CSST)      1Mbps*/
	
	printf("start the lifter control\r\n");



/**
 * Scan the index of object dictionary:
 * RegisterSetODentryCallBack(CO_Data* d, 	//Pointer to a CAN object data structure
 * 							  UNS16 wIndex, 
 * 							  UNS8 bSubindex, 
 * 							  ODCallback_t Callback)
 */

/*This Block should Contain Scrips about Callback functions on Object Dictionary*/
/*This Block should Contain Scrips about Callback functions on Object Dictionary*/
/*This Block should Contain Scrips about Callback functions on Object Dictionary*/
/*This Block should Contain Scrips about Callback functions on Object Dictionary*/
/*This Block should Contain Scrips about Callback functions on Object Dictionary*/
	
	while(1)
	{
	  

	  // if(0!=xQ_DRIVE_COMM && xQueueReceive( xQ_DRIVE_COMM, &(body_msg), (portTickType)0))
	  // {	
	  // 	
	  // 	switch(){
	  //	  case  CONTROLWORD_OPERATION_UP:
	  //	    /* Debug Message */
	  //	    printf("Receieve a ARM-Linux msg : Up \r\n");
	  //	    /* Control GPIO */
	  //	    lifter_up();	    
	  //	    break;
	  //	  case  CONTROLWORD_OPERATION_DOWN:
	  //	    /* Debug Message */
	  //	    printf("Receieve a ARM-Linux msg : Down \r\n");
	  //	    /* Control GPIO */
	  //	    lifter_down();
	  //	    break;
	  //	  case  CONTROLWORD_OPERATION_STOP:
	  //	    /* Debug Message */
	  //	    printf("Receieve a ARM-Linux msg : Stop \r\n");
	  //	    /* Control GPIO */
	  //	    lifter_stop();
	  //	    break;
	  //	  default:
	  //	    /* Debug Message */
	  //	    printf("Unknown error msg : \r\n");
	  // 	}
	  // }

	  //(void)masterSendNMTstateChange(d, 0, NMT_Reset_Node);
	  printf("lifter control is running\r\n");
	  vTaskDelay(LIFTER_CONTROL_THREAD_DELAY_TIMER);
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

void lifter_up(void)
{
	GPIO_SetBits(LIFTER_PORT, LIFTER_PIN_INA);
	GPIO_ResetBits(LIFTER_PORT, LIFTER_PIN_INB);
}

void lifter_down(void)
{
	GPIO_ResetBits(LIFTER_PORT, LIFTER_PIN_INA);
	GPIO_SetBits(LIFTER_PORT, LIFTER_PIN_INB);
}

void lifter_stop(void)
{
	GPIO_SetBits(LIFTER_PORT, LIFTER_PIN_INA);
	GPIO_SetBits(LIFTER_PORT, LIFTER_PIN_INB);
}


void start_lifter_control(void)
{
	xTaskCreate(lifter_control_thread, "lifter_control", LIFTER_CONTROL_THREAD_STACK, NULL,LIFTER_CONTROL_THREAD_PRIO, &xT_LIFTER);
	if(NULL == xT_LIFTER)
	{
	 	printf("lifter control thread creat failed!\r\n");
	}else
	{
		printf("lifter control thread creat successfully!\r\n");
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
