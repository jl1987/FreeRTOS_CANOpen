/**
  ******************************************************************************
  * @file    canopen_thread.c
  * @author  Jim
  * @version V1.0
  * @date    01-Sep-2014
  * @brief   This file provides CANOpen communication control thread functions 
  *
  *  @verbatim  
  *  
  *        
  *          ===================================================================
  *                                 How to use this 
  *          ===================================================================  
  *           
  *          1. XXXXXXXXXXXXXXXXXXXXXXXXX:
  *                 XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX;
  *                 XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX;
  *
  *          2. XXXXXXXXXXXXXXXXXXX 
  *             - XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX 
  *                 XXXXXXXXXXXXXXXXXXXXXXXXXX; 
  *             - XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  *                 XXXXXXXXXXXXXXX;
  *    
  *          3. XXXXXXXXXXXXXXXXXXXXXXXXXX, for example:
  *                 XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX;
  *             XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  *             XXXXXXXXXXXXXXXXXXXXXXXXX.
  *  
  *          4. XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  *                 XXXXXXXXXXXXXXXXXXXXXXXXX; 
  *                 
  *          5. XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  *             XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX.
  *  
  *   
  *  @note  NOTENOTENOTE 
  *        
  *  @endverbatim   
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */

#include "canopen_thread.h"
#include "stdio.h"
#include "globalstruct.h"

/* TEST */
#include "bsp_led.h"


#include "ARM_OD.h"


/*store date CAN receive*/
xQueueHandle xQ_CAN_MSG = NULL;
xTaskHandle  xT_CANOpen = NULL;

CO_DATA_STRUCT  CO_D = {NULL,NULL};


/**
 * CANOpen Data Process Thread Program
 * @brief CANOpen Data Scan, if CANbus got a CAN message, then push the message into
 *        queue(xQ_CAN_MSG), this thread scan the queue at CANOpen_THREAD_SCAN_TIMER(
 *        default:20ms) rate.
 */
static void canopen_dataprocess_thread(void * pvParameters)
{
  int i;
  unsigned char ret_canInit;
  Message RxMSG = Message_Initializer; /* 标准的CAN2.0A信息，初始化清零 */
  CANOpen_Message CAN_Rx_m;            /* CANOpen Message 包含CAN Port(CANx) */

  /* CANOpen Initialising */

  initTimer();

  /* 得到BSP的ID号，这里应为NodeID */
  //setNodeId (&ObjDict_CAN2_Data, LIFTER_ID);
	setNodeId (&ARM_OD_Data, ARM_ID);
	
	//CO_D.CO_CAN2 = &ObjDict_CAN2_Data;
	CO_D.CO_CAN2 = &ARM_OD_Data;
	CO_D.CO_CAN2->canHandle = CAN2;  //Config CANOpen Port CAN1
  printf("CANOpen OD Get The Lifer NodeID...\r\n");

  ret_canInit = canInit(CAN2,CAN_BAUD_1M);
	
  if(ret_canInit)
  {
    printf("CAN init finished...\r\n");
    /* State Machine Change to Initialisation and automatic go into Pre_operational*/
    //setState(&ObjDict_CAN2_Data, Initialisation);
		setState(&ARM_OD_Data, Initialisation);
    printf("State Machine change to Initialisation...\r\n");
    /*****************************************************************************
    * Initialisation -> Pre_operational -> StartOrStop ->
    * startSYNC | heartbeatInit | emergencyInit | PDOInit | slaveSendBootUp
    * startSYNC(CO_Data* d)       :监视OD-0x1005、0x1006，并根据其数值设定Alarm;
    * heartbeatInit(CO_Data* d)   :监视OD-0x1017，consumers心跳，而后定Alarm;
    * emergencyInit(CO_Data* d)   :监视OD-0x1003;
    * PDOInit (CO_Data * d)       :监视OD-0x1800，初始化PDO;
    * slaveSendBootUp(CO_Data* d) :发送BootUp指令。
    *****************************************************************************/
    /* State Machine change to Pro_operational */
    printf("State Machine change to Pro_operational...\r\n");
    printf("Slave Send a BootUp Message...\r\n");
  }
  else
  {
    printf("CAN init failed...\r\n");
  }

//  xQueueCreate(uxQueueLength, /* 队列能够存储的最大单元数目，即队列深度 */
//               uxItemSize)    /* 队列中数据单元的长度，以字节为单位 */
  /*create a queue can store 20 data*/
  xQ_CAN_MSG = xQueueCreate(20,sizeof(CANOpen_Message));/* 20个CAN信息？ */

  /* Success Or Fail */
  if(NULL == xQ_CAN_MSG)
  {
  	/*failed to creat the queue*/
    while(1)
    { 
      printf("!!!CANOpen Queue created failed!!!\r\n");

      vTaskDelay(100);
    }
  }else
  {
    printf("CANOpen Queue created successfully...\r\n");
    printf("<<<<<CANOpen Thread ALL CLEAR>>>>>\r\n");
  }
	
	while(1)
	{
//  xQueueReceive(xQueue,       /* 被读队列的句柄，即调用xQueueCreate()创建该队列时的返回值 */
//                pvBuffer,     /* 接收缓存指针 */
//                xTicksToWait) /* 阻塞超时时间，N个统心跳周期，portTICK_RATE_MS为毫秒*/
	 	/*Receive CANBUS data*/
	 	if(xQueueReceive(xQ_CAN_MSG, &(CAN_Rx_m), (portTickType)100))
		{
      printf("Thread get a CAN packege ");

		  /* 讲队列中的数据存储到RxMSG中 */
		  RxMSG.cob_id = (uint16_t)(CAN_Rx_m.m.StdId);
		  RxMSG.rtr = CAN_Rx_m.m.RTR;
		  RxMSG.len = CAN_Rx_m.m.DLC;
		  for(i=0;i<RxMSG.len;i++)
		  {
		    RxMSG.data[i] = CAN_Rx_m.m.Data[i]; //Transfer data[0-7] from CAN_Rx_m to RxMSG
		  }
			
			printf("from CAN%u: 0x%x| ", CAN_Rx_m.CANx,CAN_Rx_m.m.StdId);
			for(i=0;i<RxMSG.len;i++)
		  {
		    printf("0x%x ", RxMSG.data[i]);
		  }
			printf("\r\n");
      
      /* 判断是CAN1还是CAN2接收到并与访问对象字典 */
		  if((NULL != CO_D.CO_CAN2) && (1 == CAN_Rx_m.CANx))       /*Data From CAN1*/
		  {
        /*Handle The Data Receive, 此处和对象字典进行交互*/
		  	canDispatch(CO_D.CO_CAN2, &RxMSG); 
		  }
		  else if((NULL != CO_D.CO_CAN2) && (2 == CAN_Rx_m.CANx))  /*Data From CAN2*/
		  {
        /*Handle The Data Receive，此处和对象字典进行交互*/
		  	canDispatch(CO_D.CO_CAN2, &RxMSG); 
		  }
		}else
    {
     // printf("Waiting for a CAN message!\r\n");
   
			vTaskDelay(100);
    }
		bsp_LedToggle(1,100);
    vTaskDelay(CANOpen_THREAD_DELAY_TIMER);
	}


}

/**
 * CANOpen Initialization Program
 * @brief create CANOpen data process thread
 */
void canopen_init(void)
{
  /* Create CANOpen Data Process task */
	xTaskCreate(canopen_dataprocess_thread, 
              "CANOpen", 
              CANOPEN_THREAD_STACK, 
              NULL, 
              CANOpen_THREAD_PRIO, 
              &xT_CANOpen);
  /* if fail */
	if(NULL == xT_CANOpen)
	{
    printf("CANOpen dataprocess thread created failed!\r\n");
	}
  else
  {
    printf("CANOpen dataprocess thread created successfully!\r\n");

  }
}


/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
