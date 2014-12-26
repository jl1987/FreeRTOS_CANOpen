/**
  ******************************************************************************
  * @file    CAN/LoopBack/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
//#include "main.h"  //comment on 23/10/2014

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* CANOpen includes */
#include "canfestival.h"
#include "can_STM32.h"

/*TEST*/
#include "canopen_thread.h"  //test

#include "CHASSIS_OD.h"
#include "globalstruct.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern xQueueHandle xQ_CAN_MSG;
CO_DATA_STRUCT  CO_D_TEST = {NULL,NULL};  //test
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
		/* This interrupt is generated when HSE clock fails */

	if (RCC_GetITStatus(RCC_IT_CSS) != RESET)
	{
		/* At this stage: HSE, PLL are disabled (but no change on PLL config) and HSI
		is selected as system clock source */

		/* Enable HSE */
		RCC_HSEConfig(RCC_HSE_ON);

		/* Enable HSE Ready and PLL Ready interrupts */
		RCC_ITConfig(RCC_IT_HSERDY | RCC_IT_PLLRDY, ENABLE);

		/* Clear Clock Security System interrupt pending bit */
		RCC_ClearITPendingBit(RCC_IT_CSS);

		/* Once HSE clock recover, the HSERDY interrupt is generated and in the RCC ISR
		routine the system clock will be reconfigured to its previous state (before
		HSE clock failure) */
	}
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
		//USART_Send(4,4,4,4,4,USART1);
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
// void SVC_Handler(void)
// {
// }

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
// void DebugMon_Handler(void)
// {
// }

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
// void PendSV_Handler(void)
// {
// }

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
// void SysTick_Handler(void)
// {
// }

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

void USART1_IRQHandler(void)
{  
	uint8_t RX_dat; 
	CanTxMsg USART2CAN;
  
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {	
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		
		RX_dat =USART_ReceiveData(USART1);
		
		printf("USART1 Recevie Data: %x ",RX_dat);
				
		if(RX_dat == 0x01)
		{
			printf("Send a SDO Message: \r\n");
			
			USART2CAN.StdId = 0x206;
			USART2CAN.ExtId = 0x00;
			/* 是否远程帧 */
			USART2CAN.RTR = 0x00;
			/* CAN 2.0A 若用B需更改 */
			USART2CAN.IDE = CAN_ID_STD;
			/* 数据长度 */
			USART2CAN.DLC = 1;
			/* 为数据赋值 */                 

			USART2CAN.Data[0] = 0xFF;

			CAN_Transmit(CAN2, &USART2CAN);
			
		}
		else if(RX_dat == 0x02)
		{
			printf("Send a PDO Message: \r\n");
			
			USART2CAN.StdId = 0x206;
			USART2CAN.ExtId = 0x00;
			/* 是否远程帧 */
			USART2CAN.RTR = 0x00;
			/* CAN 2.0A 若用B需更改 */
			USART2CAN.IDE = CAN_ID_STD;
			/* 数据长度 */
			USART2CAN.DLC = 1;
			/* 为数据赋值 */                 

			USART2CAN.Data[0] = 0xEE;

			CAN_Transmit(CAN2, &USART2CAN);
		}else{
			printf("Nothing Send \r\n");
		}
		
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
		{
		  //USART_ClearFlag(USART1,USART_FLAG_RXNE);
		}
  }	
	
}

void UART4_IRQHandler(void)                            //﹃1い?
{
// 	int index_dump;
// 	if (USART_GetFlagStatus(UART4, USART_FLAG_RXNE) != RESET)   //??ネ钡Μい?
// 	{
// 		USART_ClearITPendingBit(UART4,   USART_IT_RXNE);       //睲埃い??в
// 		if(lidar_init_ok_P==FALSE)
// 		{
// 			if(lidar_offset_count<=62)
// 			{
// 				lidar_offset_count++;
// 			}
// 			if(lidar_offset_count>56)
// 			{
// 				if(lidar_offset_count<=63)
// 				{
// 					lidar_message[lidar_offset_count-57] = USART_ReceiveData(UART4);
// 					{
// 						if(lidar_offset_count==63)
// 						{
// 							if(lidar_message[0] == 0xa5 && lidar_message[1] == 0x5a && lidar_message[2] == 0x05 && lidar_message[3] == 0x00 && 
// 							lidar_message[4] == 0x00 && lidar_message[5] == 0x40 && lidar_message[6] == 0x81)
// 							{
// 								lidar_init_ok_P = true;
// 								lidar_offset_count = 0;
// 							}
// 						}
// 					}
// 				}
// 				
// 			}
// 		}
// 		else
// 		{
// 		   
// 		   	if(lidar_offset_count < 5)
// 			{
// 				lidar_message[lidar_offset_count] = USART_ReceiveData(UART4);
// 				lidar_offset_count++;
// 			}
// 			if(lidar_offset_count == 5)
// 			{	
// 				lidar_offset_count = 0;
// 				
// 				index_dump = (lidar_message[2]*256+lidar_message[1])/128;
// 				if(index_dump<360)
// 				{
// 					RP_Lidar_Buf[index_dump][0] = (lidar_message[4]*256+lidar_message[3])/40-20;
// 					if(RP_Lidar_Buf[index_dump][0]<0)
// 					{
// 						RP_Lidar_Buf[index_dump][0]=300;
// 					}
// 					if(RP_Lidar_Buf[index_dump][0]>300)
// 					{
// 						RP_Lidar_Buf[index_dump][0]=300;
// 					}

// 					
// 				//	USART_SendData(USART1, RP_Lidar_Buf[index_dump][0]);                                     //?癳?誹
// 				//	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}//单?癳?
// 				}
// 				
// 			}	
// 			  
// 		
// 		//	USART_SendData(USART1, USART_ReceiveData(UART4));                                     //?癳?誹
// 		//	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}//单?癳?
// 				
// 		}
// 		
// 	}
	
}

//#ifdef USE_CAN1
 /**
  * CAN1接收中断处理函数
  * @brief  
  *         在通常的处理中，CAN中断中只需调用ST库函数中的CAN接收处理函数CAN_Receive()
  *         和CANOpen中的CAN接收函数canDispatch(&object_data,&m)即可.
  *         
  *         在本程序中，CAN信息被存储在队列xQ_CAN_MSG中(队列可存储20个完整CAN2.0A信息)
  *         所以，在CANx_RX0_IQRHandler中仅仅是将接收到的数据从CAN_FIFO0中取出，传输至
  *         xQ_CAN_MSG中，整个处理流程应为：
  *         
  *         定义CAN接收变量->失能CAN中断->接收CAN总线数据->将接收到数据push到队列中->
  *         退出临界区->让出CPU占用->清除挂起中断->使能CAN中断
  *         
  * @param  None
  * @retval None
  */

void CAN1_RX0_IRQHandler(void)
{ 
//   /*进入中断*/
//   
//   int i;
//   Message RxMSG;  /* 标准的CAN2.0A信息，初始化清零 */
//   CANOpen_Message CAN_Rx_m; /* CANOpen Message 包含CAN Port(CANx) */

//   CAN_Rx_m.CANx = 1;

//   CO_D_TEST.CO_CAN1 = &ObjDict_CAN1_Data;
//   CO_D_TEST.CO_CAN1->canHandle = CAN1;

//   /* 挂起中断 */
//   CAN_ITConfig(CAN1,CAN_IT_FMP0, DISABLE);
//   printf("CAN_ITConfig\r\n");

//   /* 从CAN1 FIFO0接收数据存入CAN1_Rx_m */
//   CAN_Receive(CAN1, CAN_FIFO0, &(CAN_Rx_m.m));
//   printf("CAN_Receive\r\n");

//   printf("Thread get a CAN packege\r\n");
//   RxMSG.cob_id = (uint16_t)(CAN_Rx_m.m.StdId);
//   RxMSG.rtr = CAN_Rx_m.m.RTR;
//   RxMSG.len = CAN_Rx_m.m.DLC;
//   for(i=0;i<RxMSG.len;i++)
//   {
//     RxMSG.data[i] = CAN_Rx_m.m.Data[i]; //Transfer data[0-7] from CAN_Rx_m to RxMSG
//   }

//   printf("CAN Message Receieved: %02x|%02x %02x \r\n", RxMSG.cob_id, RxMSG.data[0],RxMSG.data[1]);

//   printf("leaving the CAN_ITConfig\r\n");

//   /*Handle The Data Receive, 此处和对象字典进行交互*/
//   printf("canDispatch\r\n");
//   //canDispatch(CO_D_TEST.CO_CAN1, &RxMSG); 
//   
//   if (RxMSG.cob_id == 0x206)
//   {
//      switch(RxMSG.data[0])
//      {
//        case  0x01:
//           /* Debug Message */
//           printf("Receieve a ARM-Linux msg : Up \r\n");
//           /* Control GPIO */
//           GPIO_SetBits(GPIOE, GPIO_Pin_11);
//           GPIO_ResetBits(GPIOE, GPIO_Pin_13);   
//           break;
//        case  0x02:
//           /* Debug Message */
//           printf("Receieve a ARM-Linux msg : Down \r\n");
//           /* Control GPIO */
//           GPIO_ResetBits(GPIOE, GPIO_Pin_11);
//           GPIO_SetBits(GPIOE, GPIO_Pin_13);
//           break;
//        case  0x03:
//           /* Debug Message */
//           printf("Receieve a ARM-Linux msg : Stop \r\n");
//           /* Control GPIO */
//           GPIO_SetBits(GPIOE, GPIO_Pin_11);
//           GPIO_SetBits(GPIOE, GPIO_Pin_13);
//           break;
//        default:
//           /* Debug Message */
//           printf("Unknown error msg : \r\n");
//      }
//   }
//   
//   
//   // if (1==canSend(CAN1,&RxMSG))
//   // {
//   //   printf("Send Succuss!\r\n" );
//   // }

//   //printf("leaving the CAN_ITConfig\r\n");

//   /* 清除挂起中断*/ 
//   CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
//   /* 使能CAN中断*/ 
//   CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);
}
/************************************************************************************/
//#endif  /* USE_CAN1 */


/**
  * @brief  This function handles CAN2 RX0 request.
  * @param  None
  * @retval None
  */
void CAN2_RX0_IRQHandler(void)
{
		CANOpen_Message CAN2_Rx_m;
	
	  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;     //中断中唤醒新任务
	
		printf("ENTER CAN2 RX IRQHandler... \r\n");
	
	  taskENTER_CRITICAL();                                  //进入中断
	
	  CAN_Receive(CAN2, CAN_FIFO0, &(CAN2_Rx_m.m));	    //从CAN2 FIFO0接收数据
	
	  CAN2_Rx_m.CANx = 2;
	
	  if(NULL != xQ_CAN_MSG)         // 向队列发送数据包
	  {
			xQueueSendFromISR(xQ_CAN_MSG, &CAN2_Rx_m, &xHigherPriorityTaskWoken);
	  }
		
	  taskEXIT_CRITICAL();
		
	  if( xHigherPriorityTaskWoken != pdFALSE )
	  {
	    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );           //强制上下文切换
  	}  
	
}


void EXTI0_IRQHandler(void)
{
//     if(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_0))
// 	{
// 		Infared0_time_end=Infared0_time;
//     Infared0_time=0;		
// 		//GPIO_SetBits(GPIOA, GPIO_Pin_4);
// 		
// 		XXXXX++;
// 		
// 		//sprintf(message_ADC,"%d\r\n",ADC3ConvertedValue*5000/0xFFF);
// 		
// 		//USART6_Puts(message_ADC);
// 	}
//     EXTI_ClearITPendingBit(EXTI_Line0);
}

void EXTI1_IRQHandler(void)
{
//     if(!GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_1))	
// 	{
// 		Infared1_time_end=Infared1_time;
// 		Infared1_time=0;
// 		//GPIO_SetBits(GPIOA, GPIO_Pin_4);
// 	}
//     EXTI_ClearITPendingBit(EXTI_Line1);
}




/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
