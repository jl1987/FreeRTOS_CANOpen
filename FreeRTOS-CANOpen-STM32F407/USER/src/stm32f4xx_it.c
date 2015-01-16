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

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Program Variable includes */
#include "globalstruct.h"

/* CANOpen includes */
#include "canfestival.h"
#include "can_STM32.h"

/* Hardware includes */
#include "ir_autocharge.h"

/* Object Dictionary includes */
#include "CHASSIS_OD.h"



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern xQueueHandle xQ_CAN_MSG;

extern AUTO_CHARGE Auto_charge;

//extern Chassis_Data CHASSIS_D;
u8 msgfromU2[8],msgfromU3[8],msgfromU5[8],buf_msgfromU5[8]; // messages from usart
extern u8 MSG_U2_R[8];
extern u8 MSG_U2_T[16];
extern u8 MSG_U3_R[8];
extern u8 MSG_U3_T[16];

//extern Chassis_Data CHASSIS_D;

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
	
	uint8_t U1RX_dat;  

	if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART1,   USART_IT_RXNE); 
		U1RX_dat=USART_ReceiveData(USART1);// & 0x7F; 
		printf("USART1 Get a Message: %d", U1RX_dat);
		motion_command = U1RX_dat;
		USART_SendData(USART1, U1RX_dat); 
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}
	}

	
}

void UART4_IRQHandler(void) 
{
// 	int index_dump;
// 	if (USART_GetFlagStatus(UART4, USART_FLAG_RXNE) != RESET)
// 	{
// 		USART_ClearITPendingBit(UART4,   USART_IT_RXNE);
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
// 				//	USART_SendData(USART1, RP_Lidar_Buf[index_dump][0]);   
// 				//	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}
// 				}
// 				
// 			}	
// 			  
// 		
// 		//	USART_SendData(USART1, USART_ReceiveData(UART4));
// 		//	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}
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

}


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
	
  Auto_charge.Infared0_time_end=Auto_charge.Infared0_time;
	Auto_charge.Infared0_time=0;
    
	EXTI_ClearITPendingBit(EXTI_Line0);
}

void EXTI1_IRQHandler(void)
{
    Auto_charge.Infared1_time_end=Auto_charge.Infared1_time;

		Auto_charge.Infared1_time=0;

    EXTI_ClearITPendingBit(EXTI_Line1);
}


void USART2_IRQHandler(void)//channel1
{
	u8 n;
// 	USART2->CR1&=0XFFFFFFEF; //禁止产生中断
// 	USART3->DR = USART2->DR;
// 	//USART_ClearFlag(USART2, USART_FLAG_RXNE);
// 	USART2->CR1|=0X00000010;  //开启中断 
	
  USART2->CR1&=0XFFFFFFEF; //禁止产生中断
	//GPIO_WriteBit(GPIOE, GPIO_Pin_5, Bit_RESET);
	DMA_Cmd(DMA1_Stream5, DISABLE);	//失能
	
  for(n=0;n<8;n++)
	{
   msgfromU2[n]=MSG_U2_R[n];
  }
  
	DMA1_Stream5->NDTR = 8;
	DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TCIF5);//清除标志
 	DMA_Cmd(DMA1_Stream5, ENABLE);	//使能
	
	//访问一次
	  USART2->SR;
	  USART2->DR;
	//访问一次
	delay(200);
  //GPIO_WriteBit(GPIOE, GPIO_Pin_5, Bit_SET);
	USART2->CR1|=0X00000010;  //开启中断 
}

void USART3_IRQHandler(void)//channel3
{
	u8 n;
// 	USART2->CR1&=0XFFFFFFEF; //禁止产生中断
// 	USART3->DR = USART2->DR;
// 	//USART_ClearFlag(USART2, USART_FLAG_RXNE);
// 	USART2->CR1|=0X00000010;  //开启中断 
	
  USART3->CR1&=0XFFFFFFEF; //禁止产生中断
	//GPIO_WriteBit(GPIOE, GPIO_Pin_5, Bit_RESET);
	DMA_Cmd(DMA1_Stream1, DISABLE);	//失能
	
  for(n=0;n<8;n++)
	{
   msgfromU3[n]=MSG_U3_R[n];
  }
  
	DMA1_Stream1->NDTR = 8;
	DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1);//清除标志
 	DMA_Cmd(DMA1_Stream1, ENABLE);	//使能
	
	//访问一次
	  USART3->SR;
	  USART3->DR;
	//访问一次
	delay(200);
  //GPIO_WriteBit(GPIOE, GPIO_Pin_5, Bit_SET);
	USART3->CR1|=0X00000010;  //开启中断 
}


// void UART5_IRQHandler(void)//channel2
// {
// 	u8  n=0,rec_count=0;
// 	u16 tiny_timeout=0,big_timeout=0;
// 	
// 	UART5->CR1&=0XFFFFFFEF; //禁止产生中断
// 	GPIO_WriteBit(LEDPORT, LED4, Bit_RESET);
//   	
// // 	msgfromU5[rec_count]=UART5->DR;
// // 	if ((msgfromU5[rec_count]==0x0A)||(rec_count>=8))
// // 		rec_count=0;
// // 	else
// // 		rec_count++;
// 	
//   while ((UART5->DR!=0X0A)&&(big_timeout++<10000))
// 	{
// 	 tiny_timeout=0;
// 		while((!USART_GetFlagStatus(Channel2_U,USART_FLAG_RXNE))&&(tiny_timeout++<1000));
// 	 
//    msgfromU5[n]=UART5->DR;

// 	 n++;
//   }
// 	
// 	
// 	GPIO_WriteBit(LEDPORT, LED4, Bit_SET);
// 	//USART_ClearITPendingBit(UART5,USART_IT_RXNE);
// 	UART5->CR1|=0X00000010;  //开启中断 
// }

u8  n=0,ifcanberead=0;
//u16 timeout=0;
void UART5_IRQHandler(void)//channel2
{
	u8 m;
	UART5->CR1&=0XFFFFFFEF; //禁止产生中断
	

// 	while((!USART_GetFlagStatus(Channel2_U,USART_FLAG_RXNE))&&
// 		    (timeout++<1000));
	 
  buf_msgfromU5[n]=UART5->DR;
	
	if (buf_msgfromU5[n]==0X23)//0X0D就是回车，0x23就是“#” 
	{
		n=0;
		for (m=0;m<8;m++)
		{
     msgfromU5[m]=buf_msgfromU5[m];
    }
  }
	n++;


	//USART_ClearITPendingBit(Channel2_U,USART_IT_RXNE);
	UART5->CR1|=0X00000010;  //开启中断 
}





/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
