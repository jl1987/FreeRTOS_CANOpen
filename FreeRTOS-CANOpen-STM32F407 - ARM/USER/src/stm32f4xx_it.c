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


/* Object Dictionary includes */
#include "ARM_OD.h"



extern uint8_t left_hand_pump_pwm_duty[5];
extern uint8_t right_hand_pump_pwm_duty[5];


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern xQueueHandle xQ_CAN_MSG;

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
		//motion_command = U1RX_dat;
		USART_SendData(USART1, U1RX_dat); 
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}
	}

	
}

void UART4_IRQHandler(void) 
{
	
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

/**************************************************************************************/
/* PMW */
void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);	


	// 12/17/2013 new added
	/*if (left_hand_pump_pwm_duty[0]>104)
	{
		left_hand_pump_pwm_duty[0] = 104;
	}
	if (left_hand_pump_pwm_duty[1]>104)
	{
		left_hand_pump_pwm_duty[1] = 104;
	}
	if (left_hand_pump_pwm_duty[2]>104)
	{
		left_hand_pump_pwm_duty[2] = 104;
	}
	if (right_hand_pump_pwm_duty[2]>104)
	{
		right_hand_pump_pwm_duty[2] = 104;
	}	*/
		
		
	/*01/23/2014 added*/	
	if (left_hand_pump_pwm_duty[0]<10)
	{
		left_hand_pump_pwm_duty[0] = 0;
	}
	if (left_hand_pump_pwm_duty[1]<10)
	{
		left_hand_pump_pwm_duty[1] = 0;
	}
	if (left_hand_pump_pwm_duty[2]<10)
	{
		left_hand_pump_pwm_duty[2] = 0;
	}
	if (right_hand_pump_pwm_duty[2]<10)
	{
		right_hand_pump_pwm_duty[2] = 0;
	}	
		

	/*	 
	// 12/19/2013 update
	TIM3->CCR2 = 1049 - right_hand_pump_pwm_duty[0]*10; // 0- 1049, duty = [0 100]. // 右大拇指

	// 12162013 new version
	TIM3->CCR1 = 1049 - left_hand_pump_pwm_duty[1]*10; // 0- 1049, duty = [0 100]. // 左食指	
	TIM3->CCR3 = 1049 - left_hand_pump_pwm_duty[2]*10; // 0- 1049, duty = [0 100]. // 左中指
	TIM3->CCR4 = 1049 - right_hand_pump_pwm_duty[2]*10; // 0- 1049, duty = [0 100]. // 右中指	
	*/	


	/*01/23/2014 new version*/
	TIM3->CCR2 = 4198 - right_hand_pump_pwm_duty[0]*41; // 0- 4198, duty = [0 100]. // 右大拇指

	// 12162013 new version
	TIM3->CCR1 = 4198 - left_hand_pump_pwm_duty[1]*41; // 0- 4198, duty = [0 100]. // 左食指	
	TIM3->CCR3 = 4198 - left_hand_pump_pwm_duty[2]*41; // 0- 4198, duty = [0 100]. // 左中指
	TIM3->CCR4 = 4198 - right_hand_pump_pwm_duty[2]*41; // 0- 4198, duty = [0 100]. // 右中指	
	

	SystemCoreClockUpdate();
  }
}


/* PMW */
void TIM4_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);


	/*// 12/17/2013 new added
	if (right_hand_pump_pwm_duty[0]>104)
	{
		right_hand_pump_pwm_duty[0] = 104;
	}
	if (right_hand_pump_pwm_duty[1]>104)
	{
		right_hand_pump_pwm_duty[1] = 104;
	}*/
	
		
	/*01/23/2014 new version*/
	if (right_hand_pump_pwm_duty[0]<10)
	{
		right_hand_pump_pwm_duty[0] = 0;
	}
	if (right_hand_pump_pwm_duty[1]<10)
	{
		right_hand_pump_pwm_duty[1] = 0;
	}


	/*// 12/16/2013 based on new pcb config	
	TIM4->CCR3 = 1049 - left_hand_pump_pwm_duty[0]*10; // 0- 1049, duty = [0 100]. // 左大拇指
  TIM4->CCR4 = 1049 - right_hand_pump_pwm_duty[1]*10; // 0- 1049, duty = [0 100]. // 右食指 
  */
	
	
	/*01/23/2014 new version*/
	TIM4->CCR3 = 4198 - left_hand_pump_pwm_duty[0]*41; // 0- 4198, duty = [0 100]. // 左大拇指
  TIM4->CCR4 = 4198 - right_hand_pump_pwm_duty[1]*41; // 0- 4198, duty = [0 100]. // 右食指 
		
	
	SystemCoreClockUpdate();
  }
}


/* PMW */
void TIM5_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);	


	/*// 12/17/2013 new added
	if (left_hand_pump_pwm_duty[3]>104)
	{
		left_hand_pump_pwm_duty[3] = 104;
	}
	if (left_hand_pump_pwm_duty[4]>104)
	{
		left_hand_pump_pwm_duty[4] = 104;
	}
	if (right_hand_pump_pwm_duty[3]>104)
	{
		right_hand_pump_pwm_duty[3] = 104;
	}
	if (right_hand_pump_pwm_duty[4]>104)
	{
		right_hand_pump_pwm_duty[4] = 104;
	}*/
		
	
	/*01/23/2014 new version*/
	if (left_hand_pump_pwm_duty[3]<10)
	{
		left_hand_pump_pwm_duty[3] = 0;
	}
	if (left_hand_pump_pwm_duty[4]<10)
	{
		left_hand_pump_pwm_duty[4] = 0;
	}
	if (right_hand_pump_pwm_duty[3]<10)
	{
		right_hand_pump_pwm_duty[3] = 0;
	}
	if (right_hand_pump_pwm_duty[4]<10)
	{
		right_hand_pump_pwm_duty[4] = 0;
	}

	
	/*// 12162013 based on new pcb config														  
	TIM5->CCR1 = 1049 - left_hand_pump_pwm_duty[3]*10; // 0- 1049, duty = [0 100]. // 左无名指
	TIM5->CCR2 = 1049 - right_hand_pump_pwm_duty[3]*10; // 0- 1049, duty = [0 100]. // 右无名指 
	TIM5->CCR3 = 1049 - left_hand_pump_pwm_duty[4]*10; // 0- 1049, duty = [0 100]. // 左小指
	TIM5->CCR4 = 1049 - right_hand_pump_pwm_duty[4]*10; // 0- 1049, duty = [0 100]. // 右小?
	*/
	
	
	/*01/23/2014 new version*/										  
	TIM5->CCR1 = 4198 - left_hand_pump_pwm_duty[3]*41; // 0- 4198, duty = [0 100]. // 左无名指
	TIM5->CCR2 = 4198 - right_hand_pump_pwm_duty[3]*41; // 0- 4198, duty = [0 100]. // 右无名指 
	TIM5->CCR3 = 4198 - left_hand_pump_pwm_duty[4]*41; // 0- 4198, duty = [0 100]. // 左小指
	TIM5->CCR4 = 4198 - right_hand_pump_pwm_duty[4]*41; // 0- 4198, duty = [0 100]. // 右小?

	SystemCoreClockUpdate();
  }
}



void EXTI0_IRQHandler(void)
{
	
 
}

void EXTI1_IRQHandler(void)
{
  
}


void USART2_IRQHandler(void)//channel1
{

}

void USART3_IRQHandler(void)//channel3
{

}

void UART5_IRQHandler(void)//channel2
{
	
}





/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
