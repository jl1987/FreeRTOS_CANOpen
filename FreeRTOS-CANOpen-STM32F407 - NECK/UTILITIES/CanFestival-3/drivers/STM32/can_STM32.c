/*
This file is edited as a part of CanFestival.

STM32F407 Port: Jim
*/


#include "stm32f4xx_can.h"
#include "can_STM32.h"
#include "canfestival.h"
#include "misc.h"




//void can_irq_handler(void);

/**
 * Initialize the hardware to receive CAN messages and start the timer for the
 * CANopen stack.
 * @param  CANx    CAN Port (CAN1:Develop PCB or CAN2:ARM_CSST PCB)
 * @param  bitrate CAN Bitrate (1M default)
 * @return 1 for succuss
 */
unsigned char canInit(CAN_TypeDef* CANx,unsigned int bitrate)
{
	GPIO_InitTypeDef 		GPIO_InitStructure;          //GPIO 
	NVIC_InitTypeDef 		NVIC_InitStructure;	         //NVIC
	CAN_InitTypeDef 		CAN_InitStructure;					 //CAN
	CAN_FilterInitTypeDef 	CAN_FilterInitStructure; //CAN Filter
	
	uint16_t prescaler;
	
	
 	/* CAN NVIC configuration */
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;    // 抢占式优先级为1
 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
 	/* CAN Birrate switch */
	switch(bitrate){
		case 1000:
			prescaler=3;
			break;
		case 500:
			prescaler=6;
			break;
		case 250:
			prescaler=12;
			break;
		case 200:
			prescaler=15;
			break;
		case 125:
			prescaler=24;
			break;
		case 100:
			prescaler=30;
			break;
		case 50:
			prescaler=60;
			break;
		case 10:
			prescaler=300;
			break;
		default:
			prescaler=3;
			break;
	}

	if(CANx == CAN1)
	{
		/* Enable GPIO clock */
		RCC_AHB1PeriphClockCmd(CAN1_GPIO_CLK, ENABLE);
		
		/* Connect CAN pins to AF9 */
		GPIO_PinAFConfig(CAN1_GPIO_PORT, CAN1_RX_SOURCE, CAN1_AF_PORT);
		GPIO_PinAFConfig(CAN1_GPIO_PORT, CAN1_TX_SOURCE, CAN1_AF_PORT); 
		
		/* Configure CAN RX and TX pins */
		GPIO_InitStructure.GPIO_Pin   = CAN1_RX_PIN | CAN1_TX_PIN;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);
		/* CAN configuration ********************************************************/  
		/* Enable CAN clock */
		RCC_APB1PeriphClockCmd(CAN1_CLK, ENABLE);
		
		/* CAN register init */
		CAN_DeInit(CANx);
		CAN_StructInit(&CAN_InitStructure);
		/* CAN cell init */
		CAN_InitStructure.CAN_TTCM = DISABLE;	//禁止时间触发通信模式
		CAN_InitStructure.CAN_ABOM = DISABLE;	//自动离线管理不使能
		CAN_InitStructure.CAN_AWUM = DISABLE;	//自动唤醒模式不使能
		CAN_InitStructure.CAN_NART = DISABLE;	//非自动重传模式不使能，即自动重传直到成功
		CAN_InitStructure.CAN_RFLM = DISABLE;	//FIFO锁定模式不使能，溢出时旧报文覆盖新报文
		CAN_InitStructure.CAN_TXFP = DISABLE;	//FIFO优先级确定，优先级由报文标识符确定
		CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;	//CAN工作在正常模式
		CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;		//重新同步跳跃宽度1个时间单位
		  
		/* CAN Baudrate = 1MBps (CAN clocked at 42 MHz) */
		/* CAN 使用APB1时钟，APB1时钟分频系数是4，,42M，见system_stm32f2xx.c   42/3=14 */
		/* CAN 频率 = APB1时钟/(CAN分频系数*(1+BS1+BS2)) */
		CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
		CAN_InitStructure.CAN_BS2 = CAN_BS2_7tq;
		CAN_InitStructure.CAN_Prescaler = prescaler;	//预分频系数
		CAN_Init(CANx, &CAN_InitStructure);
		CAN_FilterInitStructure.CAN_FilterNumber = 0;	//指定了待初始化的过滤器0
		/* CAN_FilterInitStructure.CAN_FilterNumber = 14; */
		/* 以下部分还需要特别处理，默认是全部接收 */
		/* 指定了过滤器将被初始化到的模式为标识符屏蔽位模式 */
		CAN_FilterInitStructure.CAN_FilterMode       = CAN_FilterMode_IdMask;
		CAN_FilterInitStructure.CAN_FilterScale      = CAN_FilterScale_32bit;
		CAN_FilterInitStructure.CAN_FilterIdHigh     = 0x0000;
		CAN_FilterInitStructure.CAN_FilterIdLow      = 0x0000;
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
		CAN_FilterInitStructure.CAN_FilterMaskIdLow  = 0x0000;
		/* 设定了指向过滤器的FIFO0 */
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
		CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;	//使能过滤器
		CAN_FilterInit(&CAN_FilterInitStructure);
		  
		/* Enable FIFO 0 message pending Interrupt */
		/* 打开FIFO0中断请求 */
		CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
		/* 配置NVIC */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//Test- Fix the Queue Problem 2014.-09.22

		NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
		/* 打开CAN1中断 */
		NVIC_Init(&NVIC_InitStructure);
	}
	else
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
												
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,ENABLE);
		
		/* CAN2引脚 */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);  
		
		  /* CAN register init */
		CAN_ITConfig(CAN2,CAN_IT_FMP0, DISABLE);    //CAN 接收中断禁能
		CAN_ITConfig(CAN2,CAN_IT_TME, DISABLE);
		CAN_DeInit(CAN2);													  //出示化CAN控制器
		CAN_StructInit(&CAN_InitStructure);
		/* CAN cell init */
		CAN_InitStructure.CAN_TTCM=DISABLE;  				//禁止时间触发通信方式
		CAN_InitStructure.CAN_ABOM=ENABLE;   			//启动CAN总线自动关闭管理
		CAN_InitStructure.CAN_AWUM=DISABLE;  				//禁止自动唤醒模式
		CAN_InitStructure.CAN_NART=ENABLE;   			//禁止非自动重传模式
		CAN_InitStructure.CAN_RFLM=DISABLE;	 				//FIFO 锁定模式关
		CAN_InitStructure.CAN_TXFP=DISABLE;  				//禁止发送FIFO优先级
		CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;	//CAN 工作于正常模式   
		//CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;
	//   CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;     // for  //            
	//   CAN_InitStructure.CAN_BS1=CAN_BS1_7tq;	    // 250  //            
	//   CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;     // Kbit //             
	//   CAN_InitStructure.CAN_Prescaler = 12;      	//CAN波特率=RCC_APB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler;)
	//   CAN_init_SorF = CAN_Init(CAN2 ,&CAN_InitStructure);
		CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;      // 1 . for //           
		CAN_InitStructure.CAN_BS1=CAN_BS1_7tq;	    // 7 .  1  //            
		CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;      // 6 .  M  //              
		CAN_InitStructure.CAN_Prescaler = 3;      	// 3 . bit //(CAN波特率 = RCC_APB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler;)
		CAN_Init(CAN2 ,&CAN_InitStructure); 

		/* CAN filter init */

		/* 只接受0x016的CAN信息 */
		CAN_FilterInitStructure.CAN_FilterNumber     = 14;                     //设置滤波器组 序号为0~13
		CAN_FilterInitStructure.CAN_FilterMode       = CAN_FilterMode_IdMask; //滤波器类型 屏蔽位模式
		CAN_FilterInitStructure.CAN_FilterScale      = CAN_FilterScale_32bit; //滤波器位数 32位
		CAN_FilterInitStructure.CAN_FilterIdHigh     = 0x0000;
		CAN_FilterInitStructure.CAN_FilterIdLow      = 0x0000;
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
		CAN_FilterInitStructure.CAN_FilterMaskIdLow  = 0x0000;
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;	  //接收缓冲区0
		CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;		        //滤波器使能
		CAN_FilterInit(&CAN_FilterInitStructure);		                    //初始化 滤波器
		
// 		  CAN_FilterInitStructure.CAN_FilterNumber     = 14;                     //设置滤波器组 序号为0~13
// 			CAN_FilterInitStructure.CAN_FilterMode       = CAN_FilterMode_IdMask; //滤波器类型 屏蔽位模式
// 			CAN_FilterInitStructure.CAN_FilterScale      = CAN_FilterScale_32bit; //滤波器位数 32位
// 			CAN_FilterInitStructure.CAN_FilterIdHigh     = (((u32)0x206<<21)&0xFFFF0000)>>16;
// 			CAN_FilterInitStructure.CAN_FilterIdLow      = (((u32)0x206<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF;
// 			CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;
// 			CAN_FilterInitStructure.CAN_FilterMaskIdLow  = 0xFFFF;
// 			CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;	  //接收缓冲区0
// 			CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;		        //滤波器使能
// 			CAN_FilterInit(&CAN_FilterInitStructure);		 
		
		CAN_SlaveStartBank(14);
		
		CAN_ITConfig(CAN2 ,CAN_IT_FMP0, ENABLE);
		CAN_ITConfig(CAN2 ,CAN_IT_TME, ENABLE);
		
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
		NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
  }	
	
  return 1;
}

/**
 * Send a CAN Message Passed from the CANOpen Stack
 * @param  CANx CAN_PORT used 1 or 2.
 * @param  m    Pointer to Message to send
 * @return      0-Failed, 1-Success
 */
unsigned char canSend(CAN_PORT CANx, Message *m)
{
	unsigned char ret;
	unsigned char i;
	CanTxMsg TxMessage;

	TxMessage.StdId = (uint32_t)(m->cob_id);
	TxMessage.ExtId = 0x00;
	/* 是否远程帧 */
	TxMessage.RTR = m->rtr;
	/* CAN 2.0A 若用B需更改 */
	TxMessage.IDE = CAN_ID_STD;
	//TxMessage.IDE =0x00000000;
	/* 数据长度 */
	TxMessage.DLC = m->len;
	/* 为数据赋值 */                 
	for(i=0;i<m->len;i++)
	{
		TxMessage.Data[i] = m->data[i];
	}
     
    ret = CAN_Transmit(CANx, &TxMessage);

    if(ret == CAN_TxStatus_NoMailBox)
  		return 0;	//发送失败，发送邮箱满
		else 
			return 1;	//发送成功
}

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
