/*
This file is edited as a part of CanFestival.

STM32F407 Port: Jim
*/


#include "stm32f10x_can.h"
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
			prescaler=1;
			break;
		case 500:
			prescaler=2;
			break;
		case 250:
			prescaler=4;
			break;
		case 200:
			prescaler=5;
			break;
		case 125:
			prescaler=8;
			break;
		case 100:
			prescaler=10;
			break;
		case 50:
			prescaler=20;
			break;
		case 10:
			prescaler=100;
			break;
		default:
			prescaler=1;
			break;
	}

	if(CANx == CAN1)
	{
		/* GPIO configuration ********************************************************/ 
		/* Enable GPIO clock */
		RCC_APB2PeriphClockCmd(CAN1_GPIO_CLK, ENABLE);
		#ifdef ARM_LIFTER
			GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);
		#endif
		/* CAN RX PIN */
		GPIO_InitStructure.GPIO_Pin  = CAN1_RX_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);
		/* CAN TX PIN */
		GPIO_InitStructure.GPIO_Pin   = CAN1_TX_PIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
		GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);
		/* CAN configuration ********************************************************/  
		/* Enable CAN1 clock */
		RCC_APB1PeriphClockCmd(CAN1_CLK, ENABLE);
		/* CAN register init */
		CAN_ITConfig(CAN1,CAN_IT_FMP0, DISABLE);    //CAN RX Interupt Disable
		CAN_DeInit(CAN1);													  //CAN1 Initial
		CAN_StructInit(&CAN_InitStructure);
		/* CAN cell init */
		CAN_InitStructure.CAN_TTCM = DISABLE;	//禁止时间触发通信模式
		CAN_InitStructure.CAN_ABOM = DISABLE;	//自动离线管理不使能
		CAN_InitStructure.CAN_AWUM = DISABLE;	//自动唤醒模式不使能
		CAN_InitStructure.CAN_NART = DISABLE;	//非自动重传模式不使能，即自动重传直到成功
		CAN_InitStructure.CAN_RFLM = DISABLE;	//FIFO锁定模式不使能，溢出时旧报文覆盖新报文
		CAN_InitStructure.CAN_TXFP = DISABLE;	//FIFO优先级确定，优先级由报文标识符确定
		CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; 
		CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;     // 1 . for //          
		CAN_InitStructure.CAN_BS1  = CAN_BS1_4tq;	    // 4 .  1  //           
		CAN_InitStructure.CAN_BS2  = CAN_BS2_3tq;     // 3 .  M  //             
		CAN_InitStructure.CAN_Prescaler = prescaler;  // 1 . bit //CAN = RCC_APB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler;)
		CAN_Init(CAN1 ,&CAN_InitStructure);
		/* CAN filter init */
		/* Only Recive CAN Message of 0x015  */
	//   CAN_FilterInitStructure.CAN_FilterNumber     = 0;                     //0~13
	//   CAN_FilterInitStructure.CAN_FilterMode       = CAN_FilterMode_IdMask; //ID MASK MODE
	//   CAN_FilterInitStructure.CAN_FilterScale      = CAN_FilterScale_32bit; //Filter Scale 32bit
	//   CAN_FilterInitStructure.CAN_FilterIdHigh     = (((u32)0x015<<21)&0xFFFF0000)>>16;
	//   CAN_FilterInitStructure.CAN_FilterIdLow      = (((u32)0x015<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF;
	//   CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;  //  FxR1: xxxx xxx0 101x xxxx xxxx xxxx xxxx x00x
	//   CAN_FilterInitStructure.CAN_FilterMaskIdLow  = 0xFFFF;  //  FxR2: 0000 0001 1110 0000 0000 0000 0000 0110
	//   CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
	//   CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	//   CAN_FilterInit(&CAN_FilterInitStructure);			
		/* Only Recive CAN Message of 0x015 & 0x035  */
// 		CAN_FilterInitStructure.CAN_FilterNumber     = 0;                     //0~13
// 		CAN_FilterInitStructure.CAN_FilterMode       = CAN_FilterMode_IdMask; //ID MASK MODE
// 		CAN_FilterInitStructure.CAN_FilterScale      = CAN_FilterScale_32bit; //Filter Scale 32bit
// 		CAN_FilterInitStructure.CAN_FilterIdHigh     = (((u32)0x015<<21)&0xFFFF0000)>>16;
// 		CAN_FilterInitStructure.CAN_FilterIdLow      = (((u32)0x015<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF;
// 		CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x01EF;  //  FxR1: xxxx xxx0 101x xxxx xxxx xxxx xxxx x00x
// 		CAN_FilterInitStructure.CAN_FilterMaskIdLow  = 0xFFFF;  //  FxR2: 0000 0001 1110 0000 0000 0000 0000 0110
// 		CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
// 		CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
// 		CAN_FilterInit(&CAN_FilterInitStructure);	

		/* Recive Everything */
		CAN_FilterInitStructure.CAN_FilterNumber     = 0;                     //0~13
		CAN_FilterInitStructure.CAN_FilterMode       = CAN_FilterMode_IdMask; //ID MASK MODE
		CAN_FilterInitStructure.CAN_FilterScale      = CAN_FilterScale_32bit; //Filter Scale 32bit
		CAN_FilterInitStructure.CAN_FilterIdHigh     = 0x0000;
		CAN_FilterInitStructure.CAN_FilterIdLow      = 0x0000;
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;  //  FxR1: xxxx xxx0 101x xxxx xxxx xxxx xxxx x00x
		CAN_FilterInitStructure.CAN_FilterMaskIdLow  = 0x0000;  //  FxR2: 0000 0001 1110 0000 0000 0000 0000 0110
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
		CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
		CAN_FilterInit(&CAN_FilterInitStructure);	
		
		CAN_ITConfig(CAN1 ,CAN_IT_FMP0, ENABLE);
		/* Configuration NVIC */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
		NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
		NVIC_Init(&NVIC_InitStructure);
		
	}
	else
	{
		/* GPIO configuration ********************************************************/ 
		/* Enable GPIO clock */
		RCC_APB2PeriphClockCmd(CAN1_GPIO_CLK, ENABLE);
		#ifdef ARM_LIFTER
			GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);
		#endif
		/* CAN RX PIN */
		GPIO_InitStructure.GPIO_Pin  = CAN1_RX_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);
		/* CAN TX PIN */
		GPIO_InitStructure.GPIO_Pin   = CAN1_TX_PIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
		GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);
		/* CAN configuration ********************************************************/  
		/* Enable CAN1 clock */
		RCC_APB1PeriphClockCmd(CAN1_CLK, ENABLE);
		/* CAN register init */
		CAN_ITConfig(CAN1,CAN_IT_FMP0, DISABLE);    //CAN RX Interupt Disable
		CAN_DeInit(CAN1);													  //CAN1 Initial
		CAN_StructInit(&CAN_InitStructure);
		/* CAN cell init */
		CAN_InitStructure.CAN_TTCM = DISABLE;	//禁止时间触发通信模式
		CAN_InitStructure.CAN_ABOM = DISABLE;	//自动离线管理不使能
		CAN_InitStructure.CAN_AWUM = DISABLE;	//自动唤醒模式不使能
		CAN_InitStructure.CAN_NART = DISABLE;	//非自动重传模式不使能，即自动重传直到成功
		CAN_InitStructure.CAN_RFLM = DISABLE;	//FIFO锁定模式不使能，溢出时旧报文覆盖新报文
		CAN_InitStructure.CAN_TXFP = DISABLE;	//FIFO优先级确定，优先级由报文标识符确定
		CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; 
		CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;     // 1 . for //          
		CAN_InitStructure.CAN_BS1  = CAN_BS1_4tq;	    // 4 .  1  //           
		CAN_InitStructure.CAN_BS2  = CAN_BS2_3tq;     // 3 .  M  //             
		CAN_InitStructure.CAN_Prescaler = prescaler;  // 1 . bit //CAN = RCC_APB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler;)
		CAN_Init(CAN1 ,&CAN_InitStructure);
		/* CAN filter init */
		/* Only Recive CAN Message of 0x015  */
	//   CAN_FilterInitStructure.CAN_FilterNumber     = 0;                     //0~13
	//   CAN_FilterInitStructure.CAN_FilterMode       = CAN_FilterMode_IdMask; //ID MASK MODE
	//   CAN_FilterInitStructure.CAN_FilterScale      = CAN_FilterScale_32bit; //Filter Scale 32bit
	//   CAN_FilterInitStructure.CAN_FilterIdHigh     = (((u32)0x015<<21)&0xFFFF0000)>>16;
	//   CAN_FilterInitStructure.CAN_FilterIdLow      = (((u32)0x015<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF;
	//   CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;  //  FxR1: xxxx xxx0 101x xxxx xxxx xxxx xxxx x00x
	//   CAN_FilterInitStructure.CAN_FilterMaskIdLow  = 0xFFFF;  //  FxR2: 0000 0001 1110 0000 0000 0000 0000 0110
	//   CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
	//   CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	//   CAN_FilterInit(&CAN_FilterInitStructure);			
		/* Only Recive CAN Message of 0x015 & 0x035  */
// 		CAN_FilterInitStructure.CAN_FilterNumber     = 0;                     //0~13
// 		CAN_FilterInitStructure.CAN_FilterMode       = CAN_FilterMode_IdMask; //ID MASK MODE
// 		CAN_FilterInitStructure.CAN_FilterScale      = CAN_FilterScale_32bit; //Filter Scale 32bit
// 		CAN_FilterInitStructure.CAN_FilterIdHigh     = (((u32)0x015<<21)&0xFFFF0000)>>16;
// 		CAN_FilterInitStructure.CAN_FilterIdLow      = (((u32)0x015<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF;
// 		CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x01EF;  //  FxR1: xxxx xxx0 101x xxxx xxxx xxxx xxxx x00x
// 		CAN_FilterInitStructure.CAN_FilterMaskIdLow  = 0xFFFF;  //  FxR2: 0000 0001 1110 0000 0000 0000 0000 0110
// 		CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
// 		CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
// 		CAN_FilterInit(&CAN_FilterInitStructure);	

		/* Recive Everything */
		CAN_FilterInitStructure.CAN_FilterNumber     = 0;                     //0~13
		CAN_FilterInitStructure.CAN_FilterMode       = CAN_FilterMode_IdMask; //ID MASK MODE
		CAN_FilterInitStructure.CAN_FilterScale      = CAN_FilterScale_32bit; //Filter Scale 32bit
		CAN_FilterInitStructure.CAN_FilterIdHigh     = 0x0000;
		CAN_FilterInitStructure.CAN_FilterIdLow      = 0x0000;
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;  //  FxR1: xxxx xxx0 101x xxxx xxxx xxxx xxxx x00x
		CAN_FilterInitStructure.CAN_FilterMaskIdLow  = 0x0000;  //  FxR2: 0000 0001 1110 0000 0000 0000 0000 0110
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
		CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
		CAN_FilterInit(&CAN_FilterInitStructure);	
		
		CAN_ITConfig(CAN1 ,CAN_IT_FMP0, ENABLE);
		/* Configuration NVIC */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
		NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
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
