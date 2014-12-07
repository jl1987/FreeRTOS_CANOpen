/**
  ******************************************************************************
  * @file    BMS2.0_RC/USER/src/main.c 
  * @author  Intelligent Robotics Team
  * @version V2.0
  * @date    26-March-2014
  * @brief   BMS Task Functions.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

u8 rec_inc=0;

u8 ICC_CHECK = 0;

u16 CNTBUF;
u16 CPU_rate;

u8 no_ack=1,CAN_init_SorF=0,transmit_mailbox=0;

CanRxMsg  RxMessage,RxMessage_FIFO0,RxMessage_FIFO1,RxMessage_BMS;
CanTxMsg  TXMSG_AUTO, TXMSG_CHASSIS, TXMSG_SHUN;

CanTxMsg TX_TEST;

//u8 Filter_rec=0,Filter_num=0;

u16 delay_t=0;
u8 DEAT_COUNTING=0;

u8 mailbox;

void Robot_task(void)
{
	char message[100];
	//USART_SendData(USART1,0x04);  //test Damn BUG
	
	
	/* Get Data From bq34z100 */  //-Copy From Main.c
	GetDataFromBQZ(&BMS);
	
	//USART_SendData(USART1,0x05);  //test Damn BUG
	
	
  /* Control MOS */
  if (BMS.Voltage<DEAD_VOLTAGE)
  {
		DEAT_COUNTING++;
  }else
  {
		DEAT_COUNTING=0;
  }
	
  if ((DEAT_COUNTING>=50)||(BMS.StateOfCharge==0))
  {
		//POWER_OFF;  // Didn't Flash into BMS, This is the new Version, Power Off When the Voltage is below 10v for about 50s
		POWER_ON;
  }else
  {
		POWER_ON;
  }

  /*Power Saving*/	
// 	if ((GPIO_ReadInputDataBit(POWER_DET_PORT,POWER_DET_PIN)== 1)||(GPIO_ReadInputDataBit(CHARGE_DET_PORT,CHARGE_DET_PIN)== 0))
//   {
// 		//POWER_OFF;  // Didn't Flash into BMS, This is the new Version, Power Off When the Voltage is below 10v for about 50s
// 		POWER_ON;
// 		TimerFlag.BQZ = 0;
//   }else
//   {
// 		if (TimerFlag.BQZ == 0x05)
// 		{
// 			TimerFlag.BQZ = 0;
// 			
// 			POWER_OFF;
// 			
// 		  GPIO_ResetBits(GPIOA,GPIO_Pin_2);	
// 			
// 			RunIntoStopMode();
// 		}
//   }
	
  /* LED Blink depend on Charge-Det */
  ChargeLED(BMS.StateOfCharge);

  /* Send Urgency Data to PC */
  UrgencyData(&BMS);
	
// 	USART_SendData(USART1,0xFF);
// 	//USART_SendData(USART1,BMS.AverageCurrent);
// 	USART_SendData(USART1,(BMS.AverageCurrent)>>8);
// 	USART_SendData(USART1,(BMS.AverageCurrent)&0xff);
// 	USART_SendData(USART1,0xFF);
	
	
  //AlertCheck();

	//sprintf(message,"AverageCurrent: %d \r\n", BMS.AverageCurrent);
	//USART1_Puts(message);


	/* Send Data Back to PC with Serial - For TEST */
	//data_back(BMS.Voltage, BMS.AverageCurrent, BMS.TimeToEmpty,
						//BMS.StateOfCharge, BMS.StateOfHealth);
	
  CNTBUF = TIM2->CNT;
  CPU_rate = CNTBUF/100;

}



/*** (C) COPYRIGHT 2014 CSST R&D Intelligent Robotics Research Centre *END OF FILE*/
