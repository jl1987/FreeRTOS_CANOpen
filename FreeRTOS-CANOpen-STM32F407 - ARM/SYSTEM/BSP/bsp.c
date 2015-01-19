/**
  ******************************************************************************
  * @file    bsp.c
  * @author  Jim
  * @version V1.0
  * @date    01-Sep-2014
  * @brief   This is the bsp driver function for STM32F407ZGT6.
  * @brief 	 bsp = Borad surport packet 板级支持包
  *          这是硬件底层驱动程序的主文件。每个c文件可以 #include "bsp.h" 
  *          来包含所有的外设驱动模块
  * 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */


#include "bsp.h"
#include "stm32f4xx_conf.h"


void bsp_Init(void)
{
	/*
		由于ST固件库的启动文件已经执行了CPU系统时钟的初始化，所以不必再次重复配置系统时钟。
		启动文件配置了CPU主时钟频率、内部Flash访问速度和可选的外部SRAM FSMC初始化。

		系统时钟缺省配置为168MHz，如果需要更改，可以修改 system_stm32f4xx.c 文件
	*/
   /* Enable CRC clock */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
	
	
	 /* 针对不同的应用程序，添加需要的底层驱动模块初始化函数 */
	 bsp_InitLed(); 		/* 初始LED指示灯端口 */
}




/*********************************************************************************
*描述：GPIO端口配置函数
*
*返回值：无
**********************************************************************************/

// void RCCInit(void)
// {
//   /*
//   *            @arg RCC_AHB1Periph_GPIOA:       GPIOA clock
//   *            @arg RCC_AHB1Periph_GPIOB:       GPIOB clock 
//   *            @arg RCC_AHB1Periph_GPIOC:       GPIOC clock
//   *            @arg RCC_AHB1Periph_GPIOD:       GPIOD clock
//   *            @arg RCC_AHB1Periph_GPIOE:       GPIOE clock
//   *            @arg RCC_AHB1Periph_GPIOF:       GPIOF clock
//   *            @arg RCC_AHB1Periph_GPIOG:       GPIOG clock
//   *            @arg RCC_AHB1Periph_GPIOH:       GPIOH clock
//   *            @arg RCC_AHB1Periph_GPIOI:       GPIOI clock
//   *            @arg RCC_AHB1Periph_CRC:         CRC clock
//   *            @arg RCC_AHB1Periph_BKPSRAM:     BKPSRAM interface clock
//   *            @arg RCC_AHB1Periph_CCMDATARAMEN CCM data RAM interface clock
//   *            @arg RCC_AHB1Periph_DMA1:        DMA1 clock
//   *            @arg RCC_AHB1Periph_DMA2:        DMA2 clock
//   *            @arg RCC_AHB1Periph_ETH_MAC:     Ethernet MAC clock
//   *            @arg RCC_AHB1Periph_ETH_MAC_Tx:  Ethernet Transmission clock
//   *            @arg RCC_AHB1Periph_ETH_MAC_Rx:  Ethernet Reception clock
//   *            @arg RCC_AHB1Periph_ETH_MAC_PTP: Ethernet PTP clock
//   *            @arg RCC_AHB1Periph_OTG_HS:      USB OTG HS clock
//   *            @arg RCC_AHB1Periph_OTG_HS_ULPI: USB OTG HS ULPI clock
//   *
//   *            @arg RCC_APB1Periph_TIM2:   TIM2 clock
//   *            @arg RCC_APB1Periph_TIM3:   TIM3 clock
//   *            @arg RCC_APB1Periph_TIM4:   TIM4 clock
//   *            @arg RCC_APB1Periph_TIM5:   TIM5 clock
//   *            @arg RCC_APB1Periph_TIM6:   TIM6 clock
//   *            @arg RCC_APB1Periph_TIM7:   TIM7 clock
//   *            @arg RCC_APB1Periph_TIM12:  TIM12 clock
//   *            @arg RCC_APB1Periph_TIM13:  TIM13 clock
//   *            @arg RCC_APB1Periph_TIM14:  TIM14 clock
//   *            @arg RCC_APB1Periph_WWDG:   WWDG clock
//   *            @arg RCC_APB1Periph_SPI2:   SPI2 clock
//   *            @arg RCC_APB1Periph_SPI3:   SPI3 clock
//   *            @arg RCC_APB1Periph_USART2: USART2 clock
//   *            @arg RCC_APB1Periph_USART3: USART3 clock
//   *            @arg RCC_APB1Periph_UART4:  UART4 clock
//   *            @arg RCC_APB1Periph_UART5:  UART5 clock
//   *            @arg RCC_APB1Periph_I2C1:   I2C1 clock
//   *            @arg RCC_APB1Periph_I2C2:   I2C2 clock
//   *            @arg RCC_APB1Periph_I2C3:   I2C3 clock
//   *            @arg RCC_APB1Periph_CAN1:   CAN1 clock
//   *            @arg RCC_APB1Periph_CAN2:   CAN2 clock
//   *            @arg RCC_APB1Periph_PWR:    PWR clock
//   *            @arg RCC_APB1Periph_DAC:    DAC clock

//   *            @arg RCC_APB2Periph_TIM1:   TIM1 clock
//   *            @arg RCC_APB2Periph_TIM8:   TIM8 clock
//   *            @arg RCC_APB2Periph_USART1: USART1 clock
//   *            @arg RCC_APB2Periph_USART6: USART6 clock
//   *            @arg RCC_APB2Periph_ADC1:   ADC1 clock
//   *            @arg RCC_APB2Periph_ADC2:   ADC2 clock
//   *            @arg RCC_APB2Periph_ADC3:   ADC3 clock
//   *            @arg RCC_APB2Periph_SDIO:   SDIO clock
//   *            @arg RCC_APB2Periph_SPI1:   SPI1 clock
//   *            @arg RCC_APB2Periph_SYSCFG: SYSCFG clock
//   *            @arg RCC_APB2Periph_TIM9:   TIM9 clock
//   *            @arg RCC_APB2Periph_TIM10:  TIM10 clock
//   *            @arg RCC_APB2Periph_TIM11:  TIM11 clock */
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|
//                         RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|
// 	                      RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_DMA1|
// 												RCC_AHB1Periph_DMA2|RCC_AHB1Periph_GPIOG
// 												, ENABLE);
// 												
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB1Periph_USART3| RCC_APB1Periph_UART4| 
//                         RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3|
// 	                      RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM5|
// 												RCC_APB1Periph_TIM7|RCC_APB1Periph_PWR|
// 												RCC_APB1Periph_CAN1|RCC_APB1Periph_CAN2|
// 												RCC_APB1Periph_UART5
// 												,ENABLE);
// 												
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1|RCC_APB2Periph_ADC1|
//                         RCC_APB2Periph_ADC2|RCC_APB2Periph_ADC3|RCC_APB2Periph_SPI1|
//                         RCC_APB2Periph_SYSCFG|RCC_APB2Periph_USART1|
// 												RCC_APB2Periph_USART6
// 												,ENABLE);
//  RCC_LSICmd(ENABLE);//打开LSI
//  while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY)==RESET);
// }
// void GPIOInit(void)			
// {
//  GPIO_InitTypeDef  GPIO_InitStructure;//
//  GPIO_DeInit(GPIOA);
//  GPIO_DeInit(GPIOB);
//  GPIO_DeInit(GPIOC);
//  GPIO_DeInit(GPIOD);
//  GPIO_DeInit(GPIOE);
//  GPIO_DeInit(GPIOF);
//  GPIO_DeInit(GPIOG);

//  RCC_LSICmd(ENABLE);//打开LSI
//  while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY)==RESET);
// 	
// 	//LED
// 	GPIO_InitStructure.GPIO_Pin = LED1|LED2|LED3|LED4;								//需要配置的引脚
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
// 	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
// 	GPIO_Init(LEDPORT, &GPIO_InitStructure);
// 	GPIO_WriteBit(LEDPORT, LED1|LED2|LED3|LED4, Bit_SET);
// 	//LED
// 	
// 	//FAULT
// 	GPIO_InitStructure.GPIO_Pin = Channel1_FLT|Channel2_FLT|Channel3_FLT;		
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
// 	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	        //需要配置的引脚
// 	GPIO_Init(Channel1_FLTPORT, &GPIO_InitStructure);
// 	//FAULT
// 	
// 	//RST
// 	GPIO_InitStructure.GPIO_Pin = Channel1_RST;								//需要配置的引脚
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
// 	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
// 	GPIO_Init(Channel1_RSTPORT, &GPIO_InitStructure);
// 	GPIO_WriteBit(Channel1_RSTPORT,Channel1_RST, Bit_SET);
// 	
// 	GPIO_InitStructure.GPIO_Pin = Channel2_RST|Channel3_RST;								//需要配置的引脚
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
// 	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
// 	GPIO_Init(Channel2_RSTPORT, &GPIO_InitStructure);
// 	GPIO_WriteBit(Channel2_RSTPORT,Channel2_RST|Channel2_RST, Bit_SET);
// 	//RST
// 	
// 	//U1
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;	  //tx
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//   GPIO_Init(GPIOA, &GPIO_InitStructure);
//   GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);	
// 	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);	
// 	//U1
// 	//U2-m
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;	  //tx
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//   GPIO_Init(GPIOA, &GPIO_InitStructure);
//   GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);	
// 	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);	
// 	//U2-m
// 	//U3-m
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;	  //tx
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//   GPIO_Init(GPIOB, &GPIO_InitStructure);
//   GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);	
// 	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);	
// 	//U3-m

// 	//U4-IMU
// 	/* Connect USART pins to AF7 */
// 	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
// 	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
// 	  
// 	  /* Configure USART Tx and Rx as alternate function push-pull */
// 	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
// 	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
// 	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
// 	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
// 	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
// 	  GPIO_Init(GPIOC, &GPIO_InitStructure);
// 	  
// 	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
// 	  GPIO_Init(GPIOC, &GPIO_InitStructure);
// 	//U4-IMU

// 	//U5-m
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	  //TX
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//   GPIO_Init(GPIOC, &GPIO_InitStructure);
// 	
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	    //RX
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//   GPIO_Init(GPIOD, &GPIO_InitStructure);
// 	
//   GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);	
// 	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,  GPIO_AF_UART5);	
// 	//U5-m
// 	//U6
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	  //tx
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//   GPIO_Init(GPIOC, &GPIO_InitStructure);
//   GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);	
// 	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);	
// 	//U6
// 	
// 	//T1PWM
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//   GPIO_Init(GPIOE, &GPIO_InitStructure); 
//   GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
//   GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
// 	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
// 	//T1PWM
// 	

// 	
// 	//EXTI 
//   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;       //输入模式
// 	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;   //上拉
//   GPIO_Init(GPIOG, &GPIO_InitStructure);           
//   //EXTI
// 	
// 	//CAN2 
//   /* Configure CAN RX and TX pins */
//   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
//   GPIO_Init(GPIOB, &GPIO_InitStructure);
//   GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
//   GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); 
//   //CAN2

//   //Infrared Switch S1-S9 
//   // Configure Intrared Switch pins 
//   	GPIO_InitStructure.GPIO_Pin = Infrared_S1|Infrared_S2;		
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
// 	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	        //需要配置的引脚
// 	GPIO_Init(GPIOC, &GPIO_InitStructure);

// 	GPIO_InitStructure.GPIO_Pin = Infrared_S3|Infrared_S4;		
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
// 	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	        //需要配置的引脚
// 	GPIO_Init(GPIOB, &GPIO_InitStructure);

// 	GPIO_InitStructure.GPIO_Pin = Infrared_S5|Infrared_S6|Infrared_S7|Infrared_S8|Infrared_S9;		
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
// 	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	        //需要配置的引脚
// 	GPIO_Init(GPIOF, &GPIO_InitStructure);
//   //Infrared Switch S1-S9 

//   // Force Sensor (Height 1-3)
//   // Configure ADC3 Channel 0, 1 pins as analog input for force sensors 1 and 2 respectively
//   GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
//   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//   GPIO_Init(GPIOA, &GPIO_InitStructure);

//   // Configure ADC3 Channel13 pin as analog input for force sensor 3
//   GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
//   GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//   GPIO_Init(GPIOC, &GPIO_InitStructure);

// }
// void DMAInit(void)
// {
//   	DMA_InitTypeDef DMA_InitStruct;
// //	DMA_DeInit(DMA1_Stream0);
//   	DMA_DeInit(DMA1_Stream1);
// 	DMA_DeInit(DMA1_Stream3);
//   	DMA_DeInit(DMA1_Stream5);
// 	DMA_DeInit(DMA1_Stream6);
// //  	DMA_DeInit(DMA1_Stream7);
//   	
// 	DMA_DeInit(DMA2_Stream0);
// 	DMA_DeInit(DMA2_Stream5);
//   	DMA_DeInit(DMA2_Stream7);
// /*	
// 	//U1
// 	while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE);	
//   DMA_InitStruct.DMA_Channel = DMA_Channel_4;			   //S5C4
//   DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(USART1->DR));
//   DMA_InitStruct.DMA_Memory0BaseAddr = (u32)MSG_U1_R;
//   DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;             //RX
//   DMA_InitStruct.DMA_BufferSize = (u32)0;
//   DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//   DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
//   DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//   DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//   DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
//   DMA_InitStruct.DMA_Priority = DMA_Priority_High;
//   DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
//   DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//   DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//   DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//   DMA_Init(DMA2_Stream5, &DMA_InitStruct);
//   //DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);
//   DMA_Cmd(DMA2_Stream5, ENABLE);  //接收通道
// 	
// 	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);	
// 	DMA_InitStruct.DMA_Channel = DMA_Channel_4;			   //S7C4
//   DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(USART1->DR));
//   DMA_InitStruct.DMA_Memory0BaseAddr = (u32)MSG_U1_T;
//   DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;            //TX
//   DMA_InitStruct.DMA_BufferSize = (u32)0;
//   DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//   DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
//   DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//   DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//   DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
//   DMA_InitStruct.DMA_Priority = DMA_Priority_High;
//   DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
//   DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//   DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//   DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//   DMA_Init(DMA2_Stream7, &DMA_InitStruct);
//   //DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
//   DMA_Cmd(DMA2_Stream7, ENABLE);  //接收通道
// 	//U1
// */	
// 	//U2
// 	while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);	
//   DMA_InitStruct.DMA_Channel = DMA_Channel_4;			   //S5C4
//   DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(USART2->DR));
//   DMA_InitStruct.DMA_Memory0BaseAddr = (u32)MSG_U2_R;
//   DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;             //RX
//   DMA_InitStruct.DMA_BufferSize = (u32)10;
//   DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//   DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
//   DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//   DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//   DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
//   DMA_InitStruct.DMA_Priority = DMA_Priority_High;
//   DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
//   DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//   DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//   DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//   DMA_Init(DMA1_Stream5, &DMA_InitStruct);
//   DMA_Cmd(DMA1_Stream5, ENABLE);  //接收通道
// 	
// 	while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE);	
// 	DMA_InitStruct.DMA_Channel = DMA_Channel_4;			   //S6C4
//   DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(USART2->DR));
//   DMA_InitStruct.DMA_Memory0BaseAddr = (u32)MSG_U2_T;
//   DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;            //TX
//   DMA_InitStruct.DMA_BufferSize = (u32)10;
//   DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//   DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
//   DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//   DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//   DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
//   DMA_InitStruct.DMA_Priority = DMA_Priority_High;
//   DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
//   DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//   DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//   DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//   DMA_Init(DMA1_Stream6, &DMA_InitStruct);
//   DMA_Cmd(DMA1_Stream6, ENABLE);  //接收通道
// 	//U2
// 	
// 	//U3
// 	while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);	
//   DMA_InitStruct.DMA_Channel = DMA_Channel_4;			   //S1C4
//   DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(USART3->DR));
//   DMA_InitStruct.DMA_Memory0BaseAddr = (u32)MSG_U3_R;
//   DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;             //RX
//   DMA_InitStruct.DMA_BufferSize = (u32)10;
//   DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//   DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
//   DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//   DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//   DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
//   DMA_InitStruct.DMA_Priority = DMA_Priority_High;
//   DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
//   DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//   DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//   DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//   DMA_Init(DMA1_Stream1, &DMA_InitStruct);
//   DMA_Cmd(DMA1_Stream1, ENABLE);  //接收通道
// 	
// 	while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE);	
// 	DMA_InitStruct.DMA_Channel = DMA_Channel_4;			   //S3C4
//   DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(USART3->DR));
//   DMA_InitStruct.DMA_Memory0BaseAddr = (u32)MSG_U3_T;
//   DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;            //TX
//   DMA_InitStruct.DMA_BufferSize = (u32)10;
//   DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//   DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
//   DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//   DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//   DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
//   DMA_InitStruct.DMA_Priority = DMA_Priority_High;
//   DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
//   DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//   DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//   DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//   DMA_Init(DMA1_Stream3, &DMA_InitStruct);
//   DMA_Cmd(DMA1_Stream3, ENABLE);  //接收通道
// 	//U3
// 	
// // 	//U5
// // 	while (DMA_GetCmdStatus(DMA1_Stream0) != DISABLE);	
// //   DMA_InitStruct.DMA_Channel = DMA_Channel_4;			   //S0C4
// //   DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(UART5->DR));
// //   DMA_InitStruct.DMA_Memory0BaseAddr = (u32)MSG_U5_R;
// //   DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;             //RX
// //   DMA_InitStruct.DMA_BufferSize = (u32)0;
// //   DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
// //   DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
// //   DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
// //   DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
// //   DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
// //   DMA_InitStruct.DMA_Priority = DMA_Priority_High;
// //   DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
// //   DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
// //   DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
// //   DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
// //   DMA_Init(DMA1_Stream5, &DMA_InitStruct);
// //   DMA_Cmd(DMA1_Stream5, ENABLE);  //接收通道
// // 	
// // 	while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE);	
// // 	DMA_InitStruct.DMA_Channel = DMA_Channel_4;			   //S7C4
// //   DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(UART5->DR));
// //   DMA_InitStruct.DMA_Memory0BaseAddr = (u32)MSG_U1_T;
// //   DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;            //TX
// //   DMA_InitStruct.DMA_BufferSize = (u32)0;
// //   DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
// //   DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
// //   DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
// //   DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
// //   DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
// //   DMA_InitStruct.DMA_Priority = DMA_Priority_High;
// //   DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
// //   DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
// //   DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
// //   DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
// //   DMA_Init(DMA1_Stream7, &DMA_InitStruct);
// //   DMA_Cmd(DMA1_Stream7, ENABLE);  //接收通道
// // 	//U5
// 	
//   // ADC for force sensor
//   // DMA2 Stream0 channel2 configuration for ADC3
//   while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE);	
//   DMA_InitStruct.DMA_Channel = DMA_Channel_2;
//   DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
//     
//   DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
//   DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
//   DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//   DMA_InitStruct.DMA_Priority = DMA_Priority_High;  
//   DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;  
//   DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
//   DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//   DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
//   DMA_InitStruct.DMA_BufferSize = 3;  
//   DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//   DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//   
//   DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;         
//   DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
//   
//   DMA_Init(DMA2_Stream0, &DMA_InitStruct);
//   DMA_Cmd(DMA2_Stream0, ENABLE);
//   // ADC for force sensor
// }


// void USARTInit(void)
// {
//  	USART_InitTypeDef USART_InitStructure;

// 	USART_DeInit(USART1);
// 	USART_DeInit(USART2);
// 	USART_DeInit(USART3);
// 	USART_DeInit(UART5);
// 	
// 	USART_InitStructure.USART_BaudRate = 115200;
//   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//   USART_InitStructure.USART_StopBits = USART_StopBits_1;
//   USART_InitStructure.USART_Parity = USART_Parity_No;
//   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//   USART_Init(USART1, &USART_InitStructure); 
// //  USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
// //  USART_DMACmd(USART1,USART_DMAReq_Rx, ENABLE);
// //	USART_DMACmd(USART1,USART_DMAReq_Tx, ENABLE);

// // Enable Interrupt
// 	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
// 	
//   USART_Cmd(USART1, ENABLE);
//  /*
// 	USART_InitStructure.USART_BaudRate = 19200;
//   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//   USART_InitStructure.USART_StopBits = USART_StopBits_1;
//   USART_InitStructure.USART_Parity = USART_Parity_No;
//   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//   USART_Init(USART6, &USART_InitStructure); 
//   USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
//   USART_DMACmd(USART6,USART_DMAReq_Rx, ENABLE);
// 	USART_DMACmd(USART6,USART_DMAReq_Tx, ENABLE);
//   USART_Cmd(USART6, ENABLE);
// */	
// 	//M1
// 	USART_InitStructure.USART_BaudRate = 19200;
//   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//   USART_InitStructure.USART_StopBits = USART_StopBits_1;
//   USART_InitStructure.USART_Parity = USART_Parity_No;
//   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//   USART_Init(USART2, &USART_InitStructure); 
// 	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
//   USART_DMACmd(USART2,USART_DMAReq_Rx, ENABLE);
// 	USART_DMACmd(USART2,USART_DMAReq_Tx, ENABLE);
//   USART_Cmd(USART2, ENABLE);
// 	//M1
// 	
// 	//M2
// 	USART_InitStructure.USART_BaudRate = 19200;
//   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//   USART_InitStructure.USART_StopBits = USART_StopBits_1;
//   USART_InitStructure.USART_Parity = USART_Parity_No;
//   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//   USART_Init(USART3, &USART_InitStructure); 
// 	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
//   USART_DMACmd(USART3,USART_DMAReq_Rx, ENABLE);
// 	USART_DMACmd(USART3,USART_DMAReq_Tx, ENABLE);
//   USART_Cmd(USART3, ENABLE);
// 	//M2
// 	
// 	//M3  
// 	USART_InitStructure.USART_BaudRate = 19200;
//   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//   USART_InitStructure.USART_StopBits = USART_StopBits_1;
//   USART_InitStructure.USART_Parity = USART_Parity_No;
//   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//   USART_Init(UART5, &USART_InitStructure); 
// 	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);

// //   USART_DMACmd(UART5,USART_DMAReq_Rx, ENABLE);
// // 	USART_DMACmd(UART5,USART_DMAReq_Tx, ENABLE);
//   USART_Cmd(UART5, ENABLE);
// 	//M3

// 	// IMU
//     USART_InitStructure.USART_BaudRate = 115200;
// 	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
// 	USART_InitStructure.USART_StopBits = USART_StopBits_1;
// 	USART_InitStructure.USART_Parity = USART_Parity_No;
// 	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
// 	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
// 	USART_Init(UART4, &USART_InitStructure);
// 	/* Enable USART */
// 	/* Enable USART4 Receive Data Read To Be Read Interrupt */
// 	USART_Cmd(UART4, ENABLE);
// 	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
// 	// IMU
// }
// void TIMInit(void)
// {

//   uint16_t PrescalerValue = 0;	// Prescaler of the Timer5

//   TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
//   TIM_OCInitTypeDef TIM_OCInitStructure;   
// 	//--------------TIM1的配置
//   TIM_TimeBaseStructure.TIM_Prescaler = 100;
//   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//   TIM_TimeBaseStructure.TIM_Period = HALFPWMTIMERPERIOD*2;
//   TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//   TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//   TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
//   
//   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//   TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
//   TIM_OCInitStructure.TIM_Pulse = 0;
//   TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	  //还没匹配，高!
//   TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;  //匹配之后，低!
//   TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//   TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
//   //TIM_OC1Init(TIM1, &TIM_OCInitStructure); 
//   TIM_OC2Init(TIM1, &TIM_OCInitStructure);
// 	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
//   TIM_OC4Init(TIM1, &TIM_OCInitStructure);
//   TIM_Cmd(TIM1, ENABLE);
//   TIM_CtrlPWMOutputs(TIM1, ENABLE);
//   //---------------TIM1的配置

//   //--------------TIM5的配置
//   /* Compute the prescaler value */
//   // System clock 100MHz
//   PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 24000000) - 1;


//   /* TIM5 clock enable */
//   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
//   /* Time base configuration */
//   TIM_TimeBaseStructure.TIM_Period = 800000;  //    Timer5: 24000000/1000000 = 24 hz
//   TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue; 
//   TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//   TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
//   /* TIM IT enable */
//   TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
//   /* TIM5 enable counter */
//   TIM_Cmd(TIM5, ENABLE);
//   //--------------TIM5的配置
// }



// void NVICInit(void)
// {
//  NVIC_InitTypeDef NVIC_InitStructure; 

//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//  //******************************************************************
//  
// //  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
// //  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
// //  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
// //  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
// //  NVIC_Init(&NVIC_InitStructure);

//  /* Enable the TIM5 gloabal Interrupt */
//   NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//   NVIC_Init(&NVIC_InitStructure);
//   	
//  NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
// 	
//  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; 
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//  
//  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn; 
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//  	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);  //碠甅涩?だ?? 1
// 	
//  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
//  NVIC_Init(&NVIC_InitStructure);

//  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; 
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
//  NVIC_Init(&NVIC_InitStructure);
//  
//  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; 
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
//  NVIC_Init(&NVIC_InitStructure);
//  
//  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn; 
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
//   NVIC_Init(&NVIC_InitStructure);
//  /* NVIC configuration */
//  /* Configure the Priority Group to 2 bits */

//   
//  /* Enable the USARTx Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//   
// }



// void EXTIInit(void)
// {
//   EXTI_InitTypeDef EXTI_InitStructure;
// 	
//   SYSCFG_EXTILineConfig(IRPORT,IRPIN0);
//   SYSCFG_EXTILineConfig(IRPORT,IRPIN1);
// 	
// 	EXTI_InitStructure.EXTI_Line = EXTI_Line0|EXTI_Line1;
//   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
//   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//   EXTI_Init(&EXTI_InitStructure);
// }

// u16 filter_reg_value(u8 std_or_ext,u8 reg,u16 std_id,u32 ext_id)
// {
//   u16 memo;
// 	//屏蔽寄存器的设置  
//   mask =(std_id<<18);  
//   mask^=ext_id;  
//   mask=~mask;  
//   mask<<=3;  
//   mask|=0x02; //只接收数据帧，不接收远程帧 
// 	
// 	
// 	if(std_or_ext==CAN_ID_STD)
// 	{
// 	 switch(reg)
// 	 {
// 		 case 1: {memo = (std_id<<5)&0xffff;break;}
// 			
// 		 case 2: {memo = 0;break;}
// 			
// 		 case 3: {memo = (mask>>16)&0xffff;break;}
// 			
// 		 case 4: {memo = mask&0xffff;break;}
// 			
// 		 default :break;
//    }
//   }else
// 	{		
// 	 switch(reg)
// 	 {
// 		 case 1: {memo = ((ext_id<<3) >>16)&0xffff;break;}
// 			
// 		 case 2: {memo = (u16)(ext_id<<3)|CAN_ID_EXT;break;}
// 			
// 		 case 3: {memo = (mask>>16)&0xffff;break;}
// 			
// 		 case 4: {memo = mask&0xffff;break;}
// 			
// 		 default :break;
//    }
//   }
// 	
// 	return memo;
// }

// void CANInit(void)		                        //CAN初始化
// {
//   CAN_InitTypeDef        CAN_InitStructure;
//   CAN_FilterInitTypeDef  CAN_FilterInitStructure;
// 	
//   /* CAN register init */
//   CAN_ITConfig(CAN2,CAN_IT_FMP0, DISABLE);              //CAN 接收中断禁能
//   CAN_ITConfig(CAN2,CAN_IT_TME, DISABLE);
// 	CAN_DeInit(CAN2);													  //出示化CAN控制器
//   CAN_StructInit(&CAN_InitStructure);
//   /* CAN cell init */
//   CAN_InitStructure.CAN_TTCM=DISABLE;
//   CAN_InitStructure.CAN_ABOM=ENABLE;
//   CAN_InitStructure.CAN_AWUM=DISABLE;
//   CAN_InitStructure.CAN_NART=DISABLE;
//   CAN_InitStructure.CAN_RFLM=DISABLE;		            //FIFO 锁定模式关
//   CAN_InitStructure.CAN_TXFP=DISABLE;
//   CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;	            //CAN 工作于正常模式
//   CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;        //1          //1 个周期
//   CAN_InitStructure.CAN_BS1=CAN_BS1_7tq;	      //7          //7个周期
//   CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;        //6          //6 个周期
//   CAN_InitStructure.CAN_Prescaler= 3;           //3         //3预分频(CAN波特率 = RCC_APB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler;)
//   CAN_Init(CAN2 ,&CAN_InitStructure);           //1M       //1000k
// 	/* CAN filter init */

// 	CAN_FilterInitStructure.CAN_FilterNumber=15;//滤波器1，接收VBT6的
//   	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	
//   	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
//   	CAN_FilterInitStructure.CAN_FilterIdHigh  = (((u32)0x013<<21)&0xFFFF0000)>>16;
// 	CAN_FilterInitStructure.CAN_FilterIdLow   = (((u32)0x013<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF;
// 	CAN_FilterInitStructure.CAN_FilterMaskIdHigh  = 0x01EF;// xxxx xxx0 0111x xxxx  xxxx xxxx xxxx  x00x
// 	CAN_FilterInitStructure.CAN_FilterMaskIdLow  = 0xFFFF; // 0000 0001 1110 0000 0000 0000 0000 0110
// 	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;	
//   	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;		      //滤波器使能
//   	CAN_FilterInit(&CAN_FilterInitStructure);		                  //初始化 滤波器

// 	
// 	CAN_SlaveStartBank(14);
// 	
//   CAN_ITConfig(CAN2 ,CAN_IT_FMP0, ENABLE);
// 	CAN_ITConfig(CAN2 ,CAN_IT_TME, ENABLE);
//   /* initialize the value that will be returned */
// }	

// void ADC3_Init(void)
// {
//   ADC_InitTypeDef       ADC_InitStructure;
//   ADC_CommonInitTypeDef ADC_CommonInitStructure; 

//   /* ADC Common Init **********************************************************/
//   ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;       //?ミ家Α
//   ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;         //?だ?
//   ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;    //
//   //ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; 
//   ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_15Cycles;  //?┑?
//   ADC_CommonInit(&ADC_CommonInitStructure);

//   /* ADC3 Init ****************************************************************/
//   ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;    //?竚12だ?瞯
//   //ADC_InitStructure.ADC_ScanConvMode = DISABLE;
//   ADC_InitStructure.ADC_ScanConvMode = ENABLE;   //?磞家Α?硄笵㎝硄笵??だ
//   ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;      //??家Α
// // gtz02nov  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;   //场郉?窽ゎ
//   ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;   //gtz01nov 
//   ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
//   ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;         //?誹??
//   //ADC_InitStructure.ADC_NbrOfConversion = 1;
//   ADC_InitStructure.ADC_NbrOfConversion = 3;          //?竚ADC??硄笵?
//   ADC_Init(ADC3, &ADC_InitStructure);

//   /* ADC3 regular channel12 configuration *************************************/
//   ADC_RegularChannelConfig(ADC3, ADC_Channel_0, 1, ADC_SampleTime_144Cycles);
//   ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 2, ADC_SampleTime_144Cycles);
//   ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 3, ADC_SampleTime_144Cycles);

//  /* Enable DMA request after last transfer (Single-ADC mode) */
//  	ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);//gtz01nov 

// 	ADC_ContinuousModeCmd(ADC3, ENABLE);

//   /* Enable ADC3 DMA */
//   ADC_DMACmd(ADC3, ENABLE);

//   /* Enable ADC3 */
//   ADC_Cmd(ADC3, ENABLE);

//   /* Start ADC3 Software Conversion */ 
//   ADC_SoftwareStartConv(ADC3);
// }

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
