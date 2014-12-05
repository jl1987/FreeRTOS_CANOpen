/*
This file is edited as a part of CanFestival.

STM32F407 Port: Jim

*/

#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "stm32f4xx.h"


#define WD_SLEEP

#define	FALSE					0
#define	TRUE					1

// Needed defines by Atmel lib
//#define AT91C_MASTER_CLOCK      168000000UL    //系统时钟为168M


/* 设置波特率*/
#define CAN_BAUDRATE   1000
#define CAN_BAUD_1M    1000
#define CAN_BAUD_500K  500
#define CAN_BAUD_250K  250
#define CAN_BAUD_125K  125
#define CAN_BAUD_DEFAULT  CAN_BAUD_1M

/***********CAN1-开发板*/
  //#define CAN1                        0
  #define CAN1_CLK                    RCC_APB1Periph_CAN1
  #define CAN1_RX_PIN                 GPIO_Pin_8
  #define CAN1_TX_PIN                 GPIO_Pin_9
  #define CAN1_GPIO_PORT              GPIOB
  #define CAN1_GPIO_CLK               RCC_AHB1Periph_GPIOB
  #define CAN1_AF_PORT                GPIO_AF_CAN1
  #define CAN1_RX_SOURCE              GPIO_PinSource8
  #define CAN1_TX_SOURCE              GPIO_PinSource9       

/***********CAN2-CSST_ARM*/
  //#define CAN2                       1
  #define CAN2_CLK                    RCC_APB1Periph_CAN2
  #define CAN2_RX_PIN                 GPIO_Pin_12        //RX
  #define CAN2_TX_PIN                 GPIO_Pin_13				//TX
  #define CAN2_GPIO_PORT              GPIOB
  #define CAN2_GPIO_CLK               RCC_AHB1Periph_GPIOB
  #define CAN2_AF_PORT                GPIO_AF_CAN2
  #define CAN2_RX_SOURCE              GPIO_PinSource12
  #define CAN2_TX_SOURCE              GPIO_PinSource13


// /*_CAN2*/  -This is the original define
//   //#define CAN2                       1
//   #define CAN2_CLK                    RCC_APB1Periph_CAN2
//   #define CAN2_RX_PIN                 GPIO_Pin_5                //TX
//   #define CAN2_TX_PIN                 GPIO_Pin_6				//RX
//   #define CAN2_GPIO_PORT              GPIOB
//   #define CAN2_GPIO_CLK               RCC_AHB1Periph_GPIOB
//   #define CAN2_AF_PORT                GPIO_AF_CAN2
//   #define CAN2_RX_SOURCE              GPIO_PinSource5
//   #define CAN2_TX_SOURCE              GPIO_PinSource6    


// Needed defines by Canfestival lib
#define MAX_CAN_BUS_ID                  1     
#define SDO_MAX_LENGTH_TRANSFER         32
#define SDO_MAX_SIMULTANEOUS_TRANSFERS  1
#define NMT_MAX_NODE_ID                 128
#define SDO_TIMEOUT_MS                  3000U
#define MAX_NB_TIMER                    8
#define SDO_BLOCK_SIZE                  16

// CANOPEN_BIG_ENDIAN is not defined           //Cortex-M4系列应该没有存储空间的制约
#define CANOPEN_LITTLE_ENDIAN 1	               //整个Cortex-M3系列为小端存储

#define US_TO_TIMEVAL_FACTOR 8

#define REPEAT_SDO_MAX_SIMULTANEOUS_TRANSFERS_TIMES(repeat)\
repeat
#define REPEAT_NMT_MAX_NODE_ID_TIMES(repeat)\
repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat

#define EMCY_MAX_ERRORS 8
#define REPEAT_EMCY_MAX_ERRORS_TIMES(repeat)\
repeat repeat repeat repeat repeat repeat repeat repeat


#endif /* _CONFIG_H_ */

