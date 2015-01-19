



/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stdio.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
   
/* Enables retarget of printf to  serial port (USART1 on STM32407 board) for debug purpose */   
#define SERIAL_DEBUG_ON
#define ARM_CSST
//#define ARM_ORIGINAL

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */  
//void Time_Update(void);
//void Delay(uint32_t nCount);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
