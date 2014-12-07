/**
  ******************************************************************************
  * @file    globalstruct.h
  * @author  Jim
  * @version V1.0
  * @date    01-Sep-2014
  * @brief   This is the global variable definition file.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 CSST Robot Research Center</center></h2>
  *
  ******************************************************************************
  */
 
#ifndef __GLOBAL_STRUCT_H__
#define	__GLOBAL_STRUCT_H__

/* Priority Definition */
#define CANOpen_THREAD_PRIO 		(configMAX_PRIORITIES-1) 		//CANopen数据处理任务定为最高优先级
//#define TIMER_THEAD_PRIO 			(configMAX_PRIORITIES-2) 		//定时器任务优先级
//#define BMS_CONTROL_THREAD_PRIO 	(configMAX_PRIORITIES-3) 		//BMS任务优先级
//#define CHASSIS_CONTROL_THREAD_PRIO (configMAX_PRIORITIES-4) 	//底盘任务优先级
//#define ARM_CONTROL_THREAD_PRIO 	(configMAX_PRIORITIES-5) 		//手臂任务优先级
//#define NECK_CONTROL_THREAD_PRIO 	(configMAX_PRIORITIES-6) 		//颈部任务优先级
#define LIFTER_CONTROL_THREAD_PRIO 	(configMAX_PRIORITIES-2) 		//升降台任务优先级
//#define POWERA_CONTROL_THREAD_PRIO 	(configMAX_PRIORITIES-8) 	//电源控制A(上身)  
//#define POWERB_CONTROL_THREAD_PRIO 	(configMAX_PRIORITIES-9) 	//电源控制B(下身)


/* Stack SIZE Definition */
#define CANOPEN_THREAD_STACK          500
#define TIMER_THEAD_STACK             500
#define BMS_CONTROL_THREAD_STACK      2000
#define CHASSIS_CONTROL_THREAD_STACK  2000
#define ARM_CONTROL_THREAD_STACK      2000
#define NECK_CONTROL_THREAD_STACK     2000
#define LIFTER_CONTROL_THREAD_STACK   1000
#define POWERA_CONTROL_THREAD_STACK   500
#define POWERB_CONTROL_THREAD_STACK   500

/*  Thread Delay Timer (if avilable) unit:ms */
#define CANOpen_THREAD_DELAY_TIMER           20
//#define TIMER_THEAD_DELAY_TIMER            20
//#define BMS_CONTROL_THREAD_DELAY_TIMER     20
//#define CHASSIS_CONTROL_THREAD_DELAY_TIMER 20
//#define ARM_CONTROL_THREAD_DELAY_TIMER     20
//#define NECK_CONTROL_THREAD_DELAY_TIMER    20
#define LIFTER_CONTROL_THREAD_DELAY_TIMER    20
//#define POWERA_CONTROL_THREAD_DELAY_TIMER  20
//#define POWERB_CONTROL_THREAD_DELAY_TIMER  20

/*  BSP_ID : NodeID used in CANOpen */
//#define BMS_ID     0x02
//#define CHASSIS_ID 0x03
//#define ARM_ID     0x04
//#define NECK_ID    0x05
#define LIFTER_ID    0x06
//#define POWERA_ID  0x07
//#define POWERB_ID  0x08


/** 
  * @brief  BQZ init structure definition
  */
typedef struct
{
	//u8 CNTL;			//Control()               		0x00/0x01 	N/A
	u8 SOC;				//StateOfCharge()      			0x02/0x03	%
	u8 RM;				//RemainingCapacity()  			0x04/0x05	mAh
	u8 FCC;				//FullChargeCapacity() 			0x06/0x07	mAh
	u8 VOLT;			//Voltage()     		   		0x08/0x09	mV
	u8 AI;				//AverageCurrent()     			0x0a/0x0b	mA
	u8 TEMP;			//Temperature()        			0x0c/0x0d	0.1°K
	u8 FLAGS;			//Flags()             			0x0e/0x0f	N/A
	u8 AR;  			//AtRate()             			0X10/0x11	mA
	u8 ARTTE;			//AtRateTimeToEmpty() 			0x12/0x13 	Minutes
	u8 NAC;				//NominalAvailableCapacity() 	0x14/0x15 	mAh
	u8 FAC;				//FullAvailableCapacity()   	0x16/0x17 	mAh
	u8 TTE;				//TimeToEmpty()  				0x18/0x19 	Minutes
	u8 TTF;				//TimeToFull()  				0x1a/0x1b 	Minutes
	u8 SI;				//StandbyCurrent()				0x1c/0x1d	mA
	u8 STTE;      		//StandbyTimeToEmpty()			0x1e/0x1f 	Minutes
	u8 MLI;				//MaxLoadCurrent()				0x20/0x21 	mA
	u8 MLTTE;			//MaxLoadTimeToEmpty()			0x22/0x23 	Minutes
	u8 AE;     			//AvailableEnergy()				0x24/0x25 	10 mWhr
	u8 AP;   		 	//AveragePower()				0x26/0x27 	10 mW
	u8 TTECP;			//TTEatConstantPower()  		0x28/0x29	Minutes
	u8 INTTEMP;			//Internal_Temp()				0x2a/0x2b 	0.1°K
	u8 CC;				//CycleCount()       			0x2c/0x2d 	Counts
	u8 SOH; 			//StateOfHealth()				0x2e/0x2f	%/num
	u8 CHGV;			//ChargeVoltage()				0x30/0x31 	mV
	u8 CHGI;   			//ChargeCurrent()  				0x32/0x33 	mA
	u8 PCHG;			//PassedCharge()				0x34/0x35	mAh
	u8 DOD0;			//DOD0()						0x36/0x37	HEX#
	u8 SDSG;			//SelfDischargeCurrent			0x38/0x39	mA
	u8 PKCFG;			//PackConfiguration()  			0x3a/0x3b	N/A
	u8 DCAP;			//DesignCapacity() 				0x3c/0x3d	mAh
}BQZ_typedef;

/** 
  * @brief  BMS init structure definition
  */
typedef struct
{
	/* Status -  0x01:normal 0x02:Sleep */
	u16 Status;                    	//STAS        	Status of ARM
	
	//u8 Control;     				//CNTL 			0x00/0x01 	N/A
	u16 StateOfCharge; 				//SOC			0x02/0x03	%
	u16 RemainingCapacity;			//RM			0x04/0x05	mAh
	u16 FullChargeCapacity;			//FCC			0x06/0x07	mAh
	u16 Voltage;					//VOLT			0x08/0x09	mV
	u16 AverageCurrent;				//AI			0x0a/0x0b	mA
	u16 Temperature;				//TEMP			0x0c/0x0d	0.1°K
	u16 Flags;						//FLAGS		    0x0e/0x0f	N/A
	u16 AtRate;						//AR  			0X10/0x11	mA
	u16 AtRateTimeToEmpty;			//ARTTE			0x12/0x13 	Minutes
	u16 NominalAvailableCapacity;	//NAC			0x14/0x15 	mAh
	u16 FullAvailableCapacity;		//FAC			0x16/0x17 	mAh
	u16 TimeToEmpty;				//TTE			0x18/0x19 	Minutes
	u16 TimeToFull;					//TTF			0x1a/0x1b 	Minutes
	u16 StandbyCurrent;				//SI			0x1c/0x1d	mA
	u16 StandbyTimeToEmpty;			//STTE     		0x1e/0x1f 	Minutes
	u16 MaxLoadCurrent;				//MLI			0x20/0x21 	mA
	u16 MaxLoadTimeToEmpty;			//MLTTE			0x22/0x23 	Minutes
	u16 AvailableEnergy;			//AE          	0x24/0x25 	10 mWhr
	u16 AveragePower;				//AP   		 	0x26/0x27 	10 mW
	u16 TTEatConstantPower;			//TTECP		 	0x28/0x29	Minutes
	u16 Internal_Temp;				//INTTEMP		0x2a/0x2b 	0.1°K
	u16 CycleCount;					//CC			0x2c/0x2d 	Counts
	u16 StateOfHealth;				//SOH 			0x2e/0x2f	%/num
	u16 ChargeVoltage;				//CHGV			0x30/0x31 	mV
	u16 ChargeCurrent;				//CHGI   		0x32/0x33 	mA
	u16 PassedCharge;				//PCHG			0x34/0x35 	mAh
	u16 DOD0;						//DOD0			0x36/0x37 	HEX#
	u16 SelfDischargeCurrent;		//SDSG			0x38/0x39 	mA
	u16 PackConfiguration;			//PKCFG			0x3a/0x3b 	N/A
	u16 DesignCapacity;				//DCAP			0x3c/0x3d 	mAh
} BMS_typedef;


/** 
  * @brief  Flag of Timer for each part of BMS
  */
typedef struct
{
	u32 LED;
	u32 BQZ;
	u32 ARM;
	u32 CHASSIS;
}TimerFlag_typedef;


/** 
  * @brief  CANx Struct Data[] For Charge-Det
  */
uint8_t CanSendData[8];
uint8_t CanSendDataChassis[8];
//uint8_t CanSendDataShun[8];

/** 
  * @brief  serial port transmission parameters
  */
/* Para */
#define uchar unsigned char
#define T_trsbytes 8
#define BaudRate 115200

/* Port & PIN */
#define IICPORT GPIOA
#define SDAIO   GPIO_Pin_4
#define SCLIO   GPIO_Pin_5

#define LABLEPORT  GPIOA
#define LABLEPORTB GPIOB

#define L4IO GPIO_Pin_4 
#define L5IO GPIO_Pin_5 
#define L6IO GPIO_Pin_6

#define L1IO GPIO_Pin_12 
#define L2IO GPIO_Pin_13 
#define L3IO GPIO_Pin_14

/* bq34z100 IIC Information */
#define ADDR_W 0XAA
#define ADDR_R 0XAB
#define t_dead 80000
#define delay_time 50

/** 
  * @brief  IIC PIN definition
  */
#define SDA_1 GPIO_SetBits(IICPORT,SDAIO)    //P1OUT |= BIT7	// SDA = 1 
#define SDA_0 GPIO_ResetBits(IICPORT,SDAIO)  //P1OUT &= (~BIT7)	// SDA = 0 
#define SCL_1 GPIO_SetBits(IICPORT,SCLIO)    //P1OUT |= BIT6	// SCL = 1 
#define SCL_0 GPIO_ResetBits(IICPORT,SCLIO)  // 				   SCL = 0

//(P1IN & BIT7) == BIT7
#define SDA_1_or_0  GPIO_ReadInputDataBit(IICPORT,SDAIO)

/*		     Name 		    Address*/
#define IIC_Address      0x40
#define GG_IIC_Address_W 0xAA
#define GG_IIC_Address_R 0xAB

#define SDA_OUT1 GPIOA->CRL &= 0xFFF0FFFF
#define SDA_OUT2 GPIOA->CRL |= 0X00030000
#define SDA_IN1  GPIOA->CRL &= 0xFFF0FFFF
#define SDA_IN2  GPIOA->CRL |= 0X00040000

#define ack_deadline 288


/** 
  * @brief  Battery status of Robot
  */
/* Port Number & PIN definition */
#define REDPORT  GPIOA
#define REDPIN   GPIO_Pin_8
#define BLUEPORT GPIOB
#define BLUEPIN  GPIO_Pin_15

#define CHARGE_DET_PIN  GPIO_Pin_13  //Detect charge or uncharge
#define CHARGE_DET_PORT GPIOB

#define STOP_LED_PORT GPIOA
#define STOP_LED_PIN GPIO_Pin_2
#define CHASSIS_STOP_LED_ON  GPIO_ResetBits(STOP_LED_PORT,STOP_LED_PIN);
#define CHASSIS_STOP_LED_OFF GPIO_SetBits(STOP_LED_PORT,STOP_LED_PIN);


#define CHARGE_LED_PIN  GPIO_Pin_1
#define CHARGE_LED_PORT GPIOA
#define CHARGE_LED_ON   GPIO_ResetBits(CHARGE_LED_PORT,CHARGE_LED_PIN)//charging	LED1 ON
#define CHARGE_LED_OFF  GPIO_SetBits(CHARGE_LED_PORT,CHARGE_LED_PIN)  //uncharging 	LED1 OFF

#define CHARGE_SWITCH_PIN  GPIO_Pin_14
#define CHARGE_SWTICH_PORT GPIOB
#define CHARGE_SWITCH_ON   GPIO_SetBits(CHARGE_SWTICH_PORT,CHARGE_SWITCH_PIN)   //charging	  PB14->HIGH
#define CHARGE_SWITCH_OFF  GPIO_ResetBits(CHARGE_SWTICH_PORT,CHARGE_SWITCH_PIN) //uncharging 	PB14->LOW




/** 
  * @brief DC-DC Control
  */
#define DCDC_PORT GPIOA
#define DCDC_PIN  GPIO_Pin_6

#define DCDC_ON   GPIO_ResetBits(DCDC_PORT,DCDC_PIN)  //charging	  PA6->LOW
#define DCDC_OFF  GPIO_SetBits(DCDC_PORT,DCDC_PIN)   //uncharging 	PA6->HIGH

/** 
  * @brief  MOS
  */
#define MOS_PORT  GPIOB
#define MOS_PIN   GPIO_Pin_12
#define POWER_ON  GPIO_SetBits(MOS_PORT,MOS_PIN)   //PB12
#define POWER_OFF GPIO_ResetBits(MOS_PORT,MOS_PIN)

#define DEAD_VOLTAGE 0x2710	//(hex)0x2710 = (dec)10000 mv

/*** (C) COPYRIGHT 2014 CSST R&D Intelligent Robotics Research Centre *END OF FILE*/



//extern xQueueHandle xQ_DRIVE_COMM, xQ_FLIP_COMM, xQ_ARM_COMM,xQ_HAND_COMM;
//extern const char* gNetCommandResStr[];
//extern char gNetBuffer[NET_BUFFER_MAX_NUMBER];
//extern int gNetDataSize;
#endif

/****************** (C) COPYRIGHT CSST Robot Research Center *****END OF FILE****/
