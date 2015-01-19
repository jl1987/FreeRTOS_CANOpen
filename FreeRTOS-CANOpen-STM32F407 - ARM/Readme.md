#STM32F407 FreeRTOS CANOpen Program#

This is a demo program provide a develop platform for Robot Control under STM32F407 MCU.  

**System and software i'm using now:**
<pre>
Desktop OS: Windows 8.1 Professional  
IDE:        Keil uVision V4.54.0.0
Toolchain:  MDK-ARM Standard Version-4.54.0.0
------------------------------------------------
CANOpen:        V3.0
FreeRTOS:       V8.0.1
STM Lib Driver: FwLib V1.0.2
</pre> 

**File Organization:**  
<pre>
USER:     main.c, interrupt and debug function;  
BSP:      Borad surport packet file like bsp_led;
Robot_Control: Robot control files, one main feature one file, e.g canopen_thread.c and lifter_control.c;
CMSIS:    system_stm32f4xx.c, chip configuration especially system clock;  
MDK-ARM:  startup_stm32f4xx.s NVIC vector table，memeory allocation;    
FwLib:    STM32F407 firmware files;   
FreeRTOS: FreeRTOS system files;   
CANOpen:  CAN Festival files include drivers for STM32;  
Objdict:  CANOpen Object Dictionary;  
Doc:      Readme and documentation;
</pre>

**Configuration:**  
In order to easily test program in different board, in file `main.h`, there is a few configurations to make.
<pre>
#define SERIAL_DEBUG_ON  //when defined you can use printf() to serial output 
#define ARM_CSST         //ARM_CSST means the ARM board we use on the Robot
//#define ARM_ORIGINAL   //ARM_ORIGINAL means the development board(the ST development board with joystick)
</pre>

<table>
<tbody>
<tr><td><em>HHHH</em></td><td><em>JJJJ</em></td><td><em>KKKK</em></td></tr>
<tr><td>hhhh</td><td>jjjj</td><td>kkkk</td></tr>
<tr><td>hhhh</td><td>jjjj</td><td>kkkk</td></tr>
<tr><td>hhhh</td><td>jjjj</td><td>kkkk</td></tr>
</tbody>
</table>

##Start##

The project startup file for Keil is in .../PROJ/Project.uvproj. Double click to start this program. Eclipse ARM GNU support will be added later.

###CANOpen###
The CANOpen communication function is developed based on canfestival(version 3), and all support files are contained in folder `CANOpen`.

When power on, the CANOpen communication function run automaticly, if everthing is right, the CANOpen state machine will run into `Initialisation` -> `Pre_operational`. Then CANOpen will wait for the Master(ARM Linux) message to go into `Operational` state.  

All function about CAN is in task `canopen_thread.c`, there are several important function as following:

1.CANOpen Thread Initialization:

    /**
	 * CANOpen Initialization Program
	 * @brief create CANOpen data process thread
	 */
	void canopen_init(void)

2.CANOpen Data Processing:

	/**
	  * CANOpen Data Process Thread Program
	  * @brief CANOpen Data Scan, if CANbus got a CAN message, then push the message into
	  *        queue(xQ_CAN_MSG), this thread scan the queue at CANOpen_THREAD_SCAN_TIMER(
	  *        default:20ms) rate.
	  */
	static void canopen_dataprocess_thread(void * pvParameters)

When ARM get a CAN message: First of all, the system run into IRQ, in the IRQ the CAN message will be pushed into a queue and transfered to the canopen thread, then we use function:

	canDispatch(CO_Data* d, Message *m)

to dispatch the message with Object Dictionary.

All you have to do is include the right object dictionary `.h` file at the top of the file `canopen_thread.c` and `stm32f4xx_it.c`. In this demo program, we use `LIFTER_OD`

Notice: You can find explanaton about the other function in the comments. There are a lot of detailed descriptions.

###FreeRTOS###

To create a task, use function( LED task as a example ):

	/** xTaskCreate(pvTaskCode,   指向任务的实现函数的指针(效果上仅仅是函数名)
	  *             pcName,       具有描述性的任务名。这个参数不会被FreeRTOS使用
	  *             usStackDepth, 栈空间大小，单位是字
	  *             pvParameters, 任务函数接受一个指向void的指针(void*)
	  *             uxPriority,   指定任务执行的优先级，0到最高优先级(configMAX_PRIORITIES–1)
	  *             pxCreatedTask 用于传出任务的句柄，不用则NULL代替。)
	  */
    xTaskCreate(ToggleLed4, "LED4", configMINIMAL_STACK_SIZE, NULL, LED_TASK_PRIO, NULL);

In this platform, every task(thread) is in a particular file, as lifter for example:

The task file contains in folder `Robot_Control` with a `.c` file `lifter-control.c` and two function, one is for 
Task Create and the other is the exact processing function:

	void start_lifter_control(void)  //task creat program

	void lifter_control_thread(void * pvParameters)

Other robot control block can be programed by reference to the lifter control program.

Notice: The FreeRTOS system can be configured by define different values in file `freertosconfig.h`, every configuration have a comment I made to make it clearly understandable, you can check it by yourself.

Notice: Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names:

	#define vPortSVCHandler     SVC_Handler
	#define xPortPendSVHandler  PendSV_Handler
	#define xPortSysTickHandler SysTick_Handler

##Debug##

1. Generally, you can debug your program using MDK build-in debug function and a CAN node (able to send CAN message, e.g USB-CAN) linked to your board. Use the printf() function if SERIAL_DEBUG_ON is defined in `main.h`.  

2. If you have one board only, it is feasible to debug according to the following stets:  

	- Config the `canInit(CAN_TypeDef* CANx,unsigned int bitrate)`function in file`can_STM32.c`,   
		<pre>
       	Change:
              //CAN_InitStructure.CAN_Mode=CAN_Mode_Normal; 
       	To  
              CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;`

		</pre>
		if dos so, the CAN message send from the borad with CAN port, CAN2 for example, will be recevied with CAN2 too and causing a CAN recive interrupt.
    - Plug a serial-usb between your board(USART1) and PC, you can send message with sscom32.exe software to the board, and DIY the meaning of that message in `USART1_IRQHandler()` in file `stm32fxx_it.c`, e.g:  
    	<pre>
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
			  USART2CAN.RTR = 0x00;
			  USART2CAN.IDE = CAN_ID_STD;
			  USART2CAN.DLC = 1;                 
			  USART2CAN.Data[0] = 0xFF;
			  CAN_Transmit(CAN2, &USART2CAN);	
			}
		
			while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
			{
		  	  //USART_ClearFlag(USART1,USART_FLAG_RXNE);
			}
		  }	
		}
		</pre>