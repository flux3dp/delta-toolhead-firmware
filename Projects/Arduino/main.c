/**
  ******************************************************************************
  * @file    PrinterHead_v2/main.c 
  * @author  Flux Firmware Team
  * @version V1.0.0
  * @date    20-October-2015
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 Flux</center></h2>
  *
  *
  ******************************************************************************
  */

#include "main.h"
#include "stm32f0xx_conf.h"

#if 1 //IWGD
__IO uint32_t LsiFreq = 40000;
#endif //IWGD

volatile uint32_t CmdTimeout_count=0;
volatile uint32_t Sensor_Init_Timeout_count=0;

extern Six_Axis_Sensor_State_Type Six_Axis_Sensor_State;
extern ModuleMode_Type ModuleMode;

void USART1_IRQHandler(void) 
{
	Usart1_ReadLine();	//Reading a byte until '\n' is received or timeout(10ms)
}


int main(){
	//waiting for usb connector keep in a stable
	delay_ms(50);

	SysTick_Config(SystemCoreClock / 1000);//setting system tick as 1ms

	Module_Initial();
	
	Uart1_Config();

	Six_Axis_Sensor_State = Six_Axis_Sensor_Initial();
	
	//Six_Axis_Sensor_Calibration();//what time to calibration?
	
	switch(ModuleMode){
		case FLUX_ONE_EXTRUDER_MODULE:
			
			Fan_Exhalation_Config();
	
			Fan_Inhalation_Config();
		
			Heater_Config();
		
			Set_Exhalation_Fan_PWM(255);//testing
		
			Set_Inhalation_Fan_PWM(255);//testing
		
			break;
		case FLUX_DUO_EXTRUDER_MODULE:
		
			break;
		case FLUX_LASER_MODULE:
			
			Laser_Switch_Config();
		
			break;
		case Unknow:
			//could not recognize module type
			break;
	}
	
	//watch dog config
	
	Uart1_ISR_Enable();
	
	printf("Initialed!\n");

	while(1){
		
		CommandTimeoutDetection();//command should be received within 1000ms?
		
		Xcode_Handler();
		
		PID_Handler(); //testing
		
//		Show_Sensor_RawData();
		
//		Detect_Gyro_Max();//testing
//		
//		Detect_Gyro_Tilt();
	}
}

