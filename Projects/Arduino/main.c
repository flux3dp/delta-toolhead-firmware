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

#ifdef Function_Test
	#include "LaserModule.h" //testing
	extern float Target_Temperature;
#endif
volatile uint32_t CmdTimeout_count=0;
volatile uint32_t Sensor_Init_Timeout_count=0;

extern volatile uint16_t Fan1_Count;
extern volatile uint16_t Fan2_Count;

volatile uint32_t TIM1Freq = 0;

static void RCC_HSI_Configuration(void);
static void Feed_WatchDog(void);

//extern Six_Axis_Sensor_State_Type Six_Axis_Sensor_State;
extern ModuleMode_Type ModuleMode;
extern volatile uint32_t Module_State;
extern volatile bool Debug_Mode;

void USART1_IRQHandler(void) 
{
	Usart1_ReadLine();	//Reading a byte until '\n' is received or timeout(10ms)
}

//PID control timer.Default update period:10ms
void TIM6_DAC_IRQHandler(void){
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) == SET)
    {
		PID_Handler(); //testing
		TIM_ClearITPendingBit(TIM6, /*TIM_IT_Update*/ TIM_FLAG_Update);
	}
}

void EXTI4_15_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	{
		Fan1_Count++;
		/* Clear the EXTI line 12 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line12);
	}
	
	if(EXTI_GetITStatus(EXTI_Line14) != RESET)
	{
		Fan2_Count++;
		/* Clear the EXTI line 14 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line14);
	}
}


int main(){
	
	RCC_HSI_Configuration();//setting system clock as 48M Hz
	
	SysTick_Config(SystemCoreClock / 1000);//setting system tick as 1ms
	
	delay_ms(50);//waiting for usb connector keep in stable
	
	NVIC_Configuration();//setting interrupt priority

	Module_State_Initial();
	
	Module_Recognition();//read module type
	
	Alarm_IO_Config();
	
	Uart1_Config();

	#ifdef Function_Test
		Uart1_ISR_Enable();//testing
	#endif
	
	Six_Axis_Sensor_Initial();
	
	//Six_Axis_Sensor_Calibration();//what time to calibrate?
	
	//module configuration
	switch(ModuleMode){
		case FLUX_ONE_EXTRUDER_MODULE:
			
			Fan_Exhalation_Config();
	
			Fan_Inhalation_Config();
		
			Fan_Inhalation_RPM_IO_Config();
		
			delay_ms(50);//waiting for fan stable
		
			Heater_Config();
		
			PID_Timer_Config();	
		
			#ifdef Function_Test
				Debug_Mode=TRUE;
				//Set_Exhalation_Fan_PWM(255);
			
				Set_Inhalation_Fan_PWM(255);
			#endif
		
			break;
		case FLUX_DUO_EXTRUDER_MODULE:
		
			break;
		case FLUX_LASER_MODULE:
			
			Laser_Switch_Config();
		
			#ifdef Function_Test
				Debug_Mode=TRUE;
				//Laser_Switch_On();
			#endif
			break;
			
		case Unknow:
			//could not recognize module type
			break;
	}
	
	#if Enable_IWDG
		IWDG_Configuration();
	#endif
	
	Uart1_ISR_Enable();//uart1 interrupt enable
	
	#if Enable_Debug_Msg
		if(!(Module_State & SENSOR_FAILURE))
			printf("Initialed");
		else
			printf("Sensor initial failed");
		
		printf(",Mode=");
		
		switch(ModuleMode){
			case FLUX_ONE_EXTRUDER_MODULE:	
				printf("Extruder one\n");
				break;
			case FLUX_DUO_EXTRUDER_MODULE:
				printf("Extruder duo\n");
				break;
			case FLUX_LASER_MODULE:
				printf("Laser\n");
				break;
			case Unknow:
				//could not recognize module type
				printf("unknow\n");
				break;
		}
	#endif
		
	#ifdef Function_Test
		Set_Temperature(200);//testing
	#endif
		
	
		
	while(1){
		Feed_WatchDog();//feed the watch dog or it will bite u after 800ms
	
		//CommandTimeoutDetection();//command should be received within 1000ms?
		
		Xcode_Handler();

//		Show_Sensor_RawData();
		
		if(!(Module_State & SENSOR_FAILURE)){
			Detect_Gyro_Shake();
			Detect_Gyro_Tilt();
		}
	}
}

static void RCC_HSI_Configuration(void)
{
	RCC_DeInit(); 

	RCC_HSICmd(ENABLE);

	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY)== RESET);

	RCC_HCLKConfig(RCC_SYSCLK_Div1);

	RCC_PCLKConfig(RCC_HCLK_Div1);

	FLASH_SetLatency(FLASH_Latency_1);//if(24 < HCLK <= 48 MHz) flash latency=1 else latency=0

	FLASH_PrefetchBufferCmd(ENABLE);

	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12);//set HCLK up to 48MHz

	RCC_PLLCmd(ENABLE);
	
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
	
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	
	while(RCC_GetSYSCLKSource() != 0x08);

}

static void Feed_WatchDog(void){
#if Enable_IWDG
	if(RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET)
	{
		//printf("The Stm32 has been reset by IWDG.\r\n");
		RCC_ClearFlag();
	}
	IWDG_ReloadCounter();//feed the watch dog or it will bite u after 800ms
#endif
}


