/**
  ******************************************************************************
  * @file    PrinterHead/main.c 
  * @author  Flux Firmware Team
  * @version V1.0.0
  * @date    29-January-2016
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Flux</center></h2>
  *
  *
  ******************************************************************************
  */

#include "main.h"
#include "stm32f0xx_conf.h"

#ifdef Function_Test
	#include "LaserModule.h"
#endif
uint32_t Delay_Last_Time=0;
volatile uint32_t CmdTimeout_count=0;

extern volatile uint16_t Fan1_Count;
extern volatile uint16_t Fan2_Count;
extern volatile uint16_t Fan_Exh_Count;
static void RCC_HSI_Configuration(void);
static void Feed_WatchDog(void);

//extern Six_Axis_Sensor_State_Type Six_Axis_Sensor_State;
extern ModuleMode_Type ModuleMode;
extern volatile uint32_t Module_State;
extern volatile bool Debug_Mode;
extern volatile bool Show_Sensor_Data;

extern uint16_t NTC_ADC_Value;
extern float NTC_Centigrade;
extern void Detect_Laser_Power(void);
extern void Debounce_Laser_Power(void);
extern void (*Interlock_Exti_Func)(void);

void USART1_IRQHandler(void) 
{
	Usart1_ReadLine();	//Reading a byte until '\n' is received or timeout(10ms)
}

//General control timer.Default update period:20ms
void TIM6_DAC_IRQHandler(void){
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) == SET)
    {
		//TODO
		TIM_ClearITPendingBit(TIM6, /*TIM_IT_Update*/ TIM_FLAG_Update);
		
	}
}

void EXTI2_3_IRQHandler(void){
    if(EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
        (*Interlock_Exti_Func)();
		EXTI_ClearITPendingBit(EXTI_Line2);
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
    
    if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		Fan_Exh_Count++;
		/* Clear the EXTI line 14 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
}



int main(){
	volatile uint32_t LastTime=0;
	volatile uint32_t Time_Count=0;
	
	RCC_HSI_Configuration();//setting system clock as 48M Hz
	
	SysTick_Config(SystemCoreClock / 1000);//setting system tick as 1ms
	
	delay_ms(50);//waiting for usb connector keep in stable
	
	NVIC_Configuration();//setting interrupt priority

	Module_State_Initial();
	
	Module_Recognition();//read module type
	
	Alarm_IO_Config();
	
	Self_Test_IO_Config();
	
	Uart1_Config();

	#ifdef Function_Test
		Uart1_ISR_Enable();//testing
	#endif
	
    Using_Time_Initial();
    
	Six_Axis_Sensor_Initial();

	//Module seprate configuration
	switch(ModuleMode){
        case FLUX_ONE_EXTRUDER_REV1_MODULE:
            Fan_Exhalation_RPM_IO_Config();
		case FLUX_ONE_EXTRUDER_MODULE:
	
			Fan_Exhalation_Config();
	
			Fan_Inhalation_Config();
		
			Fan_Inhalation_RPM_IO_Config();
        
			delay_ms(50);//waiting for fan stable
		
			Heater_Config();

			#ifdef Function_Test
				Reset_Module_State(NO_HELLO);
				Debug_Mode=TRUE;
				Set_Inhalation_Fan_PWM(255);
				Set_Temperature(200);//testing
			#endif
		
			break;
		case FLUX_DUO_EXTRUDER_MODULE:
		
			break;
		case FLUX_LASER_MODULE:
			
			Laser_Switch_Config();
		
			#ifdef Function_Test
				Reset_Module_State(NO_HELLO);
				Debug_Mode=TRUE;
				Laser_Switch_On();
			#endif
			break;	
		case Unknow:
			//could not recognize module type
			Set_Module_State(UNKNOW_MODULE);
			break;
	}	

	//Six_Axis_Sensor_Calibration();//what time to calibrate?

	Uart1_ISR_Enable();//uart1 interrupt enable
	
	if(!Read_Self_Test_IO()){
		Self_Test();
	}
	
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
            case FLUX_ONE_EXTRUDER_REV1_MODULE:	
                printf("Extruder one REV1\n");
				break;
			case Unknow:
				//could not recognize module type
				printf("Unknow\n");
				break;
		}
	#endif
		
	//Gerneral_Timer_Config();	
		
	#if Enable_IWDG
		IWDG_Configuration();
	#endif

	while(1){

		Feed_WatchDog();//feed the watch dog or it will bite u after 800ms
	
		Xcode_Handler();
		
        switch(ModuleMode){
            case FLUX_ONE_EXTRUDER_MODULE:
            case FLUX_ONE_EXTRUDER_REV1_MODULE:
                Temperature_Manage();
                PID_Handler(); 
                Fan_Management();
                Using_Time_Extruder_One_Record();
                break;
            case FLUX_LASER_MODULE:
                Debounce_Laser_Power();
                Detect_Laser_Power();
                Using_Time_Laser_Record();
                break;
        }
        
		Time_Count=millis()-LastTime;
		if(Time_Count >= 60){
			LastTime=millis();
			if(Show_Sensor_Data)
				Show_Sensor_RawData();
			if(ModuleMode==FLUX_ONE_EXTRUDER_MODULE || ModuleMode==FLUX_ONE_EXTRUDER_REV1_MODULE){
				NTC_ADC_Value=Read_ADC_Value(NTC_Channel);	
				//printf("rtc value=%d\n",NTC_ADC_Value);
                //NTC_Centigrade=(NTC_ADC_Value-183.8)/12.87742+24.0;
			}
		}
			
		if(!(Module_State & SENSOR_FAILURE)){
            Detect_Gyro_Harm_Posture();
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


