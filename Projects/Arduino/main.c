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





#define MaxTemp 2400

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

uint8_t Module = 1; //Defaule Module : Printer 1 : Laser 2 : NONE :0
uint8_t Mode = 2; //Default Mode


#if 1 //IWGD
__IO uint32_t LsiFreq = 40000;
#endif //IWGD


#if 0  //RTC

uint32_t AsynchPrediv = 0, SynchPrediv = 0;
uint32_t Secondfraction = 0;

RTC_TimeTypeDef  RTC_TimeStampStructure;
RTC_DateTypeDef  RTC_TimeStampDateStructure;

typedef struct {
  uint8_t tab[9];
} Table_TypeDef;
#endif //RTC



volatile uint32_t CmdTimeout_count;

void USART1_IRQHandler(void) 
{
	
	Usart1_ReadLine();//no timeout setting


}


int main(){

	
	
	SysTick_Config(SystemCoreClock / 1000);//setting system tick as 1ms

	Module_Initial();
	
	UART1_Config();
	
	Fan1_Config();
	
	Fan2_Config();
	
	Heater_Config();
	
	Six_Axis_Sensor_Initial();//must place before systic config
	
	printf("Initial!");
	
	
	
	
	//Thermistor_ADC_Initial();//heater temperature ADC convertion 
	
	//wait for usb connector stable

	
	while(1){
		
		CommandTimeoutDetection();//command should be received within 1000ms?
		
		Xcode_Handler();
		
		//Show_Sensor_RawData();
		
		//delay_ms(50);

	}
}//main

