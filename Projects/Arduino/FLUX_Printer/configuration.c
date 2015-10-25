#include "configuration.h"
#include "stm32f0xx_usart.h"
//#include "uart.h"
#include "stm32f0xx.h"
#include "command.h"
#include "defines.h"
#include <stdio.h>
ModuleMode_Type ModuleMode=Unknow;
void Module_Initial(){	
	Module_Recognition();//set up module mode
	ID_Initial();
	
}

void ID_Initial(){
	__IO uint32_t ID,checksum;
	ID=*(__IO uint32_t *)FLASH_USER_END_ADDR;
	checksum=*(__IO uint32_t *)(FLASH_USER_END_ADDR+4);
	if((ID^1) != checksum){
		Write_ID((uint32_t)0x1234);
	}
}
void Module_Recognition(){
	uint16_t ADC_Value;
	ADC_Config();	//config all ADC
	ADC_Value=Read_ADC_Value(ID0_Channel);
	
	//read adc value to recognize module and set
	if(ADC_Value >= Heater_ADC_Min_Value){
		ModuleMode = FLUX_ONE_EXTRUDER_MODULE;
	}else if(ADC_Value >= Laser_ADC_Min_Value && ADC_Value < Heater_ADC_Min_Value){
		ModuleMode = FLUX_LASER_MODULE;
	}else{
		//no such module
		ModuleMode = Unknow;
	}
}

void ADC_Config(){
	ADC_InitTypeDef     ADC_InitStructure;
  GPIO_InitTypeDef    GPIO_InitStructure;
  
  /* GPIOA Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  
  /* ADC1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* ADCs DeInit */  
  ADC_DeInit(ADC1);
}



void UART1_Config()
{				
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

	/* Configure USART1 pins:  Tx ----------------------------*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART1 pins:  Rx ----------------------------*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	

	USART_InitStructure.USART_BaudRate = 115200;//Baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
  
	//NVIC_Configuration();
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	//USART_AutoBaudRateCmd(USART1,!DISABLE);
	USART_Cmd(USART1,ENABLE);
	
	NVIC_EnableIRQ(USART1_IRQn);	
}		

void Heater_Config(){
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef        TIM_OCInitStructure;
	/* GPIOC Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); 

	/* Configure PC10 and PC11 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	//µL´£¤É¹qªý
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
//	//Configure Timer for PWM
//	TIM_TimeBaseStructure.TIM_Prescaler = 0;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseStructure.TIM_Period = (SystemCoreClock / 1000 ) - 1; //Default frequency is 1KHz
//	//ArduinoPort[pin].pwmPeriod = (SystemCoreClock / 1000 ) - 1;
//	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

//	//configure TIM1 channel 6
//	TIM_OCInitStructure.TIM_Pulse = 0; //(uint16_t) (((uint32_t) 2 * (ArduinoPort[pin].pwmPeriod - 1)) / 10); //Default duty cycle at 0%
//	//TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
//	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
//	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 

//	//Connect TIM Channels to Port Alternate Function 
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_2);        
//	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);


//	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
//	//enable Frequ and pulse update
//	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
//	
//	TIM_SelectOnePulseMode(TIM1, TIM_OPMode_Repetitive); 
//	TIM_Cmd(TIM1, ENABLE);
//	TIM_CtrlPWMOutputs(TIM1, ENABLE);   
//	TIM1->CCR1 = 0;  
SetHeaterOff();
}

void Fan1_Config(){
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_OCInitTypeDef        TIM_OCInitStructure;
		GPIO_InitTypeDef GPIO_InitStructure; 
 
		/* GPIOB clock enable */ 
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); 
	 
		/*GPIOB Configuration: alternate function push-pull */ 
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	
		//Configure Timer for PWM
		TIM_TimeBaseStructure.TIM_Prescaler = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_Period = (SystemCoreClock / 1000 ) - 1; //Default frequency is 1KHz
		//ArduinoPort[pin].pwmPeriod = (SystemCoreClock / 1000 ) - 1;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

		//configure TIM1 channel 6
		TIM_OCInitStructure.TIM_Pulse = 0; //(uint16_t) (((uint32_t) 2 * (ArduinoPort[pin].pwmPeriod - 1)) / 10); //Default duty cycle at 0%
		//TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;


		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE); 

		//Connect TIM Channels to Port Alternate Function 
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_2);        
		TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);


		TIM_OC1Init(TIM16, &TIM_OCInitStructure);
		//enable Frequ and pulse update
		TIM_OC1PreloadConfig(TIM16, TIM_OCPreload_Enable);

		TIM_SelectOnePulseMode(TIM16, TIM_OPMode_Repetitive); 
		TIM_Cmd(TIM16, ENABLE);
		TIM_CtrlPWMOutputs(TIM16, ENABLE);  
		TIM16->CCR1 = 65025;
}

void Fan2_Config(){
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_OCInitTypeDef        TIM_OCInitStructure;
		GPIO_InitTypeDef GPIO_InitStructure; 
 
		/* GPIOB clock enable */ 
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); 
	 
		/*GPIOB Configuration: alternate function push-pull */ 
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	
		//Configure Timer for PWM
		TIM_TimeBaseStructure.TIM_Prescaler = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_Period = (SystemCoreClock / 1000 ) - 1; //Default frequency is 1KHz
		//ArduinoPort[pin].pwmPeriod = (SystemCoreClock / 1000 ) - 1;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

		//configure TIM1 channel 7
		TIM_OCInitStructure.TIM_Pulse = 0; //(uint16_t) (((uint32_t) 2 * (ArduinoPort[pin].pwmPeriod - 1)) / 10); //Default duty cycle at 0%
		//TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
		
		
	//Timer configuration
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);  
		//Connect TIM Channels to Port Alternate Function 
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_2);        
		TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);

		TIM_OC1Init(TIM17, &TIM_OCInitStructure);
		//enable Frequ and pulse update
		TIM_OC1PreloadConfig(TIM17, TIM_OCPreload_Enable);
		
		TIM_SelectOnePulseMode(TIM17, TIM_OPMode_Repetitive); 
		TIM_Cmd(TIM17, ENABLE);
		TIM_CtrlPWMOutputs(TIM17, ENABLE);  
		TIM17->CCR1 = 65025;
}

void Thermistor_ADC_Config()
{
  ADC_InitTypeDef     ADC_InitStructure;
  GPIO_InitTypeDef    GPIO_InitStructure;
  
  /* GPIOA Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  
  /* ADC1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* ADCs DeInit */  
  ADC_DeInit(ADC1);
  
  /* Initialize ADC structure */
  ADC_StructInit(&ADC_InitStructure);
  
  /* Configure the ADC1 in continuous mode with a resolution equal to 12 bits  */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
  ADC_Init(ADC1, &ADC_InitStructure); 
  
  /* Convert the ADC1 Channel 0 with 239.5 Cycles as sampling time */ 
	ADC_ChannelConfig(ADC1, ADC_Channel_0 , ADC_SampleTime_239_5Cycles);  

  /* ADC Calibration */
  ADC_GetCalibrationFactor(ADC1);
  
  /* Enable the ADC peripheral */
  ADC_Cmd(ADC1, ENABLE);     
  
  /* Wait the ADRDY flag */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY)); 
  
  /* ADC1 regular Software Start Conv */ 
  ADC_StartOfConversion(ADC1);
  
}

