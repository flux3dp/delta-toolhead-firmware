#include "configuration.h"
#include "stm32f0xx_usart.h"
//#include "uart.h"
#include "stm32f0xx.h"
#include "command.h"
#include "defines.h"
#include <stdio.h>
#include "utilities.h"
#include "Six_Axis_Sensor.h"

ModuleMode_Type ModuleMode=Unknow;

extern volatile uint32_t Module_State;

static void Laser_Self_Test_IO_Config(void);

static void Extruder_One_Self_Test_IO_Config(void);

void Module_Initial(void){	
	
	Module_Recognition();//set up module mode.
	
}

void Module_Recognition(void){
	uint16_t ADC_Value;
	ADC_Config();	//config all ADC
	ADC_Value=Read_ADC_Value(ID0_Channel);
	
	//read adc value to recognize module
	if(ADC_Value >= Heater_ADC_Min_Value){
		ModuleMode = FLUX_ONE_EXTRUDER_MODULE;
	}else if(ADC_Value >= Laser_ADC_Min_Value && ADC_Value < Heater_ADC_Min_Value){
		ModuleMode = FLUX_LASER_MODULE;
	}else{
		//no such module
		ModuleMode = Unknow;
		Set_Module_State(UNKNOW_MODULE);
		//Module_State|=UNKNOW_MODULE;
	}
}

void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	
	/* set up the SysTick interrupt priority */
	NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* set up the USART1 interrupt priority */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* set up the TIM6 global interrupt priority */
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Enable and set EXTI4_15 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

//ID0 & ID1 adc configuration
void ADC_Config(void){
	//ADC_InitTypeDef     ADC_InitStructure;
	GPIO_InitTypeDef    GPIO_InitStructure;

	/* GPIOA Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* ADC1 Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ADCs DeInit */  
	ADC_DeInit(ADC1);
}

void Uart1_Config(void)
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
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART1 pins:  Rx ----------------------------*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
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
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	
	//USART_AutoBaudRateCmd(USART1,!DISABLE);
	USART_Cmd(USART1,DISABLE);
	
	NVIC_EnableIRQ(USART1_IRQn);	
}		

void Uart1_ISR_Enable(void){

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART1,ENABLE);
}

void Uart1_ISR_Disable(void){

	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

	USART_Cmd(USART1,DISABLE);
}

//PID control timer.Default update period:20ms
void Gerneral_Timer_Config(void){
	uint16_t PrescalerValue;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); 
	
	PrescalerValue = (uint16_t) (SystemCoreClock  / 1000) - 1;
	//Configure timer clock
	TIM_TimeBaseStructure.TIM_Period = 20;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	/* Prescaler configuration */
	TIM_PrescalerConfig(TIM6, PrescalerValue, TIM_PSCReloadMode_Immediate);
	
	/* TIM enable counter */
	TIM_Cmd(TIM6, ENABLE);

	/* TIM IT enable */
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
}


void Heater_Config(void){
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef        TIM_OCInitStructure;
	/* GPIOB Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); 

	/* GPIOA Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 
	
	//Configure thermal reading GPIO PA0 & NTC PA1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* Configure PB13 in alternate function pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//Configure timer clock
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = (SystemCoreClock / 1000 ) - 1; //Default frequency is 1KHz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	//configure TIM1 
	TIM_OCInitStructure.TIM_Pulse = 0; //Default duty cycle at 0%
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	

	//Connect TIM Channels to Port Alternate Function 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_2);        
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);


	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	//enable Frequ and pulse update
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_SelectOnePulseMode(TIM1, TIM_OPMode_Repetitive); 
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);   
	
	//set pwm=0
	TIM1->CCR1 = 65535;  
}

void Fan_Exhalation_Config(void){
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
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	//configure TIM1 channel 6
	TIM_OCInitStructure.TIM_Pulse = 0; //Default duty cycle at 0%
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
	TIM16->CCR1 = 65535;
}

void Fan_Inhalation_Config(void){
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
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	//configure TIM1 channel 7
	TIM_OCInitStructure.TIM_Pulse = 0;  //Default duty cycle at 0%
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
	TIM17->CCR1 = 65535;
}

void Fan_Inhalation_RPM_IO_Config(void){
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef   EXTI_InitStructure;
	/* GPIOB clock enable */ 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); 
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	//Config fan rpm reading GPIO PB12 & PB14
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* Connect EXTI12 Line to PB12 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource12);

	/* Connect EXTI14 Line to PB14 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource14);
	
	/* Configure EXTI12 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;  
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Configure EXTI14 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line14;
	EXTI_Init(&EXTI_InitStructure);

}

void Thermistor_ADC_Config(void)
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

void Laser_Switch_Config(void){
	GPIO_InitTypeDef GPIO_InitStructure; 
	/* GPIOA Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); 
	
	/* Configure PA0 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	//無提升電阻
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* Configure PB9 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	//無提升電阻
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
    //dectect laser power
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA,GPIO_Pin_0);
	GPIO_SetBits(GPIOB,GPIO_Pin_9);
}

void IWDG_Configuration(void)
{
 	RCC_LSICmd(ENABLE);                              //LSI
    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY)==RESET);

	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_32);//LSI=40kHz
	IWDG_SetReload(1000);	  //1/40k*1000=800ms ,max 0xFFF  0~4095  
	IWDG_ReloadCounter();
	IWDG_Enable();
}

void Alarm_IO_Config(void){
	GPIO_InitTypeDef    GPIO_InitStructure;
	
	/* GPIOB Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); 

	/* Configure PB0 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	//提升電阻
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	Alarm_Off();
}

void Self_Test_IO_Config(void){
	switch(ModuleMode){
			case FLUX_ONE_EXTRUDER_MODULE:	
				Extruder_One_Self_Test_IO_Config();
				break;
			case FLUX_DUO_EXTRUDER_MODULE:
				
				break;
			case FLUX_LASER_MODULE:
				Laser_Self_Test_IO_Config();
				break;
			case Unknow:
				//could not recognize module type
				
				break;
	}
}

static void Laser_Self_Test_IO_Config(void){
	GPIO_InitTypeDef    GPIO_InitStructure;
	/* GPIOA Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
}

static void Extruder_One_Self_Test_IO_Config(void){
	GPIO_InitTypeDef    GPIO_InitStructure;
	/* GPIOA Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Interlock_Exti_Config(void){
	EXTI_InitTypeDef   EXTI_InitStructure;
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource2);
	
	/* Configure EXTI12 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;  
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

void Interlock_Exti_Break(void){
    EXTI_InitTypeDef   EXTI_InitStructure;

	/* Configure EXTI12 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;  
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;
	EXTI_Init(&EXTI_InitStructure);
}


