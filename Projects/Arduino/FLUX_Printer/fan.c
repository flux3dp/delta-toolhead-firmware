#include "fan.h"
#include "stm32f0xx.h"
#include "utilities.h"
#include "command.h"
volatile uint16_t Fan1_Count=0;
volatile uint16_t Fan2_Count=0;
volatile uint8_t Inhalation_PWM=0;

void Set_Exhalation_Fan_PWM(uint8_t PWM){
	uint16_t Pwm_Value;
	Pwm_Value=(255 - PWM)*257;
	TIM16->CCR1 = Pwm_Value;
}

void Set_Inhalation_Fan_PWM(uint8_t PWM){
	uint16_t Pwm_Value;
	Inhalation_PWM=PWM;//save PWM
	if(PWM==0)
		PWM=0;
	else if(PWM<Fan_Lowest_PWM)
		PWM=Fan_Lowest_PWM;
	Pwm_Value=(255 - PWM)*257;
	TIM17->CCR1 = Pwm_Value;
}

uint8_t Read_Inhalation_Fan_PWM(void){
	Fan1_Count=0;
	Fan2_Count=0;
	delay_ms(10);
	if(Fan1_Count>=1 && Fan2_Count>=1){
		return Inhalation_PWM;
	}else{
		return 0;
	}
	
}

bool Is_Inhalation_Fan_Failed(void){
	if(Read_Inhalation_Fan_PWM()!=Inhalation_PWM)
		return TRUE;
	else{
		Reset_Module_State(FAN_FAILURE);
		return FALSE;
	}
}

