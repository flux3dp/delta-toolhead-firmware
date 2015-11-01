#include "fan.h"
#include "stm32f0xx.h"
void Set_Exhalation_Fan_PWM(uint8_t PWM){
	uint16_t Pwm_Value;
	Pwm_Value=(255 - PWM)*257;
	TIM16->CCR1 = Pwm_Value;
}

void Set_Inhalation_Fan_PWM(uint8_t PWM){
	uint16_t Pwm_Value;
	Pwm_Value=(255 - PWM)*257;
	TIM17->CCR1 = Pwm_Value;
}