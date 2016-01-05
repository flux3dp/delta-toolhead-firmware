#include "fan.h"
#include "stm32f0xx.h"
#include "utilities.h"
#include "command.h"
volatile uint16_t Fan1_Count=0;
volatile uint16_t Fan2_Count=0;
volatile uint8_t Inhalation_PWM=0;
static volatile bool Fan_Maintain_Switch=Fan2_Off;
static volatile uint32_t Fan_Switch_Start_Time=0;
static volatile uint32_t Fan_Check_Last_Time=0;
extern uint32_t Fan_Start;
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
    if(PWM!=0)
        Fan_Start=millis();
}

uint8_t Read_Inhalation_Fan_PWM(void){
    if(millis()-Fan_Start<Fan_Revolution_Time_Limit)
        return Inhalation_PWM;
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

void Fan_Management(void){
    
    if(millis()-Fan_Start < Fan_Revolution_Time_Limit-50){
        if(millis()-Fan_Switch_Start_Time >80){
            Fan_Switch_Start_Time=millis();
            if(!Fan_Maintain_Switch){
                Fan_Maintain_Switch=TRUE;
                TIM17->CCR1=0;
            }else{
                Fan_Maintain_Switch=FALSE;
                TIM17->CCR1=65535;
            }
        }
    }else{
        Set_Inhalation_Fan_PWM(Inhalation_PWM);
        if(millis()-Fan_Check_Last_Time >820){
            Fan_Check_Last_Time=millis();
            if(Inhalation_PWM!=Read_Inhalation_Fan_PWM()){
                Fan_Start=millis();
            }
        }
    }
}

