#include "heater.h"
#include "stm32f0xx.h"
#include <stdio.h>
#include "command.h"
#include "TemperatureMapping.h"
#include "fan.h"
#include "utilities.h"

float Target_Temperature = 0.0;
static uint32_t Last_PID_Time=0;
static bool Auto_PID_Completed = FALSE;
static bool pid_reset = TRUE;

uint16_t RTC_ADC_Value=0;
//For PID_Control
static float Kp=Kp_Default;
static float Ki=Ki_Default;
static float Kd=Kd_Default;
static float pTerm=0.0;
static float iTerm=0.0;
static float dTerm=0.0;
static float temp_iState=0.0;
static float temp_iState_max=PID_MAX;
static float temp_iState_min=0.0;
static float temp_dState=0.0;

//For PID_Autotune
uint32_t Current_Time,ms;
uint32_t t1=0;
uint32_t t2=0;
float input = 0.0;
int cycles = 0,ncycles=10;
bool heating = TRUE;
long t_high = 0, t_low = 0;
long bias, dxd;
float Ku, Tu;
float Kp_temp, Ki_temp, Kd_temp;
float max = 0, min = 10000;
uint8_t PWM=0;
float Current_Temperature;

//For new PID method 
float lastInput=0;

//return heater temperature
float Read_Temperature(void){
	uint32_t ADC_Value=0;
	uint8_t i;
	
	for(i=0;i<ADC_Sample_Times;i++){
		ADC_Value += Read_ADC_Value(Temperature_Channel);
		//Avg_Value += Read_ADC_Value(Temperature_Channel);
	}
	
	ADC_Value = ADC_Value/ADC_Sample_Times;

	if(ADC_Value >= 0 && ADC_Value <= 4095)
		return ((float)Temperature_Mapping[ADC_Value])/10;
	else
		return 999.9;
}

void PID_Handler(void){
	float RT=Read_Temperature();
	Auto_PID_Completed=TRUE;
	
	if(millis()-Last_PID_Time < PID_Period_Time)
		return;
	Last_PID_Time=millis();
	if(RT < Exhalation_Fan_Close_Temp){
		Set_Exhalation_Fan_PWM(0);//close fan
	}else{
		Set_Exhalation_Fan_PWM(255);//close fan
	}
	
	if(RT>900.0 || RT<=0.0001){
		Set_Heater_PWM(0);
		Set_Module_State(HARDWARE_ERROR);
		return;
	}else if(Target_Temperature <= 0.001){
        Set_Heater_PWM(0);
        return;
    }else if( RT > Max_Temperature+10 || RTC_ADC_Value>583 ){
		Set_Heater_PWM(0);
		Set_Module_State(PID_OUT_OF_CONTROL);
		return;
	}
	if(Auto_PID_Completed){
		PID_Control();
	}else{
		//if set Target_Temperature
		//PID_Autotune();
		
	}
}

void PID_Control(void){
	Set_Heater_PWM(Get_Pid_Output());
}

void Set_Temperature(float setpoint){
	pid_reset=TRUE;
	dTerm=0;
	Target_Temperature=setpoint;
	if(setpoint>0.1)
		Set_Exhalation_Fan_PWM(255);
}

//return 0~255
uint8_t Get_Pid_Output(void) {
	float pid_output;
	float pid_error;
	float current_temperature = Read_Temperature();
	
	pid_error = Target_Temperature+Temperature_Offset - current_temperature;
	if (pid_error > PID_FUNCTIONAL_RANGE) {
		pid_output = PID_MAX;
		pid_reset = TRUE;
	}else if( current_temperature> (Max_Temperature+10)){
		pid_output = 0;
		pid_reset = TRUE;
		Set_Module_State(PID_OUT_OF_CONTROL);
	}else if (pid_error < -PID_FUNCTIONAL_RANGE || Target_Temperature <= 0.0 || current_temperature> Max_Temperature) {
		pid_output = 0;
		pid_reset = TRUE;
	}else {
		if (pid_reset) {
			temp_iState = 0.0;
			pid_reset = FALSE;
		}
		pTerm = Kp * pid_error;
		
		temp_iState += pid_error;
		temp_iState = constrain(temp_iState, temp_iState_min, temp_iState_max+1.0);
		iTerm = Ki * temp_iState;

		dTerm = K2 * Kd * (current_temperature - temp_dState) + K1 * dTerm;
		//printf("1=%.1lf , 2=%.1lf , 3=%.1lf\n",K2 * Kd * (current_temperature - temp_dState),K1 * dTerm,dTerm);
		pid_output = pTerm + iTerm - dTerm;
		
		if (pid_output > PID_MAX) {
			if (pid_error > 0.0) temp_iState -= pid_error; // conditional un-integration
			pid_output = PID_MAX;
		}
		else if (pid_output <= 0.0) {
			if (pid_error <= 0.0) temp_iState -= pid_error; // conditional un-integration
			pid_output = 0.0;
		}
	}
	temp_dState = current_temperature;
	Reset_Module_State(PID_OUT_OF_CONTROL);
	//printf("E:%.1lf T:%.1lf O:%.1lf P:%.1lf I:%.1lf D:%.1lf\n",pid_error-1.0,current_temperature,pid_output,pTerm,iTerm,dTerm);
	return (uint8_t)pid_output;
}

//return 0~65535
uint16_t Get_Pid_Output_Uint16(void) {
	float pid_output;
	float pid_error;
	float current_temperature = Read_Temperature();
	
	pid_error = Target_Temperature+Temperature_Offset - current_temperature;
	if (pid_error > PID_FUNCTIONAL_RANGE) {
		pid_output = PID_MAX;
		pid_reset = TRUE;
	}
	else if (pid_error < -PID_FUNCTIONAL_RANGE || Target_Temperature <= 0.0 || current_temperature> Max_Temperature) {
		pid_output = 0;
		pid_reset = TRUE;
	}
	else {
		if (pid_reset) {
			temp_iState = 0.0;
			pid_reset = FALSE;
		}
		pTerm = Kp * pid_error;
		
		temp_iState += pid_error;
		temp_iState = constrain(temp_iState, temp_iState_min, temp_iState_max+1.0);
		iTerm = Ki * temp_iState;

		dTerm = K2 * Kd * (current_temperature - temp_dState) + K1 * dTerm;
		//printf("1=%.1lf , 2=%.1lf , 3=%.1lf\n",K2 * Kd * (current_temperature - temp_dState),K1 * dTerm,dTerm);
		pid_output = pTerm + iTerm - dTerm;
		
		if (pid_output > PID_MAX) {
			if (pid_error > 0.0) temp_iState -= pid_error; // conditional un-integration
			pid_output = PID_MAX;
		}
		else if (pid_output <= 0.0) {
			if (pid_error <= 0.0) temp_iState -= pid_error; // conditional un-integration
			pid_output = 0.0;
		}
	}
	temp_dState = current_temperature;
	//printf("E:%.1lf T:%.1lf O:%.1lf P:%.1lf I:%.1lf D:%.1lf\n",pid_error-1.0,current_temperature,pid_output,pTerm,iTerm,dTerm);
	return (uint16_t)pid_output;
}

void PID_Autotune(void){
	//TODO{}
}

//set heater pwm 0~255
void Set_Heater_PWM(uint8_t PWM){
	TIM1->CCR1 =(255 - PWM)*257;  
}

//set heater pwm 0~65535
void Set_Heater_PWM_Uint16(uint16_t PWM){
	TIM1->CCR1 =65535-PWM;  
}

void Disable_All_Heater(void){
	Set_Heater_PWM(0);
}
