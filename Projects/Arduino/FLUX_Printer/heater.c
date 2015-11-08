#include "heater.h"
#include "stm32f0xx.h"
#include <stdio.h>
#include "command.h"
#include "TemperatureMapping.h"
#include "fan.h"
float Target_Temperature = 0.0;

static bool Auto_PID_Completed = FALSE;
static bool pid_reset = TRUE;

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
uint32_t PID_Time_Count=0;
uint32_t Current_Time,ms;
uint32_t t1=0;
uint32_t t2=0;
float input = 0.0;
int cycles = 0,ncycles=10;
bool heating = TRUE;
//unsigned long temp_millis = PID_Time_Count, t1 = temp_millis, t2 = temp_millis;
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
	
	if(RT < Exhalation_Fan_Close_Temp){
		Set_Exhalation_Fan_PWM(0);//close fan
	}else{
		Set_Exhalation_Fan_PWM(255);//close fan
	}
	
	if(Target_Temperature == 0 || RT > Max_Temperature){
		Set_Heater_PWM(0);
		//fan
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
	if(PID_Time_Count>=PID_Period_Time)
	{
		Set_Heater_PWM(Get_Pid_Output());
		PID_Time_Count=0;
	}
	//PID_Compute();
	
}

uint8_t PID_Compute(void){
	float pid_output;
	uint32_t timeChange = PID_Time_Count;
	if(timeChange>=5)
	{
	  /*Compute all the working error variables*/
		float input = Read_Temperature();
		float error = Target_Temperature - input;
		float dInput;
		float output;
		iTerm+= (Ki * error);
		if(iTerm > PID_MAX) iTerm= PID_MAX;
		else if(iTerm < PID_MIN) iTerm= PID_MIN;
		dInput = (input - lastInput);

		/*Compute PID Output*/
		output = Kp * error + iTerm- Kd * dInput;

		if(output > PID_MAX) output = PID_MAX;
		else if(output < PID_MIN) output = PID_MIN;
		pid_output = output;

		/*Remember some variables for next time*/
		lastInput = input;

		PID_Time_Count=0;
		printf("T:%.1lf O:%.1lf P:%.1lf I:%.1lf D::%.1lf\n",input,pid_output,Kp * error,iTerm,Kd * dInput);
		Set_Heater_PWM(pid_output);
		return pid_output;
	}
	else return 0;
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
	
	if(Auto_PID_Completed){
		return;
	}else{
		PWM= bias = dxd = PID_MAX;
		t1=Current_Time;
		t2=Current_Time;
	}
	
	if(Target_Temperature <= 0.0)
		return;
	
	PID_Time_Count=0;
	
	Current_Temperature=Read_Temperature();

	max = max(max, Current_Temperature);
	min = min(min, Current_Temperature);
	
	Current_Time = PID_Time_Count;	//time milliseconds
	
	if (heating == TRUE && Current_Temperature > Target_Temperature) 
	{
		//printf("h1: %d\r\n",(pidc-t2));
		if (PID_Time_Count - t2 > 5000) 
		{
			heating = 0;				
						
			PWM = (bias - dxd) >> 1;	
			//printf("heating : 0   PWM:%d\r\n",PWM);	
			Set_Heater_PWM(PWM);	
			t1 = Current_Time;
			t_high = t1 - t2;
			max = Target_Temperature;
		}
	}//------------------------------------

	if (heating == FALSE && Current_Temperature < Target_Temperature) 
	{
		//printf("h0: %d\r\n",(pidc-t1));
		if (Current_Time - t1 > 5000) 
		{
			heating = TRUE;				 

			t2 = Current_Time;
			t_low = t2 - t1;
			//printf("t2: %d\r\n",t2);
			if (cycles > 0) 
			{
				long max_pow = PID_MAX;
				bias += (dxd*(t_high - t_low))/(t_low + t_high);
				bias = constrain(bias, 20, max_pow - 20);
				dxd = (bias > max_pow / 2) ? max_pow - 1 - bias : bias;

//				printf("BIAS :%lf\r\n",bias);						 
//				printf("DXD :%lf\r\n",dxd);           
//				printf("cmin :%d\r\n",min);            
//				printf("cmax :%d\r\n",max);

				if (cycles > 2) 
				{
					Ku = (4.0 * dxd) / (3.14159265 * (max - min) / 2.0);
					Tu = ((float)(t_low + t_high) / 1000.0);
//					  printf("Ku :%f\r\n",Ku);
//					  printf("Tu :%f\r\n",Tu);

					Kp_temp = 0.6 * Ku;
					Ki_temp = 2 * Kp_temp / Tu;
					Kd_temp = Kp_temp * Tu / 8;
						 
//					  printf("PID >>>>>>>>\r\n");
//					  printf("Kp :%f\r\n",Kp_temp);
//					  printf("Ki :%f\r\n",Ki_temp);
//					  printf("Kd :%f\r\n",Kd_temp);             
				}				 
			}

			PWM = (bias + dxd) >> 1;
			Set_Heater_PWM(PWM);
			cycles++;
			min = Target_Temperature;		
		}
	}//++++++++++++++++++++++++++++++++++++++++++++++++++

	if (Current_Temperature > Target_Temperature + 20) 
	{
		printf("PID_TEMP_TOO_HIGH\r\n");
		printf("PID >>>>>>>> Kp :%f,  Ki :%f,  Kd :%f\r\n",Kp_temp,Ki_temp,Kd_temp);
		Set_Heater_PWM(0);
		Target_Temperature = 0;
		return;
	}

	Current_Time = PID_Time_Count;
	// Every 2 seconds...
	if (PID_Time_Count > Current_Time + 2000) 
	{
		//p = soft_pwm[hotend];
		printf("T:%.1lf PWM:%d  cycles:%d  Current_Time:%d  t1:%d  t2:%d\r\n",Current_Temperature,PWM,cycles,Current_Time,t1,t2);

		Current_Time = PID_Time_Count;
	} // every 2 seconds

	// Over 2 minutes?
	if (((PID_Time_Count - t1) + (PID_Time_Count - t2)) > (10L*60L*1000L*2L)) 
	{
		printf("PID_TIMEOUT\r\n");
		printf("PID >>>>>>>> Kp :%f,  Ki :%f,  Kd :%f\r\n",Kp_temp,Ki_temp,Kd_temp);
		Set_Heater_PWM(0);
		Target_Temperature = 0;//­n­×§ï
		return;
	}

	if (cycles > ncycles) 
	{
		printf("PID_AUTOTUNE_FINISHED\r\n");
		#ifdef PIDTEMP
		PID_PARAM(Kp, hotend) = Kp_temp;
		PID_PARAM(Ki, hotend) = scalePID_i(Ki_temp);
		PID_PARAM(Kd, hotend) = scalePID_d(Kd_temp);
		#endif
		printf("PID >>>>>>>> Kp :%f,  Ki :%f,  Kd :%f\r\n",Kp_temp,Ki_temp,Kd_temp);
		Set_Heater_PWM(0);
		return;
	}

	//printf("pwm %d\r\n",Output);
	//pwmFrequency(HEATER,40000000);
	//analogWrite(HEATER,Output);
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
