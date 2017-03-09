#include "heater.h"
#include "stm32f0xx.h"
#include <stdio.h>
#include "command.h"
#include "TemperatureMapping.h"
#include "fan.h"
#include "utilities.h"
#include "defines.h"
float Target_Temperature = 0.0;
static uint32_t Last_PID_Time=0;
static bool Auto_PID_Completed = FALSE;
static bool pid_reset = TRUE;

static uint32_t T_Manage_Last_Time=0;
static uint32_t T_Manage_Mask_Time=0;
static float Last_Temp=0;

uint16_t NTC_ADC_Value=0;
uint32_t NTC_ADC_Value_Sampling=0;

float NTC_Centigrade=0;
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

extern ModuleMode_Type ModuleMode;

//return heater temperature
float Read_Temperature(void){
    return Read_Temperature_with_filter();
//	uint32_t ADC_Value=0;
//	uint8_t i;
//	
//	for(i=0;i<ADC_Sample_Times;i++){
//		ADC_Value += Read_ADC_Value(Temperature_Channel);
//		//Avg_Value += Read_ADC_Value(Temperature_Channel);
//	}
//	
//	ADC_Value = round((float)ADC_Value/ADC_Sample_Times);
//    
//	if(ADC_Value >= 0 && ADC_Value <= 4095)
//        switch(ModuleMode){
//			case FLUX_ONE_EXTRUDER_MODULE:
//                return ((float)Temperature_Mapping[ADC_Value])/10;
//            case FLUX_ONE_EXTRUDER_REV1_MODULE:
//            case FLUX_ONE_EXTRUDER_REV2_MODULE:
//                return ((float)Temperature_Mapping_Reverse(ADC_Value))/10;
//            default:
//                return 999.9;
//            
//        }
//	else
//		return 999.9;
}

//return heater temperature filter by 1 Standard Deviation interval
float Read_Temperature_with_filter(void){
    /* 0 to 4095 */
	uint32_t ADC_Value_Sum=0;
    double ADC_Avg=0;
	uint8_t i,count=0;
	uint16_t ADC_Val[ADC_Sample_Times],ADC_Value;
	for(i=0;i<ADC_Sample_Times;i++){
        uint16_t adc_temp = Read_ADC_Value(Temperature_Channel);
        ADC_Val[i] = adc_temp;
		ADC_Value_Sum += adc_temp;
	}
    
	ADC_Avg = (float)ADC_Value_Sum/ADC_Sample_Times;
    
    ADC_Value_Sum=0;
    for(i=0;i<50;i++){
        uint16_t adc_temp = Read_ADC_Value(Temperature_Channel);
        ADC_Avg += (float)adc_temp/ADC_Sample_Times;
        ADC_Avg -= (float)ADC_Val[i]/ADC_Sample_Times;
        if(adc_temp <= (ADC_Avg+4) || adc_temp >= (ADC_Avg-4)){
            ADC_Value_Sum += adc_temp;
            count++;
        }
    }
    
    if(count<=0)
        ADC_Value = round(ADC_Avg);
    else
        ADC_Value = round((float)ADC_Value_Sum/count);

	if(ADC_Value >= 0 && ADC_Value <= 4095)
        switch(ModuleMode){
			case FLUX_ONE_EXTRUDER_MODULE:
                return ((float)Temperature_Mapping[ADC_Value])/10;
            case FLUX_ONE_EXTRUDER_REV1_MODULE:
            case FLUX_ONE_EXTRUDER_REV2_MODULE:
                return ((float)Temperature_Mapping_Reverse(ADC_Value))/10;
            default:
                return 999.9;
            
        }
	else
		return 999.9;
}

void PID_Handler(void){
	float RT=1.0;
	Auto_PID_Completed=TRUE;

	if(millis()-Last_PID_Time < PID_Period_Time)
		return;
	Last_PID_Time=millis();
    
    RT=Read_Temperature();
	if(RT < Exhalation_Fan_Close_Temp){
		Set_Exhalation_Fan_PWM_Mask(0);//close fan
	}else{
		Set_Exhalation_Fan_PWM_Mask(255);//set fan
	}
	
	if(RT<0.001 || RT>900.0 || RT > Max_Temperature+10 || NTC_ADC_Value>583 || Get_Hardware_Error_Code(AUTO_HEAT)){//Thermal res is short or open,583=55C 454=45C 416=42C
		Set_Heater_PWM(0);
		Set_Module_State(HARDWARE_ERROR);
        if(RT<0.001)
            Set_Hardware_Error_Code(THERMAL_OPEN);
        else if(RT>900.0)
            Set_Hardware_Error_Code(THERMAL_SHORT);
        else if(RT > Max_Temperature+10)
            Set_Hardware_Error_Code(OVER_TEMPERATURE);
        else if(NTC_ADC_Value>583)
            Set_Hardware_Error_Code(NTC_OVER_TEMPERATURE);
	}else if(Target_Temperature <= 0.001){
        Set_Heater_PWM(0);
        Reset_Module_State(HARDWARE_ERROR);
        Reset_Hardware_Error_Code(THERMAL_OPEN);
        Reset_Hardware_Error_Code(THERMAL_SHORT);
        Reset_Hardware_Error_Code(OVER_TEMPERATURE);
        Reset_Hardware_Error_Code(NTC_OVER_TEMPERATURE);
        
    }else{
        if(Auto_PID_Completed){
            PID_Control();
            Reset_Module_State(HARDWARE_ERROR);
            Reset_Hardware_Error_Code(THERMAL_OPEN);
            Reset_Hardware_Error_Code(THERMAL_SHORT);
            Reset_Hardware_Error_Code(OVER_TEMPERATURE);
            Reset_Hardware_Error_Code(NTC_OVER_TEMPERATURE);
        }else{
            //if set Target_Temperature
            //PID_Autotune();
            
        }
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
    Last_Temp=0;
    T_Manage_Mask_Time=millis();
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
	}else if (pid_error < -PID_FUNCTIONAL_RANGE) { //Over temperature up to 10C
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
 
	//printf("E:%.1lf T:%.1lf O:%.1lf P:%.1lf I:%.1lf D:%.1lf\n",pid_error-1.0,current_temperature,pid_output,pTerm,iTerm,dTerm);
	return (uint8_t)pid_output;
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

void Temperature_Manage(void){
    float Current_Temp;
    float Temp_Error;
    uint32_t interval=0;
    
    if(millis()-T_Manage_Mask_Time<20000)
        return;
    interval=millis()-T_Manage_Last_Time;
    if(interval>4000){
        T_Manage_Last_Time=millis();
        if(Last_Temp<0.01){
            Last_Temp=Read_Temperature();
            return;
        }
        Current_Temp=Read_Temperature();
        Temp_Error=Target_Temperature-Current_Temp;
        //⊿砞放but放ど 2/4=0.5 /S
        if(Target_Temperature<0.01 && (Current_Temp-Last_Temp)>2){
            //Real temperature is rising but heater was closed.
            //printf("CurrT=%.2f\tLastT=%.2f\n",Current_Temp,Last_Temp);
            if(!Get_Module_State(HARDWARE_ERROR)){
                Set_Module_State(HARDWARE_ERROR);
                Set_Module_State(HEATER_FAILURE);
                Set_Hardware_Error_Code(AUTO_HEAT);
            }
        //Τ砞放but放ど硉 < 0.5/4=0.125 /S
        }else if(Target_Temperature>0.01 && Temp_Error>PID_FUNCTIONAL_RANGE && (Current_Temp-Last_Temp)<0.5){
            //Running PID but ERROR is not convergency.
            //printf("CurrT=%.2f\tLastT=%.2f\n",Current_Temp,Last_Temp);
            if(!Get_Module_State(HARDWARE_ERROR)){
                Set_Module_State(HARDWARE_ERROR);
                Set_Module_State(HEATER_FAILURE);
                Set_Hardware_Error_Code(CANNOT_HEAT);
            }
        }else{
            Reset_Module_State(HEATER_FAILURE);
            Reset_Hardware_Error_Code(AUTO_HEAT);
            Reset_Hardware_Error_Code(CANNOT_HEAT);
        }
        
        //printf("%.2f C/S\n",(Current_Temp-Last_Temp)/interval*1000);
        Last_Temp=Current_Temp;//Read_Temperature();//(Read_ADC_Value(NTC_Channel)-183.8)/12.87742+24.0;
    }
}

