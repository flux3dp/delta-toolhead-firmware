#include "Extruder_One_Module.h"
#include <stdio.h>
#include <stdlib.h>
#include "gpio.h"
#include "defines.h"
#include <string.h>
#include "command.h"
#include "heater.h"
#include "fan.h"
#include "utilities.h"
#include "Six_Axis_Sensor.h"
#include "fan.h"
#include "Using_Time.h"
extern float Target_Temperature;
//extern Module_State_Type Module_State;
extern volatile uint32_t Module_State;
extern volatile uint16_t Fan1_Count;
extern volatile uint16_t Fan2_Count;
extern volatile bool Debug_Mode;
extern volatile bool Show_Sensor_Data;
extern Kalman_Data_Struct Kal_X,Kal_Y;
extern float Degree_Now;
extern uint16_t NTC_ADC_Value;
extern float Temperature_Estimate;

void Extruder_One_Cmd_Handler(void){
	char * Command_Str;
		
	//uint16_t Real_Temperatuer=0;
	uint16_t Int_Temp;
	float 	 Float_Temp;
	//uint16_t Fan_Pwm;
	char Response_Buffer[200];
	Command_Str = strtok(NULL, " ");
	
	if(!strcmp(Command_Str, "HELLO")){	
		Debug_Mode=FALSE;
		Show_Sensor_Data=FALSE;
		Reset_Module_State(NO_HELLO);
		sprintf(Response_Buffer,"1 OK HELLO TYPE:EXTRUDER ID:%08X%08X%08X VENDOR:%s FIRMWARE:%s VERSION:%s EXTRUDER:1 HARDWARE_VERSION:0 MAX_TEMPERATURE:%.1lf USED:%u ",UUID[2],UUID[1],UUID[0],Vender,Firmware_Name,Firmware_Version,Max_Temperature,Read_Using_Time());

	}else if(!strcmp(Command_Str, "DEBUG")){
		Debug_Mode=TRUE;
		Reset_Module_State(NO_HELLO);
		sprintf(Response_Buffer,"1 OK HELLO TYPE:EXTRUDER ID:%08X%08X%08X VENDOR:%s FIRMWARE:%s VERSION:%s EXTRUDER:1 HARDWARE_VERSION:0 MAX_TEMPERATURE:%.1lf USED:%u ",UUID[2],UUID[1],UUID[0],Vender,Firmware_Name,Firmware_Version,Max_Temperature,Read_Using_Time());

	}else if(!strcmp(Command_Str, "SHOW")){
		Show_Sensor_Msg();
		Show_Sensor_Data=TRUE;
		sprintf(Response_Buffer,"1 OK ");
	}else if(!strcmp(Command_Str, "SHOW1")){
		sprintf(Response_Buffer,"1 OK x%.2f y%.2f D%.2f ",Kal_X.angle,Kal_Y.angle,Degree_Now);
	}else if(!strcmp(Command_Str, "PING")){
		//uint8_t Err_Sum;
		//error check
		if(Is_Inhalation_Fan_Failed()){
            Set_Module_State(FAN_FAILURE);
        }else{
            Reset_Module_State(FAN_FAILURE);
        }
		
		//response
		sprintf(Response_Buffer,"1 OK PONG ER:%d RT:%.1lf ",Module_State,Temperature_Estimate);
		if(Target_Temperature<0.1)
			sprintf(Response_Buffer,"%s%s",Response_Buffer,"TT:NAN ");
		else
			sprintf(Response_Buffer,"%sTT:%.1lf ",Response_Buffer,Target_Temperature);
		
		if(Get_Module_State(HARDWARE_ERROR))
            sprintf(Response_Buffer,"%sFA:%d HE:%s ",Response_Buffer,Read_Inhalation_Fan_Mask_PWM(),Get_Hardware_Error_String());
		else
            sprintf(Response_Buffer,"%sFA:%d ",Response_Buffer,Read_Inhalation_Fan_Mask_PWM());
		//reset sensor state
		Reset_Axis_Sensor_State();
		//reset alarm IO
		Alarm_Off();
        //reset SELF_RESET
        Reset_Module_State(SELF_RESET);
	}else if(!strcmp(Command_Str, "H:0")){
		Command_Str = strtok(NULL, " ");
		if(Command_Str[0]=='T' && Command_Str[1]==':'){
			Float_Temp=atof(&Command_Str[2]);
			if(Module_State & NO_HELLO){
				sprintf(Response_Buffer,"1 ER COMMAND_CANNOT_BE_PROCESSSED ");
			}else if(Float_Temp>=0 && Float_Temp<=Max_Temperature){//沒做數字檢查
				
				Set_Temperature(Float_Temp);
				sprintf(Response_Buffer,"1 OK HEATER ");
			}else{
				sprintf(Response_Buffer,"1 ER PARAM_OUT_OF_RANGE ");
			}
				
		}else{
			sprintf(Response_Buffer,"1 ER UNKNOW_COMMAND ");
		}

	}else if(!strcmp(Command_Str, "F:0")){
		Command_Str = strtok(NULL, " ");
		if(Command_Str[0]=='S' && Command_Str[1]==':' && IsNumber(&Command_Str[2])){
				Int_Temp = atoi(&Command_Str[2]);
				if(Module_State & NO_HELLO){
					sprintf(Response_Buffer,"1 ER COMMAND_CANNOT_BE_PROCESSSED ");
				}else if(Int_Temp >= 0 && Int_Temp <= 255){
					Set_Inhalation_Fan_Mask_PWM(Int_Temp);
					sprintf(Response_Buffer,"1 OK FAN ");

                    
				}else{
					sprintf(Response_Buffer,"1 ER PARAM_OUT_OF_RANGE ");
				}
		}else{
			sprintf(Response_Buffer,"1 ER UNKNOW_COMMAND ");
		}
	}else if(!strcmp(Command_Str, "USING_TIME")){
		sprintf(Response_Buffer,"1 OK %u ",Read_Using_Time());
	}else if(!strcmp(Command_Str, "DETAIL")){
		sprintf(Response_Buffer,"1 OK DATE:%s ",Firmware_Date);
	}else if(!strcmp(Command_Str, "READ_ITS")){
		sprintf(Response_Buffer,"1 OK IT=%.2f NTC=%.2f ",Read_Internal_Temperature(),(NTC_ADC_Value-183.8)/12.87742+24.0);
	}else if(!strcmp(Command_Str, "READ_TADC")){
		sprintf(Response_Buffer,"1 OK ADC=%u ",TIM1->CCR1);
	}else if(!strcmp(Command_Str, "READ_TADC_VAL")){
		sprintf(Response_Buffer,"1 OK Temp_ADC=%u ",Read_ADC_Value(Temperature_Channel));
	}else if(!strcmp(Command_Str, "READ_TEMP_TIMES")){
        Int_Temp=0;
        Command_Str = strtok(NULL, " ");
		if(Command_Str[0]=='S' && Command_Str[1]==':' && IsNumber(&Command_Str[2]))
            Int_Temp = atoi(&Command_Str[2]);
        if(Int_Temp<=0)
            Int_Temp=1;
		sprintf(Response_Buffer,"1 OK Temp=%.1lf ",Read_Temperature_times(Int_Temp));
	}else if(!strcmp(Command_Str, "READ_TEMP_TIMES_F")){
        Int_Temp=0;
        Command_Str = strtok(NULL, " ");
		if(Command_Str[0]=='S' && Command_Str[1]==':' && IsNumber(&Command_Str[2]))
            Int_Temp = atoi(&Command_Str[2]);
        if(Int_Temp<=0)
            Int_Temp=1;
		sprintf(Response_Buffer,"1 OK Temp=%.1lf ",Read_Temperature_times_with_filter(Int_Temp));
	}else if(!strcmp(Command_Str, "READ_ID")){
        uint16_t ADC_Value_ID0,ADC_Value_ID1;
        ADC_Value_ID0=Read_ADC_Value(ID0_Channel);
        ADC_Value_ID1=Read_ADC_Value(ID1_Channel);
		sprintf(Response_Buffer,"1 OK ID0=%d ID1=%d ",ADC_Value_ID0,ADC_Value_ID1);
	}else if(!strcmp(Command_Str, "READ_KAL_TEMP")){
		sprintf(Response_Buffer,"1 OK RT=%.1lf KT=%.1lf ",Read_Temperature(),Temperature_Estimate);
	}else if(!strcmp(Command_Str, "SET_TREG")){
        Command_Str = strtok(NULL, " ");
		if(Command_Str[0]=='V' && Command_Str[1]==':'){
			Int_Temp=atoi(&Command_Str[2]);
			if(Module_State & NO_HELLO){
				sprintf(Response_Buffer,"1 ER COMMAND_CANNOT_BE_PROCESSSED ");
			}else if(Int_Temp>=0 && Int_Temp<=65535){//沒做數字檢查
				
				TIM1->CCR1=Int_Temp;
				sprintf(Response_Buffer,"1 OK ADC=%u ",TIM1->CCR1);
			}else{
				sprintf(Response_Buffer,"1 ER PARAM_OUT_OF_RANGE ");
			}
				
		}else{
			sprintf(Response_Buffer,"1 ER UNKNOW_COMMAND ");
		}
		
	}else if(!strcmp(Command_Str, "GET_TEMP_ADC")){ 
        sprintf(Response_Buffer,"1 OK TEMP_ADC=\t%u ",ADC_GetConversionValue(ADC1));
    }else{
		sprintf(Response_Buffer,"1 ER UNKNOW_COMMAND ");
	}
	
	//caculate checksum and send back
	sprintf(Response_Buffer,"%s*%d",Response_Buffer,Get_Checksum(Response_Buffer,strlen(Response_Buffer)));
	printf("%s\n",Response_Buffer);
}


