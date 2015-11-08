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
extern float Target_Temperature;
//extern Module_State_Type Module_State;
extern volatile uint32_t Module_State;
extern volatile uint16_t Fan1_Count;
extern volatile uint16_t Fan2_Count;
extern volatile bool Debug_Mode;

void Extruder_One_Cmd_Handler(void){
	char * Command_Str;
		
	//uint16_t Real_Temperatuer=0;
	uint16_t Int_Temp;
	float 	 Float_Temp;
	//uint16_t Fan_Pwm;
	char Response_Buffer[200];
	Command_Str = strtok(NULL, " ");
	
	if(!strcmp(Command_Str, "HELLO")){	
		//Module_State.No_Hello=FALSE;
		//Module_State &= !NO_HELLO;
		Debug_Mode=FALSE;
		Reset_Module_State(NO_HELLO);
		sprintf(Response_Buffer,"1 OK HELLO TYPE:EXTRUDER ID:%u VENDOR:%s FIRMWARE:OHMAMA VERSION:%.4lf EXTRUDER:1 MAX_TEMPERATURE:%.1lf ",Get_UUID(),Vender,Firmware_Version,Max_Temperature);

	}else if(!strcmp(Command_Str, "DEBUG")){
		Reset_Module_State(NO_HELLO);
		sprintf(Response_Buffer,"1 OK HELLO TYPE:EXTRUDER ID:%u VENDOR:%s FIRMWARE:OHMAMA VERSION:%.4lf EXTRUDER:1 MAX_TEMPERATURE:%.1lf ",Get_UUID(),Vender,Firmware_Version,Max_Temperature);
		
	}else if(!strcmp(Command_Str, "PING")){
		//uint8_t Err_Sum;
		//error check
		if(Is_Inhalation_Fan_Failed())
			Set_Module_State(FAN_FAILURE);
		
		//response
		sprintf(Response_Buffer,"1 OK PONG ER:%d RT:%.1lf ",Module_State,Read_Temperature());
		if(Target_Temperature<0.1)
			sprintf(Response_Buffer,"%s%s",Response_Buffer,"TT:NAN ");
		else
			sprintf(Response_Buffer,"%sTT:%.1lf ",Response_Buffer,Target_Temperature);
		sprintf(Response_Buffer,"%sFA:%d ",Response_Buffer,Read_Inhalation_Fan_PWM());
		
		//reset sensor state
		Reset_Axis_Sensor_State();
		//reset alarm IO
		Alarm_Off();
	}else if(!strcmp(Command_Str, "H:0")){
		Command_Str = strtok(NULL, " ");
		if(Command_Str[0]=='T' && Command_Str[1]==':'){
			Float_Temp=atof(&Command_Str[2]);
			if(Module_State & NO_HELLO){
				sprintf(Response_Buffer,"1 ER COMMAND_CANNOT_BE_PROCESSSED ");
			}else if(Float_Temp>=0 && Float_Temp<=Max_Temperature){//�S���Ʀr�ˬd
				
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
					Set_Inhalation_Fan_PWM(Int_Temp);
					sprintf(Response_Buffer,"1 OK FAN ");
				}else{
					sprintf(Response_Buffer,"1 ER PARAM_OUT_OF_RANGE ");
				}
		}else{
			sprintf(Response_Buffer,"1 ER UNKNOW_COMMAND ");
		}
	}else{
		sprintf(Response_Buffer,"1 ER UNKNOW_COMMAND ");
	}
	
	//caculate checksum and send back
	sprintf(Response_Buffer,"%s*%d",Response_Buffer,Get_Checksum(Response_Buffer,strlen(Response_Buffer)));
	printf("%s\n",Response_Buffer);
}


