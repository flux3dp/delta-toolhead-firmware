#include "Unknow_Module.h"
#include "defines.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "command.h"
#include "utilities.h"
#include "Six_Axis_Sensor.h"

extern volatile uint32_t Module_State;
extern volatile bool Debug_Mode;
extern volatile bool Show_Sensor_Data;

void Unknow_Module_Cmd_Handler(void){
	char * Command_Str;

	char Response_Buffer[200];
	Command_Str = strtok(NULL, " ");
	
	if(!strcmp(Command_Str, "HELLO")){		
		Debug_Mode=FALSE;
		Show_Sensor_Data=FALSE;
		Reset_Module_State(NO_HELLO);
		sprintf(Response_Buffer,"1 OK HELLO TYPE:UNKNOW ID:%08X%08X%08X VENDOR:%s FIRMWARE:%s VERSION:%.4lf ",UUID[2],UUID[1],UUID[0],Vender,Firmware_Name,Firmware_Version);

	}else if(!strcmp(Command_Str, "DEBUG")){
		Debug_Mode=TRUE;
		Reset_Module_State(NO_HELLO);
		sprintf(Response_Buffer,"1 OK HELLO TYPE:UNKNOW ID:%08X%08X%08X VENDOR:%s FIRMWARE:%s VERSION:%.4lf ",UUID[2],UUID[1],UUID[0],Vender,Firmware_Name,Firmware_Version);

	}else if(!strcmp(Command_Str, "SHOW")){
		Show_Sensor_Msg();
		Show_Sensor_Data=TRUE;
		sprintf(Response_Buffer,"1 OK ");
	}else if(!strcmp(Command_Str, "PING")){
		//error check
		
		//response
		sprintf(Response_Buffer,"1 OK PONG ER:%d ",Module_State);
		
		//reset sensor state
		Reset_Axis_Sensor_State();
		//reset alarm IO
		Alarm_Off();
	}else{
		sprintf(Response_Buffer,"1 ER UNKNOW_COMMAND ");
	}
	
	//caculate checksum and send back
	sprintf(Response_Buffer,"%s*%d",Response_Buffer,Get_Checksum(Response_Buffer,strlen(Response_Buffer)));
	printf("%s\n",Response_Buffer);
}