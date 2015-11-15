#include "LaserModule.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "gpio.h"
#include "command.h"
#include "defines.h"
#include "heater.h"
#include "fan.h"
#include "Six_Axis_Sensor.h"
#include "utilities.h"
extern float Target_Temperature;
extern volatile uint32_t Module_State;
extern volatile bool Debug_Mode;
extern volatile bool Show_Sensor_Data;

void Laser_Cmd_Handler(void){
	char * Command_Str;

	char Response_Buffer[200];
	Command_Str = strtok(NULL, " ");
	
	if(!strcmp(Command_Str, "HELLO")){		
		Debug_Mode=FALSE;
		Show_Sensor_Data=FALSE;
		Reset_Module_State(NO_HELLO);
		sprintf(Response_Buffer,"1 OK HELLO TYPE:LASER ID:%u VENDOR:%s FIRMWARE:OHMAMA VERSION:%.4lf LASER ",Get_UUID(),Vender,Firmware_Version);

	}else if(!strcmp(Command_Str, "DEBUG")){
		Reset_Module_State(NO_HELLO);
		sprintf(Response_Buffer,"1 OK HELLO TYPE:LASER ID:%u VENDOR:%s FIRMWARE:OHMAMA VERSION:%.4lf LASER ",Get_UUID(),Vender,Firmware_Version);
		
	}else if(!strcmp(Command_Str, "SHOW")){
		Show_Sensor_Data=TRUE;
	}else if(!strcmp(Command_Str, "PING")){
		//error check
		if(!(Module_State&(SHAKE|TILT)))
			Laser_Switch_On();
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

void Laser_Switch_On(void){
	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	GPIO_ResetBits(GPIOB,GPIO_Pin_9);
}

void Laser_Switch_Off(void){
	GPIO_SetBits(GPIOA,GPIO_Pin_0);
	GPIO_SetBits(GPIOB,GPIO_Pin_9);
}

