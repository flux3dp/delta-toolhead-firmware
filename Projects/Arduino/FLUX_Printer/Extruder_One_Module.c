#include "Extruder_One_Module.h"
#include <stdio.h>
#include "gpio.h"
#include "defines.h"
#include <string.h>
#include "command.h"

extern bool Already_Hello;

void Extruder_One_Cmd_Handler(char * Command_Str){
	uint16_t Real_Temperatuer=0;
	uint16_t Int_Temp;
	uint16_t Fan_Pwm;
	
	Command_Str = strtok(NULL, " ");
	
	if(!strcmp(Command_Str, "HELLO")){
		//OK HELLO TYPE:EXTRUDER ID:3f1f2a VENDOR:flux\ .inc FIRMWARE:xxxxxx VERSION:0.1.9 EXTRUDER:1 MAX_TEMPERATURE:250 *[CHKSUM]\n
		
		//Pong error check();
		
		Already_Hello=TRUE;
		
		printf("OK ");
		printf("EXTRUDER ");
		printf("%d ",Read_ID());
		printf("VENDOR:%s ",Vender);
		printf("FIRMWARE:OHMAMA ");
		printf("VERSION:%.4lf ",Firmware_Version);
		printf("EXTRUDER:1 ");
		printf("MAX_TEMPERATURE:%d ",Max_Temperature/10);
		printf("*[checksum]\n");
	}else if(!strcmp(Command_Str, "PING")){
		//printf("1 OK PONG 1 ER:0 RT:123.3,212.3 TT:200.0,NAN FA:255 *[CHKSUM]\n
		printf("OK ");
		printf("PONG ");
		if(Already_Hello)
			printf("1 ");//have Hello?
		else
			printf("0 ");
		printf("ER:0 ");
		printf("RT:%lf ",((double)Real_Temperatuer/10));//­nÅªADC­È°µÂà´«
		printf("*[checksum]\n");
	}else if(!strcmp(Command_Str, "H:1")){
		printf("H:1\n");
	}else if(!strcmp(Command_Str, "H:2")){
		printf("H:2\n");
	}else if(!strcmp(Command_Str, "F:1")){
		Command_Str = strtok(NULL, " ");
		if(Command_Str[0]=='S' && Command_Str[1]==':' && IsNumber(&Command_Str[2])){
				Int_Temp = atoi(&Command_Str[2]);
				if(Int_Temp>=0 && Int_Temp<=255){
					Fan_Pwm = (-(Int_Temp - 255))*256;
				}else{
					printf("ER:2 PARAM_OUT_OF_RANGE\n\n");
					return;
				}
		}else{
			printf("ER:0 UNKNOW_COMMAND\n");
			return;
		}
		TIM16->CCR1 = Fan_Pwm;
		printf("OK FAN ");
		printf("*[checksum]\n");
		//TIM1->CCR1 = Fan_Pwm;  
	}else{
		printf("ER:0 UNKNOW_COMMAND\n");
	}
}
