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
#include "Using_Time.h"

uint32_t PDown_Start_Time=0;
uint32_t PDown_Max_Time=0;
bool Ended=TRUE;
bool Measure_Laser_Power_Down=FALSE;
void Laser_Pon(void);
void Laser_Pdown(void);
void (*Interlock_Exti_Func)(void) = Laser_Pdown;
EXTI_InitTypeDef   EXTI_InitStructure;
Laser_Status_Type Interlock_Last_Status=Laser_Power_On;
Laser_Status_Type Interlock_Status_Mask=Laser_Power_On;
Laser_Status_Type User_Switch=Laser_Power_Down;

uint32_t Laser_Power_Current_Time=0;


extern float Target_Temperature;
extern volatile uint32_t Module_State;
extern volatile bool Debug_Mode;
extern volatile bool Show_Sensor_Data;
extern Kalman_Data_Struct Kal_X,Kal_Y;
extern float Degree_Now;

void Laser_Cmd_Handler(void){
    uint32_t Laser_Pdown_Interval=0;
	char * Command_Str;
	char Response_Buffer[200];
	Command_Str = strtok(NULL, " ");
	
	if(!strcmp(Command_Str, "HELLO")){		
		Debug_Mode=FALSE;
		Show_Sensor_Data=FALSE;
		Reset_Module_State(NO_HELLO);
		sprintf(Response_Buffer,"1 OK HELLO TYPE:LASER ID:%08X%08X%08X VENDOR:%s FIRMWARE:%s VERSION:%s HARDWARE_VERSION:0 FOCAL_LENGTH:%.2lf USED:%u ",UUID[2],UUID[1],UUID[0],Vender,Firmware_Name,Firmware_Version,Read_Focal_Length(),Read_Using_Time());

	}else if(!strcmp(Command_Str, "DEBUG")){
		Debug_Mode=TRUE;
		Reset_Module_State(NO_HELLO);
		sprintf(Response_Buffer,"1 OK HELLO TYPE:LASER ID:%08X%08X%08X VENDOR:%s FIRMWARE:%s VERSION:%s HARDWARE_VERSION:0 FOCAL_LENGTH:%.2lf USED:%u ",UUID[2],UUID[1],UUID[0],Vender,Firmware_Name,Firmware_Version,Read_Focal_Length(),Read_Using_Time());

	}else if(!strcmp(Command_Str, "SHOW")){
		Show_Sensor_Msg();
		Show_Sensor_Data=TRUE;
		sprintf(Response_Buffer,"1 OK ");
	}else if(!strcmp(Command_Str, "SHOW1")){
		sprintf(Response_Buffer,"1 OK x%.2f y%.2f D%.2f ",Kal_X.angle,Kal_Y.angle,Degree_Now);
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
        //reset SELF_RESET
        Reset_Module_State(SELF_RESET);
	}else if(!strcmp(Command_Str, "WRITE")){
		float Float_Temp;
		Command_Str = strtok(NULL, " ");
		if(Command_Str[0]=='F' && Command_Str[1]=='L' && Command_Str[2]==':'){
			Float_Temp=atof(&Command_Str[3]);
			if(Write_Focal_Length(Float_Temp)){
				sprintf(Response_Buffer,"1 OK WRITE ");
			}else{
				sprintf(Response_Buffer,"1 ER FAILED ");
			}
			
				
		}else{
			sprintf(Response_Buffer,"1 ER UNKNOW_COMMAND ");
		}
	}else if(!strcmp(Command_Str, "DETAIL")){
		sprintf(Response_Buffer,"1 OK DATE:%s ",Firmware_Date);
	}else if(!strcmp(Command_Str, "USING_TIME")){
		sprintf(Response_Buffer,"1 OK %u ",Read_Using_Time());
	}else if(!strcmp(Command_Str, "START")){
        PDown_Max_Time=0;
        Measure_Laser_Power_Down=TRUE;
        Interlock_Exti_Config();
        sprintf(Response_Buffer,"OK");
    }else if(!strcmp(Command_Str, "END")){
        if(!Ended){
            Laser_Pdown_Interval=millis()-PDown_Start_Time;
            if(Laser_Pdown_Interval>PDown_Max_Time)
                PDown_Max_Time=Laser_Pdown_Interval;
            //printf("e2\n");
        }
        Ended=TRUE;
        Measure_Laser_Power_Down=FALSE;
        Interlock_Exti_Break();
        sprintf(Response_Buffer,"%d ",PDown_Max_Time);
    }else{
		sprintf(Response_Buffer,"1 ER UNKNOW_COMMAND ");
	}
	
	//caculate checksum and send back
	sprintf(Response_Buffer,"%s*%d",Response_Buffer,Get_Checksum(Response_Buffer,strlen(Response_Buffer)));
	printf("%s\n",Response_Buffer);
}

void Laser_Switch_On(void){
    User_Switch=Laser_Power_On;
	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	GPIO_ResetBits(GPIOB,GPIO_Pin_9);
}

void Laser_Switch_Off(void){
    User_Switch=Laser_Power_Down;
	GPIO_SetBits(GPIOA,GPIO_Pin_0);
	GPIO_SetBits(GPIOB,GPIO_Pin_9);
}

void Detect_Laser_Power(void){
	if(Interlock_Status_Mask==Laser_Power_On){
		Reset_Module_State(LASER_DOWN);
	}else{
		Set_Module_State(LASER_DOWN);
	}
    if(Measure_Laser_Power_Down){
        if(Ended && !GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2)){ 
            PDown_Start_Time=millis();
            Ended=FALSE;
            Interlock_Exti_Func=Laser_Pon;
            EXTI_InitStructure.EXTI_Line = EXTI_Line2;  
            EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
            EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//EXTI_Trigger_Falling;
            EXTI_InitStructure.EXTI_LineCmd = ENABLE;
            EXTI_Init(&EXTI_InitStructure);
            //printf("2F\n");
            EXTI_ClearITPendingBit(EXTI_Line2);
        }
        if(!Ended && GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2)){
            uint32_t interval_Time=millis()-PDown_Start_Time;
            Ended=TRUE;
            if(interval_Time>PDown_Max_Time)
                PDown_Max_Time=interval_Time;
            Interlock_Exti_Func=Laser_Pdown;
            EXTI_InitStructure.EXTI_Line = EXTI_Line2;  
            EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
            EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Falling;
            EXTI_InitStructure.EXTI_LineCmd = ENABLE;
            EXTI_Init(&EXTI_InitStructure);
            //printf("2O%d\n",interval_Time);
            EXTI_ClearITPendingBit(EXTI_Line2);
        }
    }
}

void Laser_Pon(void){
    uint32_t interval_Time=millis()-PDown_Start_Time;
    Ended=TRUE;
    if(interval_Time>PDown_Max_Time)
        PDown_Max_Time=interval_Time;
    
    Interlock_Exti_Func=Laser_Pdown;
    EXTI_InitStructure.EXTI_Line = EXTI_Line2;  
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    //printf("2o%d\n",interval_Time);
}

void Laser_Pdown(void){  
    PDown_Start_Time=millis();
    Ended=FALSE;
    Interlock_Exti_Func=Laser_Pon;
    EXTI_InitStructure.EXTI_Line = EXTI_Line2;  
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    //printf("2f\n");
}

void Debounce_Laser_Power(void){
    uint32_t Laser_Power_Down_Time;
    if(Read_Laser_Power()){
        if(Interlock_Last_Status==Laser_Power_On){
            //still on
            
        }else{
            //off->on
            Interlock_Last_Status=Laser_Power_On;
            Interlock_Status_Mask=Laser_Power_On;
        }
    }else{
        if(Interlock_Last_Status==Laser_Power_On){
            //on->off
            Laser_Power_Current_Time=millis();
            Interlock_Last_Status=Laser_Power_Down;
        }else{
            //still off
            Laser_Power_Down_Time=millis()-Laser_Power_Current_Time;
            if(Laser_Power_Down_Time>500){//500ms
                Interlock_Status_Mask=Laser_Power_Down;
            }
        }
    }
}
