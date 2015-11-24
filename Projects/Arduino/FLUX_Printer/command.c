#include "command.h"
#include <stdio.h>
#include "stm32f0xx_flash.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "uart.h"
#include "heater.h"
#include "defines.h"
#include "LaserModule.h"
#include "Extruder_One_Module.h"
#include "Extruder_Duo_Module.h"
#include "Unknow_Module.h"
#include "Six_Axis_Sensor.h"
#include "utilities.h"
#include "fan.h"

const char Flux_Cmd_Header[] = "1"; //FLUX command header=1,user defined header=0;
volatile uint32_t Module_State=0;
volatile bool Debug_Mode=FALSE;
volatile bool Show_Sensor_Data=FALSE;

extern Six_Axis_Sensor_State_Type Six_Axis_Sensor_State;
extern char buff_Uart1[100];//command data buffer
extern Uart_BufferType CmdBuffer;
extern ModuleMode_Type ModuleMode;
extern volatile uint32_t CmdTimeout_count;

void ToLowerCase(Uart_BufferType *buff);
void ToUpperCase(Uart_BufferType *buff);
static void Test_Extruder_One(void);
static void Test_Laser(void);
static uint32_t Test_Alarm_IO(void);

static uint32_t Test_Sensor_RW(void);

static uint32_t Test_Acceler_Range(void);

static uint32_t Test_Gyro_Range(void);

static uint32_t Test_Laser_PWM_Switch(void);

static uint32_t Test_Laser_Power_Switch(void);

static uint32_t Test_Thermal_Analog_Read(void);

static uint32_t Test_Heater_Output(void);

static uint32_t Test_Fan1_IO(void);

static uint32_t Test_Fan2_IO(void);

extern void resetUartBuffer(Uart_BufferType *buff);

void Module_State_Initial(void){
	Set_Module_State(NO_HELLO);//set initial error= no hello
	Set_Module_State(SENSOR_CALIBRATION_FAILURE);
}

void Set_Module_State(Module_State_Enum state){
	Module_State |= (uint32_t)state;
}

void Reset_Module_State(Module_State_Enum state){
	Module_State &= ~((uint32_t)state);
}

void Xcode_Handler(void){
	
	char *strSplit;	//For parameter retriving 
	//char strTemp[100];
	
	//command timeout check
	if(!Debug_Mode){
		switch(ModuleMode){
			case FLUX_ONE_EXTRUDER_MODULE:
				if(CmdTimeout_count>Extruder_Cmd_Timeout)
					Set_Temperature(0);
					if(Read_Temperature()<0.1)
						Set_Inhalation_Fan_PWM(0);
				break;
			case FLUX_DUO_EXTRUDER_MODULE:
				//close duo extruder
				if(CmdTimeout_count>Extruder_Cmd_Timeout)
					Set_Temperature(0);
				break;
			case FLUX_LASER_MODULE:
				//close laser
				if(CmdTimeout_count>Laser_Cmd_Timeout)
					Laser_Switch_Off();
				break;
			case Unknow:
				//??
				break;
		}
	}
	
	if(!CmdBuffer.Received){//Detect data ending('\n') to handle a command
		return;
	}
	
	//a command received...
	
	//Debug mode
	if(!strcmp(CmdBuffer.Data,"1 DEBUG") || !strcmp(CmdBuffer.Data,"1 debug")){
		
		Debug_Mode=TRUE;
	}else{
		//checksum caculation
		if(!Cmd_Checksum_Validation(CmdBuffer.Data,CmdBuffer.Length) && !Debug_Mode){
			resetUartBuffer(&CmdBuffer);//clear command data buffer
			return;
		}
	}
	
	
		
	//command error check()
	
	
	//check command header
	strSplit = strtok(CmdBuffer.Data, " ");
	if(strcmp(strSplit,Flux_Cmd_Header)){
		//the command is not for this module
		resetUartBuffer(&CmdBuffer);//clear command data buffer
		return;
	}
	
	ToUpperCase(&CmdBuffer);
	
	switch(ModuleMode){
			case FLUX_ONE_EXTRUDER_MODULE:
				Extruder_One_Cmd_Handler();
				CmdTimeout_count=0;
				break;
			case FLUX_DUO_EXTRUDER_MODULE:
				Extruder_Duo_Cmd_Handler();
				CmdTimeout_count=0;
				break;
			case FLUX_LASER_MODULE:
				Laser_Cmd_Handler();
				CmdTimeout_count=0;
				break;
			case Unknow:
				Unknow_Module_Cmd_Handler();
				CmdTimeout_count=0;
				break;
	}
	
	resetUartBuffer(&CmdBuffer);//clear command data buffer

}

uint32_t * Get_UUID(void){
	return ( uint32_t *)STM32F0_UUID;
}

float Read_Focal_Length(void){
	int32_t FL_Data,Checksum;
	FL_Data=*(__IO uint32_t *)FLASH_USER_END_ADDR ;
	Checksum=*(__IO uint32_t *)(FLASH_USER_END_ADDR+4);
	if((FL_Data^0x12345678) == Checksum){
		return (float)FL_Data/100;
	}else{
		return 0.0;
	}
	
}

bool Write_Focal_Length(float Focal_Length){
	uint32_t Data_Verify;
	uint32_t Data_Trans=(int32_t)(Focal_Length*100);
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 
	FLASH_ErasePage(FLASH_USER_END_ADDR);
	FLASH_ProgramWord(FLASH_USER_END_ADDR , Data_Trans);
	FLASH_ProgramWord(FLASH_USER_END_ADDR+4	, (Data_Trans^0x12345678));
	FLASH_Lock();

	Data_Verify = *(__IO uint32_t *)FLASH_USER_END_ADDR ;
	if(Data_Verify == Data_Trans)
	{	
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

bool IsNumber(char *NumberString){ 
	uint16_t i,index_last;
	index_last=strlen(NumberString);
	for(i=0;i<index_last;i++){
		if(NumberString[index_last-1]=='\r' && index_last-1>0)//number+'\r' is legal
			return TRUE;
		if(NumberString[i]<0x30 || NumberString[i]>0x39)
			return FALSE;
	}
	return TRUE;
}

bool Cmd_Checksum_Validation(char * Cmd_Data,uint16_t Length){
	char str_Temp[300];
	
	while(Length){
		if(Cmd_Data[--Length]=='*'){
			strncpy(str_Temp,Cmd_Data,Length);
			str_Temp[Length]='\0';
			if(Ascii_Checksum_Compare(str_Temp,(uint8_t)atoi(&Cmd_Data[Length+1])))
				return TRUE;
			else
				return FALSE;
			
		}
	}
	return FALSE;
}
bool Ascii_Checksum_Compare(char * str_Data,uint8_t checksum){
	//printf("checksum=%d\n",Get_Checksum(str_Data,strlen(str_Data)));
	if(Get_Checksum(str_Data,strlen(str_Data))==checksum)
		return TRUE;
	else
		return FALSE;
}

void CommandTimeoutDetection(void){
	//TODO{
	//	關雷射頭開關、加熱頭、風扇關?
	//}
}

void ToLowerCase(Uart_BufferType *buff){
	int i;
	for(i=0;i<buff->Length;i++){//to lower case
				buff->Data[i] = tolower(buff->Data[i]);
	}
}

void ToUpperCase(Uart_BufferType *buff){
	int i;
	for(i=0;i<buff->Length;i++){//to lower case
				buff->Data[i] = toupper(buff->Data[i]);
	}
}

//return 0~4095 adc value
uint16_t Read_ADC_Value(ADC_Channel_Type channel){
	ADC_InitTypeDef     ADC_InitStructure;
	/* ADCs DeInit */  
	ADC_DeInit(ADC1);

	/* Initialize ADC structure */
	ADC_StructInit(&ADC_InitStructure);

	/* Configure the ADC1 in continuous mode with a resolution equal to 12 bits  */
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
	ADC_Init(ADC1, &ADC_InitStructure); 

	/* Convert the ADC1 Channel 0 with 239.5 Cycles as sampling time */ 
	if(channel==Temperature_Channel)
		ADC_ChannelConfig(ADC1, ADC_Channel_0, ADC_SampleTime_239_5Cycles); //ADC_SampleTime_55_5Cycles 
	else if(channel==ID0_Channel)
		ADC_ChannelConfig(ADC1, ADC_Channel_6, ADC_SampleTime_239_5Cycles);  //ADC_SampleTime_239_5Cycles
	else if(channel==ID1_Channel)
		ADC_ChannelConfig(ADC1, ADC_Channel_7, ADC_SampleTime_239_5Cycles);  
	/* ADC Calibration */
	ADC_GetCalibrationFactor(ADC1);

	/* Enable the ADC peripheral */
	ADC_Cmd(ADC1, ENABLE);     

	/* Wait the ADRDY flag */
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY)); 

	/* ADC1 regular Software Start Conv */ 
	ADC_StartOfConversion(ADC1);
	
	/* Test EOC flag */
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	/* Get ADC1 converted data */
	ADC_GetConversionValue(ADC1);
	
	/* Test EOC flag */
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	/* Get ADC1 converted data */
	return ADC_GetConversionValue(ADC1);
}

bool Read_Self_Test_IO(void){
	GPIO_InitTypeDef    GPIO_InitStructure;
	/* GPIOA Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); 

	/* Configure PB0 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	return (bool)GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15);
}


void Self_Test(void){
	Self_Test_IO_Config();
	
	switch(ModuleMode){
			case FLUX_ONE_EXTRUDER_MODULE:	
				Test_Extruder_One();
				break;
			case FLUX_DUO_EXTRUDER_MODULE:
				
				break;
			case FLUX_LASER_MODULE:
				Test_Laser();
				break;
			case Unknow:
				//could not recognize module type
				printf ("%08X%08X%08X 00 %s\n",UUID[2],UUID[1],UUID[0],"0");
				break;
	}
}

static void Test_Extruder_One(void){
	int Test_Result=0;
	char buffer [33];
	char *binResult;
	
	Test_Result+=1;
	Test_Result+=Test_Alarm_IO();
	Test_Result+=Test_Sensor_RW();
	Test_Result+=Test_Acceler_Range();
	Test_Result+=Test_Gyro_Range();
	Test_Result+=Test_Thermal_Analog_Read();
	Test_Result+=Test_Heater_Output();
	Test_Result+=Test_Fan1_IO();
	Test_Result+=Test_Fan2_IO();
	
	
	binResult=int2binStr(Test_Result,buffer);
	printf ("%08X%08X%08X 10 %s\n",UUID[2],UUID[1],UUID[0],binResult);
}

static void Test_Laser(void){
	uint32_t Test_Result=0;
	char buffer[33];
	char *binResult;
	
	Test_Result+=1;
	Test_Result+=Test_Alarm_IO();
	Test_Result+=Test_Sensor_RW();
	Test_Result+=Test_Acceler_Range();
	Test_Result+=Test_Gyro_Range();
	Test_Result+=Test_Laser_PWM_Switch();
	Test_Result+=Test_Laser_Power_Switch();
	
	binResult=int2binStr(Test_Result,buffer);
	printf ("%08X%08X%08X 01 %s\n",UUID[2],UUID[1],UUID[0],binResult);
}

static uint32_t Test_Alarm_IO(void){
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	delay_ms(1);
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==0)
		return 2;
	else
		return 0;
	
}

static uint32_t Test_Sensor_RW(void){
	if(!(Module_State & SENSOR_FAILURE))
		return 4;
	else
		return 0;
	
}

static uint32_t Test_Acceler_Range(void){
	float value_x=0,value_y=0,value_z=0;
	uint8_t i;
	if(!(Module_State & SENSOR_FAILURE)){
		for(i=0;i<20;i++){
			value_x+=Read_Axis_Value(Acceler_X);
			value_y+=Read_Axis_Value(Acceler_Y);
			value_z+=Read_Axis_Value(Acceler_Z);
		}
		if(value_x>1000 || value_y>1000 || value_z>1000)
			return 0;
		else
			return 8;
	}
	return 0;
}

static uint32_t Test_Gyro_Range(void){
	float Angle_Z_Max,Angle_Z_Min,value_z;
	uint8_t i;
	if(!(Module_State & SENSOR_FAILURE)){
		Angle_Z_Max=Angle_Z_Min=Read_Axis_Value(Gyro_Z);
		for(i=0;i<20;i++){
			value_z=Read_Axis_Value(Gyro_Z);
			if(Angle_Z_Max<value_z)
				Angle_Z_Max=value_z;
			if(Angle_Z_Min>value_z)
				Angle_Z_Min=value_z;
		}
		if(ABS_F(Angle_Z_Max-Angle_Z_Min)>2000)
			return 0;
		else
			return 16;
	}
	return 0;
}



static uint32_t Test_Thermal_Analog_Read(void){
	if(Read_Temperature()<0.0001)
		return 32;
	else
		return 0;
}

static uint32_t Test_Heater_Output(void){
	TIM1->CCR1 =0;
	delay_ms(1);
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)==0)
		return 64;
	else
		return 0;
}

static uint32_t Test_Fan1_IO(void){
	TIM16->CCR1=0;
	delay_ms(1);
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==0)
		return 128;
	else
		return 0;
}

static uint32_t Test_Fan2_IO(void){
	TIM17->CCR1=0;
	delay_ms(1);
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==0)
		return 256;
	else
		return 0;
}

static uint32_t Test_Laser_PWM_Switch(void){
	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	delay_ms(1);
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2)==0)
		return 512;
	else
		return 0;
}

static uint32_t Test_Laser_Power_Switch(void){
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	delay_ms(1);
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==0)
		return 1024;
	else
		return 0;
}

