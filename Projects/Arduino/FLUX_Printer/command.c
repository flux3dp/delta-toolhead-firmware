#include "command.h"
#include <stdio.h>
#include "stm32f0xx_flash.h"
#include <stdlib.h>
#include <string.h>
#include "uart.h"
#include "heater.h"
#include "defines.h"
#include "LaserModule.h"
#include "Extruder_One_Module.h"
#include "Extruder_Duo_Module.h"
#include "Six_Axis_Sensor.h"

const char Flux_Cmd_Header[] = "1"; //FLUX command header=1,user defined header=0;

bool Already_Hello=FALSE;

extern Six_Axis_Sensor_State_Type Six_Axis_Sensor_State;
extern char buff_Uart1[100];//command data buffer
extern Uart_BufferType CmdBuffer;
//extern HeaterState_Type Heater_State;
extern ModuleMode_Type ModuleMode;

void ToLowerCase(Uart_BufferType *buff);
void ToUpperCase(Uart_BufferType *buff);

extern void resetUartBuffer(Uart_BufferType *buff);





void Xcode_Handler(){
	
	char *strSplit;	//For parameter retriving 
	char strTemp[100];
	if(!CmdBuffer.Received){//Detect data ending('\n') to handle a command
		return;
	}
	
	//a command received...
	
	//checksum caculation
	//command error check()
	
	ToUpperCase(&CmdBuffer);
	
	//check command header
	strSplit = strtok(CmdBuffer.Data, " ");
	if(!strcmp(strSplit,Flux_Cmd_Header)){
		printf("1 ");
	}else{
		//the command is not for this module
		resetUartBuffer(&CmdBuffer);//clear command data buffer
		return;
	}
	
	switch(ModuleMode){
			case FLUX_ONE_EXTRUDER_MODULE:
				Extruder_One_Cmd_Handler();
				break;
			case FLUX_DUO_EXTRUDER_MODULE:
				Extruder_Duo_Cmd_Handler();
				break;
			case FLUX_LASER_MODULE:
				Laser_Cmd_Handler();
				break;
			case Unknow:
				printf("ER:4 UNKNOW_MODULE\n");
				break;
	}
	
	resetUartBuffer(&CmdBuffer);//clear command data buffer

}

uint32_t Read_ID(){
	return *(__IO uint32_t *)FLASH_USER_END_ADDR ;
}

bool Write_ID(uint32_t ID){
	uint32_t ID_Verify;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 
	FLASH_ErasePage(FLASH_USER_END_ADDR);
	FLASH_ProgramWord(FLASH_USER_END_ADDR , ID);
	FLASH_ProgramWord(FLASH_USER_END_ADDR+4	, (ID^1));
	FLASH_Lock();

	ID_Verify = *(__IO uint32_t *)FLASH_USER_END_ADDR ;

	if(ID_Verify == ID)
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

void CommandTimeoutDetection(){
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




