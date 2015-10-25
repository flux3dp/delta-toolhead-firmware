#include "command.h"
#include <stdio.h>
#include "stm32f0xx_flash.h"
#include <stdlib.h>
#include <string.h>
#include "uart.h"
#include "heater.h"
#include "const.h"
#include "defines.h"
#include "LaserModule.h"
#include "Extruder_One_Module.h"
#include "Extruder_Duo_Module.h"

bool Already_Hello=FALSE;

extern char buff_Uart1[100];//command data buffer
extern Uart_BufferType CmdBuffer;
extern HeaterState_Type Heater_State;
extern ModuleMode_Type ModuleMode;

void ToLowerCase(Uart_BufferType *buff);
void ToUpperCase(Uart_BufferType *buff);

extern void resetUartBuffer(Uart_BufferType *buff);





void Xcode_Handler(){
	//char *PstrTemp,fan_Selection;
	uint8_t Cmd_Header=0;
	
	
	char *strSplit;
	char strTemp[100];
	if(!CmdBuffer.Received){//detect data ending('\n') to handle a command
		return;
	}
	
	//a command received...
	
	//checksum caculation
	//command error check()
	
	ToUpperCase(&CmdBuffer);
	
	//check command address
	strSplit = strtok(CmdBuffer.Data, " ");
	if(!strcmp(strSplit,"1")){
		printf("1 ");
	}else{
		//the command is not for this module
		Cmd_Header=0;
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
	
	
	
	/*
	switch(CmdBuffer.Data[0]){
		case 'a':	//PID autotune
//			pidc=0;
//			heating = 1;
//			temp_millis = pidc, t1 = temp_millis, t2 = temp_millis;
//			soft_pwm[hotend] = bias = dxd = PID_MAX;
//			Output = -(soft_pwm[hotend]-255) *257;
//			printf("init pwm:%d \r\n",Output);
//			Setpoint = 2000;
//			hotend = 1;
//			ncycles = 10;
//			PID_auto = 1;
//			printf("ok@A\r\n");	
		
				printf("hi A\n");
				break;
		case 'h':	//set heater temperature
			
				printf("hi H\n");
		
				break;
		case 'f':	//set fan1 or fan2.3 duty
				fan_Selection=CmdBuffer.Data[1];//沒檢查parameter 送錯會crash
				strTemp=&CmdBuffer.Data[2];
				if(IsNumber(strTemp)){
					intTemp = atoi(strTemp);
					if(intTemp>=0 && intTemp<=255){
						fan_pwm = (-(intTemp - 255))*256;
					}else{
						printf("parameter out of range\n");
						break;
					}
				}else{
					printf("parameter error\n");
					break;
				}
				
				switch(fan_Selection){
					case '1'://要建data structure:FAN1 FAN2
							TIM16->CCR1 = fan_pwm;
							printf("hi F1,%d\n",fan_pwm);
							break;
					case '2':
							TIM17->CCR1 = fan_pwm;
							printf("hi F2,%d\n",fan_pwm);
							break;
					default:
						
							break;
				}
				printf("hi F,%s\n",CmdBuffer.Data);
			
				break;
		case 'l':	//laser switch 
				if(CmdBuffer.Data[1]=='o'){
					SetHeaterOn();
				}else if(CmdBuffer.Data[1]=='f'){
					SetHeaterOff();
				}
				
				printf("hi L\n");
				break;
		case 'w':	//ID written
				strTemp=&CmdBuffer.Data[1];
				intTemp = atoi(strTemp);//沒檢查parameter
				printf("hi W,%d\n",intTemp);
				Write_ID(intTemp);
		
				break;
		case 'r':	//read status
				switch(CmdBuffer.Data[1]){
					case 'a':
						
						break;
					case 'c':
						printf("*************************************************\r\n");
						printf("Command List: \r\n");
						printf("A: PID Autotune\r\n");
						printf("D: Fan Detection\r\n");
						printf("F: Fan Control\r\n");
						printf("H: Heater Control\r\n");
						printf("K: Sensor Calibration\r\n");
						printf("L: Laser Control\r\n");		
						printf("Q: Clear flag\r\n");
						printf("R: Read info(A,C,E,I,M,T,V)\r\n");
						printf("S: Sensor Detection\r\n");
						printf("W: Write Device ID\r\n");
						printf("*************************************************\r\n");
						printf("ok@RC\r\n");
						break;
					case 'd':
						printf("*************************************************\r\n");
						printf("Read Device Status: \r\n");
							if(Heater_State==Heater_On){
								printf("Heater Status : ON\r\n");
								//printf("Setpoint :%d\r\n",Setpoint); 
							}else if(Heater_State==Heater_Off){
								printf("Heater Status : OFF\r\n");
							}
						printf("*************************************************\r\n");
						printf("ok@RD\r\n");
						break;
					case 'e':
						printf("*************************************************\r\n");
						printf("Event List: \r\n");
						printf("1: Boot Success\r\n");
						printf("2: Fan2 Failure\r\n");
						printf("3: Fan3 Failure\r\n");
						printf("4: Sensor Connection Success\r\n");
						printf("5: Sensor Connection Failure\r\n");
						printf("6: Sensor Gyro_x failure\r\n");
						printf("7: Sensor Gyro_y failure\r\n");
						printf("8: Sensor Gyro_z failure\r\n");
						printf("9: Sensor Accelerometer_x failure\r\n");
						printf("10: Sensor Accelerometer_y failure\r\n");
						printf("11: Sensor Accelerometer_z failure\r\n");
						printf("12: ID 0 failure\r\n");
						printf("13: ID 1 failure\r\n");
						printf("14: Over temperature\r\n");
						printf("15: Device ID Write Success\r\n");
						printf("16: Device ID Write Failure\r\n");
						printf("17: Heater On\r\n");
						printf("18: Heater Off\r\n");
						//printf("19: Fan1 On\r\n");
						//printf("20: Fan1 Off\r\n");
						//printf("21: Fan2 On\r\n");
						//printf("22: Fan2 Off\r\n");
						printf("*************************************************\r\n");
						printf("ok@RE\r\n");
						break;
					case 'h':
						printf("*************************************************\r\n");
						printf("HISTORY: \r\n");
						printf("0901-1 : add history cmd\r\n");
						printf("0901-2 : remove z cmd\r\n");
						printf("0902-1 : modify SA & SG cmd\r\n");
						printf("0902-2 : modify write device id cmd (W)\r\n");
						printf("0902-3 : modify R cmd (W)\r\n");
						printf("0904-1 : modify ID0&ID1 event message\r\n");
						printf("0908-1 : add PID autotune function\r\n");
						printf("0908-2 : modify T cmd\r\n");
						printf("0909-1 : modify sensor connection flag\r\n");
						printf("*************************************************\r\n");
						printf("ok@RH\r\n");						
						break;
					case 'i':
						printf("Device ID :%d\n\r", *(__IO uint32_t *)FLASH_USER_START_ADDR);
						printf("ok@RI\r\n");
						break;
					case 'm':
//						switch(FLUX_MODULE_MODE){
//							case FLUX_LASER_MODULE:
//								printf("FLUX Laser Module\r\n");
//								break;
//							case FLUX_3DPRINTER_MODULE:
//								printf("FLUX Printer Module\r\n");
//								break;
//						}
						printf("ok@RM\r\n");
					case 't':
//						if (FLUX_MODULE_MODE == FLUX_3DPRINTER_MODULE)
//						{	
//								//adc_s = 0;
//								//adc_s = analogRead(THERM);			
//								//printf("adc:%d >> %d  ok@T\r\n",adc_s,temps[adc_s]);
//								printf("ok@RT: %d\r\n",temps[GetTemperature_ADCValue()]);
//						}
						break;
					case 'v':
						printf("FW_Version : v%6.5f\r\n",Firmware_Version);
						printf("ok@RV\r\n");
						break;
					default:
						
						break;
				}
				
				printf("hi R\n");
				break;
		case 'k':	//sensor calibration
				printf("hi K\n");
				break;
		case 's':	//read accelerometer or gyroscope row data
				printf("hi S\n");
				break;
		case 't':	//test
				printf("hi T\n");
				break;
		case 'm':	//main count? WTF
				printf("hi M\n");
				break;
		case 'q':	//clear flag? OMG
				printf("hi Q\n");
				break;
		default:
				printf("parameter error\n");
				break;
	
	}
	*/
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




