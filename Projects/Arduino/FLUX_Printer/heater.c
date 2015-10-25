#include "heater.h"
#include "stm32f0xx.h"
#include <stdio.h>
HeaterState_Type Heater_State;
//__IO uint16_t  ADC1ConvertedValue = 0, ADC1ConvertedVoltage = 0;

uint16_t GetTemperature_ADCValue(){
		uint8_t i;
		uint16_t ADC_Value=0;

			//discard first sampling
			while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
			ADC_GetConversionValue(ADC1);
	
			/* Test EOC flag */
			while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
			/* Get ADC1 converted data */
			ADC_Value = ADC_GetConversionValue(ADC1);
	
    /* Compute the voltage */
//    ADC1ConvertedVoltage = (ADC1ConvertedValue *3300)/0xFFF;
//		v=(ADC1ConvertedVoltage)/1000;
//		mv = (ADC1ConvertedVoltage%1000)/100;
//		printf("   ADC = %d,%d V   ,value=%d",v,mv,ADC1ConvertedValue);
		
		//avgValue=(avgValue/10);
		printf("value,%d",ADC_Value);
		
		return ADC_Value;
}

