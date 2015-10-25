#include <stdint.h>
#define HeaterGPIO	GPIOB
#define HeaterPin	GPIO_Pin_13

#define Heater_ADC_Min_Value 3212
#define Laser_ADC_Min_Value 2409

void Module_Initial();
void Module_Recognition();
void ADC_Config();
void UART1_Config();
void Fan1_Config();
void Fan2_Config();
void Heater_Config();
void ID_Initial();
//void Thermistor_ADC_Config();
