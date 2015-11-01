#include <stdint.h>
#define HeaterGPIO	GPIOB
#define HeaterPin	GPIO_Pin_13

#define Heater_ADC_Min_Value 3212
#define Laser_ADC_Min_Value 2409

void Module_Initial();
void Module_Recognition();
void ADC_Config();
void Uart1_Config();
void Uart1_ISR_Enable();
void Fan_Exhalation_Config();
void Fan_Inhalation_Config();
void Heater_Config();
void ID_Initial();
void Laser_Switch_Config();
//void Thermistor_ADC_Config();
