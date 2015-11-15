#include <stdint.h>
#define HeaterGPIO	GPIOB
#define HeaterPin	GPIO_Pin_13

#define Heater_ADC_Min_Value 3212
#define Laser_ADC_Min_Value 2409

#define Alarm_On() GPIO_ResetBits(GPIOB,GPIO_Pin_0)
#define Alarm_Off() GPIO_SetBits(GPIOB,GPIO_Pin_0)

void Module_Initial(void);
void Module_Recognition(void);
void ADC_Config(void);
void Uart1_Config(void);
void Uart1_ISR_Enable(void);
void Fan_Inhalation_RPM_IO_Config(void);
void Fan_Exhalation_Config(void);
void Fan_Inhalation_Config(void);
void Heater_Config(void);
void ID_Initial(void);
void Laser_Switch_Config(void);
void NVIC_Configuration(void);
void Gerneral_Timer_Config(void);
void IWDG_Configuration(void);
void Alarm_IO_Config(void);
//void Thermistor_ADC_Config();
