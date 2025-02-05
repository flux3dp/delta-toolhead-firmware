#include <stdint.h>
#define HeaterGPIO	GPIOB
#define HeaterPin	GPIO_Pin_13

#define Extruder_One_ADC_Min_Value 3212
#define Laser_ADC_Min_Value 2409
#define Extruder_One_Rev1_ADC_Min_Value 1945
#define Extruder_One_Rev2_ADC_Min_Value 4000
#define Alarm_On() GPIO_ResetBits(GPIOB,GPIO_Pin_0)
#define Alarm_Off() GPIO_SetBits(GPIOB,GPIO_Pin_0)

void Module_Initial(void);
void Module_Recognition(void);
void ADC_Config(void);
void Uart1_Config(void);
void Uart1_ISR_Enable(void);
void Uart1_ISR_Disable(void);
void Fan_Inhalation_RPM_IO_Config(void);
void Fan_Exhalation_RPM_IO_Config(void);
void Fan_Exhalation_Config(void);
void Fan_Inhalation_Config(void);
void Heater_Config(void);
void Laser_Switch_Config(void);
void NVIC_Configuration(void);
void Gerneral_Timer_Config(void);
void IWDG_Configuration(void);
void Alarm_IO_Config(void);
void Self_Test_IO_Config(void);
//void Thermistor_ADC_Config();
void Interlock_IO_Config(void);
void Interlock_Exti_Config(void);
void Interlock_Exti_Break(void);

