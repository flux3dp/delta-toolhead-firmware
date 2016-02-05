#include <stdint.h>
#include "stm32f0xx_gpio.h"
#include "gpio.h"
#include "configuration.h"

#define FLASH_PAGE_SIZE         ((uint32_t)0x00000800)   /* FLASH Page Size */
#define FLASH_USER_START_ADDR   ((uint32_t)0x08000800)   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ((uint32_t)0x0803F800)   /* End @ of user Flash area */


#define Extruder_Cmd_Timeout 300000
#define Laser_Cmd_Timeout 2000
#define UUID STM32F0_UUID

typedef enum
{
	Temperature_Channel,
	NTC_Channel,
	ID0_Channel,
	ID1_Channel
}ADC_Channel_Type;

typedef enum
{
	STATE_OK			=0,
	UNKNOW_MODULE		=1,
	SENSOR_FAILURE		=2,
	NO_HELLO			=4,
	SENSOR_CALIBRATION_FAILURE=8,
	SHAKE				=16,
	TILT				=32,
	PID_OUT_OF_CONTROL	=64,
	FAN_FAILURE			=128,
	LASER_DOWN			=256,
}Module_State_Enum;

//typedef struct
//{
//	bool Unknow_Module;
//	bool Sensor_Failure;
//	bool No_Hello;
//	bool Shake;
//	bool Tilt;
//	bool PID_Out_Of_Control;
//	bool Fan_Failure;
//}Module_State_Type;

void Xcode_Handler(void);

bool Write_Focal_Length(float Focal_Length);

float Read_Focal_Length(void);

bool IsNumber(char *NumberString);

void CommandTimeoutDetection(void);

uint16_t Read_ADC_Value(ADC_Channel_Type channel);//return 0~4095 adc value

void Module_State_Initial(void);

uint32_t * Get_UUID(void);

void Set_Module_State(Module_State_Enum state);

void Reset_Module_State(Module_State_Enum state);

bool Cmd_Checksum_Validation(char * Cmd_Data,uint16_t Length);

bool Ascii_Checksum_Compare(char * str_Data,uint8_t checksum);

bool Read_Self_Test_IO(void);

void Self_Test(void);



