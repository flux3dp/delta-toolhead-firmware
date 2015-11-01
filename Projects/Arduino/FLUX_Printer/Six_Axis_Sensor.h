#include "gpio.h"

#define I2C_Timeout 5 			//5 millisecond

typedef enum
{
	Mems_Initial_Failed,
	Mems_Initial_Ok
}Six_Axis_Sensor_State_Type;

Six_Axis_Sensor_State_Type Six_Axis_Sensor_Initial();
void Show_Sensor_RawData();
void Six_Axis_Sensor_Calibration();
void Detect_Gyro_Max();
void Detect_Gyro_Tilt();
