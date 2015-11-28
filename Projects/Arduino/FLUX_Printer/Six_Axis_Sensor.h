#include "gpio.h"

#define I2C_Timeout 2 			//2 millisecond
#define Detect_Priod_Time 25
#define Gyro_Shake_Alarm_Value 10000.0
#define Gyro_Tilt_Alarm_Value 4500.0
#define Gyro_Tilt_Count 10
#define Gyro_Calibration_Times 20
typedef enum
{
	Mems_Initial_Failed,
	Mems_Initial_Ok
}Six_Axis_Sensor_State_Type;

typedef enum
{
	Gyro_X,
	Gyro_Y,
	Gyro_Z,
	Acceler_X,
	Acceler_Y,
	Acceler_Z
}Six_Axis_Value_Type;

Six_Axis_Sensor_State_Type Six_Axis_Sensor_Initial(void);
void Show_Sensor_Msg(void);
void Show_Sensor_RawData(void);
void Six_Axis_Sensor_Calibration(void);
void Detect_Gyro_Shake(void);
void Detect_Gyro_Tilt(void);
float Read_Axis_Value(Six_Axis_Value_Type axis);
void Reset_Axis_Sensor_State(void);
void Detect_Gyro_Harm_Posture(void);
void Show_Agle_Displacement(void);
