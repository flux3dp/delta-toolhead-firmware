#include "gpio.h"

#define I2C_Timeout 2 			//2 millisecond
#define Detect_Priod_Time 25
#define Gyro_Shake_Alarm_Value 10000.0
#define Gyro_Tilt_Alarm_Value 5000.0
#define Gyro_Tilt_Count 10
#define Gyro_Calibration_Times 20

#define Q_angle   0.001 //0.001
#define Q_gyro    0.003 //0.003
#define R_angle   0.1 //17.19 //0.03 bigger is more stable 
#define Tilt_Alarm_Degree 6.0
#define Shake_Alarm_Degree 8.0
#define Average_Moving_Time 5000
#define Kalman_Loop_Time 13 //57
#define Sample_Degree_Limit 12.0
#define Data_Amount (int)(Average_Moving_Time/Kalman_Loop_Time)
#define Data_Weight (float)1/(float)Data_Amount
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

typedef enum
{
	Angle_X,
    Angle_Y
}Angle_Type;

typedef struct
{
    float angle;
    float bias;
    float P[4];
    float K[2];
}Kalman_Data_Struct;

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
float Get_Kalman_Data(int looptime);
float kalmanCalculate(float X_newAngle, float X_newRate,float Y_newAngle, float Y_newRate,int looptime);
//float kalmanCalculate(float newAngle, float newRate,int looptime);
