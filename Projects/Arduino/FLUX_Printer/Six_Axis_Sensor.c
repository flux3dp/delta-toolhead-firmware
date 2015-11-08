#include "Six_Axis_Sensor.h"
#include "lis302dl.h"
#include <stdio.h>
#include "utilities.h"
#include "configuration.h"
#include "command.h"
#include "defines.h"
#include "LaserModule.h"
float Calibration_X=0,Calibration_Y=0,Calibration_Z=0;

uint8_t Tilt_Count=0;
Six_Axis_Sensor_State_Type Six_Axis_Sensor_State = Mems_Initial_Failed;
volatile uint32_t SensorTime_Count=0;
static uint16_t Sensor_Trigger_Count=0;
static uint32_t Trigger_Interval=0;
extern void LSM6DS3_Setup(void);
extern uint8_t LSM6DS3_RegRead(uint8_t reg);
extern void LSM6DS3_RegWrite(uint8_t reg, uint8_t data);
extern volatile uint32_t Module_State;
extern volatile bool Debug_Mode;
extern ModuleMode_Type ModuleMode;

Six_Axis_Sensor_State_Type Six_Axis_Sensor_Initial(void)
{
	uint8_t reg=0;
	LSM6DS3_Setup();	   
	
	reg = LSM6DS3_RegRead(WHO_AM_I);  

	if(reg == 0x69) //0x3B is LIS302DL , 0x69 is LSM6DS3
	{  
		LSM6DS3_RegWrite(CTRL1_XL, 0x60); //60 2g

		LSM6DS3_RegWrite(CTRL2_G, 0x60);

		LSM6DS3_RegWrite(CTRL3_C, 0x04);
		
		//bug?
		ABS_F(Read_Axis_Value(Gyro_X));
		ABS_F(Read_Axis_Value(Gyro_Y));
		ABS_F(Read_Axis_Value(Gyro_Z));

		if(ABS_F(Read_Axis_Value(Gyro_X))<0.00001)//cannot read Gyro x value
			return Mems_Initial_Failed;
		if(ABS_F(Read_Axis_Value(Gyro_Y))<0.00001)//cannot read Gyro y value
			return Mems_Initial_Failed;
		if(ABS_F(Read_Axis_Value(Gyro_Z)<0.00001))//cannot read Gyro z value
			return Mems_Initial_Failed;
		
		return Mems_Initial_Ok;
	}
	else
	{
		Set_Module_State(SENSOR_FAILURE);
		return Mems_Initial_Failed;
	}
}	

float Read_Axis_Value(Six_Axis_Value_Type axis){
	uint8_t Axis_Data[2];
	float Axis_Value=0.0;
	
	switch(axis){
		case Acceler_X:
			Axis_Data[0] = LSM6DS3_RegRead(LSM6DS3_REG_OUTX_L);
			Axis_Data[1] = LSM6DS3_RegRead(LSM6DS3_REG_OUTX_H);
			break;
		case Acceler_Y:
			Axis_Data[0] = LSM6DS3_RegRead(LSM6DS3_REG_OUTY_L);
			Axis_Data[1] = LSM6DS3_RegRead(LSM6DS3_REG_OUTY_H);
			break;
		case Acceler_Z:
			Axis_Data[0] = LSM6DS3_RegRead(LSM6DS3_REG_OUTZ_L);
			Axis_Data[1] = LSM6DS3_RegRead(LSM6DS3_REG_OUTZ_H);
			break;
		case Gyro_X:
			Axis_Data[0] = LSM6DS3_RegRead(OUTX_L_G);
			Axis_Data[1] = LSM6DS3_RegRead(OUTX_H_G);
			break;
		case Gyro_Y:
			Axis_Data[0] = LSM6DS3_RegRead(OUTY_L_G);
			Axis_Data[1] = LSM6DS3_RegRead(OUTY_H_G);
			break;
		case Gyro_Z:
			Axis_Data[0] = LSM6DS3_RegRead(OUTZ_L_G);
			Axis_Data[1] = LSM6DS3_RegRead(OUTZ_H_G);
			break;
		
	}
	Axis_Value=(((short)(Axis_Data[1] << 8)) + Axis_Data[0]) *4*9.8/ 65535;
	return Axis_Value;
}

void Six_Axis_Sensor_Calibration(void){
	float Acceler_Value;
	double Acceler_SUM;
	uint8_t i;
	//calibration X			
	Acceler_SUM=0;
	Acceler_Value=Read_Axis_Value(Acceler_X);
	if((Acceler_Value+Calibration_X) > 0 || (Acceler_Value+Calibration_X) < 0)
	{								
		for(i=0;i<100;i++)
		{
			Acceler_SUM+=Read_Axis_Value(Acceler_X);													
		}

		Acceler_Value = Acceler_SUM/100;
		Calibration_X = -Acceler_Value;
		printf(" kx : %.2f",Calibration_X);
		printf("\r\n");
	}
	
	//calibration Y
	Acceler_SUM=0;
	Acceler_Value=Read_Axis_Value(Acceler_Y);
	if((Acceler_Value+Calibration_Y) > 0 || (Acceler_Value+Calibration_Y) < 0)
	{								
		for(i=0;i<100;i++)
		{
			Acceler_SUM+=Read_Axis_Value(Acceler_Y);													
		}

		Acceler_Value = Acceler_SUM/100;
		Calibration_Y = -Acceler_Value;
		printf(" ky : %.2f",Calibration_Y);
		printf("\r\n");
	}
	
	//calibration Z	
	Acceler_SUM=0;	
	Acceler_Value=Read_Axis_Value(Acceler_Z);
	if((Acceler_Value+Calibration_Z) > 0 || (Acceler_Value+Calibration_Z) < 0)
	{								
		for(i=0;i<100;i++)
		{
			Acceler_SUM+=Read_Axis_Value(Acceler_Z);													
		}

		Acceler_Value = Acceler_SUM/100;
		Calibration_Z = -Acceler_Value;
		printf(" kz : %.2f",Calibration_Z);
		printf("\r\n");
	}
}

void Show_Sensor_RawData(void){

	printf("%.4f ",Read_Axis_Value(Gyro_X));          		
	printf("%.4f ",Read_Axis_Value(Gyro_Y));            
	printf("%.4f",Read_Axis_Value(Gyro_Z));
	printf("\n");

}

void Detect_Gyro_Shake(void){
	float Gyro_X_Value,Gyro_Y_Value,Gyro_Z_Value;
	Gyro_X_Value=Read_Axis_Value(Gyro_X);
	Gyro_Y_Value=Read_Axis_Value(Gyro_Y);
	Gyro_Z_Value=Read_Axis_Value(Gyro_Z);
	if(Gyro_X_Value>=1.8 || Gyro_X_Value<=-1.8 || Gyro_Y_Value>=1.8 || Gyro_Y_Value<=-1.8 || Gyro_Z_Value>=1.8 || Gyro_Z_Value<=-1.8){
		if(ModuleMode==FLUX_LASER_MODULE)
			Laser_Switch_Off();
		
		if(Debug_Mode){
			printf("Shake ");
			printf("%.2f ",Gyro_X_Value);          		
			printf("%.2f ",Gyro_Y_Value);            
			printf("%.2f",Gyro_Z_Value);
			printf("\n");
		}	
		
		Set_Module_State(SHAKE);
		Alarm_On();
	}
	
}

void Detect_Gyro_Tilt(void){
	float Gyro_Z_Value;
	Gyro_Z_Value=Read_Axis_Value(Gyro_Z);

	if(Trigger_Interval>=1)
		Trigger_Interval++;
	
	if(Gyro_Z_Value >= 0.4 || Gyro_Z_Value <= -0.4){
		Trigger_Interval++;
		Sensor_Trigger_Count++;
		
	}
		
	if(Sensor_Trigger_Count>=4){
		if(ModuleMode==FLUX_LASER_MODULE)
			Laser_Switch_Off();
		
		if(Debug_Mode){
			printf("Tilt");           
			printf("%.2f",Gyro_Z_Value);
			printf("\n");
		}
		Trigger_Interval=0;
		Sensor_Trigger_Count=0;
		Set_Module_State(TILT);
		Alarm_On();
	}
	
	if(Trigger_Interval>51){
		Trigger_Interval=0;
		Sensor_Trigger_Count=0;
	}
	
	SensorTime_Count=0;
}

void Reset_Axis_Sensor_State(void){
	Reset_Module_State(SHAKE);
	Reset_Module_State(TILT);
}
