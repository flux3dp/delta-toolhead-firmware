#include "Six_Axis_Sensor.h"
#include "lis302dl.h"
#include <stdio.h>
#include "utilities.h"
#include "configuration.h"
#include "command.h"
#include "defines.h"
#include "LaserModule.h"
#include <math.h>

//for detecting harm postures
static uint32_t Last_Detect_Time=0;
static uint16_t Tilt_Trigger_Count=0;
static uint16_t Shake_Trigger_Count=0;
static uint32_t Trigger_Interval=0;
static uint8_t Gyro_Value_Count=0;

//for Agle_Displacement
static float X_Angle=0.0,Y_Angle=0.0,Z_Angle=0.0;

//for gyro calibration 
static uint8_t Gyro_Calibration_Count=Gyro_Calibration_Times;
static double Z_Angle_Sum=0;
static float Z_Angle_Max,Z_Angle_Min;
static float X_Angle_Offset=0.0,Y_Angle_Offset=0.0,Z_Angle_Offset=0.0;
static float X_Acc_Offset=0.0,Y_Acc_Offset=0.0,Z_Acc_Offset=0.0;

extern void LSM6DS3_Setup(void);
extern uint8_t LSM6DS3_RegRead(uint8_t reg);
extern void LSM6DS3_RegWrite(uint8_t reg, uint8_t data);
extern volatile uint32_t Module_State;
extern volatile bool Debug_Mode;
extern ModuleMode_Type ModuleMode;

Six_Axis_Sensor_State_Type Six_Axis_Sensor_Initial(void)
{
	uint8_t reg=0;
	float Temp_Value;
	LSM6DS3_Setup();	   
	
	Set_Module_State(SENSOR_FAILURE);
	reg = LSM6DS3_RegRead(WHO_AM_I);  

	if(reg == 0x69) //0x3B is LIS302DL , 0x69 is LSM6DS3
	{  
		//Anti-aliasing filter bandwidth selection:	400 Hz
		//Accelerometer full-scale selection:		+-2g
		//Output data rate and power mode selection:416 Hz (high performance)				
		LSM6DS3_RegWrite(CTRL1_XL, 0x60);

		//Gyroscope full-scale at 125 dps:			enabled
		//Gyroscope full-scale selection:			245 dps(not applied)
		//Gyroscope output data rate selection:		416 Hz (high performance)
		LSM6DS3_RegWrite(CTRL2_G, 0x61);

		//Software reset:							normal mode
		//Big/Little Endian Data selection:			data LSB @ lower address
		//Register address automatically incremented
		//during a multiple byte access with a serial
		//interface (I2C or SPI):					enabled
		//SPI Serial Interface Mode selection:		4-wire interface
		//Push-pull/open-drain selection on INT1 and INT2 pads:
		//											push-pull mode
		//Interrupt activation level:				interrupt output pads active high
		//Block Data Update:						continuous update
		//Reboot memory content:					normal mode
		LSM6DS3_RegWrite(CTRL3_C, 0x04);
		
		//bug? recevie a large value on starting
		Read_Axis_Value(Acceler_X);
		Read_Axis_Value(Acceler_Y);
		Read_Axis_Value(Acceler_Z);
		
		Read_Axis_Value(Gyro_X);
		Read_Axis_Value(Gyro_Y);
		Read_Axis_Value(Gyro_Z);
		
		printf("Gyro x=%lf\n",Read_Axis_Value(Gyro_X));
		printf("Gyro y=%lf\n",Read_Axis_Value(Gyro_Y));
		printf("Gyro z=%lf\n",Read_Axis_Value(Gyro_Z));
		printf("Gyro x=%lf\n",Read_Axis_Value(Gyro_X));
		printf("Gyro y=%lf\n",Read_Axis_Value(Gyro_Y));
		printf("Gyro z=%lf\n",Read_Axis_Value(Gyro_Z));
		printf("Gyro x=%lf\n",Read_Axis_Value(Gyro_X));
		printf("Gyro y=%lf\n",Read_Axis_Value(Gyro_Y));
		printf("Gyro z=%lf\n",Read_Axis_Value(Gyro_Z));
		printf("Gyro x=%lf\n",Read_Axis_Value(Gyro_X));
		printf("Gyro y=%lf\n",Read_Axis_Value(Gyro_Y));
		printf("Gyro z=%lf\n",Read_Axis_Value(Gyro_Z));

		Temp_Value=ABS_F(Read_Axis_Value(Acceler_X));
		if(Temp_Value<0.0001 )//cannot read Accelero x value
		{
			printf("Acc x=%lf\n",Temp_Value);
			return Mems_Initial_Failed;
		}
			
		Temp_Value=ABS_F(Read_Axis_Value(Acceler_Y));
		if(Temp_Value<0.0001 )//cannot read Accelero y value 
		{
			printf("Acc y=%lf\n",Temp_Value);
			return Mems_Initial_Failed;
		}
		Temp_Value=ABS_F(Read_Axis_Value(Acceler_Z));
		if(Temp_Value<0.0001 )//cannot read Accelero z value
		{
			printf("Acc z=%lf\n",Temp_Value);
			return Mems_Initial_Failed;
		}
		Temp_Value=ABS_F(Read_Axis_Value(Gyro_X));
		if(Temp_Value<0.0001 || Temp_Value>50000)//cannot read Gyro x value or out of zero-rate range
		{
			printf("Gyro x=%lf\n",Temp_Value);
			return Mems_Initial_Failed;
		}
		Temp_Value=ABS_F(Read_Axis_Value(Gyro_Y));
		if(Temp_Value<0.0001 || Temp_Value>50000)//cannot read Gyro y value or out of zero-rate range
		{
			printf("Gyro y=%lf\n",Temp_Value);
			return Mems_Initial_Failed;
		}
		Temp_Value=ABS_F(Read_Axis_Value(Gyro_Z));
		if(Temp_Value<0.0001 || Temp_Value>50000)//cannot read Gyro z value or out of zero-rate range
		{
			printf("Gyro z=%lf\n",Temp_Value);
			return Mems_Initial_Failed;
		}

		Reset_Module_State(SENSOR_FAILURE);
		return Mems_Initial_Ok;
	}
	else
	{
		Set_Module_State(SENSOR_FAILURE);
		return Mems_Initial_Failed;
	}
}	

float Read_Axis_Value(Six_Axis_Value_Type axis){
	short Axis_Data=0,i;
	float Axis_Value=0.0;
	uint8_t Count=10;
	switch(axis){
			case Acceler_X:
				Axis_Data |= LSM6DS3_RegRead(LSM6DS3_REG_OUTX_L);
				Axis_Data |= LSM6DS3_RegRead(LSM6DS3_REG_OUTX_H)<<8;
				Axis_Value += (float)Axis_Data *0.061+X_Acc_Offset;
				Count=1;
				break;
			case Acceler_Y:
				Axis_Data |= LSM6DS3_RegRead(LSM6DS3_REG_OUTY_L);
				Axis_Data |= LSM6DS3_RegRead(LSM6DS3_REG_OUTY_H)<<8;
				Axis_Value += (float)Axis_Data *0.061+Y_Acc_Offset;
				Count=1;
				break;
			case Acceler_Z:
				Axis_Data |= LSM6DS3_RegRead(LSM6DS3_REG_OUTZ_L);
				Axis_Data |= LSM6DS3_RegRead(LSM6DS3_REG_OUTZ_H)<<8;
				Axis_Value += (float)Axis_Data *0.061+Z_Acc_Offset;
				Count=1;
				break;
	}
	for(i=0;i<Count;i++){
		switch(axis){

			case Gyro_X:
				Axis_Data |= LSM6DS3_RegRead(OUTX_L_G);
				Axis_Data |= LSM6DS3_RegRead(OUTX_H_G)<<8;
				Axis_Value += (float)Axis_Data *4.375+X_Angle_Offset;
				break;
			case Gyro_Y:
				Axis_Data |= LSM6DS3_RegRead(OUTY_L_G);
				Axis_Data |= LSM6DS3_RegRead(OUTY_H_G)<<8;
				Axis_Value += (float)Axis_Data *4.375+Y_Angle_Offset;
				break;
			case Gyro_Z:
				Axis_Data |= LSM6DS3_RegRead(OUTZ_L_G);
				Axis_Data |= LSM6DS3_RegRead(OUTZ_H_G)<<8;
				Axis_Value += (float)Axis_Data *4.375+Z_Angle_Offset;
				break;
		}
	}

	return Axis_Value/Count;
}

void Six_Axis_Sensor_Calibration(void){
	
	float X_Acc_Value=0,Y_Acc_Value=0,Z_Acc_Value=0;
	float Z_Gyro_Value=0;
	uint8_t i;
	const uint8_t Count=5;
	if(!Gyro_Calibration_Count)//count<=0
		return;
	for(i=0;i<Count;i++){
		X_Acc_Value+=Read_Axis_Value(Acceler_X);
		Y_Acc_Value+=Read_Axis_Value(Acceler_Y);
		Z_Acc_Value+=Read_Axis_Value(Acceler_Z);
	}
	
	X_Acc_Value=ABS_F(X_Acc_Value/Count);
	Y_Acc_Value=ABS_F(Y_Acc_Value/Count);
	Z_Acc_Value=ABS_F(Z_Acc_Value/Count);
	
	//printf("acc X'=%.4f Y'=%.4f Z'=%.4f\n",ABS_F(X_Acc_Value),ABS_F(Y_Acc_Value),ABS_F(Z_Acc_Value));
	
		
	if(X_Acc_Value<=50 && Y_Acc_Value<=50 && Z_Acc_Value<=1050 && Z_Acc_Value>=950){ //According to datasheet zero-g = +-40mg
		Z_Gyro_Value=Read_Axis_Value(Gyro_Z);
		if(Gyro_Calibration_Count==Gyro_Calibration_Times)
			Z_Angle_Max=Z_Angle_Min=Z_Gyro_Value;
		if(Z_Angle_Max<Z_Gyro_Value)
			Z_Angle_Max=Z_Gyro_Value;
		if(Z_Angle_Min>Z_Gyro_Value)
			Z_Angle_Min=Z_Gyro_Value;
		Z_Angle_Sum+=Z_Gyro_Value;
		Gyro_Calibration_Count--;
		if(!Gyro_Calibration_Count){
			if(ABS_F(Z_Angle_Max-Z_Angle_Min)<200.0){
				Z_Angle_Offset=(float)(-Z_Angle_Sum/Gyro_Calibration_Times);
				
				Reset_Module_State(SENSOR_CALIBRATION_FAILURE);		
			}else{
				Gyro_Calibration_Count=Gyro_Calibration_Times;
				Z_Angle_Sum=0;
			}				
		}
	}else{
		return;
	}

	
}

//void Six_Axis_Sensor_Calibration(void){
//	double X_Angle_Sum=0,Y_Angle_Sum=0,Z_Angle_Sum=0;
//	double X_Acc_Sum=0,Y_Acc_Sum=0,Z_Acc_Sum=0;
//	uint8_t i;
//	const uint8_t Count=100;
//	for(i=0;i<Count;i++){
//		X_Acc_Sum+=Read_Axis_Value(Acceler_X);
//		Y_Acc_Sum+=Read_Axis_Value(Acceler_Y);
//		Z_Acc_Sum+=Read_Axis_Value(Acceler_Z);
//		
//		X_Angle_Sum+=Read_Axis_Value(Gyro_X);
//		Y_Angle_Sum+=Read_Axis_Value(Gyro_Y);
//		Z_Angle_Sum+=Read_Axis_Value(Gyro_Z);
//	}
//	X_Acc_Offset=(float)(-X_Acc_Sum/Count);
//	Y_Acc_Offset=(float)(-Y_Acc_Sum/Count);
//	Z_Acc_Offset=(float)(-Z_Acc_Sum/Count)+1000.0;
//	
//	X_Angle_Offset=(float)(-X_Angle_Sum/Count);
//	Y_Angle_Offset=(float)(-Y_Angle_Sum/Count);
//	Z_Angle_Offset=(float)(-Z_Angle_Sum/Count);
//	
//	//printf("X'=%.4f Y'=%.4f Z'=%.4f , ",X_Acc_Offset,Y_Acc_Offset,Z_Acc_Offset);
//	//printf("X'=%.4f Y'=%.4f Z'=%.4f\n",X_Angle_Offset,Y_Angle_Offset,Z_Angle_Offset);
//}

void Show_Sensor_Msg(void){
	printf("Max=%lf Min=%lf\n",Z_Angle_Max,Z_Angle_Min);
	printf("offset X'=%.4f Y'=%.4f Z'=%.4f\n",X_Angle_Offset,Y_Angle_Offset,Z_Angle_Offset);
}

void Show_Sensor_RawData(void){
	float Gx,Gy,Gz;
	
	printf("%.4f\t\t",Read_Axis_Value(Acceler_X));          		
	printf("%.4f\t\t",Read_Axis_Value(Acceler_Y));            
	printf("%.4f\t\t",Read_Axis_Value(Acceler_Z));
	
	Gx=Read_Axis_Value(Gyro_X);
	Gy=Read_Axis_Value(Gyro_Y);
	Gz=Read_Axis_Value(Gyro_Z);
	
	printf("%.4f\t\t",Gx);
	printf("%.4f\t\t",Gy);
	printf("%.4f",Gz);

	printf("\n");

}

void Show_Agle_Displacement(void){
	uint32_t Dt=millis()-Last_Detect_Time;
	if(Dt < Detect_Priod_Time)
		return;
	Last_Detect_Time=millis();

	X_Angle+=(Read_Axis_Value(Gyro_X))*Dt/1000;
	Y_Angle+=(Read_Axis_Value(Gyro_Y))*Dt/1000;
	Z_Angle+=(Read_Axis_Value(Gyro_Z))*Dt/1000;

	printf("X=%.4lf  Y=%.4lf  Z=%.4lf\n",X_Angle,Y_Angle,Z_Angle);

}

void Detect_Gyro_Harm_Posture(void){
	float Gyro_Z_Value;

	Gyro_Z_Value=Read_Axis_Value(Gyro_Z);
	
	//detect shake
	if(Gyro_Z_Value>=Gyro_Shake_Alarm_Value || Gyro_Z_Value<=-Gyro_Shake_Alarm_Value){
		if(ModuleMode==FLUX_LASER_MODULE && !Debug_Mode)
			Laser_Switch_Off();
		
		Shake_Trigger_Count++;
		if(Shake_Trigger_Count>=10){
			Set_Module_State(SHAKE);
			Alarm_On();
			Shake_Trigger_Count=0;
			if(Debug_Mode){
				printf("Shake ");           
				printf("%.2f\t\t",Gyro_Z_Value);
				printf("\n");
			}	
		}
		
	}

	//detect tilt
	if(Trigger_Interval>=1)
		Trigger_Interval++;
	
	if(Gyro_Z_Value >= Gyro_Tilt_Alarm_Value || Gyro_Z_Value <= -Gyro_Tilt_Alarm_Value){
		Trigger_Interval++;
		Tilt_Trigger_Count++;
	}
		
	if(Tilt_Trigger_Count>=4){
		if(ModuleMode==FLUX_LASER_MODULE && !Debug_Mode)
			Laser_Switch_Off();

		Trigger_Interval=0;
		Tilt_Trigger_Count=0;
		Set_Module_State(TILT);
		Alarm_On();
		
		if(Debug_Mode){
			printf("Tilt ");           
			printf("%.2f",Gyro_Z_Value);
			printf("\n");
		}
	}
	
	if(Trigger_Interval>51){
		Trigger_Interval=0;
		Tilt_Trigger_Count=0;
		Shake_Trigger_Count=0;
	}

}

void Reset_Axis_Sensor_State(void){
	Reset_Module_State(SHAKE);
	Reset_Module_State(TILT);
}
