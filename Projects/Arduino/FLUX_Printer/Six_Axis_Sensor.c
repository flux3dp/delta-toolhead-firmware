#include "Six_Axis_Sensor.h"
#include "lis302dl.h"
#include <stdio.h>
#include "utilities.h"
#include "configuration.h"
#include "command.h"
#include "defines.h"
#include "LaserModule.h"
#include <math.h>
float Calibration_X=0,Calibration_Y=0,Calibration_Z=0;

uint8_t Tilt_Count=0;
Six_Axis_Sensor_State_Type Six_Axis_Sensor_State = Mems_Initial_Failed;

static uint32_t Last_Detect_Time=0;
static uint16_t Tilt_Trigger_Count=0;
static uint16_t Shake_Trigger_Count=0;
static uint32_t Trigger_Interval=0;
static uint8_t Gyro_Value_Count=0;
static float Gyro_Value_Buffer[Gyro_Tilt_Count]={0};
static float X_Angle=0.0,Y_Angle=0.0,Z_Angle=0.0;
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
	LSM6DS3_Setup();	   
	
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
	double X_Angle_Sum=0,Y_Angle_Sum=0,Z_Angle_Sum=0;
	double X_Acc_Sum=0,Y_Acc_Sum=0,Z_Acc_Sum=0;
	uint8_t i;
	const uint8_t Count=100;
	for(i=0;i<Count;i++){
		X_Acc_Sum+=Read_Axis_Value(Acceler_X);
		Y_Acc_Sum+=Read_Axis_Value(Acceler_Y);
		Z_Acc_Sum+=Read_Axis_Value(Acceler_Z);
		
		X_Angle_Sum+=Read_Axis_Value(Gyro_X);
		Y_Angle_Sum+=Read_Axis_Value(Gyro_Y);
		Z_Angle_Sum+=Read_Axis_Value(Gyro_Z);
	}
	X_Acc_Offset=(float)(-X_Acc_Sum/Count);
	Y_Acc_Offset=(float)(-Y_Acc_Sum/Count);
	Z_Acc_Offset=(float)(-Z_Acc_Sum/Count)+1000.0;
	
	X_Angle_Offset=(float)(-X_Angle_Sum/Count);
	Y_Angle_Offset=(float)(-Y_Angle_Sum/Count);
	Z_Angle_Offset=(float)(-Z_Angle_Sum/Count);
	
	//printf("X'=%.4f Y'=%.4f Z'=%.4f , ",X_Acc_Offset,Y_Acc_Offset,Z_Acc_Offset);
	//printf("X'=%.4f Y'=%.4f Z'=%.4f\n",X_Angle_Offset,Y_Angle_Offset,Z_Angle_Offset);
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
//	printf("%s%.4f\t\t",Gx>0.1?" ":"",Gx);          		
//	printf("%s%.4f\t\t",Gy>0.1?" ":"",Gy);            
//	printf("%s%.4f",Gz>0?" ":"",Gz); 

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

void Detect_Gyro_Shake(void){
	float Gyro_X_Value,Gyro_Y_Value,Gyro_Z_Value;
	Gyro_X_Value=Read_Axis_Value(Gyro_X);
	Gyro_Y_Value=Read_Axis_Value(Gyro_Y);
	Gyro_Z_Value=Read_Axis_Value(Gyro_Z);
	if(Gyro_X_Value>=Gyro_Shake_Alarm_Value || Gyro_X_Value<=-Gyro_Shake_Alarm_Value || Gyro_Y_Value>=Gyro_Shake_Alarm_Value || Gyro_Y_Value<=-Gyro_Shake_Alarm_Value || Gyro_Z_Value>=Gyro_Shake_Alarm_Value || Gyro_Z_Value<=-Gyro_Shake_Alarm_Value){
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
	
	if(Gyro_Z_Value >= Gyro_Tilt_Alarm_Value || Gyro_Z_Value <= -Gyro_Tilt_Alarm_Value){
		Trigger_Interval=1;
		Tilt_Trigger_Count++;
	}
		
	if(Tilt_Trigger_Count>=4){
		if(ModuleMode==FLUX_LASER_MODULE)
			Laser_Switch_Off();
		
		if(Debug_Mode){
			printf("Tilt");           
			printf("%.2f",Gyro_Z_Value);
			printf("\n");
		}
		Trigger_Interval=0;
		Tilt_Trigger_Count=0;
		Set_Module_State(TILT);
		Alarm_On();
	}
	
	if(Trigger_Interval>51){
		Trigger_Interval=0;
		Tilt_Trigger_Count=0;
	}

}

void Detect_Gyro_Harm_Posture(void){
	float Gyro_X_Value,Gyro_Y_Value,Gyro_Z_Value;
	//Gyro_X_Value=Read_Axis_Value(Gyro_X);
	//Gyro_Y_Value=Read_Axis_Value(Gyro_Y);
	Gyro_Z_Value=Read_Axis_Value(Gyro_Z);
	
	//detect shake
//	if(Gyro_X_Value>=Gyro_Shake_Alarm_Value || Gyro_X_Value<=-Gyro_Shake_Alarm_Value || Gyro_Y_Value>=Gyro_Shake_Alarm_Value || Gyro_Y_Value<=-Gyro_Shake_Alarm_Value || Gyro_Z_Value>=Gyro_Shake_Alarm_Value || Gyro_Z_Value<=-Gyro_Shake_Alarm_Value){
	if(Gyro_Z_Value>=Gyro_Shake_Alarm_Value || Gyro_Z_Value<=-Gyro_Shake_Alarm_Value){
		if(ModuleMode==FLUX_LASER_MODULE)
			Laser_Switch_Off();
		
		if(Debug_Mode){
			printf("Shake ");
			//printf("%.2f\t\t",Gyro_X_Value);          		
			//printf("%.2f\t\t",Gyro_Y_Value);            
			printf("%.2f\t\t",Gyro_Z_Value);
			printf("\n");
		}	
		
		Shake_Trigger_Count++;
		if(Shake_Trigger_Count>=10){
			Set_Module_State(SHAKE);
			Alarm_On();
			Shake_Trigger_Count=0;
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
		if(ModuleMode==FLUX_LASER_MODULE)
			Laser_Switch_Off();
		
		if(Debug_Mode){
			printf("Tilt ");           
			printf("%.2f",Gyro_Z_Value);
			printf("\n");
		}
		Trigger_Interval=0;
		Tilt_Trigger_Count=0;
		Set_Module_State(TILT);
		Alarm_On();
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

// --- Kalman filter module  ----------------------------------------------------------------------------

    float Q_angle  =  0.001; //0.001
    float Q_gyro   =  0.003;  //0.003
    float R_angle  =  0.03;  //0.03

    float x_angle = 0;
    float x_bias = 0;
    float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
    float dt, y, S;
    float K_0, K_1;

float kalmanCalculate(float newAngle, float newRate,int looptime) {
	dt = (float)(looptime)/1000;
	x_angle += dt * (newRate - x_bias);
	P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
	P_01 +=  - dt * P_11;
	P_10 +=  - dt * P_11;
	P_11 +=  + Q_gyro * dt;

	y = newAngle - x_angle;
	S = P_00 + R_angle;
	K_0 = P_00 / S;
	K_1 = P_10 / S;

	x_angle +=  K_0 * y;
	x_bias  +=  K_1 * y;
	P_00 -= K_0 * P_00;
	P_01 -= K_0 * P_01;
	P_10 -= K_1 * P_00;
	P_11 -= K_1 * P_01;

	return x_angle;
}

int getAccAngle(void) {
  return arctan2((int)-Read_Axis_Value(Acceler_Z), (int)-Read_Axis_Value(Acceler_X)) + 256;    // in Quid: 1024/(2*PI))
}

int getGyroRate(void) {                                             // ARef=3.3V, Gyro sensitivity=2mV/(deg/sec)
  return (int)(Read_Axis_Value(Gyro_Y)* 4.583333333);                 // in quid/sec:(1024/360)/1024 * 3.3/0.002)
}
