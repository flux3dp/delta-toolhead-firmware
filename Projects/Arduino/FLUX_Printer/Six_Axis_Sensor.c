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
static float Kalman_Data[Data_Amount];
static uint8_t Kalman_Data_Count=0;
static bool Averge_First_Time=TRUE;
static float Degree_Moving_Avg=0;
float Degree_Now=0;
static uint8_t Tilt_Trigger_Count=0;
static volatile uint32_t Tilt_Last_Time=0;
static uint8_t Shake_Trigger_Count=0;
static volatile uint32_t Shake_Last_Time=0;
static float Predict_X_Error=0,Predict_Y_Error=0;
//for Agle_Displacement
//static float X_Angle=0.0,Y_Angle=0.0,Z_Angle=0.0;

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

//for kalman filter

Kalman_Data_Struct Kal_X={0,0,{0,0,0,0},{0,0}};
Kalman_Data_Struct Kal_Y={0,0,{0,0,0,0},{0,0}};
static uint32_t Kalman_Last_Time=0;

float getGyroRate(Angle_Type angle);
double getAccAngle(Angle_Type angle);

Six_Axis_Sensor_State_Type Six_Axis_Sensor_Initial(void)
{
	uint8_t reg=0,Try_Times=5;
	float Temp_Value;
	
	LSM6DS3_Setup();	   
	
	Set_Module_State(SENSOR_FAILURE);
	 
	
	while(Try_Times--) //0x3B is LIS302DL , 0x69 is LSM6DS3
	{  
		reg = LSM6DS3_RegRead(WHO_AM_I); 
		if(reg != 0x69)
			continue;
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
		
        printf("Acc x=%lf\n",Read_Axis_Value(Acceler_X));
		printf("Acc y=%lf\n",Read_Axis_Value(Acceler_Y));
		printf("Acc z=%lf\n",Read_Axis_Value(Acceler_Z));
		printf("Acc x=%lf\n",Read_Axis_Value(Acceler_X));
		printf("Acc y=%lf\n",Read_Axis_Value(Acceler_Y));
		printf("Acc z=%lf\n",Read_Axis_Value(Acceler_Z));
        
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
			printf("ER Acc x=%lf\n",Temp_Value);
			continue;
			//return Mems_Initial_Failed;
		}
			
		Temp_Value=ABS_F(Read_Axis_Value(Acceler_Y));
		if(Temp_Value<0.0001 )//cannot read Accelero y value 
		{
			printf("ER Acc y=%lf\n",Temp_Value);
			continue;
			//return Mems_Initial_Failed;
		}
		Temp_Value=ABS_F(Read_Axis_Value(Acceler_Z));
		if(Temp_Value<0.0001 )//cannot read Accelero z value
		{
			printf("ER Acc z=%lf\n",Temp_Value);
			continue;
			//return Mems_Initial_Failed;
		}
		Temp_Value=ABS_F(Read_Axis_Value(Gyro_X));
		if(Temp_Value<0.0001 || Temp_Value>100000)//cannot read Gyro x value or out of zero-rate range
		{
			printf("ER Gyro x=%lf\n",Temp_Value);
			continue;
			//return Mems_Initial_Failed;
		}
		Temp_Value=ABS_F(Read_Axis_Value(Gyro_Y));
		if(Temp_Value<0.0001 || Temp_Value>100000)//cannot read Gyro y value or out of zero-rate range
		{
			printf("ER Gyro y=%lf\n",Temp_Value);
			continue;
			//return Mems_Initial_Failed;
		}
		Temp_Value=ABS_F(Read_Axis_Value(Gyro_Z));
		if(Temp_Value<0.0001 || Temp_Value>100000)//cannot read Gyro z value or out of zero-rate range
		{
			printf("ER Gyro z=%lf\n",Temp_Value);
			continue;
			//return Mems_Initial_Failed;
		}

		Reset_Module_State(SENSOR_FAILURE);
		return Mems_Initial_Ok;
	}
	
	Set_Module_State(SENSOR_FAILURE);
	return Mems_Initial_Failed;
}	

/* return mg or (degree/1000)/(s^2) aka mdps */
float Read_Axis_Value(Six_Axis_Value_Type axis){
	short Axis_Data=0,i;
	float Axis_Value=0.0;

    
	uint8_t Count=1;
    for(i=0;i<Count;i++){
        switch(axis){
                case Acceler_X:
                    Axis_Data |= LSM6DS3_RegRead(LSM6DS3_REG_OUTX_L);
                    Axis_Data |= LSM6DS3_RegRead(LSM6DS3_REG_OUTX_H)<<8;
                    Axis_Value += (float)Axis_Data *0.061+X_Acc_Offset;
                    break;
                case Acceler_Y:
                    Axis_Data |= LSM6DS3_RegRead(LSM6DS3_REG_OUTY_L);
                    Axis_Data |= LSM6DS3_RegRead(LSM6DS3_REG_OUTY_H)<<8;
                    Axis_Value += (float)Axis_Data *0.061+Y_Acc_Offset;
                    break;
                case Acceler_Z:
                    Axis_Data |= LSM6DS3_RegRead(LSM6DS3_REG_OUTZ_L);
                    Axis_Data |= LSM6DS3_RegRead(LSM6DS3_REG_OUTZ_H)<<8;
                    Axis_Value += (float)Axis_Data *0.061+Z_Acc_Offset;
                    break;
        }
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
	const uint8_t Count=1;
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
        if(ABS_F(Z_Angle_Max-Z_Angle_Min)<200.0){
            if(!Gyro_Calibration_Count){
				Z_Angle_Offset=(float)(-Z_Angle_Sum/Gyro_Calibration_Times);
				
				Reset_Module_State(SENSOR_CALIBRATION_FAILURE);			
            
					
			}
        }else{
            Gyro_Calibration_Count=Gyro_Calibration_Times;
            Z_Angle_Sum=0;
        }
		
	}else{
		return;
	}	
}

void Show_Sensor_Msg(void){
	printf("Max=%lf Min=%lf\n",Z_Angle_Max,Z_Angle_Min);
	printf("offset X'=%.4f Y'=%.4f Z'=%.4f\n",X_Angle_Offset,Y_Angle_Offset,Z_Angle_Offset);
}

void Show_Sensor_RawData(void){
	float Gx,Gy,Gz;
//	
//	printf("%.4f\t\t",Read_Axis_Value(Acceler_X));          		
//	printf("%.4f\t\t",Read_Axis_Value(Acceler_Y));            
//	printf("%.4f\t\t",Read_Axis_Value(Acceler_Z));
//	
//	Gx=Read_Axis_Value(Gyro_X);
//	Gy=Read_Axis_Value(Gyro_Y);
//	Gz=Read_Axis_Value(Gyro_Z);
//	
//	printf("%.4f\t\t",Gx);
//	printf("%.4f\t\t",Gy);
//	printf("%.4f\n",Gz);

    //printf("Avg=%.2f\tDx=%.2f\tDy=%.2f\tDF=%.2f\n",Degree_Moving_Avg,Kal_X.angle,Kal_Y.angle,Degree_Now);
    printf("x%.2f y%.2f D%.2f\n",Kal_X.angle,Kal_Y.angle,Degree_Now);

}


void Detect_Gyro_Harm_Posture(void){
    uint32_t Loop_Time=millis()-Kalman_Last_Time;
    float Gyro_X_Value,Gyro_Y_Value,Gyro_Z_Value;
    
    
    if(Loop_Time>=Kalman_Loop_Time){
        Kalman_Last_Time=millis();
        Degree_Now=Get_Kalman_Data(Loop_Time);

            //detect shake
        if(Abs(Predict_X_Error)> 4.0 || Abs(Predict_Y_Error)> 4.0){
            if(ModuleMode==FLUX_LASER_MODULE && !Debug_Mode)
                Laser_Switch_Off();
            
            Shake_Trigger_Count++;

//            if(Debug_Mode){
//                printf("ERR=");           
//                printf("%.2f\t%.2f",Predict_X_Error,Predict_Y_Error);
//                printf("\n");
//            }	
            if(Shake_Trigger_Count>=3){
                Set_Module_State(SHAKE);
                Alarm_On();
                Shake_Trigger_Count=0;
            }
            
        }
        if(millis()-Shake_Last_Time>511){
            Shake_Last_Time=millis();
            Shake_Trigger_Count=0;
        }
        
        //detect tilt 
        if(Abs(Predict_X_Error)> 1.0 || Abs(Predict_Y_Error)> 1.0){
            Tilt_Last_Time=millis();
//            if(Debug_Mode){
//                printf("%.2f\t%.2f",Predict_X_Error,Predict_Y_Error);
//            }
        }
        if(millis()-Tilt_Last_Time>600){
            if(Degree_Now>=Tilt_Alarm_Degree){
                Tilt_Trigger_Count++;
                if(Tilt_Trigger_Count>=14){
                    if(ModuleMode==FLUX_LASER_MODULE && !Debug_Mode){
                        Laser_Switch_Off();
                    }
                    //Alarm_On();
                    Set_Module_State(TILT);
                    Tilt_Trigger_Count=0;
                }
            }else{
                Tilt_Trigger_Count=0;
            }
        }
        //printf("t=%d\n",millis()-Kalman_Last_Time);
    }
    
    
}

void Reset_Axis_Sensor_State(void){
	Reset_Module_State(SHAKE);
	Reset_Module_State(TILT);
}

float Get_Kalman_Data(int looptime){
    
    float ACC_angle;
    float GYRO_rate;
    
    ACC_angle=getAccAngle(Angle_X);
    GYRO_rate=getGyroRate(Angle_X);
    //printf("X=%.4lf  Y=%.4lf  Z=%.4lf\n",X_Angle,Y_Angle,Z_Angle);
    //printf("y=\t%f\t%f\t%f\n",kalmanCalculate(ACC_angle,GYRO_rate,Loop_Time),ACC_angle,GYRO_rate);
    kalmanCalculate(getAccAngle(Angle_X),getGyroRate(Angle_X),getAccAngle(Angle_Y),getGyroRate(Angle_Y),looptime);
    //kalmanCalculate(getAccAngle(Angle_X),0,getAccAngle(Angle_Y),0,looptime);
    //kalmanCalculate(getAccAngle(Angle_X),getGyroRate(Angle_X),Loop_Time);
    //printf("x%.2f y%.2f\n",Kal_X.angle,Kal_Y.angle);
    //printf("x'=%.2f y'=%.2f\n",Predict_X_Error,Predict_Y_Error);
    return sqrt(Kal_X.angle*Kal_X.angle+Kal_Y.angle*Kal_Y.angle);
    
}

float kalmanCalculate(float X_newAngle, float X_newRate,float Y_newAngle, float Y_newRate,int looptime) {
//float kalmanCalculate(float newAngle, float newRate,int looptime) {   
    float dt = (float)(looptime)/1000;
    float y,S;
    
    //X angle
    Kal_X.angle += dt * (X_newRate - Kal_X.bias);
    Kal_X.P[0] +=  - dt * (Kal_X.P[2] + Kal_X.P[1]) + Q_angle * dt;
    Kal_X.P[1] +=  - dt * Kal_X.P[3];
    Kal_X.P[2] +=  - dt * Kal_X.P[3];
    Kal_X.P[3] +=  + Q_gyro * dt;

    y = X_newAngle - Kal_X.angle;
    S = Kal_X.P[0] + R_angle;
    Kal_X.K[0] = Kal_X.P[0] / S;
    Kal_X.K[1] = Kal_X.P[2] / S;

    Predict_X_Error=Kal_X.K[0] * y;
    Kal_X.angle +=  Predict_X_Error;
    Kal_X.bias  +=  Kal_X.K[1] * y;
    Kal_X.P[0] -= Kal_X.K[0] * Kal_X.P[0];
    Kal_X.P[1] -= Kal_X.K[0] * Kal_X.P[1];
    Kal_X.P[2] -= Kal_X.K[1] * Kal_X.P[0];
    Kal_X.P[3] -= Kal_X.K[1] * Kal_X.P[1];
    
    //Y angle
    Kal_Y.angle += dt * (Y_newRate - Kal_Y.bias);//predict
    Kal_Y.P[0] +=  - dt * (Kal_Y.P[2] + Kal_Y.P[1]) + Q_angle * dt;//predict P
    Kal_Y.P[1] +=  - dt * Kal_Y.P[3];
    Kal_Y.P[2] +=  - dt * Kal_Y.P[3];
    Kal_Y.P[3] +=  + Q_gyro * dt;

    y = Y_newAngle - Kal_Y.angle;//caculate error between predict x and measure
    S = Kal_Y.P[0] + R_angle;
    Kal_Y.K[0] = Kal_Y.P[0] / S;//kalman gain
    Kal_Y.K[1] = Kal_Y.P[2] / S;

    Predict_Y_Error=Kal_X.K[0] * y;
    Kal_Y.angle +=  Kal_Y.K[0] * y;//update (correction)
    Kal_Y.bias  +=  Kal_Y.K[1] * y;
    Kal_Y.P[0] -= Kal_Y.K[0] * Kal_Y.P[0];
    Kal_Y.P[1] -= Kal_Y.K[0] * Kal_Y.P[1];
    Kal_Y.P[2] -= Kal_Y.K[1] * Kal_Y.P[0];
    Kal_Y.P[3] -= Kal_Y.K[1] * Kal_Y.P[1];

    return Kal_Y.angle;
}
  
float getGyroRate(Angle_Type angle) {
    float Gyro_X=Read_Axis_Value(Gyro_X);   
    float Gyro_Y=Read_Axis_Value(Gyro_Y);   
    if(angle==Angle_X)  
        //return sqrt(Gyro_X*Gyro_X+Gyro_Y*Gyro_Y)/1000.0;
        return Read_Axis_Value(Gyro_X)/1000.0;  
        //return Read_Axis_Value(Gyro_Z)/1000.0;  
    if(angle==Angle_Y)
        //return sqrt(Gyro_X*Gyro_X+Gyro_Y*Gyro_Y)/1000.0;
        return Read_Axis_Value(Gyro_Y)/1000.0;  
        //return Read_Axis_Value(Gyro_Z)/1000.0;  
}

double getAccAngle(Angle_Type angle) {
    float Acc_X=Read_Axis_Value(Acceler_X);
    float Acc_Y=Read_Axis_Value(Acceler_Y);
    float Acc_Z=Read_Axis_Value(Acceler_Z);
    float Cos_Z=Acc_Z/1000.0;
    Acc_Y=Acc_Y>0.0?Min(Acc_Y,150):Max(Acc_Y,-150);
    Acc_X=Acc_X>0.0?Min(Acc_X,150):Max(Acc_X,-150);
    if(angle==Angle_X)
        //return acos(Cos_Z>1.0?2.0-Cos_Z:Cos_Z)*180.0/PI; //in degree
        //return atan2(sqrt(Acc_X*Acc_X+Acc_Y*Acc_Y),Acc_Z)*180/PI; //in degree
        return atan2(Acc_Y,Acc_Z>1000.0?1000.0:Acc_Z)*180.0/PI; //in degree
    if(angle==Angle_Y)
        //return acos(Cos_Z>1.0?2.0-Cos_Z:Cos_Z)*180.0/PI; //in degree
        //return atan2(sqrt(Acc_X*Acc_X+Acc_Y*Acc_Y),Acc_Z)*180/PI; //in degree
        return atan2(-Acc_X,Acc_Z>1000.0?1000.0:Acc_Z)*180.0/PI; //in degree
    
}
