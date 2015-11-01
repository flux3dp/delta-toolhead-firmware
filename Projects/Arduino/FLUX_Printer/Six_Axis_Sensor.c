#include "Six_Axis_Sensor.h"
#include "lis302dl.h"
#include <stdio.h>


float kx=0,ky=0,kz=0;

uint8_t Tilt_Count=0;
Six_Axis_Sensor_State_Type Six_Axis_Sensor_State = Mems_Initial_Failed;
volatile uint32_t SensorTime_Count=0;
static uint16_t Sensor_Trigger_Count=0;
static uint32_t Trigger_Interval=0;
extern void LSM6DS3_Setup(void);
extern uint8_t LSM6DS3_RegRead(uint8_t reg);
extern void LSM6DS3_RegWrite(uint8_t reg, uint8_t data);

Six_Axis_Sensor_State_Type Six_Axis_Sensor_Initial()
{
	uint8_t reg=0;
	LSM6DS3_Setup();	   
	
	reg = LSM6DS3_RegRead(WHO_AM_I);  
	//printf("reg=%x",reg);
	if(reg == 0x69) //0x3B is LIS302DL , 0x69 is LSM6DS3
	{  
		
		return Mems_Initial_Ok;
	}
	else
	{
		return Mems_Initial_Failed;
	}
}	

void Six_Axis_Sensor_Calibration(){
	uint8_t da[6];
	uint8_t dg[6];

	float GX,GY,GZ;
	float OX,OY,OZ;
	
	float GXsum = 0;
	float GYsum = 0;
	float GZsum = 0;
	
	uint8_t i;
	
	GX=0,GY=0,GZ=0;
	OX=0,OY=0,OZ=0;
	
	//LSM6DS3_RegWrite(CTRL1_XL, 0x68); //68 4g
	LSM6DS3_RegWrite(CTRL1_XL, 0x60); //60 2g
	//delay(200);
	LSM6DS3_RegWrite(CTRL2_G, 0x60);
	//delay(200);
	LSM6DS3_RegWrite(CTRL3_C, 0x04);
	//delay(200);   

	da[0] = LSM6DS3_RegRead(LSM6DS3_REG_OUTX_L);
	da[1] = LSM6DS3_RegRead(LSM6DS3_REG_OUTX_H);
	da[2] = LSM6DS3_RegRead(LSM6DS3_REG_OUTY_L);
	da[3] = LSM6DS3_RegRead(LSM6DS3_REG_OUTY_H);
	da[4] = LSM6DS3_RegRead(LSM6DS3_REG_OUTZ_L);
	da[5] = LSM6DS3_RegRead(LSM6DS3_REG_OUTZ_H);


	dg[0] = LSM6DS3_RegRead(OUTX_L_G);
	dg[1] = LSM6DS3_RegRead(OUTX_H_G);
	dg[2] = LSM6DS3_RegRead(OUTY_L_G);
	dg[3] = LSM6DS3_RegRead(OUTY_H_G);
	dg[4] = LSM6DS3_RegRead(OUTZ_L_G);
	dg[5] = LSM6DS3_RegRead(OUTZ_H_G);
	
	GX = (((short)(da[1] << 8)) + da[0]) *4*9.8/ 65535;
	GY = (((short)(da[3] << 8)) + da[2]) *4*9.8/ 65535;
	GZ = (((short)(da[5] << 8)) + da[4]) *4*9.8/ 65535;
				
	OX = (((short)(dg[1] << 8)) + dg[0]) *4*9.8/ 65535;
	OY = (((short)(dg[3] << 8)) + dg[2]) *4*9.8/ 65535;
	OZ = (((short)(dg[5] << 8)) + dg[4]) *4*9.8/ 65535;
	
	//calibration X							
	if((GX+kx) > 0 || (GX+kx) < 0)
	{								
		printf("\r\n KX !");
		for(i=0;i<100;i++)
		{
			da[0] = LSM6DS3_RegRead(LSM6DS3_REG_OUTX_L);
			da[1] = LSM6DS3_RegRead(LSM6DS3_REG_OUTX_H);

			GX = (((short)(da[1] << 8)) + da[0]) *4*9.8/ 65535;

			GXsum+=GX;													
		}

		GX = GXsum/100;
		printf(" GX : %.2f",GX);
		kx = -GX;
		printf(" kx : %.2f",kx);
		printf("\r\n");
	}
	
	//calibration Y
	if((GY+ky) > 0 || (GY+ky) < 0)
	{		
		printf("\r\n KY !");								
		for(i=0;i<100;i++)
		{
			da[2] = LSM6DS3_RegRead(LSM6DS3_REG_OUTY_L);
			da[3] = LSM6DS3_RegRead(LSM6DS3_REG_OUTY_H);

			GY = (((short)(da[3] << 8)) + da[2]) *4*9.8/ 65535;								

			GYsum+=GY;
			//printf("\r\n GY%d %f",i,GYsum);
		}

		GY = GYsum/100;
		printf(" GY : %.2f",GY);
		ky = -GY;
		printf(" ky : %.2f",ky);
		printf("\r\n");
	}
	
	//calibration Z							
	if((GZ+kz) != 9.8)
	{		
		printf("\r\n KZ !");		
		for(i=0;i<100;i++)
		{								
			da[4] = LSM6DS3_RegRead(LSM6DS3_REG_OUTZ_L);
			da[5] = LSM6DS3_RegRead(LSM6DS3_REG_OUTZ_H);

			GZ = (((short)(da[5] << 8)) + da[4]) *4*9.8/ 65535;									

			GZsum+=GZ;
			//printf("\r\n GZ%d %f",i,GZsum);
		}

		GZ = GZsum/100;
		printf(" GZ : %.2f",GZ);
		kz = 9.8 - GZ;
		printf(" kx : %.2f",kz);
		printf("\r\n");

	}
}

void Show_Sensor_RawData(){
	uint8_t da[6];
	uint8_t dg[6];

	float GX,GY,GZ;
	float OX,OY,OZ;

	GX=0,GY=0,GZ=0;
	OX=0,OY=0,OZ=0;

	LSM6DS3_RegWrite(CTRL1_XL, 0x60); //60 2g

	LSM6DS3_RegWrite(CTRL2_G, 0x60);

	LSM6DS3_RegWrite(CTRL3_C, 0x04);


	da[0] = LSM6DS3_RegRead(LSM6DS3_REG_OUTX_L);
	da[1] = LSM6DS3_RegRead(LSM6DS3_REG_OUTX_H);
	da[2] = LSM6DS3_RegRead(LSM6DS3_REG_OUTY_L);
	da[3] = LSM6DS3_RegRead(LSM6DS3_REG_OUTY_H);
	da[4] = LSM6DS3_RegRead(LSM6DS3_REG_OUTZ_L);
	da[5] = LSM6DS3_RegRead(LSM6DS3_REG_OUTZ_H);

	dg[0] = LSM6DS3_RegRead(OUTX_L_G);
	dg[1] = LSM6DS3_RegRead(OUTX_H_G);
	dg[2] = LSM6DS3_RegRead(OUTY_L_G);
	dg[3] = LSM6DS3_RegRead(OUTY_H_G);
	dg[4] = LSM6DS3_RegRead(OUTZ_L_G);
	dg[5] = LSM6DS3_RegRead(OUTZ_H_G);

	GX = (((short)(da[1] << 8)) + da[0]) *4*9.8/ 65535;
	GY = (((short)(da[3] << 8)) + da[2]) *4*9.8/ 65535;
	GZ = (((short)(da[5] << 8)) + da[4]) *4*9.8/ 65535;
	 
	OX = (((short)(dg[1] << 8)) + dg[0]) *4*9.8/ 65535;
	OY = (((short)(dg[3] << 8)) + dg[2]) *4*9.8/ 65535;
	OZ = (((short)(dg[5] << 8)) + dg[4]) *4*9.8/ 65535;
	 
	GX = GX + kx;
	GY = GY + ky;
	GZ = GZ + kz;
	 
	printf("%.2f ",GX);          		
	printf("%.2f ",GY);            
	printf("%.2f ",GZ);
 

	printf("%.2f ",OX);          		
	printf("%.2f ",OY);            
	printf("%.2f",OZ);
	printf("\n");

}

void Detect_Gyro_Max(){
	uint8_t da[6];
	uint8_t dg[6];

	float GX,GY,GZ;
	float OX,OY,OZ;

	GX=0,GY=0,GZ=0;
	OX=0,OY=0,OZ=0;

	LSM6DS3_RegWrite(CTRL1_XL, 0x60); //60 2g

	LSM6DS3_RegWrite(CTRL2_G, 0x60);

	LSM6DS3_RegWrite(CTRL3_C, 0x04);


	da[0] = LSM6DS3_RegRead(LSM6DS3_REG_OUTX_L);
	da[1] = LSM6DS3_RegRead(LSM6DS3_REG_OUTX_H);
	da[2] = LSM6DS3_RegRead(LSM6DS3_REG_OUTY_L);
	da[3] = LSM6DS3_RegRead(LSM6DS3_REG_OUTY_H);
	da[4] = LSM6DS3_RegRead(LSM6DS3_REG_OUTZ_L);
	da[5] = LSM6DS3_RegRead(LSM6DS3_REG_OUTZ_H);

	dg[0] = LSM6DS3_RegRead(OUTX_L_G);
	dg[1] = LSM6DS3_RegRead(OUTX_H_G);
	dg[2] = LSM6DS3_RegRead(OUTY_L_G);
	dg[3] = LSM6DS3_RegRead(OUTY_H_G);
	dg[4] = LSM6DS3_RegRead(OUTZ_L_G);
	dg[5] = LSM6DS3_RegRead(OUTZ_H_G);

	GX = (((short)(da[1] << 8)) + da[0]) *4*9.8/ 65535;
	GY = (((short)(da[3] << 8)) + da[2]) *4*9.8/ 65535;
	GZ = (((short)(da[5] << 8)) + da[4]) *4*9.8/ 65535;
	 
	OX = (((short)(dg[1] << 8)) + dg[0]) *4*9.8/ 65535;
	OY = (((short)(dg[3] << 8)) + dg[2]) *4*9.8/ 65535;
	OZ = (((short)(dg[5] << 8)) + dg[4]) *4*9.8/ 65535;
	 
	GX = GX + kx;
	GY = GY + ky;
	GZ = GZ + kz;
	
	if(OX>=1.8 || OX<=-1.8 || OY>=1.8 || OY<=-1.8 || OZ>=1.8 || OZ<=-1.8){
		printf("Shake ");
		printf("%.2f ",OX);          		
		printf("%.2f ",OY);            
		printf("%.2f",OZ);
		printf("\n");
	}
		
	
//	Accelerometer
//	printf("%.2f ",GX);          		
//	printf("%.2f ",GY);            
//	printf("%.2f ",GZ);
// 

//	Gyroscope
//	printf("%.2f ",OX);          		
//	printf("%.2f ",OY);            
//	printf("%.2f",OZ);
//	printf("\n");

}

void Detect_Gyro_Tilt(){
	uint8_t da[6];
	uint8_t dg[6];

	float GX,GY,GZ;
	float OX,OY,OZ;

	if(SensorTime_Count<=10)
		return;
	
	GX=0,GY=0,GZ=0;
	OX=0,OY=0,OZ=0;
		
	LSM6DS3_RegWrite(CTRL1_XL, 0x60); //60 2g

	LSM6DS3_RegWrite(CTRL2_G, 0x60);

	LSM6DS3_RegWrite(CTRL3_C, 0x04);


	da[0] = LSM6DS3_RegRead(LSM6DS3_REG_OUTX_L);
	da[1] = LSM6DS3_RegRead(LSM6DS3_REG_OUTX_H);
	da[2] = LSM6DS3_RegRead(LSM6DS3_REG_OUTY_L);
	da[3] = LSM6DS3_RegRead(LSM6DS3_REG_OUTY_H);
	da[4] = LSM6DS3_RegRead(LSM6DS3_REG_OUTZ_L);
	da[5] = LSM6DS3_RegRead(LSM6DS3_REG_OUTZ_H);

	dg[0] = LSM6DS3_RegRead(OUTX_L_G);
	dg[1] = LSM6DS3_RegRead(OUTX_H_G);
	dg[2] = LSM6DS3_RegRead(OUTY_L_G);
	dg[3] = LSM6DS3_RegRead(OUTY_H_G);
	dg[4] = LSM6DS3_RegRead(OUTZ_L_G);
	dg[5] = LSM6DS3_RegRead(OUTZ_H_G);

	GX = (((short)(da[1] << 8)) + da[0]) *4*9.8/ 65535;
	GY = (((short)(da[3] << 8)) + da[2]) *4*9.8/ 65535;
	GZ = (((short)(da[5] << 8)) + da[4]) *4*9.8/ 65535;
	 
	OX = (((short)(dg[1] << 8)) + dg[0]) *4*9.8/ 65535;
	OY = (((short)(dg[3] << 8)) + dg[2]) *4*9.8/ 65535;
	OZ = (((short)(dg[5] << 8)) + dg[4]) *4*9.8/ 65535;
	 
	GX = GX + kx;
	GY = GY + ky;
	GZ = GZ + kz;
	
	if(Trigger_Interval>=1)
		Trigger_Interval++;
	
	if(OZ >= 0.3 || OZ <= -0.3){
		Trigger_Interval++;
		Sensor_Trigger_Count++;
		
	}
		
	if(Sensor_Trigger_Count>=2){
		printf("Tilt");           
		printf("%.2f",OZ);
		printf("\n");
		Trigger_Interval=0;
		Sensor_Trigger_Count=0;
	}
	
	if(Trigger_Interval>51){
		Trigger_Interval=0;
		Sensor_Trigger_Count=0;
	}
	
		

//	Accelerometer
//	printf("%.2f ",GX);          		
//	printf("%.2f ",GY);            
//	printf("%.2f ",GZ);
// 

//	Gyroscope
//	printf("%.2f ",OX);          		
//	printf("%.2f ",OY);            
//	printf("%.2f",OZ);
//	printf("\n");
	SensorTime_Count=0;
}
