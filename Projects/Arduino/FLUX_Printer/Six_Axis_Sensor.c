#include "Six_Axis_Sensor.h"
#include "lis302dl.h"
#include <stdio.h>

uint8_t da[6];
uint8_t dg[6];
int AX,AY,AZ;
int AXO,AYO,AZO;
float Z_pre = 0;
float GX,GY,GZ;
float OX,OY,OZ;
float GXsum = 0;
float GYsum = 0;
float GZsum = 0;
float kx,ky,kz;
char str[10];
uint8_t acc_x=0;
uint8_t acc_y=0;
uint8_t acc_z=0;
uint8_t countx = 0;
uint8_t county = 0;
uint8_t countz = 0;
uint8_t s_conn_flag=0;
uint8_t s_send_no=0;

extern void LSM6DS3_Setup(void);
extern uint8_t LSM6DS3_RegRead(uint8_t reg);
extern void LSM6DS3_RegWrite(uint8_t reg, uint8_t data);


bool Six_Axis_Sensor_Initial()
{
	uint8_t reg=0;
	LSM6DS3_Setup();	   
	
	reg = LSM6DS3_RegRead(WHO_AM_I);  
	printf("reg=%x",reg);
	if(reg == 0x69) //0x3B is LIS302DL , 0x69 is LSM6DS3
	{  
		
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}	

void Show_Sensor_RawData(){
	AX=0,AY=0,AZ=0;
	AXO=0,AYO=0,AZO=0;
	GX=0,GY=0,GZ=0;
	OX=0,OY=0,OZ=0;
	GXsum = 0,GYsum = 0,GZsum = 0;					

	//LSM6DS3_RegWrite(LIS302DL_REG_CTRL1_XL, 0x60);	
	//LSM6DS3_RegWrite(CTRL1_XL, 0x64);
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
	 
	GX = GX + kx;
	GY = GY + ky;
	GZ = GZ + kz;

	 
	printf("\r\n [X : %.2f",GX);          		
	printf(" Y : %.2f",GY);            
	printf(" Z : %.2f]",GZ);
	printf("\r\n"); 

	printf("\r\n OX : %.1f",OX);          		
	printf(" OY : %.1f",OY);            
	printf(" OZ : %.1f",OZ);
	printf("\r\n");

	Z_pre = GZ;
	 
	pinMode(IO0,OUTPUT_PP);
	digitalWrite(IO0,HIGH);
	digitalWrite(LED1,LOW);	 
}

