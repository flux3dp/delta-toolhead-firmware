/******************** (C) COPYRIGHT 2014 STMicroelectronics ********************
*
* File Name          : LSM6DS3_ACC_GYRO_driver.h
* Author             : MSH Application Team
* Version            : v1.00
* Date               : 09/04/2014 
* Description        : LSM6DS3 ACC_GYRO driver header file
*
* HISTORY:
* Date:   09/04/2014            
* Modification: Initial Revision                
* Author:	Platform Independent Driver Generator v0.03
* Reviewed by: Armando Visconti
*-------------------------------------------------------------------------------
*
*
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LSM6DS3_ACC_GYRO_DRIVER__H
#define __LSM6DS3_ACC_GYRO_DRIVER__H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

//these could change accordingly with the architecture

#ifndef __ARCHDEP__TYPES
#define __ARCHDEP__TYPES

typedef unsigned char u8_t;
typedef unsigned short int u16_t;
typedef unsigned int u32_t;
typedef int i32_t;
typedef short int i16_t;
typedef signed char i8_t;

#endif /*__ARCHDEP__TYPES*/

/* Exported common structure --------------------------------------------------------*/

#ifndef __SHARED__TYPES
#define __SHARED__TYPES

typedef union{
	i16_t i16bit[3];
	u8_t u8bit[6];
} Type3Axis16bit_U;	

typedef union{
	i16_t i16bit;
	u8_t u8bit[2];
} Type1Axis16bit_U;

typedef union{
	i32_t i32bit;
	u8_t u8bit[4];
} Type1Axis32bit_U;

typedef enum {
  MEMS_SUCCESS				=		0x01,
  MEMS_ERROR				=		0x00	
} status_t;

#endif /*__SHARED__TYPES*/

/* Exported macro ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/************** I2C Address *****************/

#define LSM6DS3_ACC_GYRO_I2C_ADDRESS         0xD6

/************** Who am I  *******************/

#define LSM6DS3_ACC_GYRO_WHO_AM_I         0x69

/* Private Function Prototype -------------------------------------------------------*/

u8_t LSM6DS3_ACC_GYRO_ReadReg(u8_t Reg, u8_t* Data);
u8_t LSM6DS3_ACC_GYRO_WriteReg(u8_t Reg, u8_t Data); 
u8_t LSM6DS3_ACC_GYRO_WriteMem(u8_t Reg, u8_t *Bufp, u16_t len);
u8_t LSM6DS3_ACC_GYRO_ReadMem(u8_t Reg, u8_t *Bufp, u16_t len);
void LSM6DS3_ACC_GYRO_SwapHighLowByte(u8_t *bufferToSwap, u8_t numberOfByte, u8_t dimension); 


/************** Device Register  *******************/
#define LSM6DS3_ACC_GYRO_RAM_ACCESS  	0X01
#define LSM6DS3_ACC_GYRO_SENSOR_SYNC_TIME  	0X04
#define LSM6DS3_ACC_GYRO_SENSOR_SYNC_EN  	0X05
#define LSM6DS3_ACC_GYRO_FIFO_CTRL1  	0X06
#define LSM6DS3_ACC_GYRO_FIFO_CTRL2  	0X07
#define LSM6DS3_ACC_GYRO_FIFO_CTRL3  	0X08
#define LSM6DS3_ACC_GYRO_FIFO_CTRL4  	0X09
#define LSM6DS3_ACC_GYRO_FIFO_CTRL5  	0X0A
#define LSM6DS3_ACC_GYRO_ORIENT_CFG_G  	0X0B
#define LSM6DS3_ACC_GYRO_REFERENCE_G  	0X0C
#define LSM6DS3_ACC_GYRO_INT1_CTRL  	0X0D
#define LSM6DS3_ACC_GYRO_INT2_CTRL  	0X0E
#define LSM6DS3_ACC_GYRO_WHO_AM_I_REG  	0X0F
#define LSM6DS3_ACC_GYRO_CTRL1_XL  	0X10
#define LSM6DS3_ACC_GYRO_CTRL2_G  	0X11
#define LSM6DS3_ACC_GYRO_CTRL3_C  	0X12
#define LSM6DS3_ACC_GYRO_CTRL4_C  	0X13
#define LSM6DS3_ACC_GYRO_CTRL5_C  	0X14
#define LSM6DS3_ACC_GYRO_CTRL6_G  	0X15
#define LSM6DS3_ACC_GYRO_CTRL7_G  	0X16
#define LSM6DS3_ACC_GYRO_CTRL8_XL  	0X17
#define LSM6DS3_ACC_GYRO_CTRL9_XL  	0X18
#define LSM6DS3_ACC_GYRO_CTRL10_C  	0X19
#define LSM6DS3_ACC_GYRO_MASTER_CONFIG  	0X1A
#define LSM6DS3_ACC_GYRO_WAKE_UP_SRC  	0X1B
#define LSM6DS3_ACC_GYRO_TAP_SRC  	0X1C
#define LSM6DS3_ACC_GYRO_D6D_SRC  	0X1D
#define LSM6DS3_ACC_GYRO_STATUS_REG  	0X1E
#define LSM6DS3_ACC_GYRO_OUTX_L_G  	0X22
#define LSM6DS3_ACC_GYRO_OUTX_H_G  	0X23
#define LSM6DS3_ACC_GYRO_OUTY_L_G  	0X24
#define LSM6DS3_ACC_GYRO_OUTY_H_G  	0X25
#define LSM6DS3_ACC_GYRO_OUTZ_L_G  	0X26
#define LSM6DS3_ACC_GYRO_OUTZ_H_G  	0X27
#define LSM6DS3_ACC_GYRO_OUTX_L_XL  	0X28
#define LSM6DS3_ACC_GYRO_OUTX_H_XL  	0X29
#define LSM6DS3_ACC_GYRO_OUTY_L_XL  	0X2A
#define LSM6DS3_ACC_GYRO_OUTY_H_XL  	0X2B
#define LSM6DS3_ACC_GYRO_OUTZ_L_XL  	0X2C
#define LSM6DS3_ACC_GYRO_OUTZ_H_XL  	0X2D
#define LSM6DS3_ACC_GYRO_SENSORHUB1_REG  	0X2E
#define LSM6DS3_ACC_GYRO_SENSORHUB2_REG  	0X2F
#define LSM6DS3_ACC_GYRO_SENSORHUB3_REG  	0X30
#define LSM6DS3_ACC_GYRO_SENSORHUB4_REG  	0X31
#define LSM6DS3_ACC_GYRO_SENSORHUB5_REG  	0X32
#define LSM6DS3_ACC_GYRO_SENSORHUB6_REG  	0X33
#define LSM6DS3_ACC_GYRO_SENSORHUB7_REG  	0X34
#define LSM6DS3_ACC_GYRO_SENSORHUB8_REG  	0X35
#define LSM6DS3_ACC_GYRO_SENSORHUB9_REG  	0X36
#define LSM6DS3_ACC_GYRO_SENSORHUB10_REG  	0X37
#define LSM6DS3_ACC_GYRO_SENSORHUB11_REG  	0X38
#define LSM6DS3_ACC_GYRO_SENSORHUB12_REG  	0X39
#define LSM6DS3_ACC_GYRO_FIFO_STATUS1  	0X3A
#define LSM6DS3_ACC_GYRO_FIFO_STATUS2  	0X3B
#define LSM6DS3_ACC_GYRO_FIFO_STATUS3  	0X3C
#define LSM6DS3_ACC_GYRO_FIFO_STATUS4  	0X3D
#define LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L  	0X3E
#define LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_H  	0X3F
#define LSM6DS3_ACC_GYRO_TIMESTAMP0_REG  	0X40
#define LSM6DS3_ACC_GYRO_TIMESTAMP1_REG  	0X41
#define LSM6DS3_ACC_GYRO_TIMESTAMP2_REG  	0X42
#define LSM6DS3_ACC_GYRO_STEP_COUNTER_L  	0X4B
#define LSM6DS3_ACC_GYRO_STEP_COUNTER_H  	0X4C
#define LSM6DS3_ACC_GYRO_FUNC_SRC  	0X53
#define LSM6DS3_ACC_GYRO_TAP_CFG1  	0X58
#define LSM6DS3_ACC_GYRO_TAP_THS_6D  	0X59
#define LSM6DS3_ACC_GYRO_INT_DUR2  	0X5A
#define LSM6DS3_ACC_GYRO_WAKE_UP_THS  	0X5B
#define LSM6DS3_ACC_GYRO_WAKE_UP_DUR  	0X5C
#define LSM6DS3_ACC_GYRO_FREE_FALL  	0X5D
#define LSM6DS3_ACC_GYRO_MD1_CFG  	0X5E
#define LSM6DS3_ACC_GYRO_MD2_CFG  	0X5F

/************** Access Device RAM  *******************/
#define LSM6DS3_ACC_GYRO_ADDR0_TO_RW_RAM         0x62
#define LSM6DS3_ACC_GYRO_ADDR1_TO_RW_RAM         0x63
#define LSM6DS3_ACC_GYRO_DATA_TO_WR_RAM          0x64
#define LSM6DS3_ACC_GYRO_DATA_RD_FROM_RAM        0x65

#define LSM6DS3_ACC_GYRO_RAM_SIZE                2048

/************** Embedded functions register mapping  *******************/
#define LSM6DS3_ACC_GYRO_SLV0_ADD                     0x02
#define LSM6DS3_ACC_GYRO_SLV0_SUBADD                  0x03
#define LSM6DS3_ACC_GYRO_SLAVE0_CONFIG                0x04
#define LSM6DS3_ACC_GYRO_SLV1_ADD                     0x05
#define LSM6DS3_ACC_GYRO_SLV1_SUBADD                  0x06
#define LSM6DS3_ACC_GYRO_SLAVE1_CONFIG                0x07
#define LSM6DS3_ACC_GYRO_SLV2_ADD                     0x08
#define LSM6DS3_ACC_GYRO_SLV2_SUBADD                  0x09
#define LSM6DS3_ACC_GYRO_SLAVE2_CONFIG                0x0A
#define LSM6DS3_ACC_GYRO_SLV3_ADD                     0x0B
#define LSM6DS3_ACC_GYRO_SLV3_SUBADD                  0x0C
#define LSM6DS3_ACC_GYRO_SLAVE3_CONFIG                0x0D
#define LSM6DS3_ACC_GYRO_DATAWRITE_SRC_MODE_SUB_SLV0  0x0E
#define LSM6DS3_ACC_GYRO_CONFIG_PEDO_THS_MIN          0x0F
#define LSM6DS3_ACC_GYRO_CONFIG_TILT_IIR              0x10
#define LSM6DS3_ACC_GYRO_CONFIG_TILT_ACOS             0x11
#define LSM6DS3_ACC_GYRO_CONFIG_TILT_WTIME            0x12
#define LSM6DS3_ACC_GYRO_SM_STEP_THS                  0x13
#define LSM6DS3_ACC_GYRO_MAG_SI_XX                    0x24
#define LSM6DS3_ACC_GYRO_MAG_SI_XY                    0x25
#define LSM6DS3_ACC_GYRO_MAG_SI_XZ                    0x26
#define LSM6DS3_ACC_GYRO_MAG_SI_YX                    0x27
#define LSM6DS3_ACC_GYRO_MAG_SI_YY                    0x28
#define LSM6DS3_ACC_GYRO_MAG_SI_YZ                    0x29
#define LSM6DS3_ACC_GYRO_MAG_SI_ZX                    0x2A
#define LSM6DS3_ACC_GYRO_MAG_SI_ZY                    0x2B
#define LSM6DS3_ACC_GYRO_MAG_SI_ZZ                    0x2C
#define LSM6DS3_ACC_GYRO_MAG_OFFX_L                   0x2D
#define LSM6DS3_ACC_GYRO_MAG_OFFX_H                   0x2E
#define LSM6DS3_ACC_GYRO_MAG_OFFY_L                   0x2F
#define LSM6DS3_ACC_GYRO_MAG_OFFY_H                   0x30
#define LSM6DS3_ACC_GYRO_MAG_OFFZ_L                   0x31
#define LSM6DS3_ACC_GYRO_MAG_OFFZ_H                   0x32

/*******************************************************************************
* Register      : RAM_ACCESS
* Address       : 0X01
* Bit Group Name: PROG_RAM1
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_PROG_RAM1_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_PROG_RAM1_ENABLED 		 =0x01,
} LSM6DS3_ACC_GYRO_PROG_RAM1_t;

#define  	LSM6DS3_ACC_GYRO_PROG_RAM1_MASK  	0x01
status_t  LSM6DS3_ACC_GYRO_W_start_prog_ram(LSM6DS3_ACC_GYRO_PROG_RAM1_t newValue);
status_t LSM6DS3_ACC_GYRO_R_start_prog_ram(LSM6DS3_ACC_GYRO_PROG_RAM1_t *value);

/*******************************************************************************
* Register      : RAM_ACCESS
* Address       : 0X01
* Bit Group Name: CUSTOMROM1
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_CUSTOMROM1_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_CUSTOMROM1_ENABLED 		 =0x04,
} LSM6DS3_ACC_GYRO_CUSTOMROM1_t;

#define  	LSM6DS3_ACC_GYRO_CUSTOMROM1_MASK  	0x04
status_t  LSM6DS3_ACC_GYRO_W_start_customrom(LSM6DS3_ACC_GYRO_CUSTOMROM1_t newValue);
status_t LSM6DS3_ACC_GYRO_R_start_customrom(LSM6DS3_ACC_GYRO_CUSTOMROM1_t *value);

/*******************************************************************************
* Register      : RAM_ACCESS
* Address       : 0X01
* Bit Group Name: RAM_PAGE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_RAM_PAGE_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_RAM_PAGE_ENABLED 		 =0x80,
} LSM6DS3_ACC_GYRO_RAM_PAGE_t;

#define  	LSM6DS3_ACC_GYRO_RAM_PAGE_MASK  	0x80
status_t  LSM6DS3_ACC_GYRO_W_Open_RAM_Page(LSM6DS3_ACC_GYRO_RAM_PAGE_t newValue);
status_t LSM6DS3_ACC_GYRO_R_Open_RAM_Page(LSM6DS3_ACC_GYRO_RAM_PAGE_t *value);

/*******************************************************************************
* Register      : SENSOR_SYNC_TIME
* Address       : 0X04
* Bit Group Name: TPH
* Permission    : RW
*******************************************************************************/
#define  	LSM6DS3_ACC_GYRO_TPH_MASK  	0xFF
#define  	LSM6DS3_ACC_GYRO_TPH_POSITION  	0
status_t  LSM6DS3_ACC_GYRO_W_Stamping_Time_Frame(u8_t newValue);
status_t LSM6DS3_ACC_GYRO_R_Stamping_Time_Frame(u8_t *value);

/*******************************************************************************
* Register      : SENSOR_SYNC_EN
* Address       : 0X05
* Bit Group Name: SYNC_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_SYNC_EN_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_SYNC_EN_ENABLED 		 =0x01,
} LSM6DS3_ACC_GYRO_SYNC_EN_t;

#define  	LSM6DS3_ACC_GYRO_SYNC_EN_MASK  	0x01
status_t  LSM6DS3_ACC_GYRO_W_StampingEn(LSM6DS3_ACC_GYRO_SYNC_EN_t newValue);
status_t LSM6DS3_ACC_GYRO_R_StampingEn(LSM6DS3_ACC_GYRO_SYNC_EN_t *value);

/*******************************************************************************
* Register      : SENSOR_SYNC_EN
* Address       : 0X05
* Bit Group Name: HP_RST
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_HP_RST_RST_OFF 		 =0x00,
  	LSM6DS3_ACC_GYRO_HP_RST_RST_ON 		 =0x02,
} LSM6DS3_ACC_GYRO_HP_RST_t;

#define  	LSM6DS3_ACC_GYRO_HP_RST_MASK  	0x02
status_t  LSM6DS3_ACC_GYRO_W_HP_FilterRst(LSM6DS3_ACC_GYRO_HP_RST_t newValue);
status_t LSM6DS3_ACC_GYRO_R_HP_FilterRst(LSM6DS3_ACC_GYRO_HP_RST_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL1
* Address       : 0X06
* Bit Group Name: WTM_FIFO
* Permission    : RW
*******************************************************************************/
#define  	LSM6DS3_ACC_GYRO_WTM_FIFO_CTRL1_MASK  	0xFF
#define  	LSM6DS3_ACC_GYRO_WTM_FIFO_CTRL1_POSITION  	0
#define  	LSM6DS3_ACC_GYRO_WTM_FIFO_CTRL2_MASK  	0x0F
#define  	LSM6DS3_ACC_GYRO_WTM_FIFO_CTRL2_POSITION  	0
status_t  LSM6DS3_ACC_GYRO_W_FIFO_Watermark(u16_t newValue);
status_t LSM6DS3_ACC_GYRO_R_FIFO_Watermark(u16_t *value);


/*******************************************************************************
* Register      : FIFO_CTRL2
* Address       : 0X07
* Bit Group Name: TIM_PEDO_FIFO_DRDY
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_DRDY_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_DRDY_ENABLED 		 =0x40,
} LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_DRDY_t;

#define  	LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_DRDY_MASK  	0x40
status_t  LSM6DS3_ACC_GYRO_W_TIM_PEDO_FIFO_Write_En(LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_DRDY_t newValue);
status_t LSM6DS3_ACC_GYRO_R_TIM_PEDO_FIFO_Write_En(LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_DRDY_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL2
* Address       : 0X07
* Bit Group Name: TIM_PEDO_FIFO_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_EN_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_EN_ENABLED 		 =0x80,
} LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_EN_t;

#define  	LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_EN_MASK  	0x80
status_t  LSM6DS3_ACC_GYRO_W_TIM_PEDO_FIFO_En(LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_EN_t newValue);
status_t LSM6DS3_ACC_GYRO_R_TIM_PEDO_FIFO_En(LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_EN_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL3
* Address       : 0X08
* Bit Group Name: DEC_FIFO_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DATA_NOT_IN_FIFO 		 =0x00,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_XL_NO_DECIMATION 		 =0x01,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_2 		 =0x02,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_3 		 =0x03,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_4 		 =0x04,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_8 		 =0x05,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_16 		 =0x06,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_32 		 =0x07,
} LSM6DS3_ACC_GYRO_DEC_FIFO_XL_t;

#define  	LSM6DS3_ACC_GYRO_DEC_FIFO_XL_MASK  	0x07
status_t  LSM6DS3_ACC_GYRO_W_DEC_FIFO_XL(LSM6DS3_ACC_GYRO_DEC_FIFO_XL_t newValue);
status_t LSM6DS3_ACC_GYRO_R_DEC_FIFO_XL(LSM6DS3_ACC_GYRO_DEC_FIFO_XL_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL3
* Address       : 0X08
* Bit Group Name: DEC_FIFO_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_DEC_FIFO_G_DATA_NOT_IN_FIFO 		 =0x00,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_G_NO_DECIMATION 		 =0x08,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_2 		 =0x10,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_3 		 =0x18,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_4 		 =0x20,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_8 		 =0x28,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_16 		 =0x30,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_32 		 =0x38,
} LSM6DS3_ACC_GYRO_DEC_FIFO_G_t;

#define  	LSM6DS3_ACC_GYRO_DEC_FIFO_G_MASK  	0x38
status_t  LSM6DS3_ACC_GYRO_W_DEC_FIFO_G(LSM6DS3_ACC_GYRO_DEC_FIFO_G_t newValue);
status_t LSM6DS3_ACC_GYRO_R_DEC_FIFO_G(LSM6DS3_ACC_GYRO_DEC_FIFO_G_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL4
* Address       : 0X09
* Bit Group Name: DEC_FIFO_SLV0
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_DEC_FIFO_SLV0_DATA_NOT_IN_FIFO 		 =0x00,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_SLV0_NO_DECIMATION 		 =0x01,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_SLV0_DECIMATION_BY_2 		 =0x02,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_SLV0_DECIMATION_BY_3 		 =0x03,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_SLV0_DECIMATION_BY_4 		 =0x04,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_SLV0_DECIMATION_BY_8 		 =0x05,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_SLV0_DECIMATION_BY_16 		 =0x06,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_SLV0_DECIMATION_BY_32 		 =0x07,
} LSM6DS3_ACC_GYRO_DEC_FIFO_SLV0_t;

#define  	LSM6DS3_ACC_GYRO_DEC_FIFO_SLV0_MASK  	0x07
status_t  LSM6DS3_ACC_GYRO_W_DEC_FIFO_SLV0(LSM6DS3_ACC_GYRO_DEC_FIFO_SLV0_t newValue);
status_t LSM6DS3_ACC_GYRO_R_DEC_FIFO_SLV0(LSM6DS3_ACC_GYRO_DEC_FIFO_SLV0_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL4
* Address       : 0X09
* Bit Group Name: DEC_FIFO_SLV1
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_DEC_FIFO_SLV1_DATA_NOT_IN_FIFO 		 =0x00,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_SLV1_NO_DECIMATION 		 =0x08,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_SLV1_DECIMATION_BY_2 		 =0x10,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_SLV1_DECIMATION_BY_3 		 =0x18,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_SLV1_DECIMATION_BY_4 		 =0x20,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_SLV1_DECIMATION_BY_8 		 =0x28,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_SLV1_DECIMATION_BY_16 		 =0x30,
  	LSM6DS3_ACC_GYRO_DEC_FIFO_SLV1_DECIMATION_BY_32 		 =0x38,
} LSM6DS3_ACC_GYRO_DEC_FIFO_SLV1_t;

#define  	LSM6DS3_ACC_GYRO_DEC_FIFO_SLV1_MASK  	0x38
status_t  LSM6DS3_ACC_GYRO_W_DEC_FIFO_SLV1(LSM6DS3_ACC_GYRO_DEC_FIFO_SLV1_t newValue);
status_t LSM6DS3_ACC_GYRO_R_DEC_FIFO_SLV1(LSM6DS3_ACC_GYRO_DEC_FIFO_SLV1_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL4
* Address       : 0X09
* Bit Group Name: HI_DATA_ONLY
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_HI_DATA_ONLY_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_HI_DATA_ONLY_ENABLED 		 =0x40,
} LSM6DS3_ACC_GYRO_HI_DATA_ONLY_t;

#define  	LSM6DS3_ACC_GYRO_HI_DATA_ONLY_MASK  	0x40
status_t  LSM6DS3_ACC_GYRO_W_HI_DATA_ONLY(LSM6DS3_ACC_GYRO_HI_DATA_ONLY_t newValue);
status_t LSM6DS3_ACC_GYRO_R_HI_DATA_ONLY(LSM6DS3_ACC_GYRO_HI_DATA_ONLY_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL5
* Address       : 0X0A
* Bit Group Name: FIFO_MODE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_FIFO_MODE_BYPASS 		 =0x00,
  	LSM6DS3_ACC_GYRO_FIFO_MODE_FIFO 		 =0x01,
  	LSM6DS3_ACC_GYRO_FIFO_MODE_STREAM 		 =0x02,
  	LSM6DS3_ACC_GYRO_FIFO_MODE_STF 		 =0x03,
  	LSM6DS3_ACC_GYRO_FIFO_MODE_BTS 		 =0x04,
  	LSM6DS3_ACC_GYRO_FIFO_MODE_DYN_STREAM 		 =0x05,
  	LSM6DS3_ACC_GYRO_FIFO_MODE_DYN_STREAM_2 		 =0x06,
  	LSM6DS3_ACC_GYRO_FIFO_MODE_BTF 		 =0x07,
} LSM6DS3_ACC_GYRO_FIFO_MODE_t;

#define  	LSM6DS3_ACC_GYRO_FIFO_MODE_MASK  	0x07
status_t  LSM6DS3_ACC_GYRO_W_FIFO_MODE(LSM6DS3_ACC_GYRO_FIFO_MODE_t newValue);
status_t LSM6DS3_ACC_GYRO_R_FIFO_MODE(LSM6DS3_ACC_GYRO_FIFO_MODE_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL5
* Address       : 0X0A
* Bit Group Name: ODR_FIFO
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_ODR_FIFO_10Hz 		 =0x08,
  	LSM6DS3_ACC_GYRO_ODR_FIFO_25Hz 		 =0x10,
  	LSM6DS3_ACC_GYRO_ODR_FIFO_50Hz 		 =0x18,
  	LSM6DS3_ACC_GYRO_ODR_FIFO_100Hz 		 =0x20,
  	LSM6DS3_ACC_GYRO_ODR_FIFO_200Hz 		 =0x28,
  	LSM6DS3_ACC_GYRO_ODR_FIFO_400Hz 		 =0x30,
  	LSM6DS3_ACC_GYRO_ODR_FIFO_800Hz 		 =0x38,
  	LSM6DS3_ACC_GYRO_ODR_FIFO_1600Hz 		 =0x40,
  	LSM6DS3_ACC_GYRO_ODR_FIFO_3300Hz 		 =0x48,
  	LSM6DS3_ACC_GYRO_ODR_FIFO_6600Hz 		 =0x50,
  	LSM6DS3_ACC_GYRO_ODR_FIFO_13300Hz 		 =0x58,
} LSM6DS3_ACC_GYRO_ODR_FIFO_t;

#define  	LSM6DS3_ACC_GYRO_ODR_FIFO_MASK  	0x78
status_t  LSM6DS3_ACC_GYRO_W_ODR_FIFO(LSM6DS3_ACC_GYRO_ODR_FIFO_t newValue);
status_t LSM6DS3_ACC_GYRO_R_ODR_FIFO(LSM6DS3_ACC_GYRO_ODR_FIFO_t *value);

/*******************************************************************************
* Register      : ORIENT_CFG_G
* Address       : 0X0B
* Bit Group Name: ORIENT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_ORIENT_XYZ 		 =0x00,
  	LSM6DS3_ACC_GYRO_ORIENT_XZY 		 =0x01,
  	LSM6DS3_ACC_GYRO_ORIENT_YXZ 		 =0x02,
  	LSM6DS3_ACC_GYRO_ORIENT_YZX 		 =0x03,
  	LSM6DS3_ACC_GYRO_ORIENT_ZXY 		 =0x04,
  	LSM6DS3_ACC_GYRO_ORIENT_ZYX 		 =0x05,
} LSM6DS3_ACC_GYRO_ORIENT_t;

#define  	LSM6DS3_ACC_GYRO_ORIENT_MASK  	0x07
status_t  LSM6DS3_ACC_GYRO_W_Orientation(LSM6DS3_ACC_GYRO_ORIENT_t newValue);
status_t LSM6DS3_ACC_GYRO_R_Orientation(LSM6DS3_ACC_GYRO_ORIENT_t *value);

/*******************************************************************************
* Register      : ORIENT_CFG_G
* Address       : 0X0B
* Bit Group Name: SIGN_Z_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_SIGN_Z_G_POSITIVE 		 =0x00,
  	LSM6DS3_ACC_GYRO_SIGN_Z_G_NEGATIVE 		 =0x08,
} LSM6DS3_ACC_GYRO_SIGN_Z_G_t;

#define  	LSM6DS3_ACC_GYRO_SIGN_Z_G_MASK  	0x08
status_t  LSM6DS3_ACC_GYRO_W_SignZ_G(LSM6DS3_ACC_GYRO_SIGN_Z_G_t newValue);
status_t LSM6DS3_ACC_GYRO_R_SignZ_G(LSM6DS3_ACC_GYRO_SIGN_Z_G_t *value);

/*******************************************************************************
* Register      : ORIENT_CFG_G
* Address       : 0X0B
* Bit Group Name: SIGN_Y_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_SIGN_Y_G_POSITIVE 		 =0x00,
  	LSM6DS3_ACC_GYRO_SIGN_Y_G_NEGATIVE 		 =0x10,
} LSM6DS3_ACC_GYRO_SIGN_Y_G_t;

#define  	LSM6DS3_ACC_GYRO_SIGN_Y_G_MASK  	0x10
status_t  LSM6DS3_ACC_GYRO_W_SignY_G(LSM6DS3_ACC_GYRO_SIGN_Y_G_t newValue);
status_t LSM6DS3_ACC_GYRO_R_SignY_G(LSM6DS3_ACC_GYRO_SIGN_Y_G_t *value);

/*******************************************************************************
* Register      : ORIENT_CFG_G
* Address       : 0X0B
* Bit Group Name: SIGN_X_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_SIGN_X_G_POSITIVE 		 =0x00,
  	LSM6DS3_ACC_GYRO_SIGN_X_G_NEGATIVE 		 =0x20,
} LSM6DS3_ACC_GYRO_SIGN_X_G_t;

#define  	LSM6DS3_ACC_GYRO_SIGN_X_G_MASK  	0x20
status_t  LSM6DS3_ACC_GYRO_W_SignX_G(LSM6DS3_ACC_GYRO_SIGN_X_G_t newValue);
status_t LSM6DS3_ACC_GYRO_R_SignX_G(LSM6DS3_ACC_GYRO_SIGN_X_G_t *value);

/*******************************************************************************
* Register      : REFERENCE_G
* Address       : 0X0C
* Bit Group Name: REF_G
* Permission    : RW
*******************************************************************************/
#define  	LSM6DS3_ACC_GYRO_REF_G_MASK  	0xFF
#define  	LSM6DS3_ACC_GYRO_REF_G_POSITION  	0
status_t  LSM6DS3_ACC_GYRO_W_Reference_G(u8_t newValue);
status_t LSM6DS3_ACC_GYRO_R_Reference_G(u8_t *value);

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0X0D
* Bit Group Name: INT1_DRDY_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT1_DRDY_XL_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT1_DRDY_XL_ENABLED 		 =0x01,
} LSM6DS3_ACC_GYRO_INT1_DRDY_XL_t;

#define  	LSM6DS3_ACC_GYRO_INT1_DRDY_XL_MASK  	0x01
status_t  LSM6DS3_ACC_GYRO_W_DRDY_XL_on_INT1(LSM6DS3_ACC_GYRO_INT1_DRDY_XL_t newValue);
status_t LSM6DS3_ACC_GYRO_R_DRDY_XL_on_INT1(LSM6DS3_ACC_GYRO_INT1_DRDY_XL_t *value);

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0X0D
* Bit Group Name: INT1_DRDY_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT1_DRDY_G_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT1_DRDY_G_ENABLED 		 =0x02,
} LSM6DS3_ACC_GYRO_INT1_DRDY_G_t;

#define  	LSM6DS3_ACC_GYRO_INT1_DRDY_G_MASK  	0x02
status_t  LSM6DS3_ACC_GYRO_W_DRDY_G_on_INT1(LSM6DS3_ACC_GYRO_INT1_DRDY_G_t newValue);
status_t LSM6DS3_ACC_GYRO_R_DRDY_G_on_INT1(LSM6DS3_ACC_GYRO_INT1_DRDY_G_t *value);

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0X0D
* Bit Group Name: INT1_BOOT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT1_BOOT_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT1_BOOT_ENABLED 		 =0x04,
} LSM6DS3_ACC_GYRO_INT1_BOOT_t;

#define  	LSM6DS3_ACC_GYRO_INT1_BOOT_MASK  	0x04
status_t  LSM6DS3_ACC_GYRO_W_BOOT_on_INT1(LSM6DS3_ACC_GYRO_INT1_BOOT_t newValue);
status_t LSM6DS3_ACC_GYRO_R_BOOT_on_INT1(LSM6DS3_ACC_GYRO_INT1_BOOT_t *value);

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0X0D
* Bit Group Name: INT1_FTH
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT1_FTH_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT1_FTH_ENABLED 		 =0x08,
} LSM6DS3_ACC_GYRO_INT1_FTH_t;

#define  	LSM6DS3_ACC_GYRO_INT1_FTH_MASK  	0x08
status_t  LSM6DS3_ACC_GYRO_W_FIFO_TSHLD_on_INT1(LSM6DS3_ACC_GYRO_INT1_FTH_t newValue);
status_t LSM6DS3_ACC_GYRO_R_FIFO_TSHLD_on_INT1(LSM6DS3_ACC_GYRO_INT1_FTH_t *value);

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0X0D
* Bit Group Name: INT1_OVR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT1_OVR_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT1_OVR_ENABLED 		 =0x10,
} LSM6DS3_ACC_GYRO_INT1_OVR_t;

#define  	LSM6DS3_ACC_GYRO_INT1_OVR_MASK  	0x10
status_t  LSM6DS3_ACC_GYRO_W_OVERRUN_on_INT1(LSM6DS3_ACC_GYRO_INT1_OVR_t newValue);
status_t LSM6DS3_ACC_GYRO_R_OVERRUN_on_INT1(LSM6DS3_ACC_GYRO_INT1_OVR_t *value);

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0X0D
* Bit Group Name: INT1_FSS5
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT1_FSS5_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT1_FSS5_ENABLED 		 =0x20,
} LSM6DS3_ACC_GYRO_INT1_FSS5_t;

#define  	LSM6DS3_ACC_GYRO_INT1_FSS5_MASK  	0x20
status_t  LSM6DS3_ACC_GYRO_W_FSS5_on_INT1(LSM6DS3_ACC_GYRO_INT1_FSS5_t newValue);
status_t LSM6DS3_ACC_GYRO_R_FSS5_on_INT1(LSM6DS3_ACC_GYRO_INT1_FSS5_t *value);

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0X0D
* Bit Group Name: INT1_SIGN_MOT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT1_SIGN_MOT_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT1_SIGN_MOT_ENABLED 		 =0x40,
} LSM6DS3_ACC_GYRO_INT1_SIGN_MOT_t;

#define  	LSM6DS3_ACC_GYRO_INT1_SIGN_MOT_MASK  	0x40
status_t  LSM6DS3_ACC_GYRO_W_SIGN_MOT_on_INT1(LSM6DS3_ACC_GYRO_INT1_SIGN_MOT_t newValue);
status_t LSM6DS3_ACC_GYRO_R_SIGN_MOT_on_INT1(LSM6DS3_ACC_GYRO_INT1_SIGN_MOT_t *value);

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0X0D
* Bit Group Name: INT1_PEDO
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT1_PEDO_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT1_PEDO_ENABLED 		 =0x80,
} LSM6DS3_ACC_GYRO_INT1_PEDO_t;

#define  	LSM6DS3_ACC_GYRO_INT1_PEDO_MASK  	0x80
status_t  LSM6DS3_ACC_GYRO_W_PEDO_STEP_on_INT1(LSM6DS3_ACC_GYRO_INT1_PEDO_t newValue);
status_t LSM6DS3_ACC_GYRO_R_PEDO_STEP_on_INT1(LSM6DS3_ACC_GYRO_INT1_PEDO_t *value);

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0X0E
* Bit Group Name: INT2_DRDY_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT2_DRDY_XL_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT2_DRDY_XL_ENABLED 		 =0x01,
} LSM6DS3_ACC_GYRO_INT2_DRDY_XL_t;

#define  	LSM6DS3_ACC_GYRO_INT2_DRDY_XL_MASK  	0x01
status_t  LSM6DS3_ACC_GYRO_W_DRDY_XL_on_INT2(LSM6DS3_ACC_GYRO_INT2_DRDY_XL_t newValue);
status_t LSM6DS3_ACC_GYRO_R_DRDY_XL_on_INT2(LSM6DS3_ACC_GYRO_INT2_DRDY_XL_t *value);

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0X0E
* Bit Group Name: INT2_DRDY_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT2_DRDY_G_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT2_DRDY_G_ENABLED 		 =0x02,
} LSM6DS3_ACC_GYRO_INT2_DRDY_G_t;

#define  	LSM6DS3_ACC_GYRO_INT2_DRDY_G_MASK  	0x02
status_t  LSM6DS3_ACC_GYRO_W_DRDY_G_on_INT2(LSM6DS3_ACC_GYRO_INT2_DRDY_G_t newValue);
status_t LSM6DS3_ACC_GYRO_R_DRDY_G_on_INT2(LSM6DS3_ACC_GYRO_INT2_DRDY_G_t *value);

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0X0E
* Bit Group Name: INT2_FTH
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT2_FTH_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT2_FTH_ENABLED 		 =0x08,
} LSM6DS3_ACC_GYRO_INT2_FTH_t;

#define  	LSM6DS3_ACC_GYRO_INT2_FTH_MASK  	0x08
status_t  LSM6DS3_ACC_GYRO_W_FIFO_TSHLD_on_INT2(LSM6DS3_ACC_GYRO_INT2_FTH_t newValue);
status_t LSM6DS3_ACC_GYRO_R_FIFO_TSHLD_on_INT2(LSM6DS3_ACC_GYRO_INT2_FTH_t *value);

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0X0E
* Bit Group Name: INT2_OVR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT2_OVR_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT2_OVR_ENABLED 		 =0x10,
} LSM6DS3_ACC_GYRO_INT2_OVR_t;

#define  	LSM6DS3_ACC_GYRO_INT2_OVR_MASK  	0x10
status_t  LSM6DS3_ACC_GYRO_W_OVERRUN_on_INT2(LSM6DS3_ACC_GYRO_INT2_OVR_t newValue);
status_t LSM6DS3_ACC_GYRO_R_OVERRUN_on_INT2(LSM6DS3_ACC_GYRO_INT2_OVR_t *value);

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0X0E
* Bit Group Name: INT2_FSS5
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT2_FSS5_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT2_FSS5_ENABLED 		 =0x20,
} LSM6DS3_ACC_GYRO_INT2_FSS5_t;

#define  	LSM6DS3_ACC_GYRO_INT2_FSS5_MASK  	0x20
status_t  LSM6DS3_ACC_GYRO_W_FSS5_on_INT2(LSM6DS3_ACC_GYRO_INT2_FSS5_t newValue);
status_t LSM6DS3_ACC_GYRO_R_FSS5_on_INT2(LSM6DS3_ACC_GYRO_INT2_FSS5_t *value);

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0X0E
* Bit Group Name: INT2_SIGN_MOT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT2_SIGN_MOT_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT2_SIGN_MOT_ENABLED 		 =0x40,
} LSM6DS3_ACC_GYRO_INT2_SIGN_MOT_t;

#define  	LSM6DS3_ACC_GYRO_INT2_SIGN_MOT_MASK  	0x40
status_t  LSM6DS3_ACC_GYRO_W_SIGN_MOT_on_INT2(LSM6DS3_ACC_GYRO_INT2_SIGN_MOT_t newValue);
status_t LSM6DS3_ACC_GYRO_R_SIGN_MOT_on_INT2(LSM6DS3_ACC_GYRO_INT2_SIGN_MOT_t *value);

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0X0E
* Bit Group Name: INT2_PEDO
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT2_PEDO_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT2_PEDO_ENABLED 		 =0x80,
} LSM6DS3_ACC_GYRO_INT2_PEDO_t;

#define  	LSM6DS3_ACC_GYRO_INT2_PEDO_MASK  	0x80
status_t  LSM6DS3_ACC_GYRO_W_PEDO_STEP_on_INT2(LSM6DS3_ACC_GYRO_INT2_PEDO_t newValue);
status_t LSM6DS3_ACC_GYRO_R_PEDO_STEP_on_INT2(LSM6DS3_ACC_GYRO_INT2_PEDO_t *value);

/*******************************************************************************
* Register      : WHO_AM_I
* Address       : 0X0F
* Bit Group Name: WHO_AM_I_BIT
* Permission    : RO
*******************************************************************************/
#define  	LSM6DS3_ACC_GYRO_WHO_AM_I_BIT_MASK  	0xFF
#define  	LSM6DS3_ACC_GYRO_WHO_AM_I_BIT_POSITION  	0
status_t LSM6DS3_ACC_GYRO_R_WHO_AM_I(u8_t *value);

/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0X10
* Bit Group Name: BW_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_BW_XL_400Hz 		 =0x00,
  	LSM6DS3_ACC_GYRO_BW_XL_200Hz 		 =0x01,
  	LSM6DS3_ACC_GYRO_BW_XL_100Hz 		 =0x02,
  	LSM6DS3_ACC_GYRO_BW_XL_50Hz 		 =0x03,
} LSM6DS3_ACC_GYRO_BW_XL_t;

#define  	LSM6DS3_ACC_GYRO_BW_XL_MASK  	0x03
status_t  LSM6DS3_ACC_GYRO_W_BW_XL(LSM6DS3_ACC_GYRO_BW_XL_t newValue);
status_t LSM6DS3_ACC_GYRO_R_BW_XL(LSM6DS3_ACC_GYRO_BW_XL_t *value);

/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0X10
* Bit Group Name: FS_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_FS_XL_2g 		 =0x00,
  	LSM6DS3_ACC_GYRO_FS_XL_16g 		 =0x04,
  	LSM6DS3_ACC_GYRO_FS_XL_4g 		 =0x08,
  	LSM6DS3_ACC_GYRO_FS_XL_8g 		 =0x0C,
} LSM6DS3_ACC_GYRO_FS_XL_t;

#define  	LSM6DS3_ACC_GYRO_FS_XL_MASK  	0x0C
status_t  LSM6DS3_ACC_GYRO_W_FS_XL(LSM6DS3_ACC_GYRO_FS_XL_t newValue);
status_t LSM6DS3_ACC_GYRO_R_FS_XL(LSM6DS3_ACC_GYRO_FS_XL_t *value);

/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0X10
* Bit Group Name: ODR_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_ODR_XL_POWER_DOWN 		 =0x00,
  	LSM6DS3_ACC_GYRO_ODR_XL_13Hz 		 =0x10,
  	LSM6DS3_ACC_GYRO_ODR_XL_26Hz 		 =0x20,
  	LSM6DS3_ACC_GYRO_ODR_XL_52Hz 		 =0x30,
  	LSM6DS3_ACC_GYRO_ODR_XL_104Hz 		 =0x40,
  	LSM6DS3_ACC_GYRO_ODR_XL_208Hz 		 =0x50,
  	LSM6DS3_ACC_GYRO_ODR_XL_416Hz 		 =0x60,
  	LSM6DS3_ACC_GYRO_ODR_XL_833Hz 		 =0x70,
  	LSM6DS3_ACC_GYRO_ODR_XL_1660Hz 		 =0x80,
  	LSM6DS3_ACC_GYRO_ODR_XL_3330Hz 		 =0x90,
  	LSM6DS3_ACC_GYRO_ODR_XL_6660Hz 		 =0xA0,
  	LSM6DS3_ACC_GYRO_ODR_XL_13330Hz 		 =0xB0,
} LSM6DS3_ACC_GYRO_ODR_XL_t;

#define  	LSM6DS3_ACC_GYRO_ODR_XL_MASK  	0xF0
status_t  LSM6DS3_ACC_GYRO_W_ODR_XL(LSM6DS3_ACC_GYRO_ODR_XL_t newValue);
status_t LSM6DS3_ACC_GYRO_R_ODR_XL(LSM6DS3_ACC_GYRO_ODR_XL_t *value);

/*******************************************************************************
* Register      : CTRL2_G
* Address       : 0X11
* Bit Group Name: FS_125
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_FS_125_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_FS_125_ENABLED 		 =0x02,
} LSM6DS3_ACC_GYRO_FS_125_t;

#define  	LSM6DS3_ACC_GYRO_FS_125_MASK  	0x02
status_t  LSM6DS3_ACC_GYRO_W_FS_125(LSM6DS3_ACC_GYRO_FS_125_t newValue);
status_t LSM6DS3_ACC_GYRO_R_FS_125(LSM6DS3_ACC_GYRO_FS_125_t *value);

/*******************************************************************************
* Register      : CTRL2_G
* Address       : 0X11
* Bit Group Name: FS_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_FS_G_245dps 		 =0x00,
  	LSM6DS3_ACC_GYRO_FS_G_500dps 		 =0x04,
  	LSM6DS3_ACC_GYRO_FS_G_1000dps 		 =0x08,
  	LSM6DS3_ACC_GYRO_FS_G_2000dps 		 =0x0C,
} LSM6DS3_ACC_GYRO_FS_G_t;

#define  	LSM6DS3_ACC_GYRO_FS_G_MASK  	0x0C
status_t  LSM6DS3_ACC_GYRO_W_FS_G(LSM6DS3_ACC_GYRO_FS_G_t newValue);
status_t LSM6DS3_ACC_GYRO_R_FS_G(LSM6DS3_ACC_GYRO_FS_G_t *value);

/*******************************************************************************
* Register      : CTRL2_G
* Address       : 0X11
* Bit Group Name: ODR_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_ODR_G_POWER_DOWN 		 =0x00,
  	LSM6DS3_ACC_GYRO_ODR_G_13Hz 		 =0x10,
  	LSM6DS3_ACC_GYRO_ODR_G_26Hz 		 =0x20,
  	LSM6DS3_ACC_GYRO_ODR_G_52Hz 		 =0x30,
  	LSM6DS3_ACC_GYRO_ODR_G_104Hz 		 =0x40,
  	LSM6DS3_ACC_GYRO_ODR_G_208Hz 		 =0x50,
  	LSM6DS3_ACC_GYRO_ODR_G_416Hz 		 =0x60,
  	LSM6DS3_ACC_GYRO_ODR_G_833Hz 		 =0x70,
  	LSM6DS3_ACC_GYRO_ODR_G_1660Hz 		 =0x80,
} LSM6DS3_ACC_GYRO_ODR_G_t;

#define  	LSM6DS3_ACC_GYRO_ODR_G_MASK  	0xF0
status_t  LSM6DS3_ACC_GYRO_W_ODR_G(LSM6DS3_ACC_GYRO_ODR_G_t newValue);
status_t LSM6DS3_ACC_GYRO_R_ODR_G(LSM6DS3_ACC_GYRO_ODR_G_t *value);

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0X12
* Bit Group Name: SW_RESET
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_SW_RESET_NORMAL_MODE 		 =0x00,
  	LSM6DS3_ACC_GYRO_SW_RESET_RESET_DEVICE 		 =0x01,
} LSM6DS3_ACC_GYRO_SW_RESET_t;

#define  	LSM6DS3_ACC_GYRO_SW_RESET_MASK  	0x01
status_t  LSM6DS3_ACC_GYRO_W_SW_RESET(LSM6DS3_ACC_GYRO_SW_RESET_t newValue);
status_t LSM6DS3_ACC_GYRO_R_SW_RESET(LSM6DS3_ACC_GYRO_SW_RESET_t *value);

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0X12
* Bit Group Name: BLE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_BLE_LSB 		 =0x00,
  	LSM6DS3_ACC_GYRO_BLE_MSB 		 =0x02,
} LSM6DS3_ACC_GYRO_BLE_t;

#define  	LSM6DS3_ACC_GYRO_BLE_MASK  	0x02
status_t  LSM6DS3_ACC_GYRO_W_BLE(LSM6DS3_ACC_GYRO_BLE_t newValue);
status_t LSM6DS3_ACC_GYRO_R_BLE(LSM6DS3_ACC_GYRO_BLE_t *value);

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0X12
* Bit Group Name: IF_INC
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_IF_INC_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_IF_INC_ENABLED 		 =0x04,
} LSM6DS3_ACC_GYRO_IF_INC_t;

#define  	LSM6DS3_ACC_GYRO_IF_INC_MASK  	0x04
status_t  LSM6DS3_ACC_GYRO_W_IF_Addr_Incr(LSM6DS3_ACC_GYRO_IF_INC_t newValue);
status_t LSM6DS3_ACC_GYRO_R_IF_Addr_Incr(LSM6DS3_ACC_GYRO_IF_INC_t *value);

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0X12
* Bit Group Name: SIM
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_SIM_4_WIRE 		 =0x00,
  	LSM6DS3_ACC_GYRO_SIM_3_WIRE 		 =0x08,
} LSM6DS3_ACC_GYRO_SIM_t;

#define  	LSM6DS3_ACC_GYRO_SIM_MASK  	0x08
status_t  LSM6DS3_ACC_GYRO_W_SPI_Mode(LSM6DS3_ACC_GYRO_SIM_t newValue);
status_t LSM6DS3_ACC_GYRO_R_SPI_Mode(LSM6DS3_ACC_GYRO_SIM_t *value);

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0X12
* Bit Group Name: PP_OD
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_PP_OD_PUSH_PULL 		 =0x00,
  	LSM6DS3_ACC_GYRO_PP_OD_OPEN_DRAIN 		 =0x10,
} LSM6DS3_ACC_GYRO_PP_OD_t;

#define  	LSM6DS3_ACC_GYRO_PP_OD_MASK  	0x10
status_t  LSM6DS3_ACC_GYRO_W_PadSel(LSM6DS3_ACC_GYRO_PP_OD_t newValue);
status_t LSM6DS3_ACC_GYRO_R_PadSel(LSM6DS3_ACC_GYRO_PP_OD_t *value);

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0X12
* Bit Group Name: INT_ACT_LEVEL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT_ACT_LEVEL_ACTIVE_HI 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT_ACT_LEVEL_ACTIVE_LO 		 =0x20,
} LSM6DS3_ACC_GYRO_INT_ACT_LEVEL_t;

#define  	LSM6DS3_ACC_GYRO_INT_ACT_LEVEL_MASK  	0x20
status_t  LSM6DS3_ACC_GYRO_W_INT_ACT_LEVEL(LSM6DS3_ACC_GYRO_INT_ACT_LEVEL_t newValue);
status_t LSM6DS3_ACC_GYRO_R_INT_ACT_LEVEL(LSM6DS3_ACC_GYRO_INT_ACT_LEVEL_t *value);

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0X12
* Bit Group Name: BDU
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_BDU_CONTINUOS 		 =0x00,
  	LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE 		 =0x40,
} LSM6DS3_ACC_GYRO_BDU_t;

#define  	LSM6DS3_ACC_GYRO_BDU_MASK  	0x40
status_t  LSM6DS3_ACC_GYRO_W_BDU(LSM6DS3_ACC_GYRO_BDU_t newValue);
status_t LSM6DS3_ACC_GYRO_R_BDU(LSM6DS3_ACC_GYRO_BDU_t *value);

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0X12
* Bit Group Name: BOOT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_BOOT_NORMAL_MODE 		 =0x00,
  	LSM6DS3_ACC_GYRO_BOOT_REBOOT_MODE 		 =0x80,
} LSM6DS3_ACC_GYRO_BOOT_t;

#define  	LSM6DS3_ACC_GYRO_BOOT_MASK  	0x80
status_t  LSM6DS3_ACC_GYRO_W_BOOT(LSM6DS3_ACC_GYRO_BOOT_t newValue);
status_t LSM6DS3_ACC_GYRO_R_BOOT(LSM6DS3_ACC_GYRO_BOOT_t *value);

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0X13
* Bit Group Name: STOP_ON_FTH
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_STOP_ON_FTH_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_STOP_ON_FTH_ENABLED 		 =0x01,
} LSM6DS3_ACC_GYRO_STOP_ON_FTH_t;

#define  	LSM6DS3_ACC_GYRO_STOP_ON_FTH_MASK  	0x01
status_t  LSM6DS3_ACC_GYRO_W_STOP_ON_FTH(LSM6DS3_ACC_GYRO_STOP_ON_FTH_t newValue);
status_t LSM6DS3_ACC_GYRO_R_STOP_ON_FTH(LSM6DS3_ACC_GYRO_STOP_ON_FTH_t *value);

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0X13
* Bit Group Name: MODE3_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_MODE3_EN_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_MODE3_EN_ENABLED 		 =0x02,
} LSM6DS3_ACC_GYRO_MODE3_EN_t;

#define  	LSM6DS3_ACC_GYRO_MODE3_EN_MASK  	0x02
status_t  LSM6DS3_ACC_GYRO_W_MODE3_Enable(LSM6DS3_ACC_GYRO_MODE3_EN_t newValue);
status_t LSM6DS3_ACC_GYRO_R_MODE3_Enable(LSM6DS3_ACC_GYRO_MODE3_EN_t *value);

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0X13
* Bit Group Name: I2C_DISABLE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_I2C_DISABLE_I2C_AND_SPI 		 =0x00,
  	LSM6DS3_ACC_GYRO_I2C_DISABLE_SPI_ONLY 		 =0x04,
} LSM6DS3_ACC_GYRO_I2C_DISABLE_t;

#define  	LSM6DS3_ACC_GYRO_I2C_DISABLE_MASK  	0x04
status_t  LSM6DS3_ACC_GYRO_W_I2C_DISABLE(LSM6DS3_ACC_GYRO_I2C_DISABLE_t newValue);
status_t LSM6DS3_ACC_GYRO_R_I2C_DISABLE(LSM6DS3_ACC_GYRO_I2C_DISABLE_t *value);

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0X13
* Bit Group Name: DRDY_MSK
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_DRDY_MSK_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_DRDY_MSK_ENABLED 		 =0x08,
} LSM6DS3_ACC_GYRO_DRDY_MSK_t;

#define  	LSM6DS3_ACC_GYRO_DRDY_MSK_MASK  	0x08
status_t  LSM6DS3_ACC_GYRO_W_DRDY_MSK(LSM6DS3_ACC_GYRO_DRDY_MSK_t newValue);
status_t LSM6DS3_ACC_GYRO_R_DRDY_MSK(LSM6DS3_ACC_GYRO_DRDY_MSK_t *value);

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0X13
* Bit Group Name: FIFO_TEMP_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_FIFO_TEMP_EN_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_FIFO_TEMP_EN_ENABLED 		 =0x10,
} LSM6DS3_ACC_GYRO_FIFO_TEMP_EN_t;

#define  	LSM6DS3_ACC_GYRO_FIFO_TEMP_EN_MASK  	0x10
status_t  LSM6DS3_ACC_GYRO_W_FIFO_TEMP_EN(LSM6DS3_ACC_GYRO_FIFO_TEMP_EN_t newValue);
status_t LSM6DS3_ACC_GYRO_R_FIFO_TEMP_EN(LSM6DS3_ACC_GYRO_FIFO_TEMP_EN_t *value);

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0X13
* Bit Group Name: INT2_ON_INT1
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT2_ON_INT1_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT2_ON_INT1_ENABLED 		 =0x20,
} LSM6DS3_ACC_GYRO_INT2_ON_INT1_t;

#define  	LSM6DS3_ACC_GYRO_INT2_ON_INT1_MASK  	0x20
status_t  LSM6DS3_ACC_GYRO_W_INT2_ON_INT1(LSM6DS3_ACC_GYRO_INT2_ON_INT1_t newValue);
status_t LSM6DS3_ACC_GYRO_R_INT2_ON_INT1(LSM6DS3_ACC_GYRO_INT2_ON_INT1_t *value);

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0X13
* Bit Group Name: SLEEP_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_SLEEP_G_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_SLEEP_G_ENABLED 		 =0x40,
} LSM6DS3_ACC_GYRO_SLEEP_G_t;

#define  	LSM6DS3_ACC_GYRO_SLEEP_G_MASK  	0x40
status_t  LSM6DS3_ACC_GYRO_W_SleepMode_G(LSM6DS3_ACC_GYRO_SLEEP_G_t newValue);
status_t LSM6DS3_ACC_GYRO_R_SleepMode_G(LSM6DS3_ACC_GYRO_SLEEP_G_t *value);

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0X13
* Bit Group Name: BW_SCAL_ODR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_BW_SCAL_ODR_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED 		 =0x80,
} LSM6DS3_ACC_GYRO_BW_SCAL_ODR_t;

#define  	LSM6DS3_ACC_GYRO_BW_SCAL_ODR_MASK  	0x80
status_t  LSM6DS3_ACC_GYRO_W_BW_Fixed_By_ODR(LSM6DS3_ACC_GYRO_BW_SCAL_ODR_t newValue);
status_t LSM6DS3_ACC_GYRO_R_BW_Fixed_By_ODR(LSM6DS3_ACC_GYRO_BW_SCAL_ODR_t *value);

/*******************************************************************************
* Register      : CTRL5_C
* Address       : 0X14
* Bit Group Name: ST_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_ST_XL_NORMAL_MODE 		 =0x00,
  	LSM6DS3_ACC_GYRO_ST_XL_POS_SIGN_TEST 		 =0x01,
  	LSM6DS3_ACC_GYRO_ST_XL_NEG_SIGN_TEST 		 =0x02,
  	LSM6DS3_ACC_GYRO_ST_XL_NA 		 =0x03,
} LSM6DS3_ACC_GYRO_ST_XL_t;

#define  	LSM6DS3_ACC_GYRO_ST_XL_MASK  	0x03
status_t  LSM6DS3_ACC_GYRO_W_SelfTest_XL(LSM6DS3_ACC_GYRO_ST_XL_t newValue);
status_t LSM6DS3_ACC_GYRO_R_SelfTest_XL(LSM6DS3_ACC_GYRO_ST_XL_t *value);

/*******************************************************************************
* Register      : CTRL5_C
* Address       : 0X14
* Bit Group Name: ST_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_ST_G_NORMAL_MODE 		 =0x00,
  	LSM6DS3_ACC_GYRO_ST_G_POS_SIGN_TEST 		 =0x04,
  	LSM6DS3_ACC_GYRO_ST_G_NA 		 =0x08,
  	LSM6DS3_ACC_GYRO_ST_G_NEG_SIGN_TEST 		 =0x0C,
} LSM6DS3_ACC_GYRO_ST_G_t;

#define  	LSM6DS3_ACC_GYRO_ST_G_MASK  	0x0C
status_t  LSM6DS3_ACC_GYRO_W_SelfTest_G(LSM6DS3_ACC_GYRO_ST_G_t newValue);
status_t LSM6DS3_ACC_GYRO_R_SelfTest_G(LSM6DS3_ACC_GYRO_ST_G_t *value);

/*******************************************************************************
* Register      : CTRL6_G
* Address       : 0X15
* Bit Group Name: LP_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_LP_XL_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_LP_XL_ENABLED 		 =0x10,
} LSM6DS3_ACC_GYRO_LP_XL_t;

#define  	LSM6DS3_ACC_GYRO_LP_XL_MASK  	0x10
status_t  LSM6DS3_ACC_GYRO_W_LowPower_XL(LSM6DS3_ACC_GYRO_LP_XL_t newValue);
status_t LSM6DS3_ACC_GYRO_R_LowPower_XL(LSM6DS3_ACC_GYRO_LP_XL_t *value);

/*******************************************************************************
* Register      : CTRL6_G
* Address       : 0X15
* Bit Group Name: DEN_LVL2_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_DEN_LVL2_EN_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_DEN_LVL2_EN_ENABLED 		 =0x20,
} LSM6DS3_ACC_GYRO_DEN_LVL2_EN_t;

#define  	LSM6DS3_ACC_GYRO_DEN_LVL2_EN_MASK  	0x20
status_t  LSM6DS3_ACC_GYRO_W_DEN_LVL2_EN(LSM6DS3_ACC_GYRO_DEN_LVL2_EN_t newValue);
status_t LSM6DS3_ACC_GYRO_R_DEN_LVL2_EN(LSM6DS3_ACC_GYRO_DEN_LVL2_EN_t *value);

/*******************************************************************************
* Register      : CTRL6_G
* Address       : 0X15
* Bit Group Name: DEN_LVL_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_DEN_LVL_EN_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_DEN_LVL_EN_ENABLED 		 =0x40,
} LSM6DS3_ACC_GYRO_DEN_LVL_EN_t;

#define  	LSM6DS3_ACC_GYRO_DEN_LVL_EN_MASK  	0x40
status_t  LSM6DS3_ACC_GYRO_W_DEN_LVL_EN(LSM6DS3_ACC_GYRO_DEN_LVL_EN_t newValue);
status_t LSM6DS3_ACC_GYRO_R_DEN_LVL_EN(LSM6DS3_ACC_GYRO_DEN_LVL_EN_t *value);

/*******************************************************************************
* Register      : CTRL6_G
* Address       : 0X15
* Bit Group Name: DEN_EDGE_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_DEN_EDGE_EN_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_DEN_EDGE_EN_ENABLED 		 =0x80,
} LSM6DS3_ACC_GYRO_DEN_EDGE_EN_t;

#define  	LSM6DS3_ACC_GYRO_DEN_EDGE_EN_MASK  	0x80
status_t  LSM6DS3_ACC_GYRO_W_DEN_EDGE_EN(LSM6DS3_ACC_GYRO_DEN_EDGE_EN_t newValue);
status_t LSM6DS3_ACC_GYRO_R_DEN_EDGE_EN(LSM6DS3_ACC_GYRO_DEN_EDGE_EN_t *value);

/*******************************************************************************
* Register      : CTRL7_G
* Address       : 0X16
* Bit Group Name: HPM_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_HPM_G_NORMAL_MODE 		 =0x00,
  	LSM6DS3_ACC_GYRO_HPM_G_REF_SIGNAL 		 =0x10,
  	LSM6DS3_ACC_GYRO_HPM_G_NORMAL_MODE_2 		 =0x20,
  	LSM6DS3_ACC_GYRO_HPM_G_AUTO_RESET_ON_INT 		 =0x30,
} LSM6DS3_ACC_GYRO_HPM_G_t;

#define  	LSM6DS3_ACC_GYRO_HPM_G_MASK  	0x30
status_t  LSM6DS3_ACC_GYRO_W_HPFilter_Mode_G(LSM6DS3_ACC_GYRO_HPM_G_t newValue);
status_t LSM6DS3_ACC_GYRO_R_HPFilter_Mode_G(LSM6DS3_ACC_GYRO_HPM_G_t *value);

/*******************************************************************************
* Register      : CTRL7_G
* Address       : 0X16
* Bit Group Name: HP_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_HP_EN_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_HP_EN_ENABLED 		 =0x40,
} LSM6DS3_ACC_GYRO_HP_EN_t;

#define  	LSM6DS3_ACC_GYRO_HP_EN_MASK  	0x40
status_t  LSM6DS3_ACC_GYRO_W_HPFilter_En(LSM6DS3_ACC_GYRO_HP_EN_t newValue);
status_t LSM6DS3_ACC_GYRO_R_HPFilter_En(LSM6DS3_ACC_GYRO_HP_EN_t *value);

/*******************************************************************************
* Register      : CTRL7_G
* Address       : 0X16
* Bit Group Name: LP_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_LP_EN_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_LP_EN_ENABLED 		 =0x80,
} LSM6DS3_ACC_GYRO_LP_EN_t;

#define  	LSM6DS3_ACC_GYRO_LP_EN_MASK  	0x80
status_t  LSM6DS3_ACC_GYRO_W_LP_Mode(LSM6DS3_ACC_GYRO_LP_EN_t newValue);
status_t LSM6DS3_ACC_GYRO_R_LP_Mode(LSM6DS3_ACC_GYRO_LP_EN_t *value);

/*******************************************************************************
* Register      : CTRL8_XL
* Address       : 0X17
* Bit Group Name: FDS
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_FDS_FILTER_OFF 		 =0x00,
  	LSM6DS3_ACC_GYRO_FDS_FILTER_ON 		 =0x04,
} LSM6DS3_ACC_GYRO_FDS_t;

#define  	LSM6DS3_ACC_GYRO_FDS_MASK  	0x04
status_t  LSM6DS3_ACC_GYRO_W_FDS(LSM6DS3_ACC_GYRO_FDS_t newValue);
status_t LSM6DS3_ACC_GYRO_R_FDS(LSM6DS3_ACC_GYRO_FDS_t *value);

/*******************************************************************************
* Register      : CTRL9_XL
* Address       : 0X18
* Bit Group Name: XEN_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_XEN_XL_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_XEN_XL_ENABLED 		 =0x08,
} LSM6DS3_ACC_GYRO_XEN_XL_t;

#define  	LSM6DS3_ACC_GYRO_XEN_XL_MASK  	0x08
status_t  LSM6DS3_ACC_GYRO_W_XEN_XL(LSM6DS3_ACC_GYRO_XEN_XL_t newValue);
status_t LSM6DS3_ACC_GYRO_R_XEN_XL(LSM6DS3_ACC_GYRO_XEN_XL_t *value);

/*******************************************************************************
* Register      : CTRL9_XL
* Address       : 0X18
* Bit Group Name: YEN_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_YEN_XL_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_YEN_XL_ENABLED 		 =0x10,
} LSM6DS3_ACC_GYRO_YEN_XL_t;

#define  	LSM6DS3_ACC_GYRO_YEN_XL_MASK  	0x10
status_t  LSM6DS3_ACC_GYRO_W_YEN_XL(LSM6DS3_ACC_GYRO_YEN_XL_t newValue);
status_t LSM6DS3_ACC_GYRO_R_YEN_XL(LSM6DS3_ACC_GYRO_YEN_XL_t *value);

/*******************************************************************************
* Register      : CTRL9_XL
* Address       : 0X18
* Bit Group Name: ZEN_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_ZEN_XL_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_ZEN_XL_ENABLED 		 =0x20,
} LSM6DS3_ACC_GYRO_ZEN_XL_t;

#define  	LSM6DS3_ACC_GYRO_ZEN_XL_MASK  	0x20
status_t  LSM6DS3_ACC_GYRO_W_ZEN_XL(LSM6DS3_ACC_GYRO_ZEN_XL_t newValue);
status_t LSM6DS3_ACC_GYRO_R_ZEN_XL(LSM6DS3_ACC_GYRO_ZEN_XL_t *value);

/*******************************************************************************
* Register      : CTRL10_C
* Address       : 0X19
* Bit Group Name: SIGN_MOTION_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_SIGN_MOTION_EN_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_SIGN_MOTION_EN_ENABLED 		 =0x01,
} LSM6DS3_ACC_GYRO_SIGN_MOTION_EN_t;

#define  	LSM6DS3_ACC_GYRO_SIGN_MOTION_EN_MASK  	0x01
status_t  LSM6DS3_ACC_GYRO_W_SignifcantMotion(LSM6DS3_ACC_GYRO_SIGN_MOTION_EN_t newValue);
status_t LSM6DS3_ACC_GYRO_R_SignifcantMotion(LSM6DS3_ACC_GYRO_SIGN_MOTION_EN_t *value);

/*******************************************************************************
* Register      : CTRL10_C
* Address       : 0X19
* Bit Group Name: PEDO_RST_STEP
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_PEDO_RST_STEP_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_PEDO_RST_STEP_ENABLED 		 =0x02,
} LSM6DS3_ACC_GYRO_PEDO_RST_STEP_t;

#define  	LSM6DS3_ACC_GYRO_PEDO_RST_STEP_MASK  	0x02
status_t  LSM6DS3_ACC_GYRO_W_PedoStepReset(LSM6DS3_ACC_GYRO_PEDO_RST_STEP_t newValue);
status_t LSM6DS3_ACC_GYRO_R_PedoStepReset(LSM6DS3_ACC_GYRO_PEDO_RST_STEP_t *value);

/*******************************************************************************
* Register      : CTRL10_C
* Address       : 0X19
* Bit Group Name: XEN_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_XEN_G_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_XEN_G_ENABLED 		 =0x08,
} LSM6DS3_ACC_GYRO_XEN_G_t;

#define  	LSM6DS3_ACC_GYRO_XEN_G_MASK  	0x08
status_t  LSM6DS3_ACC_GYRO_W_XEN_G(LSM6DS3_ACC_GYRO_XEN_G_t newValue);
status_t LSM6DS3_ACC_GYRO_R_XEN_G(LSM6DS3_ACC_GYRO_XEN_G_t *value);

/*******************************************************************************
* Register      : CTRL10_C
* Address       : 0X19
* Bit Group Name: YEN_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_YEN_G_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_YEN_G_ENABLED 		 =0x10,
} LSM6DS3_ACC_GYRO_YEN_G_t;

#define  	LSM6DS3_ACC_GYRO_YEN_G_MASK  	0x10
status_t  LSM6DS3_ACC_GYRO_W_YEN_G(LSM6DS3_ACC_GYRO_YEN_G_t newValue);
status_t LSM6DS3_ACC_GYRO_R_YEN_G(LSM6DS3_ACC_GYRO_YEN_G_t *value);

/*******************************************************************************
* Register      : CTRL10_C
* Address       : 0X19
* Bit Group Name: ZEN_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_ZEN_G_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_ZEN_G_ENABLED 		 =0x20,
} LSM6DS3_ACC_GYRO_ZEN_G_t;

#define  	LSM6DS3_ACC_GYRO_ZEN_G_MASK  	0x20
status_t  LSM6DS3_ACC_GYRO_W_ZEN_G(LSM6DS3_ACC_GYRO_ZEN_G_t newValue);
status_t LSM6DS3_ACC_GYRO_R_ZEN_G(LSM6DS3_ACC_GYRO_ZEN_G_t *value);

/*******************************************************************************
* Register      : CTRL10_C
* Address       : 0X19
* Bit Group Name: FUNC_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_FUNC_EN_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED 		 =0x04,
} LSM6DS3_ACC_GYRO_FUNC_EN_t;

#define  	LSM6DS3_ACC_GYRO_FUNC_EN_MASK  	0x04
status_t  LSM6DS3_ACC_GYRO_W_FUNC_EN(LSM6DS3_ACC_GYRO_FUNC_EN_t newValue);
status_t LSM6DS3_ACC_GYRO_R_FUNC_EN(LSM6DS3_ACC_GYRO_FUNC_EN_t *value);

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0X1A
* Bit Group Name: MASTER_ON
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_MASTER_ON_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_MASTER_ON_ENABLED 		 =0x01,
} LSM6DS3_ACC_GYRO_MASTER_ON_t;

#define  	LSM6DS3_ACC_GYRO_MASTER_ON_MASK  	0x01
status_t  LSM6DS3_ACC_GYRO_W_I2C_MASTER_Enable(LSM6DS3_ACC_GYRO_MASTER_ON_t newValue);
status_t LSM6DS3_ACC_GYRO_R_I2C_MASTER_Enable(LSM6DS3_ACC_GYRO_MASTER_ON_t *value);

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0X1A
* Bit Group Name: IRON_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_IRON_EN_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_IRON_EN_ENABLED 		 =0x02,
} LSM6DS3_ACC_GYRO_IRON_EN_t;

#define  	LSM6DS3_ACC_GYRO_IRON_EN_MASK  	0x02
status_t  LSM6DS3_ACC_GYRO_W_IronCorrection_EN(LSM6DS3_ACC_GYRO_IRON_EN_t newValue);
status_t LSM6DS3_ACC_GYRO_R_IronCorrection_EN(LSM6DS3_ACC_GYRO_IRON_EN_t *value);

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0X1A
* Bit Group Name: PASS_THRU_MODE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_PASS_THRU_MODE_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_PASS_THRU_MODE_ENABLED 		 =0x04,
} LSM6DS3_ACC_GYRO_PASS_THRU_MODE_t;

#define  	LSM6DS3_ACC_GYRO_PASS_THRU_MODE_MASK  	0x04
status_t  LSM6DS3_ACC_GYRO_W_PASS_THRU_MODE(LSM6DS3_ACC_GYRO_PASS_THRU_MODE_t newValue);
status_t LSM6DS3_ACC_GYRO_R_PASS_THRU_MODE(LSM6DS3_ACC_GYRO_PASS_THRU_MODE_t *value);

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0X1A
* Bit Group Name: PULL_UP_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_PULL_UP_EN_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_PULL_UP_EN_ENABLED 		 =0x08,
} LSM6DS3_ACC_GYRO_PULL_UP_EN_t;

#define  	LSM6DS3_ACC_GYRO_PULL_UP_EN_MASK  	0x08
status_t  LSM6DS3_ACC_GYRO_W_PULL_UP_EN(LSM6DS3_ACC_GYRO_PULL_UP_EN_t newValue);
status_t LSM6DS3_ACC_GYRO_R_PULL_UP_EN(LSM6DS3_ACC_GYRO_PULL_UP_EN_t *value);

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0X1A
* Bit Group Name: START_CONFIG
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_START_CONFIG_XL_G_DRDY 		 =0x00,
  	LSM6DS3_ACC_GYRO_START_CONFIG_EXT_INT2 		 =0x10,
} LSM6DS3_ACC_GYRO_START_CONFIG_t;

#define  	LSM6DS3_ACC_GYRO_START_CONFIG_MASK  	0x10
status_t  LSM6DS3_ACC_GYRO_W_SensorHUB_Trigger_Sel(LSM6DS3_ACC_GYRO_START_CONFIG_t newValue);
status_t LSM6DS3_ACC_GYRO_R_SensorHUB_Trigger_Sel(LSM6DS3_ACC_GYRO_START_CONFIG_t *value);

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0X1A
* Bit Group Name: DATA_VAL_SEL_FIFO
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_DATA_VAL_SEL_FIFO_XL_G_DRDY 		 =0x00,
  	LSM6DS3_ACC_GYRO_DATA_VAL_SEL_FIFO_SHUB_DRDY 		 =0x40,
} LSM6DS3_ACC_GYRO_DATA_VAL_SEL_FIFO_t;

#define  	LSM6DS3_ACC_GYRO_DATA_VAL_SEL_FIFO_MASK  	0x40
status_t  LSM6DS3_ACC_GYRO_W_DATA_VAL_SEL_FIFO(LSM6DS3_ACC_GYRO_DATA_VAL_SEL_FIFO_t newValue);
status_t LSM6DS3_ACC_GYRO_R_DATA_VAL_SEL_FIFO(LSM6DS3_ACC_GYRO_DATA_VAL_SEL_FIFO_t *value);

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0X1A
* Bit Group Name: DRDY_ON_INT1
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_DRDY_ON_INT1_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_DRDY_ON_INT1_ENABLED 		 =0x80,
} LSM6DS3_ACC_GYRO_DRDY_ON_INT1_t;

#define  	LSM6DS3_ACC_GYRO_DRDY_ON_INT1_MASK  	0x80
status_t  LSM6DS3_ACC_GYRO_W_DRDY_ON_INT1(LSM6DS3_ACC_GYRO_DRDY_ON_INT1_t newValue);
status_t LSM6DS3_ACC_GYRO_R_DRDY_ON_INT1(LSM6DS3_ACC_GYRO_DRDY_ON_INT1_t *value);

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0X1B
* Bit Group Name: Z_WU
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_Z_WU_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_Z_WU_DETECTED 		 =0x01,
} LSM6DS3_ACC_GYRO_Z_WU_t;

#define  	LSM6DS3_ACC_GYRO_Z_WU_MASK  	0x01
status_t LSM6DS3_ACC_GYRO_R_Z_WU(LSM6DS3_ACC_GYRO_Z_WU_t *value);

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0X1B
* Bit Group Name: Y_WU
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_Y_WU_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_Y_WU_DETECTED 		 =0x02,
} LSM6DS3_ACC_GYRO_Y_WU_t;

#define  	LSM6DS3_ACC_GYRO_Y_WU_MASK  	0x02
status_t LSM6DS3_ACC_GYRO_R_Y_WU(LSM6DS3_ACC_GYRO_Y_WU_t *value);

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0X1B
* Bit Group Name: X_WU
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_X_WU_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_X_WU_DETECTED 		 =0x04,
} LSM6DS3_ACC_GYRO_X_WU_t;

#define  	LSM6DS3_ACC_GYRO_X_WU_MASK  	0x04
status_t LSM6DS3_ACC_GYRO_R_X_WU(LSM6DS3_ACC_GYRO_X_WU_t *value);

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0X1B
* Bit Group Name: WU_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_WU_EV_STATUS_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_WU_EV_STATUS_DETECTED 		 =0x08,
} LSM6DS3_ACC_GYRO_WU_EV_STATUS_t;

#define  	LSM6DS3_ACC_GYRO_WU_EV_STATUS_MASK  	0x08
status_t LSM6DS3_ACC_GYRO_R_WU_EV_STATUS(LSM6DS3_ACC_GYRO_WU_EV_STATUS_t *value);

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0X1B
* Bit Group Name: SLEEP_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_SLEEP_EV_STATUS_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_SLEEP_EV_STATUS_DETECTED 		 =0x10,
} LSM6DS3_ACC_GYRO_SLEEP_EV_STATUS_t;

#define  	LSM6DS3_ACC_GYRO_SLEEP_EV_STATUS_MASK  	0x10
status_t LSM6DS3_ACC_GYRO_R_SLEEP_EV_STATUS(LSM6DS3_ACC_GYRO_SLEEP_EV_STATUS_t *value);

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0X1B
* Bit Group Name: FF_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_FF_EV_STATUS_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_FF_EV_STATUS_DETECTED 		 =0x20,
} LSM6DS3_ACC_GYRO_FF_EV_STATUS_t;

#define  	LSM6DS3_ACC_GYRO_FF_EV_STATUS_MASK  	0x20
status_t LSM6DS3_ACC_GYRO_R_FF_EV_STATUS(LSM6DS3_ACC_GYRO_FF_EV_STATUS_t *value);

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0X1C
* Bit Group Name: Z_TAP
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_Z_TAP_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_Z_TAP_DETECTED 		 =0x01,
} LSM6DS3_ACC_GYRO_Z_TAP_t;

#define  	LSM6DS3_ACC_GYRO_Z_TAP_MASK  	0x01
status_t LSM6DS3_ACC_GYRO_R_Z_TAP(LSM6DS3_ACC_GYRO_Z_TAP_t *value);

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0X1C
* Bit Group Name: Y_TAP
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_Y_TAP_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_Y_TAP_DETECTED 		 =0x02,
} LSM6DS3_ACC_GYRO_Y_TAP_t;

#define  	LSM6DS3_ACC_GYRO_Y_TAP_MASK  	0x02
status_t LSM6DS3_ACC_GYRO_R_Y_TAP(LSM6DS3_ACC_GYRO_Y_TAP_t *value);

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0X1C
* Bit Group Name: X_TAP
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_X_TAP_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_X_TAP_DETECTED 		 =0x04,
} LSM6DS3_ACC_GYRO_X_TAP_t;

#define  	LSM6DS3_ACC_GYRO_X_TAP_MASK  	0x04
status_t LSM6DS3_ACC_GYRO_R_X_TAP(LSM6DS3_ACC_GYRO_X_TAP_t *value);

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0X1C
* Bit Group Name: TAP_SIGN
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_TAP_SIGN_POS_SIGN 		 =0x00,
  	LSM6DS3_ACC_GYRO_TAP_SIGN_NEG_SIGN 		 =0x08,
} LSM6DS3_ACC_GYRO_TAP_SIGN_t;

#define  	LSM6DS3_ACC_GYRO_TAP_SIGN_MASK  	0x08
status_t LSM6DS3_ACC_GYRO_R_TAP_SIGN(LSM6DS3_ACC_GYRO_TAP_SIGN_t *value);

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0X1C
* Bit Group Name: DOUBLE_TAP_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_DOUBLE_TAP_EV_STATUS_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_DOUBLE_TAP_EV_STATUS_DETECTED 		 =0x10,
} LSM6DS3_ACC_GYRO_DOUBLE_TAP_EV_STATUS_t;

#define  	LSM6DS3_ACC_GYRO_DOUBLE_TAP_EV_STATUS_MASK  	0x10
status_t LSM6DS3_ACC_GYRO_R_DOUBLE_TAP_EV_STATUS(LSM6DS3_ACC_GYRO_DOUBLE_TAP_EV_STATUS_t *value);

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0X1C
* Bit Group Name: SINGLE_TAP_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_SINGLE_TAP_EV_STATUS_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_SINGLE_TAP_EV_STATUS_DETECTED 		 =0x20,
} LSM6DS3_ACC_GYRO_SINGLE_TAP_EV_STATUS_t;

#define  	LSM6DS3_ACC_GYRO_SINGLE_TAP_EV_STATUS_MASK  	0x20
status_t LSM6DS3_ACC_GYRO_R_SINGLE_TAP_EV_STATUS(LSM6DS3_ACC_GYRO_SINGLE_TAP_EV_STATUS_t *value);

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0X1C
* Bit Group Name: TAP_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_TAP_EV_STATUS_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_TAP_EV_STATUS_DETECTED 		 =0x40,
} LSM6DS3_ACC_GYRO_TAP_EV_STATUS_t;

#define  	LSM6DS3_ACC_GYRO_TAP_EV_STATUS_MASK  	0x40
status_t LSM6DS3_ACC_GYRO_R_TAP_EV_STATUS(LSM6DS3_ACC_GYRO_TAP_EV_STATUS_t *value);

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0X1D
* Bit Group Name: DSD_XL
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_DSD_XL_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_DSD_XL_DETECTED 		 =0x01,
} LSM6DS3_ACC_GYRO_DSD_XL_t;

#define  	LSM6DS3_ACC_GYRO_DSD_XL_MASK  	0x01
status_t LSM6DS3_ACC_GYRO_R_DSD_XL(LSM6DS3_ACC_GYRO_DSD_XL_t *value);

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0X1D
* Bit Group Name: DSD_XH
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_DSD_XH_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_DSD_XH_DETECTED 		 =0x02,
} LSM6DS3_ACC_GYRO_DSD_XH_t;

#define  	LSM6DS3_ACC_GYRO_DSD_XH_MASK  	0x02
status_t LSM6DS3_ACC_GYRO_R_DSD_XH(LSM6DS3_ACC_GYRO_DSD_XH_t *value);

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0X1D
* Bit Group Name: DSD_YL
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_DSD_YL_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_DSD_YL_DETECTED 		 =0x04,
} LSM6DS3_ACC_GYRO_DSD_YL_t;

#define  	LSM6DS3_ACC_GYRO_DSD_YL_MASK  	0x04
status_t LSM6DS3_ACC_GYRO_R_DSD_YL(LSM6DS3_ACC_GYRO_DSD_YL_t *value);

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0X1D
* Bit Group Name: DSD_YH
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_DSD_YH_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_DSD_YH_DETECTED 		 =0x08,
} LSM6DS3_ACC_GYRO_DSD_YH_t;

#define  	LSM6DS3_ACC_GYRO_DSD_YH_MASK  	0x08
status_t LSM6DS3_ACC_GYRO_R_DSD_YH(LSM6DS3_ACC_GYRO_DSD_YH_t *value);

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0X1D
* Bit Group Name: DSD_ZL
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_DSD_ZL_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_DSD_ZL_DETECTED 		 =0x10,
} LSM6DS3_ACC_GYRO_DSD_ZL_t;

#define  	LSM6DS3_ACC_GYRO_DSD_ZL_MASK  	0x10
status_t LSM6DS3_ACC_GYRO_R_DSD_ZL(LSM6DS3_ACC_GYRO_DSD_ZL_t *value);

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0X1D
* Bit Group Name: DSD_ZH
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_DSD_ZH_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_DSD_ZH_DETECTED 		 =0x20,
} LSM6DS3_ACC_GYRO_DSD_ZH_t;

#define  	LSM6DS3_ACC_GYRO_DSD_ZH_MASK  	0x20
status_t LSM6DS3_ACC_GYRO_R_DSD_ZH(LSM6DS3_ACC_GYRO_DSD_ZH_t *value);

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0X1D
* Bit Group Name: D6D_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_D6D_EV_STATUS_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_D6D_EV_STATUS_DETECTED 		 =0x40,
} LSM6DS3_ACC_GYRO_D6D_EV_STATUS_t;

#define  	LSM6DS3_ACC_GYRO_D6D_EV_STATUS_MASK  	0x40
status_t LSM6DS3_ACC_GYRO_R_D6D_EV_STATUS(LSM6DS3_ACC_GYRO_D6D_EV_STATUS_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X1E
* Bit Group Name: XLDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_XLDA_NO_DATA_AVAIL 		 =0x00,
  	LSM6DS3_ACC_GYRO_XLDA_DATA_AVAIL 		 =0x01,
} LSM6DS3_ACC_GYRO_XLDA_t;

#define  	LSM6DS3_ACC_GYRO_XLDA_MASK  	0x01
status_t LSM6DS3_ACC_GYRO_R_XLDA(LSM6DS3_ACC_GYRO_XLDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X1E
* Bit Group Name: GDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_GDA_NO_DATA_AVAIL 		 =0x00,
  	LSM6DS3_ACC_GYRO_GDA_DATA_AVAIL 		 =0x02,
} LSM6DS3_ACC_GYRO_GDA_t;

#define  	LSM6DS3_ACC_GYRO_GDA_MASK  	0x02
status_t LSM6DS3_ACC_GYRO_R_GDA(LSM6DS3_ACC_GYRO_GDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X1E
* Bit Group Name: EV_BOOT
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_EV_BOOT_NO_BOOT_RUNNING 		 =0x00,
  	LSM6DS3_ACC_GYRO_EV_BOOT_BOOT_IS_RUNNING 		 =0x08,
} LSM6DS3_ACC_GYRO_EV_BOOT_t;

#define  	LSM6DS3_ACC_GYRO_EV_BOOT_MASK  	0x08
status_t LSM6DS3_ACC_GYRO_R_EV_BOOT(LSM6DS3_ACC_GYRO_EV_BOOT_t *value);

/*******************************************************************************
* Register      : FIFO_STATUS1
* Address       : 0X3A
* Bit Group Name: DIFF_FIFO
* Permission    : RO
*******************************************************************************/
#define  	LSM6DS3_ACC_GYRO_DIFF_FIFO_STATUS1_MASK  	0xFF
#define  	LSM6DS3_ACC_GYRO_DIFF_FIFO_STATUS1_POSITION  	0
#define  	LSM6DS3_ACC_GYRO_DIFF_FIFO_STATUS2_MASK  0xF
#define  	LSM6DS3_ACC_GYRO_DIFF_FIFO_STATUS2_POSITION  	0
status_t LSM6DS3_ACC_GYRO_R_FIFONumOfEntries(u16_t *value);

/*******************************************************************************
* Register      : FIFO_STATUS2
* Address       : 0X3B
* Bit Group Name: FIFO_EMPTY
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_FIFO_EMPTY_FIFO_NOT_EMPTY 		 =0x00,
  	LSM6DS3_ACC_GYRO_FIFO_EMPTY_FIFO_EMPTY 		 =0x10,
} LSM6DS3_ACC_GYRO_FIFO_EMPTY_t;

#define  	LSM6DS3_ACC_GYRO_FIFO_EMPTY_MASK  	0x10
status_t LSM6DS3_ACC_GYRO_R_FIFOEmpty(LSM6DS3_ACC_GYRO_FIFO_EMPTY_t *value);

/*******************************************************************************
* Register      : FIFO_STATUS2
* Address       : 0X3B
* Bit Group Name: FIFO_FULL
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_FIFO_FULL_FIFO_NOT_FULL 		 =0x00,
  	LSM6DS3_ACC_GYRO_FIFO_FULL_FIFO_FULL 		 =0x20,
} LSM6DS3_ACC_GYRO_FIFO_FULL_t;

#define  	LSM6DS3_ACC_GYRO_FIFO_FULL_MASK  	0x20
status_t LSM6DS3_ACC_GYRO_R_FIFOFull(LSM6DS3_ACC_GYRO_FIFO_FULL_t *value);

/*******************************************************************************
* Register      : FIFO_STATUS2
* Address       : 0X3B
* Bit Group Name: OVERRUN
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_OVERRUN_NO_OVERRUN 		 =0x00,
  	LSM6DS3_ACC_GYRO_OVERRUN_OVERRUN 		 =0x40,
} LSM6DS3_ACC_GYRO_OVERRUN_t;

#define  	LSM6DS3_ACC_GYRO_OVERRUN_MASK  	0x40
status_t LSM6DS3_ACC_GYRO_R_OVERRUN(LSM6DS3_ACC_GYRO_OVERRUN_t *value);

/*******************************************************************************
* Register      : FIFO_STATUS2
* Address       : 0X3B
* Bit Group Name: WTM
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_WTM_BELOW_WTM 		 =0x00,
  	LSM6DS3_ACC_GYRO_WTM_ABOVE_OR_EQUAL_WTM 		 =0x80,
} LSM6DS3_ACC_GYRO_WTM_t;

#define  	LSM6DS3_ACC_GYRO_WTM_MASK  	0x80
status_t LSM6DS3_ACC_GYRO_R_WaterMark(LSM6DS3_ACC_GYRO_WTM_t *value);

/*******************************************************************************
* Register      : FIFO_STATUS3
* Address       : 0X3C
* Bit Group Name: FIFO_PATTERN
* Permission    : RO
*******************************************************************************/
#define  	LSM6DS3_ACC_GYRO_FIFO_STATUS3_PATTERN_MASK  	0xFF
#define  	LSM6DS3_ACC_GYRO_FIFO_STATUS3_PATTERN_POSITION  	0
#define  	LSM6DS3_ACC_GYRO_FIFO_STATUS4_PATTERN_MASK  	0x03
#define  	LSM6DS3_ACC_GYRO_FIFO_STATUS4_PATTERN_POSITION  	0
status_t LSM6DS3_ACC_GYRO_R_FIFOPattern(u16_t *value);

/*******************************************************************************
* Register      : FUNC_SRC
* Address       : 0X53
* Bit Group Name: SENS_HUB_END
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_SENS_HUB_END_STILL_READING 		 =0x00,
  	LSM6DS3_ACC_GYRO_SENS_HUB_END_OP_COMPLETED 		 =0x01,
} LSM6DS3_ACC_GYRO_SENS_HUB_END_t;

#define  	LSM6DS3_ACC_GYRO_SENS_HUB_END_MASK  	0x01
status_t LSM6DS3_ACC_GYRO_R_SENS_HUB_END(LSM6DS3_ACC_GYRO_SENS_HUB_END_t *value);

/*******************************************************************************
* Register      : FUNC_SRC
* Address       : 0X53
* Bit Group Name: SOFT_IRON_END
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_SOFT_IRON_END_NOT_COMPLETED 		 =0x00,
  	LSM6DS3_ACC_GYRO_SOFT_IRON_END_COMPLETED 		 =0x02,
} LSM6DS3_ACC_GYRO_SOFT_IRON_END_t;

#define  	LSM6DS3_ACC_GYRO_SOFT_IRON_END_MASK  	0x02
status_t LSM6DS3_ACC_GYRO_R_SOFT_IRON_END(LSM6DS3_ACC_GYRO_SOFT_IRON_END_t *value);

/*******************************************************************************
* Register      : FUNC_SRC
* Address       : 0X53
* Bit Group Name: PEDO_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_PEDO_EV_STATUS_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_PEDO_EV_STATUS_DETECTED 		 =0x10,
} LSM6DS3_ACC_GYRO_PEDO_EV_STATUS_t;

#define  	LSM6DS3_ACC_GYRO_PEDO_EV_STATUS_MASK  	0x10
status_t LSM6DS3_ACC_GYRO_R_PEDO_EV_STATUS(LSM6DS3_ACC_GYRO_PEDO_EV_STATUS_t *value);

/*******************************************************************************
* Register      : FUNC_SRC
* Address       : 0X53
* Bit Group Name: TILT_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_TILT_EV_STATUS_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_TILT_EV_STATUS_DETECTED 		 =0x20,
} LSM6DS3_ACC_GYRO_TILT_EV_STATUS_t;

#define  	LSM6DS3_ACC_GYRO_TILT_EV_STATUS_MASK  	0x20
status_t LSM6DS3_ACC_GYRO_R_TILT_EV_STATUS(LSM6DS3_ACC_GYRO_TILT_EV_STATUS_t *value);

/*******************************************************************************
* Register      : FUNC_SRC
* Address       : 0X53
* Bit Group Name: SIGN_MOT_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_SIGN_MOT_EV_STATUS_NOT_DETECTED 		 =0x00,
  	LSM6DS3_ACC_GYRO_SIGN_MOT_EV_STATUS_DETECTED 		 =0x40,
} LSM6DS3_ACC_GYRO_SIGN_MOT_EV_STATUS_t;

#define  	LSM6DS3_ACC_GYRO_SIGN_MOT_EV_STATUS_MASK  	0x40
status_t LSM6DS3_ACC_GYRO_R_SIGN_MOT_EV_STATUS(LSM6DS3_ACC_GYRO_SIGN_MOT_EV_STATUS_t *value);

/*******************************************************************************
* Register      : TAP_CFG1
* Address       : 0X58
* Bit Group Name: LIR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_LIR_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_LIR_ENABLED 		 =0x01,
} LSM6DS3_ACC_GYRO_LIR_t;

#define  	LSM6DS3_ACC_GYRO_LIR_MASK  	0x01
status_t  LSM6DS3_ACC_GYRO_W_LIR(LSM6DS3_ACC_GYRO_LIR_t newValue);
status_t LSM6DS3_ACC_GYRO_R_LIR(LSM6DS3_ACC_GYRO_LIR_t *value);

/*******************************************************************************
* Register      : TAP_CFG1
* Address       : 0X58
* Bit Group Name: TAP_Z_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_TAP_Z_EN_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_TAP_Z_EN_ENABLED 		 =0x02,
} LSM6DS3_ACC_GYRO_TAP_Z_EN_t;

#define  	LSM6DS3_ACC_GYRO_TAP_Z_EN_MASK  	0x02
status_t  LSM6DS3_ACC_GYRO_W_TAP_Z_EN(LSM6DS3_ACC_GYRO_TAP_Z_EN_t newValue);
status_t LSM6DS3_ACC_GYRO_R_TAP_Z_EN(LSM6DS3_ACC_GYRO_TAP_Z_EN_t *value);

/*******************************************************************************
* Register      : TAP_CFG1
* Address       : 0X58
* Bit Group Name: TAP_Y_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_TAP_Y_EN_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_TAP_Y_EN_ENABLED 		 =0x04,
} LSM6DS3_ACC_GYRO_TAP_Y_EN_t;

#define  	LSM6DS3_ACC_GYRO_TAP_Y_EN_MASK  	0x04
status_t  LSM6DS3_ACC_GYRO_W_TAP_Y_EN(LSM6DS3_ACC_GYRO_TAP_Y_EN_t newValue);
status_t LSM6DS3_ACC_GYRO_R_TAP_Y_EN(LSM6DS3_ACC_GYRO_TAP_Y_EN_t *value);

/*******************************************************************************
* Register      : TAP_CFG1
* Address       : 0X58
* Bit Group Name: TAP_X_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_TAP_X_EN_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_TAP_X_EN_ENABLED 		 =0x08,
} LSM6DS3_ACC_GYRO_TAP_X_EN_t;

#define  	LSM6DS3_ACC_GYRO_TAP_X_EN_MASK  	0x08
status_t  LSM6DS3_ACC_GYRO_W_TAP_X_EN(LSM6DS3_ACC_GYRO_TAP_X_EN_t newValue);
status_t LSM6DS3_ACC_GYRO_R_TAP_X_EN(LSM6DS3_ACC_GYRO_TAP_X_EN_t *value);

/*******************************************************************************
* Register      : TAP_CFG1
* Address       : 0X58
* Bit Group Name: TILT_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_TILT_EN_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_TILT_EN_ENABLED 		 =0x20,
} LSM6DS3_ACC_GYRO_TILT_EN_t;

#define  	LSM6DS3_ACC_GYRO_TILT_EN_MASK  	0x20
status_t  LSM6DS3_ACC_GYRO_W_TILT_EN(LSM6DS3_ACC_GYRO_TILT_EN_t newValue);
status_t LSM6DS3_ACC_GYRO_R_TILT_EN(LSM6DS3_ACC_GYRO_TILT_EN_t *value);

/*******************************************************************************
* Register      : TAP_CFG1
* Address       : 0X58
* Bit Group Name: PEDO_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_PEDO_EN_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_PEDO_EN_ENABLED 		 =0x40,
} LSM6DS3_ACC_GYRO_PEDO_EN_t;

#define  	LSM6DS3_ACC_GYRO_PEDO_EN_MASK  	0x40
status_t  LSM6DS3_ACC_GYRO_W_PEDO_EN(LSM6DS3_ACC_GYRO_PEDO_EN_t newValue);
status_t LSM6DS3_ACC_GYRO_R_PEDO_EN(LSM6DS3_ACC_GYRO_PEDO_EN_t *value);

/*******************************************************************************
* Register      : TAP_CFG1
* Address       : 0X58
* Bit Group Name: TIMER_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_TIMER_EN_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_TIMER_EN_ENABLED 		 =0x80,
} LSM6DS3_ACC_GYRO_TIMER_EN_t;

#define  	LSM6DS3_ACC_GYRO_TIMER_EN_MASK  	0x80
status_t  LSM6DS3_ACC_GYRO_W_TIMER_EN(LSM6DS3_ACC_GYRO_TIMER_EN_t newValue);
status_t LSM6DS3_ACC_GYRO_R_TIMER_EN(LSM6DS3_ACC_GYRO_TIMER_EN_t *value);

/*******************************************************************************
* Register      : TAP_THS_6D
* Address       : 0X59
* Bit Group Name: TAP_THS
* Permission    : RW
*******************************************************************************/
#define  	LSM6DS3_ACC_GYRO_TAP_THS_MASK  	0x1F
#define  	LSM6DS3_ACC_GYRO_TAP_THS_POSITION  	0
status_t  LSM6DS3_ACC_GYRO_W_TAP_THS(u8_t newValue);
status_t LSM6DS3_ACC_GYRO_R_TAP_THS(u8_t *value);

/*******************************************************************************
* Register      : TAP_THS_6D
* Address       : 0X59
* Bit Group Name: SIXD_THS
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_SIXD_THS_80_degree 		 =0x00,
  	LSM6DS3_ACC_GYRO_SIXD_THS_70_degree 		 =0x20,
  	LSM6DS3_ACC_GYRO_SIXD_THS_60_degree 		 =0x40,
  	LSM6DS3_ACC_GYRO_SIXD_THS_50_degree 		 =0x60,
} LSM6DS3_ACC_GYRO_SIXD_THS_t;

#define  	LSM6DS3_ACC_GYRO_SIXD_THS_MASK  	0x60
status_t  LSM6DS3_ACC_GYRO_W_SIXD_THS(LSM6DS3_ACC_GYRO_SIXD_THS_t newValue);
status_t LSM6DS3_ACC_GYRO_R_SIXD_THS(LSM6DS3_ACC_GYRO_SIXD_THS_t *value);

/*******************************************************************************
* Register      : INT_DUR2
* Address       : 0X5A
* Bit Group Name: SHOCK
* Permission    : RW
*******************************************************************************/
#define  	LSM6DS3_ACC_GYRO_SHOCK_MASK  	0x03
#define  	LSM6DS3_ACC_GYRO_SHOCK_POSITION  	0
status_t  LSM6DS3_ACC_GYRO_W_SHOCK_Duration(u8_t newValue);
status_t LSM6DS3_ACC_GYRO_R_SHOCK_Duration(u8_t *value);

/*******************************************************************************
* Register      : INT_DUR2
* Address       : 0X5A
* Bit Group Name: QUIET
* Permission    : RW
*******************************************************************************/
#define  	LSM6DS3_ACC_GYRO_QUIET_MASK  	0x0C
#define  	LSM6DS3_ACC_GYRO_QUIET_POSITION  	2
status_t  LSM6DS3_ACC_GYRO_W_QUIET_Duration(u8_t newValue);
status_t LSM6DS3_ACC_GYRO_R_QUIET_Duration(u8_t *value);

/*******************************************************************************
* Register      : INT_DUR2
* Address       : 0X5A
* Bit Group Name: DUR
* Permission    : RW
*******************************************************************************/
#define  	LSM6DS3_ACC_GYRO_DUR_MASK  	0xF0
#define  	LSM6DS3_ACC_GYRO_DUR_POSITION  	4
status_t  LSM6DS3_ACC_GYRO_W_DUR(u8_t newValue);
status_t LSM6DS3_ACC_GYRO_R_DUR(u8_t *value);

/*******************************************************************************
* Register      : WAKE_UP_THS
* Address       : 0X5B
* Bit Group Name: WK_THS
* Permission    : RW
*******************************************************************************/
#define  	LSM6DS3_ACC_GYRO_WK_THS_MASK  	0x3F
#define  	LSM6DS3_ACC_GYRO_WK_THS_POSITION  	0
status_t  LSM6DS3_ACC_GYRO_W_WK_THS(u8_t newValue);
status_t LSM6DS3_ACC_GYRO_R_WK_THS(u8_t *value);

/*******************************************************************************
* Register      : WAKE_UP_THS
* Address       : 0X5B
* Bit Group Name: INACTIVITY_ON
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INACTIVITY_ON_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INACTIVITY_ON_ENABLED 		 =0x40,
} LSM6DS3_ACC_GYRO_INACTIVITY_ON_t;

#define  	LSM6DS3_ACC_GYRO_INACTIVITY_ON_MASK  	0x40
status_t  LSM6DS3_ACC_GYRO_W_INACTIVITY_ON(LSM6DS3_ACC_GYRO_INACTIVITY_ON_t newValue);
status_t LSM6DS3_ACC_GYRO_R_INACTIVITY_ON(LSM6DS3_ACC_GYRO_INACTIVITY_ON_t *value);

/*******************************************************************************
* Register      : WAKE_UP_THS
* Address       : 0X5B
* Bit Group Name: SINGLE_DOUBLE_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_DOUBLE_TAP 		 =0x00,
  	LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_SINGLE_TAP 		 =0x80,
} LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_t;

#define  	LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_MASK  	0x80
status_t  LSM6DS3_ACC_GYRO_W_SINGLE_DOUBLE_TAP_EV(LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_t newValue);
status_t LSM6DS3_ACC_GYRO_R_SINGLE_DOUBLE_TAP_EV(LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_t *value);

/*******************************************************************************
* Register      : WAKE_UP_DUR
* Address       : 0X5C
* Bit Group Name: SLEEP_DUR
* Permission    : RW
*******************************************************************************/
#define  	LSM6DS3_ACC_GYRO_SLEEP_DUR_MASK  	0x0F
#define  	LSM6DS3_ACC_GYRO_SLEEP_DUR_POSITION  	0
status_t  LSM6DS3_ACC_GYRO_W_SLEEP_DUR(u8_t newValue);
status_t LSM6DS3_ACC_GYRO_R_SLEEP_DUR(u8_t *value);

/*******************************************************************************
* Register      : WAKE_UP_DUR
* Address       : 0X5C
* Bit Group Name: TIMER_HR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_TIMER_HR_6_4ms 		 =0x00,
  	LSM6DS3_ACC_GYRO_TIMER_HR_25us 		 =0x10,
} LSM6DS3_ACC_GYRO_TIMER_HR_t;

#define  	LSM6DS3_ACC_GYRO_TIMER_HR_MASK  	0x10
status_t  LSM6DS3_ACC_GYRO_W_TIMER_HR(LSM6DS3_ACC_GYRO_TIMER_HR_t newValue);
status_t LSM6DS3_ACC_GYRO_R_TIMER_HR(LSM6DS3_ACC_GYRO_TIMER_HR_t *value);

/*******************************************************************************
* Register      : WAKE_UP_DUR
* Address       : 0X5C
* Bit Group Name: WAKE_DUR
* Permission    : RW
*******************************************************************************/
#define  	LSM6DS3_ACC_GYRO_WAKE_DUR_MASK  	0x60
#define  	LSM6DS3_ACC_GYRO_WAKE_DUR_POSITION  	5
status_t  LSM6DS3_ACC_GYRO_W_WAKE_DUR(u8_t newValue);
status_t LSM6DS3_ACC_GYRO_R_WAKE_DUR(u8_t *value);

/*******************************************************************************
* Register      : FREE_FALL
* Address       : 0X5D
* Bit Group Name: FF_DUR
* Permission    : RW
*******************************************************************************/
#define  	LSM6DS3_ACC_GYRO_FF_FREE_FALL_DUR_MASK  	0xF8
#define  	LSM6DS3_ACC_GYRO_FF_FREE_FALL_DUR_POSITION  	3
#define  	LSM6DS3_ACC_GYRO_FF_WAKE_UP_DUR_MASK  	0x80
#define  	LSM6DS3_ACC_GYRO_FF_WAKE_UP_DUR_POSITION  	7
status_t  LSM6DS3_ACC_GYRO_W_FF_Duration(u8_t newValue);
status_t LSM6DS3_ACC_GYRO_R_FF_Duration(u8_t *value);


/*******************************************************************************
* Register      : FREE_FALL
* Address       : 0X5D
* Bit Group Name: FF_THS
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_FF_THS_5 		 =0x00,
  	LSM6DS3_ACC_GYRO_FF_THS_7 		 =0x01,
  	LSM6DS3_ACC_GYRO_FF_THS_8 		 =0x02,
  	LSM6DS3_ACC_GYRO_FF_THS_10 		 =0x03,
  	LSM6DS3_ACC_GYRO_FF_THS_11 		 =0x04,
  	LSM6DS3_ACC_GYRO_FF_THS_13 		 =0x05,
  	LSM6DS3_ACC_GYRO_FF_THS_15 		 =0x06,
  	LSM6DS3_ACC_GYRO_FF_THS_16 		 =0x07,
} LSM6DS3_ACC_GYRO_FF_THS_t;

#define  	LSM6DS3_ACC_GYRO_FF_THS_MASK  	0x07
status_t  LSM6DS3_ACC_GYRO_W_FF_THS(LSM6DS3_ACC_GYRO_FF_THS_t newValue);
status_t LSM6DS3_ACC_GYRO_R_FF_THS(LSM6DS3_ACC_GYRO_FF_THS_t *value);

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0X5E
* Bit Group Name: INT1_TIMER
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT1_TIMER_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT1_TIMER_ENABLED 		 =0x01,
} LSM6DS3_ACC_GYRO_INT1_TIMER_t;

#define  	LSM6DS3_ACC_GYRO_INT1_TIMER_MASK  	0x01
status_t  LSM6DS3_ACC_GYRO_W_TimerEvRouteInt1(LSM6DS3_ACC_GYRO_INT1_TIMER_t newValue);
status_t LSM6DS3_ACC_GYRO_R_TimerEvRouteInt1(LSM6DS3_ACC_GYRO_INT1_TIMER_t *value);

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0X5E
* Bit Group Name: INT1_TILT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT1_TILT_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT1_TILT_ENABLED 		 =0x02,
} LSM6DS3_ACC_GYRO_INT1_TILT_t;

#define  	LSM6DS3_ACC_GYRO_INT1_TILT_MASK  	0x02
status_t  LSM6DS3_ACC_GYRO_W_TiltEvOnInt1(LSM6DS3_ACC_GYRO_INT1_TILT_t newValue);
status_t LSM6DS3_ACC_GYRO_R_TiltEvOnInt1(LSM6DS3_ACC_GYRO_INT1_TILT_t *value);

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0X5E
* Bit Group Name: INT1_6D
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT1_6D_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT1_6D_ENABLED 		 =0x04,
} LSM6DS3_ACC_GYRO_INT1_6D_t;

#define  	LSM6DS3_ACC_GYRO_INT1_6D_MASK  	0x04
status_t  LSM6DS3_ACC_GYRO_W_6DEvOnInt1(LSM6DS3_ACC_GYRO_INT1_6D_t newValue);
status_t LSM6DS3_ACC_GYRO_R_6DEvOnInt1(LSM6DS3_ACC_GYRO_INT1_6D_t *value);

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0X5E
* Bit Group Name: INT1_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT1_TAP_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT1_TAP_ENABLED 		 =0x08,
} LSM6DS3_ACC_GYRO_INT1_TAP_t;

#define  	LSM6DS3_ACC_GYRO_INT1_TAP_MASK  	0x08
status_t  LSM6DS3_ACC_GYRO_W_TapEvOnInt1(LSM6DS3_ACC_GYRO_INT1_TAP_t newValue);
status_t LSM6DS3_ACC_GYRO_R_TapEvOnInt1(LSM6DS3_ACC_GYRO_INT1_TAP_t *value);

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0X5E
* Bit Group Name: INT1_FF
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT1_FF_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT1_FF_ENABLED 		 =0x10,
} LSM6DS3_ACC_GYRO_INT1_FF_t;

#define  	LSM6DS3_ACC_GYRO_INT1_FF_MASK  	0x10
status_t  LSM6DS3_ACC_GYRO_W_FFEvOnInt1(LSM6DS3_ACC_GYRO_INT1_FF_t newValue);
status_t LSM6DS3_ACC_GYRO_R_FFEvOnInt1(LSM6DS3_ACC_GYRO_INT1_FF_t *value);

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0X5E
* Bit Group Name: INT1_WU
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT1_WU_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT1_WU_ENABLED 		 =0x20,
} LSM6DS3_ACC_GYRO_INT1_WU_t;

#define  	LSM6DS3_ACC_GYRO_INT1_WU_MASK  	0x20
status_t  LSM6DS3_ACC_GYRO_W_WUEvOnInt1(LSM6DS3_ACC_GYRO_INT1_WU_t newValue);
status_t LSM6DS3_ACC_GYRO_R_WUEvOnInt1(LSM6DS3_ACC_GYRO_INT1_WU_t *value);

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0X5E
* Bit Group Name: INT1_SINGLE_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_ENABLED 		 =0x40,
} LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_t;

#define  	LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_MASK  	0x40
status_t  LSM6DS3_ACC_GYRO_W_SingleTapOnInt1(LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_t newValue);
status_t LSM6DS3_ACC_GYRO_R_SingleTapOnInt1(LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_t *value);

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0X5E
* Bit Group Name: INT1_SLEEP
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT1_SLEEP_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT1_SLEEP_ENABLED 		 =0x80,
} LSM6DS3_ACC_GYRO_INT1_SLEEP_t;

#define  	LSM6DS3_ACC_GYRO_INT1_SLEEP_MASK  	0x80
status_t  LSM6DS3_ACC_GYRO_W_SleepEvOnInt1(LSM6DS3_ACC_GYRO_INT1_SLEEP_t newValue);
status_t LSM6DS3_ACC_GYRO_R_SleepEvOnInt1(LSM6DS3_ACC_GYRO_INT1_SLEEP_t *value);

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0X5F
* Bit Group Name: INT2_TIMER
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT2_TIMER_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT2_TIMER_ENABLED 		 =0x01,
} LSM6DS3_ACC_GYRO_INT2_TIMER_t;

#define  	LSM6DS3_ACC_GYRO_INT2_TIMER_MASK  	0x01
status_t  LSM6DS3_ACC_GYRO_W_TimerEvRouteInt2(LSM6DS3_ACC_GYRO_INT2_TIMER_t newValue);
status_t LSM6DS3_ACC_GYRO_R_TimerEvRouteInt2(LSM6DS3_ACC_GYRO_INT2_TIMER_t *value);

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0X5F
* Bit Group Name: INT2_TILT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT2_TILT_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT2_TILT_ENABLED 		 =0x02,
} LSM6DS3_ACC_GYRO_INT2_TILT_t;

#define  	LSM6DS3_ACC_GYRO_INT2_TILT_MASK  	0x02
status_t  LSM6DS3_ACC_GYRO_W_TiltEvOnInt2(LSM6DS3_ACC_GYRO_INT2_TILT_t newValue);
status_t LSM6DS3_ACC_GYRO_R_TiltEvOnInt2(LSM6DS3_ACC_GYRO_INT2_TILT_t *value);

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0X5F
* Bit Group Name: INT2_6D
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT2_6D_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT2_6D_ENABLED 		 =0x04,
} LSM6DS3_ACC_GYRO_INT2_6D_t;

#define  	LSM6DS3_ACC_GYRO_INT2_6D_MASK  	0x04
status_t  LSM6DS3_ACC_GYRO_W_6DEvOnInt2(LSM6DS3_ACC_GYRO_INT2_6D_t newValue);
status_t LSM6DS3_ACC_GYRO_R_6DEvOnInt2(LSM6DS3_ACC_GYRO_INT2_6D_t *value);

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0X5F
* Bit Group Name: INT2_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT2_TAP_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT2_TAP_ENABLED 		 =0x08,
} LSM6DS3_ACC_GYRO_INT2_TAP_t;

#define  	LSM6DS3_ACC_GYRO_INT2_TAP_MASK  	0x08
status_t  LSM6DS3_ACC_GYRO_W_TapEvOnInt2(LSM6DS3_ACC_GYRO_INT2_TAP_t newValue);
status_t LSM6DS3_ACC_GYRO_R_TapEvOnInt2(LSM6DS3_ACC_GYRO_INT2_TAP_t *value);

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0X5F
* Bit Group Name: INT2_FF
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT2_FF_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT2_FF_ENABLED 		 =0x10,
} LSM6DS3_ACC_GYRO_INT2_FF_t;

#define  	LSM6DS3_ACC_GYRO_INT2_FF_MASK  	0x10
status_t  LSM6DS3_ACC_GYRO_W_FFEvOnInt2(LSM6DS3_ACC_GYRO_INT2_FF_t newValue);
status_t LSM6DS3_ACC_GYRO_R_FFEvOnInt2(LSM6DS3_ACC_GYRO_INT2_FF_t *value);

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0X5F
* Bit Group Name: INT2_WU
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT2_WU_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT2_WU_ENABLED 		 =0x20,
} LSM6DS3_ACC_GYRO_INT2_WU_t;

#define  	LSM6DS3_ACC_GYRO_INT2_WU_MASK  	0x20
status_t  LSM6DS3_ACC_GYRO_W_WUEvOnInt2(LSM6DS3_ACC_GYRO_INT2_WU_t newValue);
status_t LSM6DS3_ACC_GYRO_R_WUEvOnInt2(LSM6DS3_ACC_GYRO_INT2_WU_t *value);

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0X5F
* Bit Group Name: INT2_SINGLE_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT2_SINGLE_TAP_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT2_SINGLE_TAP_ENABLED 		 =0x40,
} LSM6DS3_ACC_GYRO_INT2_SINGLE_TAP_t;

#define  	LSM6DS3_ACC_GYRO_INT2_SINGLE_TAP_MASK  	0x40
status_t  LSM6DS3_ACC_GYRO_W_SingleTapOnInt2(LSM6DS3_ACC_GYRO_INT2_SINGLE_TAP_t newValue);
status_t LSM6DS3_ACC_GYRO_R_SingleTapOnInt2(LSM6DS3_ACC_GYRO_INT2_SINGLE_TAP_t *value);

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0X5F
* Bit Group Name: INT2_SLEEP
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM6DS3_ACC_GYRO_INT2_SLEEP_DISABLED 		 =0x00,
  	LSM6DS3_ACC_GYRO_INT2_SLEEP_ENABLED 		 =0x80,
} LSM6DS3_ACC_GYRO_INT2_SLEEP_t;

#define  	LSM6DS3_ACC_GYRO_INT2_SLEEP_MASK  	0x80
status_t  LSM6DS3_ACC_GYRO_W_SleepEvOnInt2(LSM6DS3_ACC_GYRO_INT2_SLEEP_t newValue);
status_t LSM6DS3_ACC_GYRO_R_SleepEvOnInt2(LSM6DS3_ACC_GYRO_INT2_SLEEP_t *value);
/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : GetGyroData
* Permission    : RO 
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_Get_GetGyroData(u8_t *buff); 
/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : GetAccData
* Permission    : RO 
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_Get_GetAccData(u8_t *buff); 
/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : GetSnsr3Data
* Permission    : RO 
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_Get_GetSnsr3Data(u8_t *buff); 
/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : GetSnsr4Data
* Permission    : RO 
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_Get_GetSnsr4Data(u8_t *buff); 
/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : GetFIFOData
* Permission    : RO 
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_Get_GetFIFOData(u8_t *buff); 
/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : GetTimestamp
* Permission    : RO 
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_Get_GetTimestamp(u8_t *buff); 
/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : GetStepCounter
* Permission    : RO 
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_Get_GetStepCounter(u8_t *buff); 

/* program the RAM with code */
status_t LSM6DS3_ACC_GYRO_init_RAM(u8_t *code_bufp);

/************** Program Pedometer Threshold  *******************/
status_t  LSM6DS3_ACC_GYRO_W_PedoThreshold(u8_t newValue);
#endif
