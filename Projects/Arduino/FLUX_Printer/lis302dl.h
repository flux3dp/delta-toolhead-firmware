/******************************************************************************
 * Project        : HAN ESE PRJ2, PRJ1V & PRJ1D
 * File           : LIS302DL implementation
 * Copyright      : 2013 HAN Embedded Systems Engineering
 ******************************************************************************
  Change History:

    Version 1.0 - May 2013
    > Initial revision

******************************************************************************
   Hardware:
                            LIS302DL breakout board v13
                       Vdd +-------------+
                        |  |             |
   ------------+        +--+Vcc          |
               |   GND|----+GND          |
   I2C1_SCL/PB6+-----------+SCL/SPC      |
   I2C1_SDA/PB7+-----------+MOSI/SDA/SDI |
               |   GND|----+MISO/SDO     |
            PB8+-----------+CS           |
               |           |             |
   ------------+       nc--+INT1         |
                       nc--+INT2         |
                           |             |
                           +-------------+

******************************************************************************/
#ifndef _LIS302DL_H_
#define _LIS302DL_H_

#include "stm32f0xx.h"

// ----------------------------------------------------------------------------
// Defines
// ----------------------------------------------------------------------------

//james 

#define LIS302DL_ADDR         ((uint16_t)(0xD7))//((uint16_t)(0x3B)) //0x38 
#define LSM6DS3_ADDR          ((uint16_t)(0xD4))   // SA0=0 , Write=0

// Registers
#define WHO_AM_I  ((uint8_t)(0x0F))

//LSM6DS3
#define FIFO_CTRL1   ((uint8_t)(0x06))
#define FIFO_CTRL2   ((uint8_t)(0x07))
#define FIFO_CTRL3   ((uint8_t)(0x08))
#define FIFO_CTRL4   ((uint8_t)(0x09))
#define FIFO_CTRL5   ((uint8_t)(0x0A))
#define CTRL1_XL     ((uint8_t)(0x10))   //60
#define CTRL4_C      ((uint8_t)(0x13))
#define FIFO_STATUS1 ((uint8_t)(0x3A))
#define FIFO_STATUS2 ((uint8_t)(0x3B))
#define FIFO_DATA_OUT_L ((uint8_t)(0x3E))



//#define FIFO_CTRL5  ((uint8_t)(0x06))

//#define LIS302DL_REG_CTRL_REG1 ((uint8_t)(0x10))  //0x20
#define CTRL1_XL  ((uint8_t)(0x10))   //60
#define CTRL2_G  ((uint8_t)(0x11))    //60
#define CTRL3_C  ((uint8_t)(0x12))    //04
#define CTRL9_XL  ((uint8_t)(0x18))   //38
#define CTRL10_C  ((uint8_t)(0x19))   //38
#define WAKE_UP_SRC  ((uint8_t)(0x1B)) //0F
#define D6D_SRC  ((uint8_t)(0x1D))     //20
#define STATUS_REG  ((uint8_t)(0x1E))  //07


#define OUTX_L_G  ((uint8_t)(0x22))  //07
#define OUTX_H_G  ((uint8_t)(0x23))  //07
#define OUTY_L_G  ((uint8_t)(0x24))  //07
#define OUTY_H_G  ((uint8_t)(0x25))  //07
#define OUTZ_L_G  ((uint8_t)(0x26))  //07
#define OUTZ_H_G  ((uint8_t)(0x27))  //07


#define LIS302DL_REG_OUTX_L    ((uint8_t)(0x28))  
#define LIS302DL_REG_OUTX      ((uint8_t)(0x29))
#define LIS302DL_REG_OUTY_L    ((uint8_t)(0x2A))
#define LIS302DL_REG_OUTY      ((uint8_t)(0x2B))
#define LIS302DL_REG_OUTZ_L    ((uint8_t)(0x2C))
#define LIS302DL_REG_OUTZ      ((uint8_t)(0x2D))


#define LSM6DS3_REG_CTRL_REG1   ((uint8_t)(0x10))  
#define LSM6DS3_REG_OUTX_L      ((uint8_t)(0x28)) 
#define LSM6DS3_REG_OUTX_H      ((uint8_t)(0x29))
#define LSM6DS3_REG_OUTY_L      ((uint8_t)(0x2A))  
#define LSM6DS3_REG_OUTY_H      ((uint8_t)(0x2B)) 
#define LSM6DS3_REG_OUTZ_L      ((uint8_t)(0x2C))  
#define LSM6DS3_REG_OUTZ_H      ((uint8_t)(0x2D))


// ----------------------------------------------------------------------------
// Function prototypes
// ----------------------------------------------------------------------------
void LIS302DL_Setup(void);
uint8_t LIS302DL_RegRead(uint8_t reg);
void LIS302DL_RegWrite(uint8_t reg, uint8_t data);

#endif /* _LIS302DL_H_ */
