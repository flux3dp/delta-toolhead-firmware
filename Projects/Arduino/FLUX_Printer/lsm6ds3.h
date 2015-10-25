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
#ifndef _LSM6DS3_H_
#define _LSM6DS3_H_

#include "stm32f0xx.h"

// ----------------------------------------------------------------------------
// Defines
// ----------------------------------------------------------------------------
#define LSM6DS3_ADDR          ((uint16_t)(0x6B))//((uint16_t)(0x38))

// Registers
#define LSM6DS3_REG_WHO_AM_I  ((uint8_t)(0x0F)) 
#define LSM6DS3_REG_CTRL_REG1 ((uint8_t)(0x20))
#define LSM6DS3_REG_OUTX      ((uint8_t)(0x29))
#define LSM6DS3_REG_OUTY      ((uint8_t)(0x2B))
#define LSM6DS3_REG_OUTZ      ((uint8_t)(0x2D))

// ----------------------------------------------------------------------------
// Function prototypes
// ----------------------------------------------------------------------------
void LSM6DS3_Setup(void);
uint8_t LSM6DS3_RegRead(uint8_t reg);
void LSM6DS3_RegWrite(uint8_t reg, uint8_t data);

#endif /* _LIS302DL_H_ */
