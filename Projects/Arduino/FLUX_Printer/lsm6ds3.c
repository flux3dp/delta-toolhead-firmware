/******************************************************************************
 * Project        : HAN ESE PRJ2, PRJ1V & PRJ1D
 * File           : LIS302DL implementation
 * Copyright      : 2013 HAN Embedded Systems Engineering
 ******************************************************************************
  Change History:

    Version 1.0 - May 2013
    > Initial revision

******************************************************************************/
#include "lsm6ds3.h"

/**
  * @brief  This function initializes I2C1.
  * @param  None
  * @retval None
  */
void LSM6DS3_Setup(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef  I2C_InitStructure;
  
  RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);

  //(#) Enable peripheral clock using RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2Cx, ENABLE)
  //    function for I2C1 or I2C2.
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  
  //(#) Enable SDA, SCL  and SMBA (when used) GPIO clocks using 
  //    RCC_AHBPeriphClockCmd() function. 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  
  // Enable PB8 and make it active high
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIOB->BSRR = GPIO_BSRR_BS_8;
  
  //(#) Peripherals alternate function: 
  //    (++) Connect the pin to the desired peripherals' Alternate 
  //         Function (AF) using GPIO_PinAFConfig() function.
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_1);

  //    (++) Configure the desired pin in alternate function by:
  //         GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;

  //    (++) Select the type, OpenDrain and speed via  
  //         GPIO_PuPd, GPIO_OType and GPIO_Speed members
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  //    (++) Call GPIO_Init() function.
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  //(#) Program the Mode, Timing , Own address, Ack and Acknowledged Address 
  //    using the I2C_Init() function.
  I2C_StructInit(&I2C_InitStructure);
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
  I2C_InitStructure.I2C_DigitalFilter = 0;
  //aven I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_Mode = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0;
  I2C_InitStructure.I2C_Timing = 0x40B22536;
  I2C_Init(I2C1, &I2C_InitStructure);
  
  //(#) Optionally you can enable/configure the following parameters without
  //    re-initialization (i.e there is no need to call again I2C_Init() function):
  //    (++) Enable the acknowledge feature using I2C_AcknowledgeConfig() function.
  //    (++) Enable the dual addressing mode using I2C_DualAddressCmd() function.
  //    (++) Enable the general call using the I2C_GeneralCallCmd() function.
  //    (++) Enable the clock stretching using I2C_StretchClockCmd() function.
  //    (++) Enable the PEC Calculation using I2C_CalculatePEC() function.
  //    (++) For SMBus Mode:
  //         (+++) Enable the SMBusAlert pin using I2C_SMBusAlertCmd() function.

  //(#) Enable the NVIC and the corresponding interrupt using the function
  //    I2C_ITConfig() if you need to use interrupt mode.
  
  //(#) When using the DMA mode 
  //   (++) Configure the DMA using DMA_Init() function.
  //   (++) Active the needed channel Request using I2C_DMACmd() function.
  
  //(#) Enable the I2C using the I2C_Cmd() function.
  I2C_Cmd(I2C1, ENABLE);

  //(#) Enable the DMA using the DMA_Cmd() function when using DMA mode in the 
  //    transfers. 
}

/**
  * @brief  This function reads data from a register of the LIS302DL sensor.
  * @param  reg: register to be read
  * @retval 8-bit register contents
  */
uint8_t LSM6DS3_RegRead(uint8_t reg)
{
  uint8_t tmp = 0xFF;
  
//I2C1->CR2 = (uint32_t)(LIS302DL_ADDR) | (uint32_t)(1 << 16) | I2C_CR2_START ;
  I2C_TransferHandling(I2C1, LSM6DS3_ADDR, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
  while(!(I2C1->ISR & I2C_ISR_TXIS)){;}

//I2C1->TXDR = (uint8_t)(addr);
  I2C_SendData(I2C1, reg);
  while(!(I2C1->ISR & I2C_ISR_TC)){;}
    
//I2C1->CR2 = (uint32_t)(LIS302DL_ADDR) | (uint32_t)(1 << 16) | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START;
  I2C_TransferHandling(I2C1, LSM6DS3_ADDR, 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
  while(!(I2C1->ISR & I2C_ISR_RXNE)){;}
    
//tmp = (uint8_t)I2C1->RXDR;
  tmp = I2C_ReceiveData(I2C1);
    
  while(!(I2C1->ISR & I2C_ISR_STOPF)){;}
  I2C1->ICR = I2C_ICR_STOPCF;
  
  return(tmp);
}

/**
  * @brief  This function writes data to a register of the LIS302DL sensor.
  * @param  reg: register to be written
  *         data: data to be written to the selected register
  * @retval None
  */
void LSM6DS3_RegWrite(uint8_t reg, uint8_t data)
{
//I2C1->CR2 |= (uint32_t)(LIS302DL_ADDR) | (uint32_t)(1 << 16) | I2C_CR2_RELOAD | I2C_CR2_START ;
  I2C_TransferHandling(I2C1, LSM6DS3_ADDR, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);
  while(!(I2C1->ISR & I2C_ISR_TXIS)){;}
  
//I2C1->TXDR = (uint8_t)(addr);
  I2C_SendData(I2C1, reg);
  while(!(I2C1->ISR & I2C_ISR_TCR)){;}
    
//I2C1->CR2 |= I2C_CR2_AUTOEND;
  I2C_AutoEndCmd(I2C1, ENABLE);
  while(!(I2C1->ISR & I2C_ISR_TXIS)){;}
    
//I2C1->TXDR = data;
  I2C_SendData(I2C1, data);
  
//I2C1->CR2 &= ~I2C_CR2_RELOAD;
  I2C_ReloadCmd(I2C1, DISABLE);
    
  while(!(I2C1->ISR & I2C_ISR_STOPF)){;}
  I2C1->ICR = I2C_ICR_STOPCF;
}
