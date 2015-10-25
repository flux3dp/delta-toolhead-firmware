#include <stdint.h>
#include "stm32f0xx_gpio.h"
#include "gpio.h"
#include "configuration.h"

//#include "uart.h"
#define FLASH_PAGE_SIZE         ((uint32_t)0x00000400)   /* FLASH Page Size */
#define FLASH_USER_START_ADDR   ((uint32_t)0x08000800)   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ((uint32_t)0x08007000)   /* End @ of user Flash area */
#define SetHeaterOn() GPIO_SetBits(HeaterGPIO,HeaterPin);
#define SetHeaterOff() GPIO_ResetBits(HeaterGPIO,HeaterPin);

typedef enum
{
  Temperature_Channel,
	ID0_Channel,
	ID1_Channel
}ADC_Channel_Type;


void Xcode_Handler();

bool Write_ID(uint32_t ID);

uint32_t Read_ID();

bool IsNumber(char *NumberString);

void CommandTimeoutDetection();

uint16_t Read_ADC_Value(ADC_Channel_Type channel);



