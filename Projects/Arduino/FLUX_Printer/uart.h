#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
#include "gpio.h"

#define Uart_Sprinter USART1
#define Baudrate 115200;
#define CmdBufferLength 100

typedef struct
{
	USART_TypeDef 	*UARTX;
	uint16_t		Size;				// record the total size can be used
	char 			*Data;
	uint16_t		Length;				// record data length
	bool			Received;
	uint32_t		*TimeoutCounter;
}Uart_BufferType;

char *itoa(int value, char *string, int radix);
void USART_SendByte(USART_TypeDef* USARTx,uint16_t data);
void USART_write(USART_TypeDef* USARTx,uint8_t* data,uint16_t len);
int fputc(int ch, FILE *f);
void Usart1_ReadLine(void);
void resetUartBuffer(Uart_BufferType *buff);
