#include "uart.h"
#include "string.h"
#include <stdarg.h>
#include "gpio.h"
extern uint32_t CmdTimeout_count;

char Uart1_DataBuffer [CmdBufferLength] = "";
//char buffTemp [CmdBufferLength] = "";

Uart_BufferType CmdBuffer={Uart_Sprinter,CmdBufferLength,Uart1_DataBuffer,0,FALSE,&CmdTimeout_count };

int fputc(int ch, FILE *f)
{
  while(USART_GetFlagStatus(Uart_Sprinter, USART_FLAG_TXE) == RESET);
 
  USART_SendData(Uart_Sprinter, ch);
 
  return(ch);
}


void Usart1_ReadLine(void){

	if(CmdBuffer.Length>0 && *CmdBuffer.TimeoutCounter > 10){ //command timeout(10ms) checking 
		resetUartBuffer(&CmdBuffer);
	}
	*CmdBuffer.TimeoutCounter=0;//reset timeout counter
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){	//Receive interupt
   		
			char c = USART_ReceiveData(USART1);  
			if(CmdBuffer.Length > CmdBuffer.Size){ //command buffer overflow
				resetUartBuffer(&CmdBuffer);
				return;
			}
			if(!CmdBuffer.Received){
				if(c != '\n'){
					CmdBuffer.Data[CmdBuffer.Length++]=c;
					CmdBuffer.Received=FALSE;
				}else{	//receive ending character:'\n'
					CmdBuffer.Received=TRUE;
					//CmdBuffer.Data[CmdBuffer.Length++]='\n';
					CmdBuffer.Data[CmdBuffer.Length]='\0';
					if(CmdBuffer.Data[CmdBuffer.Length-1]=='\r')
						CmdBuffer.Data[--CmdBuffer.Length]='\0';
					//printf("%s\n",CmdBuffer.Data);
				}
			}
			
			
	}

}

void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...){ 

	const char *s;
    int d;
   
    char buf[16];
    va_list ap;
    va_start(ap, Data);
	//RS485Transmit();
	while(*Data!=0){				                          //耞琌笷才﹃挡才
		if(*Data==0x5c){									  //'\'
			switch (*++Data){
				case 'r':							          //ó才
					while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
					USART_SendData(USARTx, 0x0d);	   

					Data++;
					break;
				case 'n':							          //传︽才
					while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
					USART_SendData(USARTx, 0x0a);	
					Data++;
					break;
				
				default:
					Data++;
				    break;
			}
			
			 
		}
		else if(*Data=='%'){									  //
			switch (*++Data){				
				case 's':										  //才﹃
                	s = va_arg(ap, const char *);
                	for ( ; *s; s++) {
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                    	USART_SendData(USARTx,*s);
						
                	}
					Data++;
                	break;
            	case 'd':										  //秈
                	d = va_arg(ap, int);
                	itoa(d, buf, 10);
                	for (s = buf; *s; s++) {
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                    	USART_SendData(USARTx,*s);
						
                	}
					Data++;
                	break;
				default:
					Data++;
				    break;
			}		 
		}
		else{
			while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
			USART_SendData(USARTx, *Data++);
		}
		//while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
	}
	
	//RS485Receive();
}


/******************************************************
		俱计沮锣才﹃ㄧ计
        char *itoa(int value, char *string, int radix)
		radix=10 夹ボ琌10秈	獶秈锣传挡狦0;  

	    ㄒd=-379;
		磅︽	itoa(d, buf, 10); 
		
		buf="-379"							   			  
**********************************************************/
char *itoa(int value, char *string, int radix)
{
    int     i, d;
    int     flag = 0;
    char    *ptr = string;

    /* This implementation only works for decimal numbers. */
    if (radix != 10)
    {
        *ptr = 0;
        return string;
    }

    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    /* if this is a negative value insert the minus sign. */
    if (value < 0)
    {
        *ptr++ = '-';

        /* Make the value positive. */
        value *= -1;
    }

    for (i = 10000; i > 0; i /= 10)
    {
        d = value / i;

        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    /* Null terminate the string. */
    *ptr = 0;

    return string;

} /* NCL_Itoa */

void resetUartBuffer(Uart_BufferType *buff){
	buff->Size=500;
	buff->Received=FALSE;
	buff->Length=0;
	*buff->TimeoutCounter=0;
}


