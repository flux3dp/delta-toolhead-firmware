/**
  ******************************************************************************
  * @file    Arduino/serial.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    23-March-2012
  * @brief   Arduino serial communication API for STM32F051 Discovery Kit
  *       Interrupt receive function is USART2_IRQHandler    
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

//#include "main.h"

#if 1
#include "stm32f0xx_conf.h"
#include "utilities.h"
#include "serial.h"
#include "gpio.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#if 1
//for printf
#include <rt_misc.h>
#pragma import(__use_no_semihosting_swi)
struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;
#endif


//int fputc(int ch, FILE *f)
//{
//    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
// 
//  USART_SendData(USART1, ch);
// 
//  return(ch);
//}
 
int fgetc(FILE *f)
{
    char ch;
 
    while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
 
    ch = USART_ReceiveData(USART1);
 
  return((int)ch);
}

int ferror(FILE *f)
{
  /* Your implementation of ferror */
  return EOF;
}
 
void _ttywrch(int ch)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
 
  USART_SendData(USART1, ch);
}
 
void _sys_exit(int return_code)
{
label:  goto label;  /* endless loop */
}


#if 0
/* Private typedef -----------------------------------------------------------*/
int i,m1,m2;
char d[4];
//long Setpoint=0;

double Setpoint=0, Input=0, Output=0;
//char RX_buff[10];


//#define ThermistorPIN 0                 // Analog Pin 0

float vcc = 4.91;                       // only used for display purposes, if used
                                        // set to the measured Vcc.
//float pad = 9850;                       // balance/pad resistor value, set this to
float pad = 200000;            
                                        // the measured resistance of your pad resistor
float thermr = 10000;                   // thermistor nominal resistance

int therm[100]; 
long sum =0;
int flag =0;
int fan_flag = 0;
int count =0;
int fan1_pwm=0;
int fan2_pwm=0;

int freq1=0;
int freq2=0;



int16_t ax, ay, az;
#define OUTPUT_READABLE_ACCELGYRO
int8_t chip_ID;
float x,y,z;
//int16_t z_data[100];

#endif

  



/* Private define ------------------------------------------------------------*/
/*Arduino Serial communication stream*/
Serial_TypeDef Serial = {
  0,
  begin_fcn,
  available_fcn,
  flush_fcn,
  read_fcn,
  peek_fcn,
  write_fcn,
  print_fcn,
  println_fcn,
	printint_fcn,//aven
	printlong_fcn,//aven
	
};

//Reception definitions
static char rcv_buff[BUFFER_RCV_SIZE]; //Reception buffer
static char* pt_rcv_read = rcv_buff;   //Circular read ptr
static char* pt_rcv_write = rcv_buff;  //Circular write ptr
static volatile uint16_t nb_rcv_buff_char = 0;  //Nb of char to read in rcv_buff
static uint16_t rcv_buff_overload=0;   //Nb of char refused cause rcv_buff is full (overload)
static char rcv_buff_full=0;           //rcv_buff is full. pt_rcv_write


/* Private function prototypes -----------------------------------------------*/
static void GPIO_Configuration_USART1(void);
static void UART1_Init(uint32_t baud_rate);
static USART_TypeDef* USARTx = USART1;
//static void NVIC_Configuration(void);
#if 0
static void GPIO_Configuration_USART2(void);
static void UART2_Init(uint32_t baud_rate);
static USART_TypeDef* USARTx = USART2;
#endif
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  UART2_Init  Configures and initializes the UART module.
  * @param  Serial baud rate communication speed
  * @retval None
  */
#if 1
void UART1_Init(uint32_t baud_rate){
  /*The internal UART module module is set to :
	default Tx and Rx pins. 
	115200 baud
	8-bit data, no parity. 
	1 STOP bit. 
      */
  USART_InitTypeDef USART_InitStructure;

  GPIO_Configuration_USART1();
     
  USART_InitStructure.USART_BaudRate = baud_rate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);  
  
  //NVIC_Configuration();
  
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  USART_Cmd(USART1, ENABLE);

  NVIC_EnableIRQ(USART1_IRQn);
}
#endif	
	
	
#if 0	
static void UART2_Init(uint32_t baud_rate){
  /*The internal UART module module is set to :
	default Tx and Rx pins. 
	115200 baud
	8-bit data, no parity. 
	1 STOP bit. 
      */
  USART_InitTypeDef USART_InitStructure;

  GPIO_Configuration_USART2();
     
  USART_InitStructure.USART_BaudRate = baud_rate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);  
  
  NVIC_Configuration();
  
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

  USART_Cmd(USART2, ENABLE);  
}
#endif

/**
  * @brief  putchar Output a character to the usart
  * @param  int character to output
  * @retval None
  */
#if 0
int putchar(int chr)
{
   /* mask out any high bits */
   while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
      /* null */ ;
   USART_SendData(USARTx, (char)(chr & 0x7f));
 
   return chr;  
}
#endif


/**
  * @brief  begin_fcn Configure Arduino communication port
  * @param  baudRate = usart speed
  * @retval None
  */
void begin_fcn(uint32_t baudRate) {
  //UART2_Init(baudRate);
	UART1_Init(baudRate);
  Serial.baudRate = baudRate;
  pinSetMode(0, INPUT_AF); //pin 0 is Rx
  pinSetMode(1, OUTPUT_AF); //pin 1 is Tx
}

/**
  * @brief  available_fcn Return the nb of received char
  * @param  None
  * @retval Nb of char to process from rcv_buffer 
  */
uint16_t available_fcn(void){
  return(nb_rcv_buff_char);
}


/**
  * @brief  flush_fcn Reset all rcv_buff parameters
  * @param  None
  * @retval None
  */
void flush_fcn(void){
  pt_rcv_read = rcv_buff;   //reset circular read ptr
  pt_rcv_write = rcv_buff;  //reset circular write ptr
  nb_rcv_buff_char = 0; //reset Nb of char to read in rcv_buff
  rcv_buff_overload=0;  //reset Nb of char refused cause rcv_buff is full (overload)
  rcv_buff_full=0;         //reset rcv_buff is full.   
}


/**
  * @brief  read_fcn Return the first not read received char 
  * @param  None
  * @retval rcv char
  */
char read_fcn(void){
  char l_readChar = (char)-1; //Init with no char to read result
  if(nb_rcv_buff_char){
    //If there is char to read
    l_readChar = *pt_rcv_read++;
    nb_rcv_buff_char--;
    rcv_buff_full = 0; //At least 1 char space free in rcv_buff
    if(pt_rcv_read > &rcv_buff[BUFFER_RCV_SIZE]) {
      //pointer rollover rcv_buffer
      pt_rcv_read = rcv_buff;
    }    
  }
  return(l_readChar);
}


/**
  * @brief  peek_fcn Return the first char from buff but doesn't move read ptr
  * @param  None
  * @retval 1st rcv char
  */
char peek_fcn(void){
  char l_readChar = (char)-1; //Init with no char in rcv buff
  if(nb_rcv_buff_char){
    //If there is char to peek
    l_readChar = *pt_rcv_read;
  }
  return(l_readChar);
}


/**
  * @brief  write_fcn Send a char on usart
  * @param  char to send
  * @retval None
  */
void write_fcn(unsigned char byte){
  putchar(byte);
}

/**
  * @brief  print_fcn Send a string on usart
  * @param  string to send
  * @retval None
  */
void print_fcn(char *string){
  char *l_i = string;
  do{
    putchar(*l_i++);
  } while(*l_i != 0);  
}

/**
  * @brief  println_fcn Send a string on usart with carriage return
  * @param  string to send
  * @retval None
  */
void println_fcn(char *string){
  print_fcn(string);
  write_fcn('\r');
  write_fcn('\n');  
}

void printint_fcn(int a){
  
	/* mask out any high bits */
   while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
      /* null */ ;
   USART_SendData(USARTx, (int)(a & 0x7f));
 
   //return chr;  
  write_fcn('\r');
  write_fcn('\n');  
}


void printlong_fcn(long b){
  
	/* mask out any high bits */
   while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
      /* null */ ;
   USART_SendData(USARTx, (long)(b & 0x7f));
 
   //return chr;  
  write_fcn('\r');
  write_fcn('\n');  
}

/**
  * @brief  GPIO_Configuration_USART2 Configures the different GPIO ports for UART.
  * @param  None
  * @retval None
  */
#if 1
void GPIO_Configuration_USART1(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);  

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
 
  GPIO_PinAFConfig(UART_PORT, RX_PIN_SOURCE, GPIO_AF_1);
  GPIO_PinAFConfig(UART_PORT, TX_PIN_SOURCE, GPIO_AF_1);
   
//ADC	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
  /* Configure UART1 pins:  Rx and Tx ----------------------------*/
  //GPIO_InitStructure.GPIO_Pin =  TX_PIN | RX_PIN;
  GPIO_InitStructure.GPIO_Pin =  RX_PIN | TX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_Init(UART_PORT, &GPIO_InitStructure); 
  
}
#endif

#if 0
static void GPIO_Configuration_USART2(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);  

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
 
  GPIO_PinAFConfig(UART_PORT, RX_PIN_SOURCE, GPIO_AF_1);
  GPIO_PinAFConfig(UART_PORT, TX_PIN_SOURCE, GPIO_AF_1);
   
  /* Configure UART1 pins:  Rx and Tx ----------------------------*/
  GPIO_InitStructure.GPIO_Pin =  TX_PIN | RX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_Init(UART_PORT, &GPIO_InitStructure); 
  
}
#endif


/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
//static void NVIC_Configuration(void)
//{
//	NVIC_InitTypeDef NVIC_InitStructure;

//	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

//	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//	
//}


//int cot=0;
//char RX_BB[256];
//char rx_bb [100] = "";


#if 0

void sendcmd(char *string)
{
    while(*string)
		{
        
        USART_SendData(USART1, (unsigned short int) *string++);
 
        
        while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    }
}

char rx_bb [100] = "";

char * readcmd()
{    
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
		{         
        char c = USART_ReceiveData(USART1);         
        if(c != '\r' && c != '\n')
				{
            sprintf (rx_bb, "%s%c", rx_bb, c);
					  //leng = strlen(rx_bb);
					  //sprintf (ww, "%s%c", ww, c);
					  //leng = sizeof(ww);
					  //sprintf (st, "leng = %d", leng);
					
					  
					  //Serial.println(st);
        }
				else
				{
            char rx_bb2 [100] = "";
            memcpy(rx_bb2,rx_bb, strlen(rx_bb)); 
            memset(rx_bb, 0, strlen(rx_bb));             
            return rx_bb2;
					  //Serial.println("2");
        }
    }
    return "";
}

#endif

/**
  * @brief  This function handles USART2 global interrupt request.
  * @param  None
  * @retval None
  */
#if 0
void USART1_IRQHandler(void)
{
	//scanf("%s",data);	
	char * data = readcmd(); 
	
	if(data[0] == 'T' || data[0] == 't')
       { 
				 printf("\r\n TTT");
				 t_flag = 1;
				 
			 }
			 		 
	 //char * data = readcmd(); 
	//Serial.println("2");
	  //printf("looks");
	 //digitalWrite(FAN2,HIGH);
	#if 0
		 //if(data[0] == 'H' | data[0] == 'h')
	   if(data[0] == 'H' || data[0] == 'h')
     {
         m1 = 0;
         Setpoint = 0;
         //m2=0;
			   if(data[1]!= NULL)
				 {	 
           for(i = 0; i < 4; i++)
           {
             d[i]=0;
             d[i]=data[i+1];          
           }
					 
           m1 = atoi(d);
           printf("\r\n Heater :%d",m1);
					 
           if(m1 <= 2625 && m1 >= 0)
           {			
             //Serial.println("ssss");					 
             if(fan_flag == 0 )
             {					 
               freq2 = 800;
               fan1_pwm = 255;
						   pwmFrequency(FAN2,freq2);
               //setPwmFrequency(FAN2, freq2);
               //delay_ms(100);
						 
               analogWrite(FAN1,fan1_pwm);
               analogWrite(FAN2,fan1_pwm);
						 
               fan_flag = 1;
               //delay_ms(10);					 
              } 
					 
              flag = 1;
              Setpoint = m1;
							printf("\r\n PID ON!!");
            }
					}					 
       }
       
       
       if(data[0] == 'O' || data[0] == 'o')
       {
        if(flag == 1)
        {
          analogWrite(HEATER,0);
          analogWrite(FAN1,0);
          digitalWrite(FAN2,0);
          fan_flag = 0;
          flag = 0;
          Setpoint = 0;
          fan1_pwm = 0;
					printf("\r\n PID OFF!!");
                
        }        
       }
			 
			 if(data[0] == 'T' || data[0] == 't')
       { 
         //if(flag == 0)
         //{
				 
           for(i=0;i<10 ;i++)
           {  
             //therm[i]=analogRead(THERM);
             sum += therm[i];
           }
           //free(therm);
           sum = (sum/10);
           //sum=pgm_read_word(&temps[sum]);
           sum=temps[sum];
					 			   
           printf("\r\n Celsius :%d",sum);				 
                 
       }
			 
			 if(data[0] == 'P' || data[0] == 'p')
       {
         m1 = 0;
         
				 if(data[1]!= NULL)
				 {
				   for(i = 0; i < 4; i++)
           {
              d[i]=0;
              d[i]=data[i+1];         
           }
         
           m1 = atoi(d);
         
           freq2 = m1;
         
           //setPwmFrequency(FAN1, m1);
				   pwmFrequency(FAN2,freq2);
           //setPwmFrequency(FAN2, m1);
					 printf("\r\n Set freq2 :%d",freq2);
            
        }
			 }
			 
		 if(data[0] == 'F' || data[0] == 'f')
     {
         m1 = 0;
			 
			   if(data[1]!= NULL)
				 {
           for(i = 0; i < 4; i++)
           {
              d[i]=0;
              d[i]=data[i+1];         
           }
         
           m1 = atoi(d);
           fan1_pwm = m1;
				 
           if(m1 <= 256 && m1 >= 0 )
           {           
             analogWrite(FAN1,m1);
             analogWrite(FAN2,m1);
						 printf("\r\n Set fan :%d",m1);
						 
             fan_flag = 1;
           
             if(m1 == 0)
             {
               fan_flag = 0;           
             }          
           }
				  }					 
       }


       if(data[0] == 'R' || data[0] == 'r')
       {
          for(i=0;i<10 ;i++)
          {  
             //therm[i]=analogRead(THERM);
             sum += therm[i];
          }
          //free(therm);
          sum = (sum/10);
          //sum=pgm_read_word(&temps[sum]);
          sum=temps[sum];
					
					
					printf("\r\n T :%d",sum);
					            
					printf("\r\n X5 :%d",Setpoint);        
        
					printf("\r\n flag :%d",flag); 
        
				  printf("\r\n X7 :%d",freq2); 
        
				  printf("\r\n X8 :%d",fan1_pwm);
        
				  printf("\r\n fan_flag :%d",fan_flag);

        }			 
     	 
        if(data[0] == 'L' || data[0] == 'l')
        {
          m1 = 0;
					if(data[1]!= NULL)
				  {
            for(i = 0; i < 4; i++)
            {
              d[i]=0;
              d[i]=data[i+1];         
            }
         
            m1 = atoi(d);
       
            if(m1 == 0)
            {
               digitalWrite(FAN2,LOW);
							 printf("\r\n Set Laser Power Digital Low ");
            }
       
            else if(m1 < 256)
            {
              //analogWrite(FAN1,m1);
              analogWrite(FAN2,m1);
							printf("\r\n Set Laser Power :%d",m1);
            }
           }
				 }
				#endif 
//*******************************************************************************************
// Sensor
//*******************************************************************************************				 
#if 0
				 if(data[0] == 'S' || data[0] == 's')
         {
           x = 0;
           y = 0;
           z = 0; 
           count = 0;
           //chip_ID = accelgyro.getDeviceID();
					 printf("\r\n MPU6050 Chip ID is %x",chip_ID);
           
           if(chip_ID == 56)//0x38
           {
						 printf("\r\n Sensor test is Pass !");
          
             for(i=0 ; i<100 ; i++)
             {
               accelgyro.getAcceleration(&ax, &ay, &az);
               Serial.println((float)az/1673,6); 
               z = ((float)az/1673,6);
               Serial.println(z);
               if(z > 9.5)
               {
                 Serial.print("LOW :");
                 Serial.println(count);
                 count++;
               }
                //z_sum +=((float)az/1673,6);
               delay(100);         
              }
              //z = (z_sum/100);
              //Serial.print("z :");
              //Serial.println(z);
          
              //x = ((float)ax/1673,6);
              //y = ((float)ay/1673,6);
              //z = ((float)az/1673,6);
          
              //Serial.print(x);
              //Serial.print(" ");
              //Serial.print(y);
              //Serial.print(" ");
              //Serial.println(z);
          
              if(count > 50)
              {
                digitalWrite(IO0,LOW);
                Serial.println("IO0 LOW");
                								
              }
            }
            else
            {            
							printf("\r\n Sensor id not match !");
            }
        /*for(;;)
         {
         // read raw accel/gyro measurements from device
           accelgyro.getAcceleration(&ax, &ay, &az);
           Serial.print("Accel Data:\t");
           Serial.print((float)ax/1673,6); Serial.print("\t");
           Serial.print((float)ay/1673,6); Serial.print("\t");
           Serial.println((float)az/1673,6); //1672= 2*9.8/36864//use the RANGE is +-4g, so the resolution is 9.8*4/2048=0.01914
           delay(100); 
         }*/
      
          } 
				 #endif
}
#endif

#if 0
void USART2_IRQHandler(void)
{
  uint16_t l_tmp; //Received char from USART2
  
  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
    USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    //Read one byte from the receive data register
    l_tmp  = USART_ReceiveData(USART2);
    if(rcv_buff_full){
      //rcv buffer is full ==> trash rcv char
      rcv_buff_overload++;
    } else {
      *pt_rcv_write++ = (char)l_tmp; //put rcv char in buffer
      nb_rcv_buff_char++; //One char more to be read in buffer
      if(pt_rcv_write > &rcv_buff[BUFFER_RCV_SIZE]) {
        //pointer rollover rcv_buffer
        pt_rcv_write = rcv_buff;
      }
      if(pt_rcv_write == pt_rcv_read) {
        //Test if rcv buffer is full
        rcv_buff_full = 1;
      }      
    }
  }
  USART_ClearITPendingBit(USART2, USART_IT_ORE);
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
#endif
