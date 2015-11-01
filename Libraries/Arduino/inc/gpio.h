/**
  ******************************************************************************
  * @file    Arduino/gpio.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    23-March-2012
  * @brief   Arduino GPIO API for STM32F051 Discovery Kit
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
/* Define to prevent recursive inclusion -------------------------------------*/  
#ifndef _GPIO_PIN_CONF_H_
#define _GPIO_PIN_CONF_H_

#ifdef __cplusplus
extern "C"{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"



/* Exported types ------------------------------------------------------------*/
typedef unsigned char byte ;
typedef unsigned char BYTE ;
typedef unsigned char boolean ;
typedef enum {FALSE = 0, TRUE = !FALSE} bool;  
  
/*
 * GPIO register maps and devices
 * This structure defines Arduino Pin's functionality 
 * and mapping on STM32
 */
typedef struct
{
  uint16_t PinNum;
  GPIO_TypeDef* GPIOx; 
  uint8_t PinMode;
  uint16_t pwmPeriod;
}GPIO_PinTypeDef;
  

/* Exported constants --------------------------------------------------------*/
//GPIO Pin value
#define LOW 0
#define HIGH 1


//Pin configuration definition Input is odd, output is even. Struc PinMode field 
// A simple way to know INPUT or OUTPUT request will be to test bit0
#define NOT_CONFIGURED 0 //Pin is not configured
#define INPUT 0x01 //Pin is used as input
#define OUTPUT 0x02 //Pin is used as output
#define INPUT_AF 0x3 //Pin is configured as Input Alternate Function (GPIO_Mode_AF)
#define OUTPUT_AF 0x4 //Pin is configured as Output Alternate Function (GPIO_Mode_AF)
#define INPUT_AN 0x5 //Pin is configured as Input analog (GPIO_Mode_AN)
#define OUTPUT_AN 0x6 //Pin is configured as Output analog (GPIO_Mode_AN)
#define INPUT_NOPULL 0x7 //Pin is configured as Input NO pull_up/pull_down (GPIO_PuPd_NOPULL)
#define OUTPUT_OD 0x8 //Pin is configured as Output Open_drain (GPIO_OType_OD)
#define INPUT_PU 0x9 //Pin is configured as Input Pull_up (GPIO_PuPd_UP)
#define OUTPUT_PP 0x0A //Pin is configured as Output Push_Pull (GPIO_OType_PP)
#define INPUT_PD 0x0B //Pin is configured as Input Pull_down (GPIO_PuPd_DOWN)

//aven - pin define
#define ID0  2    //PA6
#define ID1  1    //PA7
#define TEMP  7
#define THERM 8   //PA0
#define F2FB	9
#define HEATER 10 //PB13
#define F1FB    11
#define FAN1    15 //PB8
#define FAN2    14 //PB9
#define SDA   16 //PB7
#define SCL   17 //PB6
#define INT2  18
#define INT1  19
#define LED1  21 //PB2
#define IO1   22
#define IO0	  23
#define LD4 27
#define LD3 28


/*
#define HEATER 9//2 //PB13
#define FAN1 5//4 //PB10
#define FAN2 6//5 //PB11

#define F1FB 8
#define F2FB 10

#define THERM A0
#define TEMP A1

#define IO0 4
#define IO1 3
*/


//#define PID_MAX 255.0
//#define PID_MIN 0.0

#if 0
//Define Variables we'll be connecting to
double Setpoint=0, Input=0, Output=0;
//char RX_buff[10];
char d[4];
int m1,m2;

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


//aven - pin define


//Analog Pin number definition
#define A0 14 
#define A1 15 
#define A2 16 
#define A3 17 
#define A4 18 
#define A5 19

#define IS_ARDUINO_PIN(PIN) (((PIN) == 0) || \
                             ((PIN) == 1) || \
                             ((PIN) == 2) || \
                             ((PIN) == 3) || \
                             ((PIN) == 4) || \
                             ((PIN) == 5) || \
                             ((PIN) == 6) || \
                             ((PIN) == 7) || \
                             ((PIN) == 8) || \
                             ((PIN) == 9) || \
                             ((PIN) == 10) || \
                             ((PIN) == 11) || \
                             ((PIN) == 12) || \
                             ((PIN) == 13) || \
                             ((PIN) == 14) || \
                             ((PIN) == 15) || \
                             ((PIN) == 16) || \
                             ((PIN) == 17) || \
                             ((PIN) == 18) || \
                             ((PIN) == 19) || \
                             ((PIN) == 20) || \
                             ((PIN) == 21))

#define IS_ARDUINO_MODE(MODE) (((MODE) == INPUT) || \
                               ((MODE) == OUTPUT))


#define IS_PWM_PIN(pin)  (((PIN) == 3) || \
                             ((PIN) == 5) || \
                             ((PIN) == 6) || \
                             ((PIN) == 9) || \
                             ((PIN) == 10)|| \
                             ((PIN) == 11))

#define IS_ADC_PIN(pin)  (((PIN) == A0) || \
                             ((PIN) == A1) || \
                             ((PIN) == A2) || \
                             ((PIN) == A3) || \
                             ((PIN) == A4)|| \
                             ((PIN) == A5))

/* Exported functions ------------------------------------------------------- */ 
void pinMode(uint16_t, uint16_t); 
void pinSetMode(uint16_t, uint16_t);
uint16_t digitalRead(uint16_t);
void digitalWrite(uint16_t, uint16_t);
uint16_t analogRead(uint16_t);
void analogWrite(uint16_t, uint16_t);
void pwmFrequency(uint16_t, uint16_t);
GPIO_PinTypeDef getPinDef(uint16_t);

#ifdef __cplusplus
}
#endif

#endif /* _GPIO_PIN_CONF_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


