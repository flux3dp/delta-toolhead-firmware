/**
  ******************************************************************************
  * @file    BeeBluetooth/setup.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    23-March-2012
  * @brief   Arduino Pin configuration
  *          Configure pin A0 to A5 for ADC
  *          Pin A2 is not configured to test that analogRead() from loop 
  *          automatically configure A2 for ADC
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
#include "setup.h"
#include "stm32f0xx_conf.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
 
 
/**
  * @brief  config Arduino processing function
  * @param  None
  * @retval None
  */
void setup(void)
{ 
  //For I2C
  //int error;
  //uint8_t c;
  //Wire.begin(); //For I2C
  //Fastwire::setup(400, true);
  //For I2C
  
  //Serial.begin(115200);
	//pinMode(HEATER,OUTPUT_AF);// HEATER
  //pinMode(FAN1,OUTPUT_AF);// FAN1
  //pinMode(FAN2,OUTPUT_AF);// FAn2
    
  //pinMode(F1FB,INPUT);//PB0 F1FB
  //pinMode(F2FB,INPUT);//PB2 F2FB
    
  //pinMode(THERM,INPUT);//
  //Input = analogRead(THERM);

  //pinMode(IO0,OUTPUT);

  //myPID.SetMode(AUTOMATIC);
  
  //freq2 = 1;
  //setPwmFrequency(FAN2, freq2);
	//pwmFrequency(FAN2,freq2);
  //delay(100);
  //fan1_pwm = 255;
  //analogWrite(FAN1,fan1_pwm);
  //analogWrite(FAN2,fan1_pwm);
  //fan_flag = 1;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
  //Serial.println("Initializing I2C devices...");
  //accelgyro.initialize();
  //Serial.println("Testing device connections...");
  //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 	
/*	
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  //A2 not configured
  pinMode(A3,INPUT);
  pinMode(A4,INPUT);
  pinMode(A5,INPUT);
*/
	
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
