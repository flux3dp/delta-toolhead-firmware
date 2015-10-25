#if 0
//#include "PID_v1.h"
#include "time.h"
#include "loop.h"

millis()
{   
 //clock_t t1, t2;
 unsigned long tt;
	
 tt = clock();   

 //int i;

 //for(i = 0;i < 1000000 ;i++)   
 //{   
   //   int x = 90;  
 //}   
 //t2 = clock();   

 //float diff = (((float)t2 - (float)t1) / 1000000.0F ) * 1000;   
 //printf("%f",diff);   

 return tt;   
 }
/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID_SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;
 
   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;
	 
	   if(ITerm > outMax) ITerm= outMax;
	   else if(ITerm < outMin) ITerm= outMin;
   }
}

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
void setPID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection)
{
	
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
	inAuto = false;
	
	PID_SetOutputLimits(0, 255);				//default output limit corresponds to 
												//the arduino pwm limits

  SampleTime = 100;							//default Controller Sample Time is 0.1 seconds

  PID_SetControllerDirection(ControllerDirection);
  PID_SetTunings(Kp, Ki, Kd);

	gettime = millis();
  lastTime =  gettime - SampleTime;				
}
 
 
/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
_Bool PID_Compute()
{
   if(!inAuto) return false;
   now = millis();
   timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
	    input = *myInput;
      error = *mySetpoint - input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      dInput = (input - lastInput);
 
      /*Compute PID Output*/
      output = kp * error + ITerm- kd * dInput;
      
	  if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
	  *myOutput = output;
	  
      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
	  return true;
   }
   else return false;
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void PID_SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;
 
   dispKp = Kp; dispKi = Ki; dispKd = Kd;
   
   SampleTimeInSec = ((double)SampleTime)/1000;  
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
 
  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}
  
/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void PID_SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}


/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/ 
void PID_SetMode(int Mode)
{
    newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        PID_Initialize();
    }
    inAuto = newAuto;
}
 
/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/ 
void PID_Initialize()
{
   ITerm = *myOutput;
   lastInput = *myInput;
   if(ITerm > outMax) ITerm = outMax;
   else if(ITerm < outMin) ITerm = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID_SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	  kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }   
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID_GetKp(){ return  dispKp; }
double PID_GetKi(){ return  dispKi;}
double PID_GetKd(){ return  dispKd;}
int PID_GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID_GetDirection(){ return controllerDirection;}

/**
  ******************************************************************************
  * @file    TEST/loop.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    23-March-2012
  * @brief   Arduino functions test for STM32F051 Discovery Kit
  *          Get ADC value from pin A0 to A6.
  *          Set different frequencies to PWM pins 3,5,6,9,10 and 11
  *          Configure Rx/Tx pins 0 and 1 and use Serial "object" to print info
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
#include <stdio.h>
#include "loop.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/* Global variables ----------------------------------------------------------*/
uint16_t G_ADCValue=0;
float G_ReadVal =0;
char G_SendBuff[64];


void loop()
{
	
 while(1)
 {
   

 }	 
	
		//while(1)
		//{
    //USART_SendData(USART1, 'AAA');
	  //USART_SendData(USART1, 'A');
		//
		//RxBuffer[RXSIZE] = 
	  //G_SendBuff[0] = USART_ReceiveData(USART1);
    
	  //if(G_SendBuff[0]!=0)
		//{
      //USART_SendData(USART1, G_SendBuff[0]);
		//}			
		
		 //G_SendBuff[0] = 0;
	  //}
		//for(i=0;i<RXSIZE;i++)
		//{
		 //RxBuffer[RXSIZE] = USART_ReceiveData(USART1);	
		 //if(RxBuffer>0)
     //{
			 //USART_SendData(USART1, RxBuffer[i]);
     //}			
	  //}

		
	

  	

uint16_t i = 0;
char L_readChar;
   //pinMode(motor_cmd,OUTPUT);
pwmFrequency(pwm_pin_3,1000); //set pwm frequencies
pwmFrequency(pwm_pin_5,2000); //it is not a standart arduino function
pwmFrequency(pwm_pin_6,3000);
pwmFrequency(pwm_pin_9,4000);
pwmFrequency(pwm_pin_10,5000);
pwmFrequency(pwm_pin_11,6000);
//Serial.begin(115200); //Open com on uart 0-1 pins

    while(1){
     analogWrite(pwm_pin_3,i*10);
     analogWrite(pwm_pin_5,i*10);
     analogWrite(pwm_pin_6,i*10);
     analogWrite(pwm_pin_9,i*10);
     analogWrite(pwm_pin_10,i*10);
     analogWrite(pwm_pin_11,i*10);
     delay(2000); //Wait for 1 sec 
     G_ADCValue = analogRead(A2); 
     G_ReadVal = (float)(POWER_SUPPLY * G_ADCValue) / 4095;
     sprintf(G_SendBuff,"%f Volt",G_ReadVal);
     Serial.print("ADC read: ");
     Serial.println(G_SendBuff);     
     i++;
     if(i>10) i=0;
     while(Serial.available()) {
       L_readChar = Serial.read();
       Serial.write(L_readChar);
     }
     Serial.println("");     
     
    }
	
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
#endif
