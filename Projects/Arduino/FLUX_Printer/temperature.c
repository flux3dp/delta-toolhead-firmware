#include "temperature.h"



#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define BOOL unsigned char
#define TRUE 1
#define FALSE 0
#define HO 0
#define HA 32768
#define HF 65535


int Kp,Ki,Kd,MinOutput,MaxOutput; 

int PID_Kp, PID_Ki, PID_Kd;
int PID_Integrated;
int PID_Prev_Input;
int PID_MinOutput, PID_MaxOutput;
BOOL PID_First_Time;

int ErrValue, DiffValue, Result;
int Err;
int Setpoint=0;
int Input=0;
int Output=0;

uint32_t temp_millis=0;
uint32_t t1=0;
uint32_t t2=0;

//float input = 0.0;
int cycles = 0;
bool heating = 1;

  //unsigned long temp_millis = SysTick->VAL, t1 = temp_millis, t2 = temp_millis;
long t_high = 0, t_low = 0;
float Ku, Tu;
float Kp_temp, Ki_temp, Kd_temp;
int cmax = 2625, cmin = 0;
uint32_t pidc=0;	
int soft_pwm[1];
long bias, dxd;
#define PID_MAX 255
long max_pow;
int p;
//int pwm;
uint8_t PID_auto=0;
int temp=0; 
uint8_t hotend=0;
uint8_t ncycles=0;


void Init_PID(int Kp,int Ki,int Kd,int MinOutput,int MaxOutput)
{
  PID_Kp         = Kp;
  PID_Ki         = Ki;
  PID_Kd         = Kd;
  PID_MinOutput  = MinOutput;
  PID_MaxOutput  = MaxOutput;
  PID_Integrated = 0.0;
  PID_Prev_Input = 0.0;
  PID_First_Time = TRUE;
}


int PID_Calculate(int Setpoint,int Input)
{
  Err = Setpoint - Input;
  //printf("\r\n %d - %d = %d",Setpoint,Input,Err);	
  // --- calculate proportional value ---
  ErrValue  = Err * PID_Kp;
  //printf("\r\n %d  x %d = %d",Err,PID_Kp,ErrValue);
	
  // --- Calculate integrated value ---
#if 0	
  if(Err > 0)
  {
    PID_Ki = 0;
	}

  if(Err < 20)
	{
    PID_Ki = 1;
	}		
#endif

if(Err > 0)
{	
  PID_Integrated = PID_Integrated - (Err * PID_Ki);
}

if(Err < 0)
{
	PID_Integrated = PID_Integrated + (Err * PID_Ki);
}	
	//printf("\r\n %f + (%f * %f) = %f",PID_Integrated,Err,PID_Ki,PID_Integrated);
		
  // limit it to output minimum and maximum
  if (PID_Integrated < PID_MinOutput) 
    PID_Integrated = PID_MinOutput;
  if (PID_Integrated > PID_MaxOutput)
    PID_Integrated = PID_MaxOutput;

  // --- calculate derivative value ---
  if (PID_First_Time)
  {
    // to avoid a huge DiffValue the first time (PID_Prev_Input = 0)
    PID_First_Time = FALSE;
    PID_Prev_Input = Input;
  }
  DiffValue = (Input - PID_Prev_Input) * PID_Kd;
	//printf("\r\n (%f - %f ) * %f = %f",Input,PID_Prev_Input,PID_Kd);
  PID_Prev_Input = Input;

  // --- calculate total ---
  Result = ErrValue + PID_Integrated - DiffValue; // mind the minus sign!!!
	
	//printf("\r\n %d + %d - %d = %d",ErrValue,PID_Integrated,DiffValue,Result);
	//printf("\r\n ErrValue : %f",ErrValue);
	//printf("\r\n PID_Integrated : %f",PID_Integrated);
	//printf("\r\n DiffValue : %f",DiffValue);
	
	//printf("\r\n PID_MinOutput : %f",PID_MinOutput);
	//printf("\r\n PID_MaxOutput : %f",PID_MaxOutput);
	
	
	
  // limit it to output minimum and maximum
  if (Result < PID_MinOutput) 
    Result = PID_MinOutput;
  if (Result > PID_MaxOutput)
    Result = PID_MaxOutput;
	
	//printf("\r\n Result1 %d",Result);
	Result = -(Result - 2625);
	//printf("\r\n Result2 %d",Result);
	Result = Result * 25;
	//printf("\r\n Result3 %d",Result);
	if(Result > 65535)
	{
		Result = 65535;
	}
	
	if(Result < 0)
	{
    Result = 0;
	}		
	
  return (Result);
}

void PID_autotune(){
		//PID_auto = 1;
		i=0;
		adc_s = 0;
		addc_s = 0;
		pidc=0;
		heating = 1;
	    temp_millis = pidc, t1 = temp_millis, t2 = temp_millis;
	    //analogWrite(HEATER,65535);
	    //soft_pwm[hotend] = bias = dxd = PID_MAX / 2;
		soft_pwm[hotend] = bias = dxd = PID_MAX;
	    Output = -(soft_pwm[hotend]-255) *257;
	    printf("init pwm:%d \r\n",Output);
			
		Setpoint = 2000;
		hotend = 1;
		ncycles = 10;
		//pwmFrequency(HEATER,40000000);	
		//analogWrite(HEATER,pwm);
		//PID_autotune(2000,1,10);
		PID_auto = 1;
		printf("ok@A\r\n");
		
		
		adc_s=0;
			addc_s=0;
			cmax=0;
			cmin=0;

			pidc++;

			//printf("pidc : %d\r\n",pidc);			
			//Read temp
			addc_s = analogRead(THERM);
			adc_s = temps[addc_s];

			cmax = MAX(cmax, adc_s);
			cmin = MIN(cmin, adc_s);

			//printf("cmax : %d\r\n",cmax);
			//printf("cmin : %d\r\n",cmin);

			if (heating == 1 && adc_s > Setpoint) 
			{
				printf("h1: %d\r\n",(pidc-t2));
				if (pidc - t2 > 5000) 
				{
					heating = 0;				
								
					soft_pwm[hotend] = (bias - dxd) >> 1;
					Output = -(soft_pwm[hotend]-255) *257;		
					printf("heating : 0   PWM:%d\r\n",Output);	
					pwmFrequency(HEATER,40000000);
					analogWrite(HEATER,Output);			
					t1 = pidc;
					t_high = t1 - t2;
					cmax = Setpoint;
				}
			}//------------------------------------

			if (heating == 0 && adc_s < Setpoint) 
			{
				printf("h0: %d\r\n",(pidc-t1));
				if (pidc - t1 > 5000) 
				{
					heating = 1;				 

					t2 = pidc;
					t_low = t2 - t1;
					printf("t2: %d\r\n",t2);
					if (cycles > 0) 
					{
						max_pow = PID_MAX;
						bias += (dxd*(t_high - t_low))/(t_low + t_high);
						bias = constrain(bias, 20, max_pow - 20);
						dxd = (bias > max_pow / 2) ? max_pow - 1 - bias : bias;

						printf("BIAS :%lf\r\n",bias);						 
						printf("DXD :%lf\r\n",dxd);           
						printf("cmin :%d\r\n",cmin);            
						printf("cmax :%d\r\n",cmax);

						if (cycles > 2) 
						{
							Ku = (4.0 * dxd) / (3.14159265 * (cmax - cmin) / 2.0);
							Tu = ((float)(t_low + t_high) / 1000.0);
							  printf("Ku :%f\r\n",Ku);
							  printf("Tu :%f\r\n",Tu);

							Kp_temp = 0.6 * Ku;
							Ki_temp = 2 * Kp_temp / Tu;
							Kd_temp = Kp_temp * Tu / 8;
								 
							  printf("PID >>>>>>>>\r\n");
							  printf("Kp :%f\r\n",Kp_temp);
							  printf("Ki :%f\r\n",Ki_temp);
							  printf("Kd :%f\r\n",Kd_temp);             
						}				 
					}

					soft_pwm[hotend] = (bias + dxd) >> 1;
					Output = -(soft_pwm[hotend]-255) *257;
					printf("heating : 1   PWM:%d\r\n",Output);
					pwmFrequency(HEATER,40000000);
					analogWrite(HEATER,Output);
					cycles++;
					cmin = temp;		
				}
			}//++++++++++++++++++++++++++++++++++++++++++++++++++

			if (adc_s > Setpoint + 200) 
			{
				printf("PID_TEMP_TOO_HIGH\r\n");
				printf("PID >>>>>>>> Kp :%f,  Ki :%f,  Kd :%f\r\n",Kp_temp,Ki_temp,Kd_temp);
				//analogWrite(HEATER,HF);
				//Output = HF;
				pwmFrequency(HEATER,40000000);
				analogWrite(HEATER,HF);
				PID_auto = 0;
				Setpoint = 0;
				pidc = 0;
			}

			// Every 2 seconds...
			if (pidc > temp_millis + 18000) 
			{
				p = soft_pwm[hotend];
				printf("ok T:%d S:%d  @%d  PWM:%d  cycles:%d  pidc:%d  t1:%d  t2:%d  %d\r\n",adc_s,Setpoint,p,Output,cycles,pidc,t1,t2,i);

				temp_millis = pidc;
				i++;
			} // every 2 seconds

			// Over 2 minutes?
			if (((pidc - t1) + (pidc - t2)) > (10L*60L*1000L*2L*2L)) 
			{
				printf("PID_TIMEOUT\r\n");
				printf("PID >>>>>>>> Kp :%f,  Ki :%f,  Kd :%f\r\n",Kp_temp,Ki_temp,Kd_temp);
				//analogWrite(HEATER,HF);
				//Output = HF;
				pwmFrequency(HEATER,40000000);
				analogWrite(HEATER,HF);
				PID_auto = 0;
				Setpoint = 0;
				pidc = 0;
				//return;
			}

			if (cycles > ncycles) 
			{
				printf("PID_AUTOTUNE_FINISHED\r\n");
				#ifdef PIDTEMP
				PID_PARAM(Kp, hotend) = Kp_temp;
				PID_PARAM(Ki, hotend) = scalePID_i(Ki_temp);
				PID_PARAM(Kd, hotend) = scalePID_d(Kd_temp);
				#endif
				printf("PID >>>>>>>> Kp :%f,  Ki :%f,  Kd :%f\r\n",Kp_temp,Ki_temp,Kd_temp);
				//analogWrite(HEATER,HF);
				//Output = HF;
				pwmFrequency(HEATER,40000000);
				analogWrite(HEATER,HF);
				PID_auto = 0;
				Setpoint = 0;
				pidc = 0;
				//return;
			}
}

