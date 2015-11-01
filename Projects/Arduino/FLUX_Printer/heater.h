#include <stdint.h>

#define Max_Temperature	235.0
#define Temperature_Offset 0.0
#define PID_Period_Time 1
#define Kp_Default 30.0
#define Ki_Default 0.52
#define Kd_Default 10000.0
//#define Kp_Default 30.0
//#define Ki_Default 0.53
//#define Kd_Default 10000.0
//#define Kp_Default 30.0
//#define Ki_Default 0.5
//#define Kd_Default 10000.0
#define PID_FUNCTIONAL_RANGE 10.0 //PID control from error=10,or output=Maxmum.
#define PID_MAX	255.0
#define PID_MIN	0.0
#define max(x, y) (((x) > (y)) ? (x) : (y))
#define min(x, y) (((x) < (y)) ? (x) : (y))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define K1 0.95 // Smoothing factor within the PID
#define K2 (1.0 - K1)
#define ADC_Sample_Times 50

float	Read_Temperature();//return temperature*10
void	Set_Heater_PWM(uint8_t PWM);//set heater pwm 0~255
uint8_t Get_Pid_Output();
void	PID_Control();
void	PID_Handler();
void 	Disable_All_Heater();
void 	PID_Autotune();
uint8_t PID_Compute();
uint16_t Get_Pid_Output_Uint16();
void Set_Heater_PWM_Uint16(uint16_t PWM);
