#include <stdint.h>
#include "gpio.h"

#define Fan_Lowest_PWM  200
#define Fan_Revolution_Time_Limit 850
#define Fan2_On         TRUE
#define Fan2_Off        FALSE
void	Set_Exhalation_Fan_PWM(uint8_t PWM);//set small fan pwm 0~255
void	Set_Inhalation_Fan_PWM(uint8_t PWM);//set Big 2 fan pwm 0~255
uint8_t Read_Inhalation_Fan_PWM(void);
bool Is_Inhalation_Fan_Failed(void);
void Fan_Management(void);
