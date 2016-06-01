#include <stdint.h>
#include "gpio.h"

#define Fan_Lowest_PWM  200
#define Fan_Mask_Time 5000
#define Fan_Regular_Check_Time 973
#define Fan2_On         TRUE
#define Fan2_Off        FALSE
void	Set_Exhalation_Fan_PWM(uint8_t PWM);//set small fan pwm 0~255
void	Set_Exhalation_Fan_PWM_Mask(uint8_t PWM);
void	Set_Inhalation_Fan_PWM(uint8_t PWM);//set Big 2 fan pwm 0~255
void Set_Inhalation_Fan_Mask_PWM(uint8_t PWM);
uint8_t Read_Inhalation_Fan_PWM(void);
uint8_t Read_Inhalation_Fan_Mask_PWM(void);
uint8_t Read_Exhalation_Fan_PWM(void);
bool Is_Inhalation_Fan_Failed(void);
void Fan_Management(void);

