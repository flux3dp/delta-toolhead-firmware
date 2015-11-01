
#define Laser_Switch_On() GPIO_ResetBits(GPIOA,GPIO_Pin_0);
#define Laser_Switch_Off() GPIO_SetBits(GPIOA,GPIO_Pin_0);

void Laser_Cmd_Handler();


