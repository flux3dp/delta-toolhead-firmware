#define Read_Laser_Power() GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2)

typedef enum
{
	Laser_Power_On,
	Laser_Power_Down
}Interlock_Status_Type;

void Laser_Cmd_Handler(void);
void Laser_Switch_On(void);
void Laser_Switch_Off(void);
void Detect_Laser_Power(void);
void Debounce_Laser_Power(void);


