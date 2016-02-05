#include "gpio.h"

#define Using_Time_Start_Addr             ((uint32_t)0x0803E800)
#define Using_Time_Last_Addr        ((uint32_t)0x0803EFF8)    
#define Using_Time_Backup_Start_Addr      ((uint32_t)0x0803F000)
#define Using_Time_Backup_Last_Addr ((uint32_t)0x0803F7F8) 
uint32_t Read_Using_Time(void);
void Using_Time_Extruder_One_Record(void);
void Using_Time_Laser_Record(void);
bool Using_Time_Initial(void);

