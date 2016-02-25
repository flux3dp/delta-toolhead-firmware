#include "utilities.h"
#include "Using_Time.h"
#include "command.h"
#include "heater.h"
#include <stdio.h>
#include "LaserModule.h"

static volatile uint32_t Using_Last_Time=0;
uint32_t Current_Using_Time_Addr=Using_Time_Start_Addr;
uint32_t Current_Using_Time_Backup_Addr=Using_Time_Backup_Start_Addr;

bool Check_Using_Time_Validation(uint32_t Addr);
bool Erase_Using_Time_Page(uint32_t Addr);
bool Write_Using_Time(uint32_t Addr,uint32_t data);
uint32_t Read_Rom_Data(uint32_t Addr);
bool Add_Using_Time(uint16_t time_minute);
uint32_t Laser_Time_Count=0;

extern Laser_Status_Type User_Switch;
bool Using_Time_Initial(void){
    //find maxmun number or no data
    uint32_t Max_Using_Time_Addr=0;
    uint32_t Max_Using_Time=0;
    uint32_t Max_Using_Time_Backup_Addr=0;
    uint32_t Max_Using_Time_Backup=0;
    int i;
    for(i=0;i<256;i++){
        if(Check_Using_Time_Validation(Using_Time_Start_Addr+i*8)){
            if(Read_Rom_Data(Using_Time_Start_Addr+i*8)>Max_Using_Time){
                Max_Using_Time=Read_Rom_Data(Using_Time_Start_Addr+i*8);
                Max_Using_Time_Addr=Using_Time_Start_Addr+i*8;
            }
        }
        if(Check_Using_Time_Validation(Using_Time_Backup_Start_Addr+i*8)){
            if(Read_Rom_Data(Using_Time_Backup_Start_Addr+i*8)>Max_Using_Time_Backup){
                Max_Using_Time_Backup=Read_Rom_Data(Using_Time_Backup_Start_Addr+i*8);
                Max_Using_Time_Backup_Addr=Using_Time_Backup_Start_Addr+i*8;
            }
        }
    }
    printf("Max_addr=%x\t Back_addr=%x\n",Max_Using_Time_Addr,Max_Using_Time_Backup_Addr);
    if(Max_Using_Time_Addr==0 && Max_Using_Time_Backup_Addr==0){
        Current_Using_Time_Addr=Using_Time_Start_Addr;
    }else{
        if(Max_Using_Time_Backup>Max_Using_Time){
            Current_Using_Time_Addr=Max_Using_Time_Backup_Addr-FLASH_PAGE_SIZE;
        }else{
            Current_Using_Time_Addr=Max_Using_Time_Addr;
        }
    }
    printf("Current addr=%x\n",Current_Using_Time_Addr);
}

bool Erase_Using_Time_Page(uint32_t Addr){
    Uart1_ISR_Disable();//Uart interrupt will cause a shutdown when writing flash
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 
	FLASH_ErasePage(Addr);
    FLASH_Lock();
    Uart1_ISR_Enable();
}

bool Write_Using_Time(uint32_t Addr,uint32_t data){
    uint32_t Data_Verify;
	uint32_t Data_Trans=data;
    Uart1_ISR_Disable();//Uart interrupt will cause a shutdown when writing flash
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 
	//FLASH_ErasePage(Addr);
	FLASH_ProgramWord(Addr , Data_Trans);
	FLASH_ProgramWord(Addr+4	, (Data_Trans^0x12345678));
	FLASH_Lock();
    Uart1_ISR_Enable();
	Data_Verify = *(__IO uint32_t *)Addr ;
	if(Data_Verify == Data_Trans)
	{	
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}
bool Check_Using_Time_Validation(uint32_t Addr){
    int32_t FL_Data,Checksum;
	FL_Data=*(__IO uint32_t *)Addr ;
	Checksum=*(__IO uint32_t *)(Addr+4);
	if((FL_Data^0x12345678) == Checksum){
		return TRUE;
	}else{
		return FALSE;
	}
}

bool Add_Using_Time(uint16_t time_minute){
    uint32_t current_data;
    //Synchronize
    if(Check_Using_Time_Validation(Current_Using_Time_Addr)){
        if(!Check_Using_Time_Validation(Current_Using_Time_Addr+FLASH_PAGE_SIZE)){
            Erase_Using_Time_Page(Using_Time_Backup_Start_Addr);
            Write_Using_Time(Current_Using_Time_Addr+FLASH_PAGE_SIZE,Read_Rom_Data(Current_Using_Time_Addr));
        }
    }else{
        if(Check_Using_Time_Validation(Current_Using_Time_Addr+FLASH_PAGE_SIZE)){
            Erase_Using_Time_Page(Using_Time_Start_Addr);
            Write_Using_Time(Current_Using_Time_Addr,Read_Rom_Data(Current_Using_Time_Addr+FLASH_PAGE_SIZE));
        }else{
            Erase_Using_Time_Page(Using_Time_Start_Addr);
            Write_Using_Time(Current_Using_Time_Addr,0);
            Erase_Using_Time_Page(Using_Time_Backup_Start_Addr);
            Write_Using_Time(Current_Using_Time_Addr+FLASH_PAGE_SIZE,0);
        }
    }
    
    current_data=Read_Rom_Data(Current_Using_Time_Addr);
    //check data end
    if(Current_Using_Time_Addr==Using_Time_Last_Addr){
        Erase_Using_Time_Page(Using_Time_Start_Addr);
        Erase_Using_Time_Page(Using_Time_Backup_Start_Addr);
        Current_Using_Time_Addr=Using_Time_Start_Addr;
        
    }else{
        Current_Using_Time_Addr+=8;
    }
    
    Write_Using_Time(Current_Using_Time_Addr,current_data+time_minute);
    Write_Using_Time(Current_Using_Time_Addr+FLASH_PAGE_SIZE,Read_Rom_Data(Current_Using_Time_Addr));
}

void Using_Time_Extruder_One_Record(void){
    uint32_t interval=millis()-Using_Last_Time;
    if(interval > 300000){// 5 minutes
        Using_Last_Time=millis();
        if(Read_Temperature()>50.0)//50
            Add_Using_Time(5);
    }
}

void Using_Time_Laser_Record(void){
    uint32_t interval=millis()-Using_Last_Time;
    if(interval > 1000){
        Using_Last_Time=millis();
         
        if(User_Switch==Laser_Power_On){
            Laser_Time_Count++;
        }
        if(Laser_Time_Count>=60){
            Laser_Time_Count=0;
            Add_Using_Time(1); 
        }
    }
    
}

uint32_t Read_Using_Time(void){
    //Synchronize
    if(Check_Using_Time_Validation(Current_Using_Time_Addr)){
        return Read_Rom_Data(Current_Using_Time_Addr);
    }else if(Check_Using_Time_Validation(Current_Using_Time_Addr+FLASH_PAGE_SIZE)){
        return Read_Rom_Data(Current_Using_Time_Addr+FLASH_PAGE_SIZE);
    }else{
        return 0;
    }
    
}


