#define Firmware_Version	"1.0.4"
#define Firmware_Name		"EXCALIBUR"
#define Firmware_Date       "20160217"
#define STM32F0_UUID ((uint32_t *)0x1FFFF7AC)
#define Enable_IWDG 1
#define Enable_Debug_Msg 0
//#define Function_Test 1

static const char Vender[] = "FLUX\\ .inc";

typedef enum
{
	FLUX_ONE_EXTRUDER_MODULE,
	FLUX_DUO_EXTRUDER_MODULE,
	FLUX_LASER_MODULE,
	Unknow
}ModuleMode_Type;

/*
Version history
1.0.0   20160129    Add extruder one using time
1.0.1   20160129    Fix flash using time bug(shut down when flash and uart interrup at the same time)
1.0.2   20160201    Flash using time a whole page
1.0.3   20160202    Add measure laser power down function(factory test)
1.0.4   20160217    Add debounce laser power function(500ms)






*/
