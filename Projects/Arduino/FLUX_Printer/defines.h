#define Firmware_Version	-2.01112
#define Firmware_Name		"OHMAMA"
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
