#define Firmware_Version	1.0924
#define STM32F0_UUID ((uint32_t *)0x1FFFF7AC)
#define Enable_IWDG 1
#define Enable_Debug_Msg 1
#define Function_Test

static const char Vender[] = "FLUX\\ .inc";

typedef enum
{
	FLUX_ONE_EXTRUDER_MODULE,
	FLUX_DUO_EXTRUDER_MODULE,
	FLUX_LASER_MODULE,
	Unknow
}ModuleMode_Type;
