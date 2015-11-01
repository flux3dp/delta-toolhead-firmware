#define Firmware_Version	1.0922

//#define PRINT_DEBUG_MESSAGE 1
//#define FLUX_LASER_MODULE 1
//#define FLUX_3DPRINTER_MODULE 2

////define this firmware working mode
//#define FLUX_MODULE_MODE FLUX_3DPRINTER_MODULE

static const char Vender[] = "FLUX\\ .inc";

typedef enum
{
	FLUX_ONE_EXTRUDER_MODULE,
	FLUX_DUO_EXTRUDER_MODULE,
	FLUX_LASER_MODULE,
	Unknow
}ModuleMode_Type;
