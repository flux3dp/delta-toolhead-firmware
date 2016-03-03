#define Firmware_Version	"1.0.8"
#define Firmware_Name		"EXCALIBUR"//"TESTING"//
#define Firmware_Date       "20160301"
#define STM32F0_UUID ((uint32_t *)0x1FFFF7AC)
#define Enable_IWDG 1
#define Enable_Debug_Msg 0
//#define Function_Test 1

static const char Vender[] = "FLUX\\ Inc.";

typedef enum
{
	FLUX_ONE_EXTRUDER_MODULE,
	FLUX_DUO_EXTRUDER_MODULE,
	FLUX_LASER_MODULE,
	Unknow
}ModuleMode_Type;

/*
Version history
1.0.0   20160129    Add extruder one using time.
1.0.1   20160129    Fix flash using time bug(shut down when flash and uart interrup at the same time).
1.0.2   20160201    Flash using time a whole page.
1.0.3   20160202    Add measure laser power down function(factory test).
1.0.4   20160217    Add debounce laser power function(500ms) & fix laser using time.
1.0.5   20160224    Fix heater can be set even thermal not work. Enable internal temperature sensor.
1.0.6   20160225    Lower ceiling temperature to 45 celsius degree.
1.0.7   20160226    Change ceiling temperature to 42 celsius degree.
1.0.8   20160226    Fix HARDWARE_ERROR bug.


*/
