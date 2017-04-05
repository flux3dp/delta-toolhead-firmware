#define Firmware_Version	"1.2.17"
#define Firmware_Name		"EXCALIBUR"//"gyro_test"//
#define Firmware_Date       "20170405"
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
    FLUX_ONE_EXTRUDER_REV1_MODULE,//Compare to FLUX_ONE_EXTRUDER_MODULE:Inverting heater thermal temperature mapping¡B3 tires exhalation fan¡Bcorrespond factory test
    FLUX_ONE_EXTRUDER_REV2_MODULE,//Compare to FLUX_ONE_EXTRUDER_REV1_MODULE:Replace 3 tires exhalation fan to 2 tires.
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
1.0.9   20160307    Heater monitoring
1.1.0   20160308    Add temperature management feature.
1.1.1   20160311    Bug:Hardware failure cause error 512.
1.1.2   20160314    Alter factory testing items.
1.2.0   20160505    Replace heater thermal,add extruder one rev1.
1.2.1   20160601    Change NTC upper temperature to 55C.
1.2.2   20160621    Addtional error code of hardware error. <-*
1.2.3   20160830    Change NTC factory test temperature.
1.2.4   20160831    Change heater output factory test for extruder R1.
1.2.5   20160907    Add new hardware extruder one R2 & self-reset alarm.
1.2.6   20160929    Revise: if HARDWARE_ERROR has not appeared , it would not response "HE: " with PING command.
1.2.7   20161006    NTC sampling.
1.2.8   20161123    Revise hello command messages for new extruder hardware.
1.2.9   20170123    Revise extruder command timeout 300s to 30s
1.2.10  20170216    Add exhalation fan error code.
1.2.11  20170217    Extruder_One_Rev2_ADC_Min_Value 4080->4000.
1.2.12  20170217    Exhalation fan error bug fixed.
1.2.13  20170302    Fix temperature managment process.
1.2.14  20170308    Improve PID control process of heater.
1.2.15  20170313    Add temperature filter to improve heater realiability.
1.2.16  20170314    Add kalman filter to keep heater temperature stable.
1.2.17  20170328    1.Temperature management using kalman estimated temperature. 2.Fixed error code bug.
*/
