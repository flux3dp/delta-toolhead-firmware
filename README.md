**Version history**

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

1.2.2   20160621    Addtional error code of hardware error.

1.2.3   20160830    Change NTC factory test temperature.

1.2.4   20160831    Change heater output factory test for extruder R1.


**Printer head firmware protocol**

* 取得基本狀態

➥  `1 HELLO *[CHKSUM]\n` 或 ➥  `1 DEBUG\n`

⇠ `1 OK HELLO TYPE:EXTRUDER ID:3f1f2a VENDOR:flux\ .inc FIRMWARE:xxxxxx VERSION:0.1.9 EXTRUDER:1 MAX_TEMPERATURE:250 USED:180  *[CHKSUM]\n`

⇠ `1 OK HELLO TYPE:LASER ID:3f1f2a VENDOR:flux\ .inc FIRMWARE:xxxxxx VERSION:0.1.9 FOCAL_LENGTH:3.1 USED:180 *[CHKSUM]\n`

➥  `1 PING *[CHKSUM]\n`

⇠ `1 OK PONG ER:0 RT:123.3 TT:200.0 FA:255 *[CHKSUM]\n` (for extruder)

⇠ `1 OK PONG ER:0 *[CHKSUM]\n` (for laser)

Note: PONG 1=有HELLO過, RT = real temp, TT = target temp, FA = fan speed (0~255), ER: 錯誤代碼, 0=無

Note2:if TT:NAN that is target temperature had not been setted


* 溫度控制

➥  `1 H:0 T:200.0 *[CHKSUM]\n`  or number or NAN

⇠ `1 OK HEATER *[CHKSUM]\n`


* 風扇控制

➥  `1 F:0 S:255 *[CHKSUM]\n` (0~255) 強度

⇠ `1 OK FAN *[CHKSUM]\n`

* 焦距寫入

➥  `1 WRITE FL:-3.3 *[CHKSUM]\n`  Focal length in millimeter

⇠ `1 OK WRITE *[CHKSUM]\n`

⇠ `1 ER FAILED\n`

* ERROR CODE


⇠ `1 ER UNKNOW_COMMAND\n`

⇠ `1 ER WRONG_NO_PARM\n`

⇠ `1 ER PARAM_OUT_OF_RANGE\n`

⇠ `1 ER COMMAND_CANNOT_BE_PROCESSSED\n`

⇠ `1 ER FAILLED\n`

⇠ `1 ER:1 UNKNOW_MODULE\n`

⇠ `1 ER:2 SENSOR_FAILURE\n`

⇠ `1 ER:4 NO_HELLO\n`

⇠ `1 ER:8 SENSOR_CALIBRATION_FAILURE\n`

⇠ `1 ER:16 SHAKE\n`

⇠ `1 ER:32 TILT\n`

⇠ `1 ER:64 HARDWARE_ERROR\n`

⇠ `1 ER:128 FAN_FAILURE\n`

⇠ `1 ER:256 LASER_DOWN\n` 

⇠ `1 ER:512 HEATER_FAILURE\n` 

* Hardware Error

⇠ `THERMAL_SHORT` 

⇠ `THERMAL_OPEN` 

⇠ `OVER_TEMP` 

⇠ `NTC_OVER_TEMP` 

⇠ `AUTO_HEAT` 

⇠ `CANNOT_HEAT` 

e.g 
⇠ `1 OK PONG ER:64 RT:383.3 TT:200.0 FA:255 HE:OVER_TEMP,AUTO_HEAT *[CHKSUM]\n` (for extruder)

Note: ER are general error responses,all commands could respond,ER:1~128 are only responded by PONG.

*ER:64 HARDWARE_ERROR

  1.Thermal short`
  
  2.Thernal open`
  
  3.Heater over temperature(245°C)`
  
  4.NTC over temperature(42°C)`
  
  5.Heat automaticlly(trigger ER 512)`
  
  6.Heater can't be heat(trigger ER 512)`
  

DEBUG MODE:

  `Show gyrosope sensor data`
  
  `Command has no timeout`
  
  `No chechsum check`
  
COMMON MODE:

  `Laser head has 1s timeout`
  
  `Print head has 5 minutes timeout`
  
  `It do not show sensor data automatically`
  
