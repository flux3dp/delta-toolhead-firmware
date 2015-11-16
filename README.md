Printer head firmware protocol

* 取得基本狀態

➥  `1 HELLO *[CHKSUM]\n` 或 ➥  `1 DEBUG\n`

⇠ `1 OK HELLO TYPE:EXTRUDER ID:3f1f2a VENDOR:flux\ .inc FIRMWARE:xxxxxx VERSION:0.1.9 EXTRUDER:1 MAX_TEMPERATURE:250 *[CHKSUM]\n`

⇠ `1 OK HELLO TYPE:LASER ID:3f1f2a VENDOR:flux\ .inc FIRMWARE:xxxxxx VERSION:0.1.9 FOCAL_LENGTH:3.1 *[CHKSUM]\n`

➥  `1 PING *[CHKSUM]\n`

⇠ `1 OK PONG ER:0 RT:123.3,212.3 TT:200.0,230.0 FA:255 *[CHKSUM]\n` (for extruder)

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

➥  `1 FL:-3.3 *[CHKSUM]\n`  Focal length in millimeter

⇠ `1 OK *[CHKSUM]\n`

⇠ `1 ER FAILLED\n`

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

⇠ `1 ER:64 PID_OUT_OF_CONTROL\n`

⇠ `1 ER:128 FAN_FAILURE\n`

Note: ER are general error responses,all commands could respond,ER:1~64 are only responded by PONG.

DEBUG MODE:

  `Show gyrosope sensor data`
  
  `Command has no timeout`
  
  `No chechsum check`
  
COMMON MODE:

  `Laser head has 1s timeout`
  
  `Print head has 5 minutes timeout`
  
  `It do not show sensor data automatically`
  
