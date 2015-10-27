Printer head firmware protocol

* 取得基本狀態

➥  `1 HELLO *[CHKSUM]\n`

⇠ `1 OK HELLO TYPE:EXTRUDER ID:3f1f2a VENDOR:flux\ .inc FIRMWARE:xxxxxx VERSION:0.1.9 EXTRUDER:1 MAX_TEMPERATURE:250 *[CHKSUM]\n`

➥  `1 PING *[CHKSUM]\n`

⇠ `1 OK PONG 1 ER:0 RT:123.3,212.3 TT:200.0,230.0 FA:255 *[CHKSUM]\n` (for extruder)

⇠ `1 OK PONG 1 ER:0 *[CHKSUM]\n` (for laser)

Note: PONG 1=有HELLO過, RT = real temp, TT = target temp, FA = fan speed (0~255), ER: 錯誤代碼, 0=無

Note2:if TT:NAN that is target temperature had not been setted


* 溫度控制

➥  `1 H:1 T:200.0 *[CHKSUM]\n`  or number or NAN

⇠ `1 OK HEATER *[CHKSUM]\n`


* 風扇控制

➥  `1 F:1 S:255 *[CHKSUM]\n` (0~255) 強度

⇠ `1 OK FAN *[CHKSUM]\n`


* ERROR CODE


⇠ `1 ER:0 UNKNOW_COMMAND\n`

⇠ `1 ER:1 WRONG_NO_PARM\n`

⇠ `1 ER:2 PARAM_OUT_OF_RANGE\n`

⇠ `1 ER:3 COMMAND_CANNOT_BE_PROCESSSED\n`

⇠ `1 ER:4 UNKNOW_MODULE\n`

⇠ `1 ER:5 SENSOR_FAILED\n`

⇠ `1 ER:6 NO_HELLO\n`

⇠ `1 ER:7 SHAKE\n`

⇠ `1 ER:8 TILT\n`

⇠ `1 ER:9 Temperature is too high\n`

⇠ `1 ER:10 Fan do not work\n`

Note: ER:0~3 is general response,all commands must be respond.ER:4~10 is only responded by PONG.
