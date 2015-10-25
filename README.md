PrinterHead Firmware

##PrinterHead Xcode command:
```
A                                                                            : PID Autotune
ok@A

H + O + num (0 ~ 2625)                                                       : set heater temp
ok@HO+num

H + F                                                                        : set heater off
ok@HF

F + 1(Fan1) + num (0 ~ 255)                                                  : set fan1 duty
ok@F1+num

F + 2(Fan2) + O                                                              : set fan2 on
ok@F2O

F + 2(Fan2) + F                                                              : set fan2 off
ok@F2F

W + num                                                                      : write device id
ok@W

R + (A,C,E,I,M,T,V)                                                          : read board info (A:all, C:cmd list I:device id, M:module, T:read temp V:fw version)
ok@RA
ok@RC
ok@RE
ok@RI
ok@RM
ok@RT : + degree
ok@RV

K
ok@K
                                                                            : sensor calibration
S + (A,G)                                                                    : read sensor raw data
ok@S_A
ok@S_G

Q                                                                            : clear flag
ok@Q


Event	1	Boot Success
	2	Fan2 Failure
	3	Fan3 Failure
	4	Sensor Connection Success
	5	Sensor Connection Failure
	6	Sensor Gyro_x failure
	7	Sensor Gyro_y failure
	8	Sensor Gyro_z failure
	9	Sensor Accelerometer_x failure
	10	Sensor Accelerometer_y failure
	11	Sensor Accelerometer_z failure
	12	ID 0 failure
	13	ID 1 failure
	14	Over temperature
	15	Device write success
	16	Device write fail
	17	Heater On
	18	Heater Off

```