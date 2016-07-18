// command.h

#ifndef _COMMAND_h
#define _COMMAND_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#endif

#define MySerial            Serial
#define COMMAND_NOT_FOUND   ""
#define COMMAND_HELLO       "HELLO"
#define COMMAND_PING        "PING"


void commandHandler(void);
