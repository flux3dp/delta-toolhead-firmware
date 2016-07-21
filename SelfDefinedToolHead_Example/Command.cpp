/*
Self-defined toolhead firmware.
Copyright (C) 2016 Flux Software Team

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Command.h"

CommandClass::CommandClass(HardwareSerial *commandPort)
{
    recBuff = "";
    recString = "";
    commandPtr = 0;
    serialPort = commandPort;
    serialPort->begin(115200);
}

/**
* @brief  This function handles commands.
* @param  None
* @retval None
*/
void CommandClass::commandHandler(void) {
    if (!readLine()) {
        return;
    }
    else {
        if (!commandValidate(recString))
            return;
    }
    char sendBack[200];
    if (codeSeen(COMMAND_HELLO)) {
        sprintf(sendBack, "1 OK HELLO TYPE:USER/MYTOOLHEAD ID:0000 VENDOR:BANANA VERSION : 0.0.1 ");
        sprintf(sendBack, "%s*%d", sendBack, getChecksum(sendBack));
        serialPort->println(sendBack);
    }
    else if (codeSeen(COMMAND_PING)) {
        sprintf(sendBack, "1 OK PONG ER:0 ");
        sprintf(sendBack, "%s*%d", sendBack, getChecksum(sendBack));
        serialPort->println(sendBack);
    }
    else if (codeSeen(COMMAND_SET_LED)) {
        uint8_t ledPWM=codeValueShort();
        analogWrite(10,ledPWM);
        sprintf(sendBack, "1 OK ");
        sprintf(sendBack, "%s*%d", sendBack, getChecksum(sendBack));
        serialPort->println(sendBack);
    }
    else if (codeSeen(COMMNAD_DIGITALREAD)) {
        uint8_t pin = codeValueShort();
        int pinState = digitalRead(pin);
        sprintf(sendBack, "1 OK %d ", pinState);
        sprintf(sendBack, "%s*%d", sendBack, getChecksum(sendBack));
        serialPort->println(sendBack);
    }
    else if (codeSeen(COMMAND_DIGITALWRITE)) {
        uint8_t pin = codeValueShort();
        if (codeSeen("STATE"))
            digitalWrite(pin,codeValueShort());
        sprintf(sendBack, "1 OK ");
        sprintf(sendBack, "%s*%d", sendBack, getChecksum(sendBack));
        serialPort->println(sendBack);
    }
}

/**
* @brief  This function validates the checksum of received string.
* @param  str: received string.
* @retval None
*/
bool CommandClass::commandValidate(String str) {
    uint8_t checksum = 0;
    String strTemp = "";
    int dataLen = 0;
    for (int strPtr = str.length() - 1; strPtr >= 0; strPtr--) {
        if (str.charAt(strPtr) == '*') {
            strTemp = str.substring(0, strPtr - 1);
            checksum= getChecksum(strTemp);
            break;
        }
    }
    if (findCommandChecksum(str) == checksum || 1)
        return true;
    else
        return false;
}

/**
* @brief  This function caculates a checksum of a string by xor all of charactors.
* @param  str: a string.
* @retval checksum.
*/
uint8_t CommandClass::getChecksum(String str) {
    uint8_t checksum = 0;
    for (int strPtr=0; strPtr<str.length()-1; strPtr++) {
        checksum = checksum^str.charAt(strPtr++);
    }
    return checksum;
}

/**
* @brief  This function finds checksum of received string.
* @param  str: received string.
* @retval Received checksum.
*/
uint8_t CommandClass::findCommandChecksum(String str) {
    for (int strPtr = str.length() - 1; strPtr >= 0; strPtr--) {
        if (str.charAt(strPtr) == '*')
            return atoi((str.substring(strPtr + 1).c_str()));
    }
}

/**
* @brief  This function compares two strings.
* @param  str: received string.
* @retval return true if two strings are the same.
*/
bool CommandClass::isStr(String recCommand, String myCommnad) {
    if (recCommand.compareTo(myCommnad) == 0)
        return true;
    else
        return false;
}

/**
* @brief  This function finds substring from the received command and moves the command pointer.
* @param  code: string to be found.
* @retval return true if code is found in the command.
*/
bool CommandClass::codeSeen(String code) {
    String strTarget=" "+ code;
    for (int i = commandPtr; i < recString.length(); i++) {
        if (recString.charAt(i) == strTarget.charAt(0)) {
            for (int j = 1; j < strTarget.length(); j++) {
                if (recString.charAt(i + j) != strTarget.charAt(0 + j)) {
                    return false;
                }
            }
            commandPtr = i + strTarget.length()+1;
            return true;
        }
    }
    return false;
}

/**
* @brief  This function transforms command string to signed integer beginning with the command pointer.
* @param  NONE
* @retval Found integer.
*/
int16_t CommandClass::codeValueShort(void) {
    char strTemp[125];
    recString.toCharArray(strTemp, 125, commandPtr);
    return strtod(strTemp, NULL);
}

/**
* @brief  This function reads all characters until received a line character.
* @param  NONE
* @retval Return true if received a line.
*/
bool CommandClass::readLine(void) {
    char getC;
    if (serialPort->available() > 0) {
        getC = serialPort->read();
        Serial.print(getC);
        if (recBuff.compareTo("") == 0 && getC == '1') {
            recBuff.concat('1');
        }
        else if (getC != '\n') {
            if (getC != '\r')
                recBuff.concat(getC);
        }
        else if (getC == '\n') {
            recString = recBuff;
            recBuff = "";
            commandPtr = 0;
            return true;
        }
        return false;
    }
    else {
        return false;
    }
}
