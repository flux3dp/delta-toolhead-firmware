#include "cmd.h"

String recBuff = "";
String recString = "";
int commandPtr = 0;

void readLine(void);
int16_t code_value_short(void);
bool codeSeen(String code);
bool isStr(String recCommand, String myCommnad);
bool find(String str, String subString);
String getCommand(String str);
bool commandValidate(String str);
uint8_t getChecksum(String str);

void commandHandler(void) {
    readLine();
    if (isStr(recString, COMMAND_NOT_FOUND)) {
        return;
    }
    else {
        if (!commandValidate(recString))
            return;
    }

    if (codeSeen(COMMAND_PING)) {
        Serial.print("PpinG:");
        Serial.println(code_value_short());
        if (codeSeen("Z")) {
            Serial.print("Z:");
            Serial.println(code_value_short());
        }
        if (codeSeen("Y")) {
            Serial.print("Y:");
            Serial.println(code_value_short());
        }
        if (codeSeen("a")) {
            Serial.print("a:");
            Serial.println(code_value_short());
        }
        if (codeSeen("b")) {
            Serial.print("b:");
            Serial.println(code_value_short());
        }
    }
}

bool commandValidate(String str) {
    uint8_t checksum = 0;
    String strTemp = "";
    int dataLen = 0;
    for (int strPtr = str.length() - 1; strPtr >= 0; strPtr--) {
        if (str.charAt(strPtr) == '*') {
            strTemp = str.substring(0, strPtr - 1);
            while (dataLen<strPtr) {
                checksum = checksum^str.charAt(dataLen++);
            }
            break;
        }
    }
    if (getChecksum(str) == checksum || 1)
        return true;
    else
        return false;

}

uint8_t getChecksum(String str) {
    for (int strPtr = str.length() - 1; strPtr >= 0; strPtr--) {
        if (str.charAt(strPtr) == '*')
            return atoi((str.substring(strPtr + 1).c_str()));
    }
}

String getCommand(String str) {
    uint8_t pos;
    for (int strPtr = 2; strPtr < str.length() - 1; strPtr++) {
        if (str.charAt(strPtr) == ' ') {
            pos = strPtr;
            return str.substring(2, pos);
        }
    }
    return "";
}
bool isStr(String recCommand, String myCommnad) {
    if (recCommand.compareTo(myCommnad) == 0)
        return true;
    else
        return false;
}


bool find(String str, String subString) {

    for (int i = commandPtr; i < recString.length(); i++) {
        if (recString.charAt(i) == subString.charAt(0)) {
            for (int j = 1; j < subString.length(); j++) {
                if (recString.charAt(i + j) != subString.charAt(0 + j)) {
                    return false;
                }
            }
            commandPtr= i + subString.length();
            return true;
        }
    }
    return false;
}
bool codeSeen(String code) {    
    return find(recString, code);
}
int16_t code_value_short(void) {
    char strTemp[125];
    recString.toCharArray(strTemp, 125, commandPtr);
    //char strTemp= str.substring(commandPtr).c_str();
    return strtod(strTemp, NULL);
}

void readLine(void) {
    char getC;
    if (MySerial.available() > 0) {
        getC = MySerial.read();
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
            return;
        }
        return;
    }
    else {
        return;
    }
}
