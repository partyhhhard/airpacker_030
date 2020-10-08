
#ifndef _GCODE_H
#define _GCODE_H

#include "string.h"
#include <stdbool.h>
#include "stdio.h"
#include "stdint.h"
#include <stdlib.h>
#include "cmsis_os.h"

#define MAX_CMD_SIZE 40
#define ARRAY_SIZE(_x)	(sizeof(_x)/sizeof(_x[0]))

#define LAYER_HEIGHT_LABEL                              ";POLYGON_LAYER_HEIGHT="
#define FIRST_LAYER_HEIGHT_LABEL                        ";POLYGON_FIRST_LAYER_HEIGHT="
#define BASIC_FEEDRATE_LABEL                            ";POLYGON_BASIC_FEEDRATE="

#define POLYGON_SOLUBLE_SUPPORT                         ";POLYGON_SOLUBLE="

#define POLYGON_SETTINGS_END                            ";POLYGON_END_PARAMS"

#define POLYGON_AVG_TIME_QUALITY                        ";TASK_TIME_POLYGON_0="                                                    
#define POLYGON_AVG_TIME_STANDART                       ";TASK_TIME_POLYGON_1="
#define POLYGON_AVG_TIME_FAST                           ";TASK_TIME_POLYGON_2="
#define POLYGON_AVG_TIME_SKETCH                         ";TASK_TIME_POLYGON_3="

class GCode
{
public:
    int commandLength;
    uint16_t params;
    uint16_t params2;
    uint16_t M;
    float X;
    float Y;
    float Z;
    float L;
    inline void setX(float value)
    {
      params |= 8;
      X = value;
    }
    inline void setY(float value)
    {
      params |= 16;
      Y = value;
    }
    inline void setZ(float value)
    {
      params |= 32;
      Z = value;
    }

    inline bool hasM()
    {
        return ((params & 2)!=0);
    }

 
    inline void setFormatError() {
        params2 |= 32768;
    }
    inline bool hasFormatError() {
        return ((params2 & 32768)!=0);
    }
    

 
        
   
    bool parseAscii(char *line,bool fromSerial);

    static int executeCommand( char *cmd, int commandLength = 0, unsigned int filePos = 0, int fromFile = 0 );

private:
    void debugCommandBuffer();
    void checkAndPushCommand();
    static void requestResend();
    inline float parseFloatValue(char *s, int *error )
    {
        error = 0;
        char *endPtr;
        while(*s == 32) s++; // skip spaces
        float f = (strtod(s, &endPtr));
        if(s == endPtr) {
          f=0.0; // treat empty string "x " as "x0"
          error++;
        }
        return f;
    }
    inline long parseLongValue(char *s)
    {
        char *endPtr;
        while(*s == 32) s++; // skip spaces
        long l = (strtol(s, &endPtr, 10));
        if(s == endPtr) l=0; // treat empty string argument "p " as "p0"
        return l;
    }

    static uint8_t commandReceiving[MAX_CMD_SIZE]; ///< Current received command.
    static uint8_t commandsReceivingWritePosition; ///< Writing position in gcode_transbuffer.

};

#endif