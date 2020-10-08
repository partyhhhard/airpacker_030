#include "gcode.h"
#include "pid_controller.h"
#include "temp_sensor.h"

extern PIDControl pid;


int GCode::executeCommand( char *cmd, int commandLength, unsigned int filePos, int fromFile )
{
  char buf[MAX_CMD_SIZE];
  int buflen;
  char c;
  GCode code;
  memset( &code, 0, sizeof( code ) );
  int res = 0;
  code.commandLength = commandLength; // будем складывать размеры всех команд что бы знать размер текущей очереди в байтах
  int32_t strPosition = 0;
  do
  {
    // Wait for a free place in command buffer
    // Scan next command from string
    
    buflen = 0;
    do
    {
      c = *(cmd++);
      strPosition++;
      if(c == 0 || c == '\n') break;
      buf[buflen++] = c;
    }
    while( buflen < MAX_CMD_SIZE );
    
    
    if(code.parseAscii((char *)buf,false) && (code.params & 518))   // Success
    {
      if( code.hasM() ) {
        switch( code.M ) {
        case 1: {
          if( code.X > 0 && code.Y && code.Z > 0 ) {
            PIDInit( &pid, code.X, code.Y, code.Z, 
             0.1, 0.0, HEATER_TIMER_DEF_PERIOD, 
             AUTOMATIC, DIRECT);   	
            resetPid( &pid );
          }
          
        }break;
        
        case 2: {
          
        }break;
        
        }
      }
          
    }
  }
  while(c);

  return res;
}



/**
Converts a ascii GCode line into a GCode structure.
*/
bool GCode::parseAscii(char *line,bool fromSerial)
{
  char *pos = line;
  //int text_counter = 0;
  params = 0;
  params2 = 0;
  
  int error = 0;
  char c;
  while ( (c = *(pos++)) )
  {
    switch(c)
    {
    case 'M':
    case 'm':
      {
        M = parseLongValue(pos) & 0xffff;
        params |= 2;
        break;
      }
    case 'X':
    case 'x':
      {
        X = parseFloatValue(pos, &error);
        params |= 8;
        break;
      }
    case 'Y':
    case 'y':
      {
        Y = parseFloatValue(pos, &error);
        params |= 16;
        break;
      }
    case 'Z':
    case 'z':
      {
        Z = parseFloatValue(pos, &error);
        params |= 32;
        break;
      }
  
    case 'L':
    case 'l':
      {
        L = parseFloatValue(pos, &error);
        params2 |= 512;
        params |= 4096; // Needs V2 for saving
        break;
      }

    default:
      break;
    }
  }
  return true;
}
