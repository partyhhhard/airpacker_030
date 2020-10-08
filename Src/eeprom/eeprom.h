#ifndef EEPROM_H_INCLUDED
#define EEPROM_H_INCLUDED

//#include "indicator.h"
#include "stdint.h"
#include "stm32f030x8.h"


typedef struct {
  int targetTemp;
  float targetMotorPwm;
  float targetBlowerPwm;
  float accDist;
  float decDist;
} tWorkSettings;

typedef enum {
  I2C_ERR = 0,
  I2C_OK = 1,
} tI2cResult;


#define DEV_ADDR        0xA0
#define DEV_MEM_SIZE    32*1024     //bytes
#define DEV_ADDR_SIZE   1           //bytes
#define DEV_PAGE_SIZE   32          //bytes

#define PROFILES_MAX_QTY    4


tI2cResult eepromWriteByte( uint16_t addr, uint8_t *data );
tI2cResult eepromReadByte( uint16_t addr, uint8_t *data );
tI2cResult eepromWriteHalfWord( uint16_t addr, uint8_t *data );
tI2cResult eepromReadHalfWord( uint16_t addr, uint8_t *data );
tI2cResult eepromWriteWord( uint16_t addr, uint8_t *data );
tI2cResult eepromReadWord( uint16_t addr, uint8_t *data );
tI2cResult eepromWriteFloat( uint16_t addr, uint8_t *data );
tI2cResult eepromReaFloat( uint16_t addr, uint8_t *data );

tI2cResult eepromWriteData( uint16_t addr, uint8_t *data, int size );
tI2cResult eepromReadData( uint16_t addr, uint8_t *data, int size );


#endif
