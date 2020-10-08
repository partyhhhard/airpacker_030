
#include "eeprom.h"
#include "math.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "stm32f0xx_hal_i2c.h"

extern I2C_HandleTypeDef hi2c1;





tI2cResult eepromWriteByte( uint16_t addr, uint8_t *data )
{
  tI2cResult res = I2C_ERR;
  if( HAL_OK == HAL_I2C_Mem_Write( &hi2c1, DEV_ADDR, addr, 2, data, 1, 2 ) ) {
    res = I2C_OK;
  }
  osDelay(2);
  return res;
}

tI2cResult eepromReadByte( uint16_t addr, uint8_t *data )
{
  tI2cResult res = I2C_ERR;
  if( HAL_OK == HAL_I2C_Mem_Write( &hi2c1, DEV_ADDR, addr, 2, data, 1, 2 ) ) {
    res = I2C_OK;
  }
  osDelay(2);
  return res;
}

tI2cResult eepromWriteData( uint16_t addr, uint8_t *data, int size ) 
{
   int trycnt = 5;
  tI2cResult res = I2C_ERR;
  for( int i = 0; i < size; i++ ) {
    trycnt = 5;
    while( trycnt > 0 ) {
      if( HAL_OK == HAL_I2C_Mem_Write( &hi2c1, DEV_ADDR, addr + i, 2, (uint8_t*)&data[i], 1, 2 ) ) {
        res = I2C_OK;
        break;
      }
     trycnt--;
    }
   osDelay(5);
  }
  return res;
}
tI2cResult eepromReadData( uint16_t addr, uint8_t *data, int size )
{
  int trycnt = 5;
  tI2cResult res = I2C_ERR;
  for( int i = 0; i < size; i++ ) {
    trycnt = 5;
    while( trycnt > 0 ) {
      if( HAL_OK == HAL_I2C_Mem_Read( &hi2c1, DEV_ADDR, addr + i, 2, (uint8_t*)&data[i], 1, 2 ) ) {
        res = I2C_OK;
        break;
      }
     trycnt--;
    }
   osDelay(5);
  }
  return res;
}