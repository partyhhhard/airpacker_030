
#include "pcconn.h"
#include "math.h"
#include "stdio.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "string.h"
#include "eeprom.h"
#include "stm32f0xx_hal_i2c.h"
#include "indicator.h"
#include "gcode.h"


#define TX_BUFFER_SIZE        30
#define RX_BUFFER_SIZE        30
#define SEND_STATE_PERIOD       5000

typedef struct {
  uint16_t crc;
  tWorkSettings data;
} tSettingsProfile;


typedef struct {
  uint16_t crc;
  tSettingsProfile storedProfile[PROFILES_MAX_QTY];
} tEEPROM;




extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;
extern tDeviceCurrentState  dcs;

uint8_t __byte = 0;
uint8_t rxIndex = 0;
char rxBuffer[RX_BUFFER_SIZE];
char txBuffer[TX_BUFFER_SIZE];

volatile uint32_t receiveCnt = 0;
volatile uint32_t transmitCnt = 0;

volatile uint8_t canReceive = 1;
uint8_t incomeDataReady = 0;
volatile uint8_t canTransmit = 1;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  receiveCnt++;
  rxBuffer[rxIndex] = __byte;
  if( __byte == 0x0D || __byte == 0x0A || __byte == '\0' ) {
    canReceive = 0;
    incomeDataReady = 1;
  }
  else {
    rxIndex++;
  }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  transmitCnt++;
  canTransmit = 1;
}


void debug_( char *msg, float value = 0 )
{
  memset( txBuffer, 0, sizeof( txBuffer ) );
  snprintf( txBuffer, sizeof( txBuffer ), "%s:%3.2f", msg, value );
  HAL_UART_Transmit_IT( &huart1, (uint8_t*)&txBuffer[0], strlen( txBuffer ) );
}

#pragma optimize = none
void pcconnTaskFunc( void const *argument ) 
{
    
//  int startB = 100;
//  int startS = 100;
//  int startT = 110;
//  uint16_t addr = 0;
//  for( int i = 0; i < PROFILES_MAX_QTY; i++ ) {
//    eeprom.storedProfile[i].data.targetBlower = startB;
//    eeprom.storedProfile[i].data.targetSpeed = startS;
//    eeprom.storedProfile[i].data.targetTemp = startT;
//    eeprom.storedProfile[i].crc = 0;
//    startB -= 12;
//    startS -= 8;
//    startT -= 5;
//    eepromWriteData( addr, (uint8_t*)&eeprom.storedProfile[i], sizeof( tSettingsProfile ) );
//    addr += sizeof( tSettingsProfile );
//  }
  osDelay(2000);
  tEEPROM eeprom;
  memset( &eeprom, 0, sizeof( eeprom ) );
  uint16_t addr = 0;
  for( int i = 0; i < PROFILES_MAX_QTY; i++ ) {
    eepromReadData( addr, (uint8_t*)&eeprom.storedProfile[i], sizeof( tSettingsProfile ) );
    addr += sizeof( tSettingsProfile );
  }
  
  int msCounter = 0; 

  while(1) {
  
    if( msCounter == SEND_STATE_PERIOD && canTransmit == 1 ) {
      snprintf( txBuffer, sizeof( txBuffer ), "airPackerV1.0 T = %d C\r\n", dcs.temperature );
      HAL_UART_Transmit_IT( &huart1, (uint8_t*)&txBuffer[0], strlen( txBuffer ) );
      msCounter = 0;
    }
//    if( ( msCounter % 300 == 0 ) && ( dcs.state == STATE_WORKING || dcs.state == STATE_IDLE ) ) {
//     
////       snprintf( txBuffer, sizeof( txBuffer ), "T = %d C; M = %5.2f%%; H = %d%% B = %d%%\r\n", 
////                dcs.temperature, dcs.motorPwm, 
////                dcs.heaterPwm, dcs.blowerPwm );
//      snprintf( txBuffer, sizeof( txBuffer ), "M = %5.2f%%\r\n", dcs.motorPwm );
//      HAL_UART_Transmit_IT( &huart1, (uint8_t*)&txBuffer[0], strlen( txBuffer ) );
//
//    }
    if( incomeDataReady ) {
      __HAL_UART_SEND_REQ( &huart1, UART_RXDATA_FLUSH_REQUEST);
     
      HAL_UART_Transmit_IT( &huart1, (uint8_t*)&rxBuffer, strlen( rxBuffer ) );
      GCode::executeCommand( (char*)rxBuffer );
      
      memset( &rxBuffer, 0, sizeof( rxBuffer ) );
      rxIndex = 0;
      incomeDataReady = 0;
      canReceive = 1;
    }
    msCounter++;
    osDelay( 1 );
  }
}