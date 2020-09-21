
#include "pcconn.h"
#include "math.h"
#include "stdio.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "string.h"
#include "eeprom.h"
#include "stm32f0xx_hal_i2c.h"
#include "indicator.h"


#define TX_BUFFER_SIZE        40
#define RX_BUFFER_SIZE        40
#define SEND_STATE_PERIOD       1000

extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;
extern tDeviceCurrentState  deviceCurrentState;

uint8_t __byte = 0;
uint8_t rxIndex = 0;
char rxBuffer[RX_BUFFER_SIZE];
char txBuffer[TX_BUFFER_SIZE];

volatile int receiveCnt = 0;
volatile int transmitCnt = 0;

volatile uint8_t canReceive = 1;
uint8_t incomeDataReady = 0;
volatile uint8_t canTransmit = 1;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  receiveCnt++;
  rxBuffer[rxIndex] = __byte;
  if( __byte == 0x0D || __byte == 0x0A) {
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

#pragma optimize = none
void pcconnTaskFunc( void const *argument ) 
{
  int msCounter = 0; 
  osDelay(1000);

 // __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC | UART_IT_RXNE  );
  //HAL_UART_Receive_IT( &huart1, (uint8_t*)__byte, 1 );
  while(1) {
  
    if( msCounter == SEND_STATE_PERIOD && canTransmit == 1 ) {
      snprintf( txBuffer, sizeof( txBuffer ), "airPackerV1.0\r\n" );
      HAL_UART_Transmit_IT( &huart1, (uint8_t*)&txBuffer[0], strlen( txBuffer ) );
      osDelay(1);
      snprintf( txBuffer, sizeof( txBuffer ), "temp   %d*C\r\n", deviceCurrentState.temperature );
      HAL_UART_Transmit_IT( &huart1, (uint8_t*)&txBuffer[0], strlen( txBuffer ) );
      osDelay(1);
      snprintf( txBuffer, sizeof( txBuffer ), "heater %d \r\n", deviceCurrentState.heaterPwm );
      HAL_UART_Transmit_IT( &huart1, (uint8_t*)&txBuffer[0], strlen( txBuffer ) );
      osDelay(1);
      snprintf( txBuffer, sizeof( txBuffer ), "motor  %d \r\n", deviceCurrentState.motorPwm );
      HAL_UART_Transmit_IT( &huart1, (uint8_t*)&txBuffer[0], strlen( txBuffer ) );
      osDelay(1);
      snprintf( txBuffer, sizeof( txBuffer ), "blower %d \r\n", deviceCurrentState.blowerPwm );
      HAL_UART_Transmit_IT( &huart1, (uint8_t*)&txBuffer[0], strlen( txBuffer ) );
      osDelay(1);
      msCounter = 0;
    }
    if( incomeDataReady ) {
      __HAL_UART_SEND_REQ( &huart1, UART_RXDATA_FLUSH_REQUEST);
      
      rxIndex = 0;
      incomeDataReady = 0;
      canReceive = 1;
    }
    msCounter++;
    osDelay( 1 );
  }
}