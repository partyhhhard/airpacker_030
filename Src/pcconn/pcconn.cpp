
#include "pcconn.h"
#include "math.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "string.h"

#define UART_BUFFER_SIZE        40
#define SEND_STATE_PERIOD       1000

extern UART_HandleTypeDef huart1;

uint8_t rxBuffer[UART_BUFFER_SIZE];
uint8_t txBuffer[UART_BUFFER_SIZE];

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  //HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, 10,0xFFFF);
}

/* UART RX complete callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  //HAL_UART_Transmit(&huart1, (uint8_t *)rxBuffer, 10,0xFFFF);
}

void pcconnTaskFunc( void const *argument ) 
{
  
  const char uartMsg[] = "airPackerV1.0| T=%d*C\r\n";

  int msCounter = 0;
  while(1) {
  
    HAL_UART_Transmit_IT( &huart1, (uint8_t*)uartMsg, strlen( uartMsg ) );
    msCounter++;
    osDelay( 1 );
  }
}