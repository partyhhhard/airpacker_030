#include "indicator.h"
#include "math.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "stm32f0xx_hal_spi.h"

uint32_t timePressed[6] = {0};
uint32_t timeReleased[6] = {0};

#define LONG_BUTON_MIN_TIME                 500

extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim6;
EXTI_HandleTypeDef lineButtonOne;
EXTI_HandleTypeDef lineButtonTwo;

EXTI_HandleTypeDef lineGoButton;
EXTI_HandleTypeDef lineEditButton;
EXTI_HandleTypeDef lineLeftButton;
EXTI_HandleTypeDef lineRightButton;
EXTI_ConfigTypeDef buttonExti;

uint32_t irqDisableTime;


const uint8_t numToLcd[10] = { dig0, dig1, dig2, dig3, dig4, dig5, dig6, dig7, dig8, dig9 };
uint8_t currentR = 0;
uint8_t r[4] = {0};
volatile bool buttonHandled = true;


volatile uint8_t changeHeaterState;

tButton buttonOne = { RELEASED, BUTTON_ONE_PIN, 1, 0, 2 };
tButton buttonTwo = { RELEASED, BUTTON_TWO_PIN, 1, 0, 2 };
tButton buttonEdit = { RELEASED, BUTTON_EDIT_PIN, 1, 0, 2 };
tButton buttonGo = { RELEASED, BUTTON_GO_PIN, 1, 0, 2 };
tButton buttonLeft = { RELEASED, BUTTON_LEFT_PIN, 1, 0, 2 };
tButton buttonRight = { RELEASED, BUTTON_RIGHT_PIN, 1, 0, 2 };




tDeviceMenuState deviceMenuState;
 
  
tDeviceCurrentState  deviceCurrentState;


void divideIntoDigits( uint16_t number ) {
  r[0] = number % 10;
  r[0] = numToLcd[r[0]];
  r[1] = number % 100 / 10;
  r[1] = numToLcd[r[1]];
  r[2] = number % 1000 / 100;
  r[2] = numToLcd[r[2]];

}

extern "C" void enableR0( void ) {
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
}
extern "C" void disableR0( void ) {
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
}

extern "C" void enableR1( void ) {
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
}
extern "C" void disableR1( void ) {
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
}

extern "C" void enableR2( void ) {
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
}
extern "C" void disableR2( void ) {
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
}

extern "C" void enableR3( void ) {
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
}
extern "C" void disableR3( void ) {
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
}

extern "C" void disableAllR( void ) {
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,GPIO_PIN_RESET);
}


void showValue( int *value, tMenuState *state  ) {
  
  divideIntoDigits( *value );
  disableAllR();
  uint8_t valueType = 0;
  if( currentR == 0 ) {
    //disableR2();
    uint8_t byte = r[0];//numToLcd[i];//~(((data[i]<<1) | (1 << 7)));
    //seg[i] = byte;
    cs0();
    HAL_SPI_Transmit( &hspi1, (uint8_t*)&byte, 1, 100 );
//    while( HAL_SPI_GetState( &hspi1 ) != HAL_SPI_STATE_READY ) {
//      //cnt++;
//    }
    //cnt = 0;
    cs1();
    cs0();
    cs1();
    enableR0();
  }
  else if( currentR == 1 ) {
    //disableR0();
    uint8_t byte = r[1];//numToLcd[i];//~(((data[i]<<1) | (1 << 7)));
    //seg[i] = byte;
    cs0();
    HAL_SPI_Transmit( &hspi1, (uint8_t*)&byte, 1, 100 );
//    while( HAL_SPI_GetState( &hspi1 ) != HAL_SPI_STATE_READY ) {
//      //cnt++;
//    }
    //cnt = 0;
    cs1();
    cs0();
    cs1();
    enableR1();
    
  }
  else if( currentR == 2 ) {
    //disableR1();
    uint8_t byte = r[2];//numToLcd[i];//~(((data[i]<<1) | (1 << 7)));
    //seg[i] = byte;
    cs0();
    HAL_SPI_Transmit( &hspi1, (uint8_t*)&byte, 1, 100);
//    while( HAL_SPI_GetState( &hspi1 ) != HAL_SPI_STATE_READY ) {
//      //cnt++;
//    }
    //cnt = 0;
    cs1();
    cs0();
    cs1();
    enableR2();
  }
  else if( currentR == 3 ) {
    if( *state == SHOW_TEMP  ) valueType = CHAR_TEMPERATURE;
    else if( *state == SHOW_SPEED  ) valueType = CHAR_SPEED;
    else if( *state == EDIT_TIME ) valueType = CHAR_TIME;
    uint8_t byte = valueType;
    cs0();
    HAL_SPI_Transmit( &hspi1, (uint8_t*)&byte, 1, 100);
//    while( HAL_SPI_GetState( &hspi1 ) != HAL_SPI_STATE_READY ) {
//      //cnt++;
//    }
    //cnt = 0;
    cs1();
    cs0();
    cs1();
    enableR3();
  }
  currentR++;
  if( currentR == 4 ) currentR = 0;
  
}


void buttonOneHandler( void )
{
}
void buttonTwoHandler( void )
{
}
void buttonGoHandler( uint32_t *result )
{
  if( buttonGo.ticksBeforeCheck == 0 ) {
    if( buttonGo.handled == 1 ) {
      *result = 0; 
      return;
    }
    if( buttonGo.state == RELEASED ) {
      if( deviceCurrentState.state == STATE_WAITING ) {
        deviceCurrentState.state = STATE_WORKING;
      }
      else {
        deviceCurrentState.state = STATE_WAITING;
      }
    }
    //buttonGo.ticksBeforeCheck = 1000;
    buttonGo.handled = 1;
    *result = 1;
  }
  if( buttonGo.ticksBeforeCheck > 0 ) buttonGo.ticksBeforeCheck--;
  return;
}

void buttonEditHandler( uint32_t *result )
{
  
  if( buttonEdit.handled == 1 ) {
    *result = 0; 
    return;
  }
  switch( deviceMenuState.menuState )
  {
  case SHOW_TEMP: deviceMenuState.menuState = SHOW_SPEED; break;
  case SHOW_SPEED: deviceMenuState.menuState = EDIT_TIME; break;
  case EDIT_TIME: deviceMenuState.menuState = SHOW_TEMP; break;
  }
  buttonEdit.handled = 1;
  *result = 1;
  return;
}
void buttonLeftHandler( uint32_t *result )
{
  if( buttonLeft.ticksBeforeCheck == 0 ) {
    if( buttonLeft.handled == 1 ) {
      *result = 0; 
      return;
    }
    switch( deviceMenuState.menuState )
    {
    case SHOW_TEMP:
      if( HAL_GetTick() - timePressed[2] > LONG_BUTON_MIN_TIME && buttonLeft.state == PRESSED ) {
        deviceCurrentState.workSetting.targetTemp -= 5;
        buttonLeft.ticksBeforeCheck = 75;
      }
      else if (buttonLeft.state == RELEASED) {
        deviceCurrentState.workSetting.targetTemp--;
        buttonLeft.handled = 1;
        buttonRight.ticksBeforeCheck = 2;
      }
      if( deviceCurrentState.workSetting.targetTemp <= 0 ) deviceCurrentState.temperature = 0;
      break;
    case SHOW_SPEED:
      if(  HAL_GetTick() - timePressed[2] > LONG_BUTON_MIN_TIME && buttonLeft.state == PRESSED ) {
        deviceCurrentState.workSetting.targetSpeed -= 5;
        buttonLeft.ticksBeforeCheck = 75;
      }
      else if ( buttonLeft.state == RELEASED) {
        deviceCurrentState.workSetting.targetSpeed--;
        buttonLeft.handled = 1;
        buttonRight.ticksBeforeCheck = 2;
      }
      if( deviceCurrentState.workSetting.targetSpeed <= 0 ) deviceCurrentState.motorPwm = 0;
      break;
    }
  }
  if( buttonLeft.ticksBeforeCheck > 0 ) buttonLeft.ticksBeforeCheck--;
  return;
}
void buttonRightHandler( uint32_t *result )
{
  if (buttonRight.ticksBeforeCheck == 0)
  {
    if (buttonRight.handled == 1)
    {
      *result = 0;
      return;
    }
    switch (deviceMenuState.menuState)
    {
    case SHOW_TEMP:
      if (HAL_GetTick() - timePressed[3] > LONG_BUTON_MIN_TIME && buttonRight.state == PRESSED)
      {
        deviceCurrentState.workSetting.targetTemp += 5;
        buttonRight.ticksBeforeCheck = 75;
      }
      else if (buttonRight.state == RELEASED)
      {
        deviceCurrentState.workSetting.targetTemp++;
        buttonRight.handled = 1;
        buttonRight.ticksBeforeCheck = 2;
      }
      break;
    case SHOW_SPEED:
      if (HAL_GetTick() - timePressed[3] > LONG_BUTON_MIN_TIME && buttonRight.state == PRESSED)
      {
        deviceCurrentState.workSetting.targetSpeed += 5;
        buttonRight.ticksBeforeCheck = 75;
      }
      else if (buttonRight.state == RELEASED)
      {
        deviceCurrentState.workSetting.targetSpeed++;
        buttonRight.handled = 1;
        buttonRight.ticksBeforeCheck = 2;
      }
      break;
    }
  }
  if( buttonRight.ticksBeforeCheck > 0 ) buttonRight.ticksBeforeCheck--;
  return;
}

uint32_t time = 0;
int timeFromLastButton( void ) {
  
  uint32_t max = 0;
  uint32_t min = 0x0FFFFFFFF;
  
  uint32_t curTime = HAL_GetTick();
  for( int i = 0; i < 4; i++ ) {
    uint32_t dt = curTime - timePressed[i];
    if( dt > max ) max = dt;
    if( dt < min ) min = dt;
  }
  for( int i = 0; i < 4; i++ ) {
    uint32_t dt = curTime - timeReleased[i];
    if( dt > max ) max = dt;
    if( dt < min ) min = dt;
  }
time = min;
  return min;
}
void showMenu()
{
  static int8_t updateValuePeriod = 2;
  
  if( updateValuePeriod == 0 ) {
    switch( deviceMenuState.menuState ) 
    {
    case SHOW_TEMP:
     showValue( &deviceCurrentState.temperature, &deviceMenuState.menuState );
//      if( timeFromLastButton() > 1600 && deviceMenuState.flagEdit != 1 ) {
//        showValue( &deviceCurrentState.temperature, &deviceMenuState.menuState );
//      }
//      else {
//        showValue( &deviceCurrentState.workSetting.targetTemp, &deviceMenuState.menuState );
//      }
      break;
      
    case SHOW_SPEED:   
       if( timeFromLastButton() > 1600 && deviceMenuState.flagEdit != 1 ) {
        showValue( &deviceCurrentState.motorPwm, &deviceMenuState.menuState );
      }
      else {
        showValue( &deviceCurrentState.workSetting.targetSpeed, &deviceMenuState.menuState );
      }
      break;
      
    case EDIT_TIME:
      
      break;
      

    }
    updateValuePeriod = 2;
  }
  
  updateValuePeriod--;
}

void indicatorTaskFunc( const void *argument )
{
  uint8_t data[7] = { sega, segb, segc, segd, sege, segf, segg };
  
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_14,GPIO_PIN_SET);
  cs0();
  
  osDelay(1);
  cs1();
  osDelay(1);
  cs0();
  osDelay(2);
  cs1();

  uint16_t cnt = 0;
  uint16_t sleepCnt = 0;
  
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  uint32_t result;

  while(1) {
    
    showMenu();
    buttonEditHandler( &result );
    buttonGoHandler( &result );
    buttonLeftHandler( &result );
    buttonRightHandler( &result );
    
    
    osDelay(1);
  }
}

int cnt[4] = {0};

#pragma optimize = none
void irqDelay( int delayCounter ) {
  for( int i = 0; i < delayCounter; i++ ) {
     __NOP();
  }
}

#pragma optimize = none
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
  irqDelay( 1000 );

  switch(GPIO_Pin)
  {
  case BUTTON_ONE_PIN: {
    if( HAL_GPIO_ReadPin( BUTTON_PORT, BUTTON_ONE_PIN ) == GPIO_PIN_RESET ) {
      timePressed[4] = HAL_GetTick();
      buttonOne.state = PRESSED;
      buttonOne.handled = 0;
    }
    else {
      timeReleased[4] = HAL_GetTick();
      buttonOne.timePressed = timeReleased[4] - timePressed[4];
      buttonOne.state = RELEASED;
      buttonOne.handled = 0;
    }
    }
    break;
  
  case BUTTON_TWO_PIN: {
    if( HAL_GPIO_ReadPin( BUTTON_PORT, BUTTON_TWO_PIN ) == GPIO_PIN_RESET ) {
      timePressed[5] = HAL_GetTick();
      buttonTwo.state = PRESSED;
      buttonTwo.handled = 0;
    }
    else {
      timeReleased[5] = HAL_GetTick();
      buttonTwo.timePressed = timeReleased[5] - timePressed[5];
      buttonTwo.state = RELEASED;
      buttonTwo.handled = 0;
    }
    }
    break;
  
  case BUTTON_EDIT_PIN:
    if( HAL_GPIO_ReadPin( BUTTON_PORT, BUTTON_EDIT_PIN ) == GPIO_PIN_RESET ) {
      timePressed[0] = HAL_GetTick();
      buttonEdit.state = PRESSED;
      buttonEdit.handled = 0;
    }
    else {
      timeReleased[0] = HAL_GetTick();
      buttonEdit.timePressed = timeReleased[0] - timePressed[0];
      buttonEdit.state = RELEASED;
      buttonEdit.handled = 0;
      
    }
    break;
    
  case BUTTON_GO_PIN:
    if( HAL_GPIO_ReadPin( BUTTON_PORT, BUTTON_GO_PIN ) == GPIO_PIN_RESET ) {
      timePressed[1] = HAL_GetTick();
      buttonGo.state = PRESSED;
      buttonGo.handled = 0;
    }
    else {
      timeReleased[1] = HAL_GetTick();
      buttonGo.timePressed = timeReleased[1] - timePressed[1];
      buttonGo.state = RELEASED;
      buttonGo.handled = 0;
    }
    break;
    
  case BUTTON_LEFT_PIN:
    if( HAL_GPIO_ReadPin( BUTTON_PORT, BUTTON_LEFT_PIN ) == GPIO_PIN_RESET ) {
      timePressed[2] = HAL_GetTick();
      buttonLeft.state = PRESSED;
      buttonLeft.handled = 0;
    }
    else {
      timeReleased[2] = HAL_GetTick();
      buttonLeft.timePressed = timeReleased[2] - timePressed[2];
      buttonLeft.state = RELEASED;
      buttonLeft.handled = 0;
    }
    break;
    
  case BUTTON_RIGHT_PIN:
    if( HAL_GPIO_ReadPin( BUTTON_PORT, BUTTON_RIGHT_PIN ) == GPIO_PIN_RESET ) {
      timePressed[3] = HAL_GetTick();
      buttonRight.state = PRESSED;
      buttonRight.handled = 0;
    }
    else {
      timeReleased[3] = HAL_GetTick();
      buttonRight.timePressed = timeReleased[3] - timePressed[3];
      buttonRight.state = RELEASED;
      buttonRight.handled = 0;
    }
    break;
    
  }
  HAL_NVIC_EnableIRQ( EXTI4_15_IRQn );
  
          
}