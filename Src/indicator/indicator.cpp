#include "indicator.h"
#include "motor.h"
#include "math.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "stm32f0xx_hal_spi.h"

#define BUTTON_MIN_TIME                  75
#define LONG_BUTON_MIN_TIME                 500
#define TIME_WAIT_AFTER_CHANGE              1200

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

int8_t updateValuePeriod = 2;
const uint8_t numToLcd[10] = { dig0, dig1, dig2, dig3, dig4, dig5, dig6, dig7, dig8, dig9 };
uint8_t currentR = 0;
uint8_t r[4] = {0};
volatile bool buttonHandled = true;

tButton buttonOne = { RELEASED, BUTTON_ONE_PIN, 1, 0, 2 };
tButton buttonTwo = { RELEASED, BUTTON_TWO_PIN, 1, 0, 2 };
tButton buttonIdle = { RELEASED, BUTTON_IDLE_PIN, 1, 0, 2 };
tButton buttonGo = { RELEASED, BUTTON_GO_PIN, 1, 0, 2 };
tButton buttonLeft = { RELEASED, BUTTON_LEFT_PIN, 1, 0, 2 };
tButton buttonRight = { RELEASED, BUTTON_RIGHT_PIN, 1, 0, 2 };


tButton *bList[6] = {
  &buttonGo,
  &buttonIdle,
  &buttonLeft,
  &buttonRight,
  &buttonOne,
  &buttonTwo
};
  
tDeviceCurrentState  dcs;


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


void showValue( int showState, int *value, tMenuState *state  ) {
  
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


void buttonOneHandler( tDeviceCurrentState *cs )
{
  if( buttonOne.ticksBeforeCheck == 0 && buttonOne.handled == 0 ) {
    switch( buttonOne.state )
    {
    case PRESSED:
      if( ( HAL_GetTick() - buttonOne.timePressed ) > LONG_BUTON_MIN_TIME * 2 ) {
        cs->menuState = EDIT_TIME;
        //cs->editTimeExpired = TIME_WAIT_AFTER_CHANGE;
      }
      buttonOne.ticksBeforeCheck = 125;
      //buttonOne.handled = 1;
      break;
      
    case RELEASED:
      if( buttonOne.pressToReleaseTime > BUTTON_MIN_TIME &&
         buttonOne.pressToReleaseTime < LONG_BUTON_MIN_TIME * 2 ) {
        if( cs->state == STATE_STOPPED  ) {
          cs->timeMode = ( cs->timeMode == TIME_MODE_ON  ) ? TIME_MODE_OFF : TIME_MODE_ON;       
        }
      }
      cs->menuState = SHOW_TEMP;
      buttonOne.ticksBeforeCheck = 75;
      buttonOne.handled = 1;
      break;
    }
  }
   if( buttonOne.ticksBeforeCheck > 0 ) buttonOne.ticksBeforeCheck--;
}
void buttonTwoHandler( tDeviceCurrentState *cs )
{
  
  //  if( buttonIdle.handled == 0 ) {
//    switch( deviceMenuState.menuState )
//    {
//    case SHOW_TEMP: deviceMenuState.menuState = SHOW_SPEED; break;
//    case SHOW_SPEED: deviceMenuState.menuState = EDIT_TIME; break;
//    case EDIT_TIME: deviceMenuState.menuState = SHOW_TEMP; break;
//    }
//    buttonIdle.handled = 1;
//  }
//  return;
  
//  if( buttonTwo.ticksBeforeCheck == 0  && buttonTwo.handled == 0) {
//    switch( buttonTwo.state )
//    {
//    case PRESSED:
//      cs->state = STATE_IDLE;
//      buttonTwo.handled = 1;
//      break;
//      
//    case RELEASED:
////      if( buttonTwo.timePressed < LONG_BUTON_MIN_TIME ) {
////        if( cs->state != STATE_IDLE ) cs->state = STATE_IDLE;
////      }
//      cs->state = STATE_STOPPED;
//      buttonTwo.handled = 1;
//      break;
//    }
//  }
//   if( buttonTwo.ticksBeforeCheck > 0 ) buttonTwo.ticksBeforeCheck--;
}
void buttonGoHandler( tDeviceCurrentState *cs )
{
  if( buttonGo.ticksBeforeCheck == 0 && buttonGo.handled == 0 ) {
    if( buttonGo.state == RELEASED && buttonGo.pressToReleaseTime > BUTTON_MIN_TIME ) {
      if( cs->state == STATE_STOPPED ) {
        cs->state = STATE_WAIT_MIN_TEMP;
        buttonLeft.ticksBeforeCheck = 20;
      }
      else {
        cs->state = STATE_DECELERATION;
        buttonLeft.ticksBeforeCheck = 2;
      }
    }
    buttonGo.handled = 1;
  }
  if( buttonGo.ticksBeforeCheck > 0 ) buttonGo.ticksBeforeCheck--;
  return;
}

void buttonIdleHandler( tDeviceCurrentState *cs )
{ 
   if( buttonIdle.ticksBeforeCheck == 0 && buttonIdle.handled == 0 ) {
     if( cs->state == STATE_STOPPED ) {
       switch( buttonIdle.state )
       {
       case PRESSED:
         cs->state = STATE_IDLE;
         buttonIdle.ticksBeforeCheck = 125;
         buttonIdle.handled = 1;
         break;
         
       case RELEASED:
         cs->state = STATE_STOPPED;
         buttonIdle.ticksBeforeCheck = 125;
         buttonIdle.handled = 1;
         break;
       }
     }
  }
  if( buttonIdle.ticksBeforeCheck > 0 ) buttonIdle.ticksBeforeCheck--;

}
void buttonLeftHandler( tDeviceCurrentState *cs )
{
  if( buttonLeft.ticksBeforeCheck == 0 && buttonLeft.handled == 0 ) {
  

    switch( cs->menuState )
    {
    case SHOW_TEMP: { 
      
      if( HAL_GetTick() - buttonLeft.timePressed > LONG_BUTON_MIN_TIME && buttonLeft.state == PRESSED ) {
        cs->workSetting.targetTemp -= 3;
        buttonLeft.ticksBeforeCheck = 125;
      }
      else if (buttonLeft.state == RELEASED) {
        cs->workSetting.targetTemp--;
        buttonLeft.handled = 1;
        buttonLeft.ticksBeforeCheck = 2;
      }
      if( cs->workSetting.targetTemp <= 0 ) cs->temperature = 0;
    }
    break;
    
    case SHOW_SPEED: {
      if(  HAL_GetTick() - buttonLeft.timePressed > LONG_BUTON_MIN_TIME && buttonLeft.state == PRESSED ) {
        cs->workSetting.targetSpeed -= 3;
        buttonLeft.ticksBeforeCheck = 125;
      }
      else if ( buttonLeft.state == RELEASED) {
        cs->workSetting.targetSpeed--;
        buttonLeft.handled = 1;
        buttonLeft.ticksBeforeCheck = 2;
      }
      if( cs->workSetting.targetSpeed <= 0 ) cs->motorPwm = 0;   /// wtf!!!!!!!!!!!!!!!!!
    }
    break;
      
    case EDIT_TIME: {
      if(  HAL_GetTick() - buttonLeft.timePressed > LONG_BUTON_MIN_TIME && buttonLeft.state == PRESSED ) {
        cs->timeModeOneCycleDuration -= 5;
        buttonLeft.ticksBeforeCheck = 125;
      }
      else if ( buttonLeft.state == RELEASED) {
        cs->timeModeOneCycleDuration--;
        buttonLeft.handled = 1;
        buttonLeft.ticksBeforeCheck = 2;
      }
      //cs->editTimeExpired += 1200;
      if( cs->timeModeOneCycleDuration <= 0 ) dcs.timeModeOneCycleDuration = 0;
    }
    break;
    }
  }
  if( buttonLeft.ticksBeforeCheck > 0 ) buttonLeft.ticksBeforeCheck--;
}

void buttonRightHandler( tDeviceCurrentState *cs )
{
  if (buttonRight.ticksBeforeCheck == 0 && buttonRight.handled == 0)
  {
    switch( cs->menuState )
    {
    case SHOW_TEMP:
      if (HAL_GetTick() - buttonRight.timePressed > LONG_BUTON_MIN_TIME && buttonRight.state == PRESSED)
      {
        cs->workSetting.targetTemp += 3;
        buttonRight.ticksBeforeCheck = 125;
      }
      else if (buttonRight.state == RELEASED)
      {
        cs->workSetting.targetTemp++;
        buttonRight.handled = 1;
        buttonRight.ticksBeforeCheck = 2;
      }
      break;
    case SHOW_SPEED:
      if (HAL_GetTick() - buttonRight.timePressed > LONG_BUTON_MIN_TIME && buttonRight.state == PRESSED)
      {
        cs->workSetting.targetSpeed += 3;
        buttonRight.ticksBeforeCheck = 125;
      }
      else if (buttonRight.state == RELEASED)
      {
        cs->workSetting.targetSpeed++;
        buttonRight.handled = 1;
        buttonRight.ticksBeforeCheck = 2;
      }
      break;
      
    case EDIT_TIME: {
      if (HAL_GetTick() - buttonRight.timePressed > LONG_BUTON_MIN_TIME && buttonRight.state == PRESSED)
      {
        cs->timeModeOneCycleDuration += 5;
        buttonRight.ticksBeforeCheck = 125;
      }
      else if (buttonRight.state == RELEASED)
      {
        cs->timeModeOneCycleDuration++;
        buttonRight.handled = 1;
        buttonRight.ticksBeforeCheck = 2;
      }
      //cs->editTimeExpired += 1200;
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
  for( int i = 0; i < 6; i++ ) {
    uint32_t dt = curTime - bList[i]->timePressed;
    if( dt > max ) max = dt;
    if( dt < min ) min = dt;
  }
  for( int i = 0; i < 6; i++ ) {
    uint32_t dt = curTime - bList[i]->timeReleased;
    if( dt > max ) max = dt;
    if( dt < min ) min = dt;
  }
  time = min;
  return min;
}

void showMenu( tDeviceCurrentState *cs )
{
  
  if( updateValuePeriod == 0 ) {
    switch( cs->menuState ) 
    {
    case SHOW_SPEED: {
      int pwm = (int)((float)dcs.workSetting.targetSpeed / 100.0 + 0.5f);
      showValue( 1, &pwm, &cs->menuState );
    }
    break;
    
    case SHOW_TEMP: {
      if( timeFromLastButton() > TIME_WAIT_AFTER_CHANGE && buttonLeft.state == RELEASED && buttonRight.state == RELEASED ) {
        showValue(1,  &cs->temperature, &cs->menuState );
      }
      else {
        showValue( 1, &cs->workSetting.targetTemp, &cs->menuState );
      }
    }
    break;
    
    case EDIT_TIME: {
        showValue( 1, &cs->timeModeOneCycleDuration, &cs->menuState );
    }
    break;
    }
    updateValuePeriod = 2;
  }
  updateValuePeriod--;
}

void indicatorTaskFunc( const void *argument )
{
  //uint8_t data[7] = { sega, segb, segc, segd, sege, segf, segg };
  osDelay(1000);
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

  
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  

  while(1) {
    
    showMenu( &dcs );
    
    buttonGoHandler( &dcs);
    buttonIdleHandler( &dcs );
    buttonLeftHandler( &dcs );
    buttonRightHandler( &dcs );
    buttonOneHandler( &dcs );
    buttonTwoHandler( &dcs );
    
   
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
  case BUTTON_GO_PIN:
    if( HAL_GPIO_ReadPin( BUTTON_PORT, BUTTON_GO_PIN ) == GPIO_PIN_RESET ) {
      buttonGo.timePressed = HAL_GetTick();
      buttonGo.state = PRESSED;
    }
    else {
      buttonGo.timeReleased = HAL_GetTick();
      buttonGo.pressToReleaseTime = buttonGo.timeReleased - buttonGo.timePressed;
      buttonGo.state = RELEASED;
    }
    buttonGo.handled = 0;
    break;
    
  case BUTTON_IDLE_PIN:
    if( HAL_GPIO_ReadPin( BUTTON_PORT, BUTTON_IDLE_PIN ) == GPIO_PIN_RESET ) {
      buttonIdle.timePressed = HAL_GetTick();
      buttonIdle.state = PRESSED;
      buttonIdle.handled = 0;
    }
    else {
      buttonIdle.timeReleased = HAL_GetTick();
      buttonIdle.pressToReleaseTime =  buttonIdle.timeReleased - buttonIdle.timePressed;
      buttonIdle.state = RELEASED;
      buttonIdle.handled = 0;
      
    }
    break;
    
  case BUTTON_LEFT_PIN:
    if( HAL_GPIO_ReadPin( BUTTON_PORT, BUTTON_LEFT_PIN ) == GPIO_PIN_RESET ) {
      buttonLeft.timePressed = HAL_GetTick();
      buttonLeft.state = PRESSED;
    }
    else {
      buttonLeft.timeReleased = HAL_GetTick();
      buttonLeft.pressToReleaseTime = buttonLeft.timeReleased - buttonLeft.timePressed;
      buttonLeft.state = RELEASED;
    }
    buttonLeft.handled = 0;
    break;
    
  case BUTTON_RIGHT_PIN:
    if( HAL_GPIO_ReadPin( BUTTON_PORT, BUTTON_RIGHT_PIN ) == GPIO_PIN_RESET ) {
      buttonRight.timePressed= HAL_GetTick();
      buttonRight.state = PRESSED;
    }
    else {
      buttonRight.timeReleased = HAL_GetTick();
      buttonRight.pressToReleaseTime = buttonRight.timeReleased - buttonRight.timePressed;
      buttonRight.state = RELEASED;
    }
    buttonRight.handled = 0;
    break;
    
  case BUTTON_ONE_PIN: {
    if( HAL_GPIO_ReadPin( BUTTON_PORT, BUTTON_ONE_PIN ) == GPIO_PIN_RESET ) {
      buttonOne.timePressed = HAL_GetTick();
      buttonOne.state = PRESSED;
    }
    else {
      buttonOne.timeReleased = HAL_GetTick();
      buttonOne.pressToReleaseTime = buttonOne.timeReleased - buttonOne.timePressed;
      buttonOne.state = RELEASED;
    }
    buttonOne.handled = 0;
  }
  break;
  
  case BUTTON_TWO_PIN: {
    if( HAL_GPIO_ReadPin( BUTTON_PORT, BUTTON_TWO_PIN ) == GPIO_PIN_RESET ) {
      buttonTwo.timePressed = HAL_GetTick();
      buttonTwo.state = PRESSED;
    }
    else {
      buttonTwo.timeReleased = HAL_GetTick();
      buttonTwo.pressToReleaseTime = buttonTwo.timeReleased - buttonTwo.timePressed;
      buttonTwo.state = RELEASED;
    }
     buttonTwo.handled = 0;
    }
    break;
    
  }
  HAL_NVIC_EnableIRQ( EXTI4_15_IRQn );
  
          
}