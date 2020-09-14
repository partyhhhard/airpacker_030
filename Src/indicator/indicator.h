#ifndef INDICATOR_H
#define INDICATOR_H

#include "stdint.h"

//#define sega ((uint8_t)0x40)
//#define segb ((uint8_t)0x20)
//#define segc ((uint8_t)0x10)
//#define segd ((uint8_t)0x08)
//#define sege ((uint8_t)0x04)
//#define segf ((uint8_t)0x02)
//#define segg ((uint8_t)0x01)

#define BUTTON_RIGHT_PIN     GPIO_PIN_12
#define BUTTON_LEFT_PIN      GPIO_PIN_13
#define BUTTON_GO_PIN        GPIO_PIN_14
#define BUTTON_EDIT_PIN      GPIO_PIN_15

#define BUTTON_PORT      GPIOB


#define sega    ((uint8_t)0x7F)
#define segb    ((uint8_t)0x3F)
#define segc    ((uint8_t)0x5F)
#define segd    ((uint8_t)0x6F)
#define sege    ((uint8_t)0x77)
#define segf    ((uint8_t)0x7B)
#define segg    ((uint8_t)0x7D)

//#define dig0    ((uint8_t)sega&segb&segc&segd&sege&segf)
//#define dig1    ((uint8_t)segb&segc)
//#define dig2    ((uint8_t)sega&segb&segd&sege&segg)
//#define dig3    ((uint8_t)sega&segb&segc&segd&segg)
//#define dig4    ((uint8_t)segf&segg&segb&segc)
//#define dig5    ((uint8_t)sega&segf&segg&segc&segd)
//#define dig6    ((uint8_t)sega&segf&sege&segg&segd&segc)
//#define dig7    ((uint8_t)sega&segb&segc&sega)
//#define dig8    ((uint8_t)sega&segb&segc&segd&sege&segf&segg)
//#define dig9    ((uint8_t)sega&segb&segc&segd&segf&segg)

#define dig0    ((uint8_t)(segf & sege & segd & segc & segb & sega))
#define dig1    ((uint8_t)(segc & segb | ~sega))
#define dig2    ((uint8_t)( segg & sege & segd & segb & sega ))
#define dig3    ((uint8_t)(sega & segb & segc & segd & segg))
#define dig4    ((uint8_t)(segf&segg&segb&segc | ~sega ))
#define dig5    ((uint8_t)(sega&segf&segg&segc&segd) )
#define dig6    ((uint8_t)(sega&segf&sege&segg&segd&segc) )
#define dig7    ((uint8_t)(sega&segb&segc&sega))
#define dig8    ((uint8_t)(sega&segb&segc&segd&sege&segf&segg) )
#define dig9    ((uint8_t)(sega&segb&segc&segd&segf&segg))

#define CHAR_TEMPERATURE  ((uint8_t)( sega & segb & segf & segg ))
#define CHAR_SPEED          ((uint8_t)( segc & segd & sege | ~sega ))
#define CHAR_TIME           ((uint8_t)( sega & segb & segf & segg & sege ))




#define cs1()    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET)
#define cs0()     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET)


typedef enum {
  PRESSED = 0,
  RELEASED,
  LONG,
} tButtonState;

typedef struct {
  tButtonState state;
  uint16_t pin;
  uint8_t handled;
  uint32_t timePressed;
  uint16_t ticksBeforeCheck;
} tButton;


extern void indicatorTaskFunc( const void *argument );

void buttonGoHandler( );

void buttonEditHandler( );

void buttonLeftHandler( );

void buttonRightHandler( );

typedef struct {
  int targetTemp;
  int targetSpeed;
  int targetBlower;
  int time;
} tWorkSetting;

typedef enum {
  STATE_WAITING = 0,
  STATE_EDIT_SETTINGS,
  STATE_WORKING,
} tDeviceState;

typedef struct {
  tDeviceState state;
  tWorkSetting workSetting;
  int temperature;
  uint16_t heaterPwm;
  int motorPwm;
  int blowerPwm;
  bool heaterEnabled;
  bool motorEnabled;
  bool blowerEnabled;
  float motorControlVoltage;
  uint8_t minTempAchieved;
  uint32_t stateChangedTime;
  
} tDeviceCurrentState;

typedef enum {
  SHOW_TEMP = 0,
  SHOW_SPEED,
  EDIT_TIME,
} tMenuState;

typedef struct {
  tMenuState menuState;
  bool flagEdit;
} tDeviceMenuState;

#endif