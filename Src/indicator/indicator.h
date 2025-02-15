#ifndef INDICATOR_H
#define INDICATOR_H

#include "stdint.h"
#include "eeprom.h"

//#define sega ((uint8_t)0x40)
//#define segb ((uint8_t)0x20)
//#define segc ((uint8_t)0x10)
//#define segd ((uint8_t)0x08)
//#define sege ((uint8_t)0x04)
//#define segf ((uint8_t)0x02)
//#define segg ((uint8_t)0x01)
#define BUTTON_ONE_PIN       GPIO_PIN_10
#define BUTTON_TWO_PIN       GPIO_PIN_11
#define BUTTON_RIGHT_PIN     GPIO_PIN_12
#define BUTTON_LEFT_PIN      GPIO_PIN_13
#define BUTTON_IDLE_PIN      GPIO_PIN_14
#define BUTTON_GO_PIN        GPIO_PIN_15

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
#define CHAR_TIME           ((uint8_t)( sega & segc & segd & segg & segf ))




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
  uint16_t ticksBeforeCheck;
  uint32_t pressToReleaseTime;
  uint32_t timePressed;
  uint32_t timeReleased;
} tButton;

typedef enum {
  SHOW_TEMP = 0,
  SHOW_SPEED,
  EDIT_TIME,
  EDIT_SPEED,
} tMenuState;

typedef enum {
  TIME_MODE_OFF = 0,
  TIME_MODE_ON,
} tTimeMode;

typedef enum {
  STATE_STOPPED = 0,
  STATE_WAIT_MIN_TEMP,
  STATE_ACCELERATION,
  STATE_EDIT_SETTINGS,
  STATE_WORKING,
  STATE_DECELERATION,
  STATE_IDLE,
} tDeviceState;

typedef struct {
  tDeviceState state;
  tWorkSettings workSetting;
  
  uint8_t needStart;
  uint8_t needStop;
  uint8_t minTempAchieved;
  uint8_t needSave;
  
  float acceleration;
  float accelerationDist;
  float decelerationDist;
  
  int temperature;
  float heaterPwm;
  float motorPwm;
  float blowerPwm;
  uint8_t heaterEnabled;
  uint8_t motorEnabled;
  uint8_t blowerEnabled;
  tTimeMode timeMode;
  int timeModeOneCycleDuration;
  float motorControlVoltage;
  tMenuState menuState;
  int editTimeExpired;
  bool flagEdit;
} tDeviceCurrentState;


extern void indicatorTaskFunc( const void *argument );

void buttonGoHandler( tDeviceCurrentState *cs );
void buttonEditHandler( tDeviceCurrentState *cs );
void buttonLeftHandler( tDeviceCurrentState *cs );
void buttonRightHandler( tDeviceCurrentState *cs );


#endif