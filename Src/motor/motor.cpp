
#include "temp_sensor.h"
#include "string.h"
#include "motor.h"
#include "math.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "indicator.h"

#define RESET_TEMPERATURE       135

extern IWDG_HandleTypeDef hiwdg;
extern TIM_HandleTypeDef htim1;
#define BLOWER_CHANNEL   TIM_CHANNEL_4
#define MOTOR_CHANNEL    TIM_CHANNEL_1

extern void debug_( char *msg, float value = 0 );
extern tDeviceCurrentState  dcs;

extern float CurTemp;

float calcAcceleration( float startSpd, float endSpd, float distance )
{
  return (endSpd - startSpd) / 2 * distance;
}

//float calculateVelocity( 

void setMotorPwm( int value ) {

  //if( dcs.motorPwm == value ) return;
  if( value > TIM_DEFAULT_PERIOD ) {
    value = TIM_DEFAULT_PERIOD;
  }
  if( value < 0 ) value = 0;
  if( value > 1 ) dcs.motorEnabled = 1;
  uint16_t ccr = value;//TIM_DEFAULT_PERIOD / 101 * value;
  __HAL_TIM_SET_COMPARE( &htim1, MOTOR_CHANNEL, ccr );
  //dcs.motorPwm = ccr;
}
void disableMotor( void ) {
  if( dcs.motorEnabled ) {
    
    dcs.motorPwm = 0;
    setMotorPwm( 0 );
    dcs.motorEnabled = 0;
  }
 
}


void setBlowerPwm( int value ) {

    if(value > ( TIM_DEFAULT_PERIOD ) ) value = TIM_DEFAULT_PERIOD;
  if( value > 0 ) dcs.blowerEnabled = 1;
  __HAL_TIM_SET_COMPARE( &htim1, BLOWER_CHANNEL, value );
  //dcs.blowerCCR = value;
  return;
}
void disableBlower( void ) {
  //if( dcs.blowerEnabled ) {
    //dcs.blowerEnabled = 0;
    //dcs.blowerCCR = 0;
    __HAL_TIM_SET_COMPARE( &htim1, BLOWER_CHANNEL, 0 );//setBlowerPwm( 0 );
  //}
 
}
void disableAll( void ) {
  disableMotor();
  disableBlower();
}

bool isEqual( float par1, float par2 )
{
  const float eps = 0.001;
  if( fabs( par1 - par2 ) < eps ) return true;
  return false;
}

uint32_t curTime = 0;
uint32_t startTime = 0;

void fixTime( void )
{
  startTime = HAL_GetTick();
}
uint32_t timeFromFix( void )
{
  return ( HAL_GetTick() - startTime );
}

/**
  * @brief  Function implementing the motor and blower control thread.
  * @param  argument: Not used 
  * @retval None
  */
#pragma optimize = none
void deviceControlTaskFunc( void const *argument ) 
{
  
  osDelay(2000);
  
  uint16_t iwdgReloadPeriod = 3000;
  uint8_t taskPeriod = 1;
  
  
  HAL_TIM_PWM_Start( &htim1, MOTOR_CHANNEL );
  HAL_TIM_PWM_Start( &htim1, BLOWER_CHANNEL ); 
  
  disableAll();
  
  
  
  uint16_t blowerStartDelayValue = 750;    
  
  float minStartSpeed = (float)TIM_ONE_PERCENT * 15.0;
  int blowerStartDelay = blowerStartDelayValue;

  startTime = 0;
  
  dcs.timeMode = TIME_MODE_OFF;
 
  
  while( 1 ) {
    
    if( dcs.needStart ) {
      dcs.needStart = 0;
      acceleration = calcAcceleration( minStartSpeed, dcs.workSetting.targetMotorPwm, accDistance );
      dcs.state = STATE_WAIT_MIN_TEMP;
    }
    if( dcs.needStop ) {
      dcs.needStop = 0;
      acceleration = calcAcceleration( minStartSpeed, dcs.motorPwm, accDistance );
      dcs.state = STATE_DECELERATION;
      fixTime();
    }
    
    switch( dcs.state )
    {
    case STATE_WAIT_MIN_TEMP: {
      if( dcs.minTempAchieved ) {
        dcs.state = STATE_ACCELERATION;
        dcs.motorPwm = minStartSpeed;
        setMotorPwm( (int)( dcs.motorPwm + 0.5f ) );
        fixTime();
      }
      else {
        // do nothing ^)
      }
    }
    break;
    
    case STATE_ACCELERATION: {
      if( dcs.motorPwm < dcs.workSetting.targetMotorPwm ) {
        dcs.motorPwm = dcs.motorPwm + acceleration * timeFromFix() / 1000;
        setMotorPwm( (int)(dcs.motorPwm + 0.5f ) );
      }
      else {
        dcs.motorPwm = dcs.workSetting.targetMotorPwm;
        setMotorPwm( (int)( dcs.motorPwm + .5f ) );
        dcs.state = STATE_WORKING;
      }
    }
    break;
    
    case STATE_WORKING: {
      if( isEqual( dcs.motorPwm, dcs.workSetting.targetMotorPwm ) == false ) {
        dcs.motorPwm = dcs.workSetting.targetMotorPwm;
        setMotorPwm( (int)( dcs.motorPwm + 0.5f ) );
      }
      else {
        // do nothing
      }
    } break;
    
    case STATE_DECELERATION: {
      if( dcs.motorPwm > minStartSpeed ) {
        dcs.motorPwm = dcs.motorPwm - acceleration * timeFromFix() / 1000;
        setMotorPwm( (int)( dcs.motorPwm + 0.5f ) );
      }
      else {
        dcs.state = STATE_STOPPED;
        dcs.motorPwm = 0;
        dcs.blowerPwm = 0;
        disableAll();
        dcs.needSave = 1;
      }
    } break;
    
    case STATE_STOPPED: {
       // do nothing
    } break;
      
    case STATE_IDLE: {
      setMotorPwm( TIM_DEFAULT_PERIOD / 100 * 30 );
    } break;
    }
    
    if( iwdgReloadPeriod <= 0 ) {
      __HAL_IWDG_RELOAD_COUNTER( &hiwdg );
      iwdgReloadPeriod = 3000;
    }
    
    osDelay(taskPeriod);
    iwdgReloadPeriod -= taskPeriod;
  }
}