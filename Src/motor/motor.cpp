
#include "temp_sensor.h"
#include "string.h"
#include "motor.h"
#include "math.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "indicator.h"

#define RESET_TEMPERATURE       125

extern IWDG_HandleTypeDef hiwdg;
extern TIM_HandleTypeDef htim1;
#define BLOWER_CHANNEL   TIM_CHANNEL_4
#define MOTOR_CHANNEL    TIM_CHANNEL_1


extern tDeviceMenuState deviceMenuState;
extern tDeviceCurrentState  deviceCurrentState;

extern float CurTemp;
int currentSpeedPercent;
tMotorAndBlowerSettings speedSettings;


void setMotorPwm( int value ) {

  //if( deviceCurrentState.motorPwm == value ) return;
  if( value > (TIM_DEFAULT_PERIOD - (TIM_DEFAULT_PERIOD / 100 * 2) ) ) {
    value = TIM_DEFAULT_PERIOD - (TIM_DEFAULT_PERIOD / 100 * 2 );
  }
  if( value < 0 ) value = 0;
  if( value > 1 ) deviceCurrentState.motorEnabled = 1;
  uint16_t ccr = value;//TIM_DEFAULT_PERIOD / 101 * value;
  __HAL_TIM_SET_COMPARE( &htim1, MOTOR_CHANNEL, ccr );
  deviceCurrentState.motorCCR = ccr;
}
void disableMotor( void ) {
  //if( deviceCurrentState.motorEnabled ) {
    deviceCurrentState.motorEnabled = 0;
    setMotorPwm( 0 );
  //}
 
}


void setBlowerPwm( int value ) {

    //if(value > ( TIM_DEFAULT_PERIOD ) ) value = TIM_DEFAULT_PERIOD;
    if( value > 0 ) deviceCurrentState.blowerEnabled = 1;
  __HAL_TIM_SET_COMPARE( &htim1, BLOWER_CHANNEL, value );
  deviceCurrentState.blowerCCR = value;
  return;
}
void disableBlower( void ) {
  //if( deviceCurrentState.blowerEnabled ) {
    deviceCurrentState.blowerEnabled = 0;
    setBlowerPwm( 0 );
  //}
 
}
void disableAll( void ) {
  disableMotor();
  disableBlower();
}
const int ACC = 5000;
const int DEC = 2000;
int accTime = ACC;

int accStep = 0;


int decTime = 0;
int decStep = 0;
/**
  * @brief  Function implementing the motor and blower control thread.
  * @param  argument: Not used 
  * @retval None
  */
void deviceControlTaskFunc( void const *argument ) 
{
  int iwdgReloadPeriod = 3000;
  int taskPeriod = 1;
  memset( &speedSettings, 0, sizeof( speedSettings ) );
  
  speedSettings.blowerSpeed = 50;
  speedSettings.motorSpeed = 50;
  
  HAL_TIM_PWM_Start( &htim1, MOTOR_CHANNEL );
  HAL_TIM_PWM_Start( &htim1, BLOWER_CHANNEL ); 
  
  disableAll();
  int minStartSpeed = TIM_ONE_PERCENT * 5;
  

  
  
//  while( 1 ) {
//    setMotorPwm( deviceCurrentState.workSetting.targetSpeed );
//    setBlowerPwm( deviceCurrentState.workSetting.targetSpeed / 5 );
//    osDelay(taskPeriod);
//  }
  while( 1 ) {
    
    switch( deviceCurrentState.state )
    {
    case STATE_WORKING:
      if( deviceCurrentState.minTempAchieved ) {
        if( accTime == ACC ) {
          accStep = deviceCurrentState.workSetting.targetSpeed / ( accTime / taskPeriod );
          deviceCurrentState.motorPwm = minStartSpeed;
          setBlowerPwm( TIM_DEFAULT_PERIOD );
          //setMotorPwm( TIM_DEFAULT_PERIOD );
          //osDelay(100);
        }
        if( accTime > 0 ) {
          deviceCurrentState.motorPwm += accStep;
          setMotorPwm( deviceCurrentState.motorPwm );
          accTime -= taskPeriod;
        }
        else {
          setMotorPwm( deviceCurrentState.workSetting.targetSpeed );
        }
   //     setMotorPwm( deviceCurrentState.workSetting.targetSpeed );
 //       setBlowerPwm( TIM_DEFAULT_PERIOD / 100 * 99 );
      }
      decTime = DEC;
 
      break;
    
    case STATE_WAITING:
            if( decTime == DEC ) {
        decStep = deviceCurrentState.motorPwm / ( decTime / taskPeriod );
        //deviceCurrentState.motorPwm = minStartSpeed;
        decTime = DEC;
      }
      if( decTime > 0 ) {
        deviceCurrentState.motorPwm -= decStep;
        setMotorPwm( deviceCurrentState.motorPwm );
        decTime -= taskPeriod;
      }
      else {
        if( deviceCurrentState.temperature > 78 ) setMotorPwm( 2000 );
        else {
          disableMotor();
          accTime = ACC;
        }
      }
      disableBlower();
      //disableAll();
      
      break;
      
    case STATE_IDLE: {
        setMotorPwm( TIM_DEFAULT_PERIOD / 100 * 50 );
    }
    break;
    }
    
    if( deviceCurrentState.temperature > RESET_TEMPERATURE ) {
        HAL_NVIC_SystemReset();
    }
    if( iwdgReloadPeriod <= 0 ) {
      __HAL_IWDG_RELOAD_COUNTER( &hiwdg );
      iwdgReloadPeriod = 3000;
    }

    osDelay(taskPeriod);
    iwdgReloadPeriod -= taskPeriod;
  }
}