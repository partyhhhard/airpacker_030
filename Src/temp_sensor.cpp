





#include "temp_sensor.h"
#include "math.h"
#include "string.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "motor.h"
#include "indicator.h"
#include "pid_controller.h"


extern tDeviceCurrentState  dcs;

extern ADC_HandleTypeDef hadc;
extern TIM_HandleTypeDef htim3;

#define HEATER_CHANNEL  TIM_CHANNEL_1

#define DEF_PID_P   1.2    
#define DEF_PID_I   0.22     //0.2  //2.0
#define DEF_PID_D   2.0

PIDControl pid;


float currentDT = 0;
uint16_t currentPwm = 0;

uint16_t adcBuffer[2];
int adcIndex = 0;
uint8_t adcDataReadyFlag = 0;

#pragma optimize = none
void setHeaterPwm( int value ) {

  if( value < 0 ) value = 0;
  int ccr = HEATER_TIMER_DEF_PERIOD / 101 * value;
  if( ccr > (HEATER_TIMER_DEF_PERIOD - (HEATER_TIMER_DEF_PERIOD / 100) * 2 ) ) {
    ccr = HEATER_TIMER_DEF_PERIOD - (HEATER_TIMER_DEF_PERIOD / 100) * 2;
  }
  
  if( ccr > 0 ) dcs.heaterEnabled = 1;
  if( ccr < 0 ) ccr = 0;
 TIM3->CCR1 = ccr;
 dcs.heaterPwm = ccr;
}
void disableHeater( void ) {
  if( dcs.heaterEnabled != 0 ) {
    dcs.heaterEnabled = 0;
    setHeaterPwm( 0 );
  }
}

#pragma optimize = none
static float calcTempByV( float adcV, float r0, float t0, float betaT0, float R1 )
{
  t0+= 273.15;			
  float i = ( BASE_VOLTAGE - adcV ) / R1;
  float r = adcV/i;
  float _t=logf(r/r0)/betaT0 + 1/t0;
  return (1/_t - 273.15);
}
#pragma optimize = none
float getSensorTemperature( void )
{
  
  uint16_t adcValue = adcBuffer[0];
  float vTable = (float)adcValue/4096.0*BASE_VOLTAGE;
  float tTable = calcTempByV( vTable, 100e3/*TABLE_BASE_R*/, 25, 4334, 4.64e3 );
  return tTable;
}

const uint8_t motbufsize = 10;
uint16_t motbuf[motbufsize];
uint8_t motbufindex = 0; 
#pragma optimize = none
float getMotorControlVoltage( void  )
{
  uint16_t avg = 0;
  uint16_t sum = 0;
  for( int i = 0; i < motbufsize; i++ ) {
    sum += motbuf[i];
  }
  avg = sum / motbufsize;
    
  float volt = (float)avg/4096.0*BASE_VOLTAGE;
  return volt;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC)) 
    {
      adcBuffer[adcIndex] = HAL_ADC_GetValue(hadc); 
      adcIndex++;
      __HAL_ADC_CLEAR_FLAG( hadc, ADC_FLAG_EOC );
      
    }
    if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOS))  
    {
      adcIndex = 0;
      motbuf[motbufindex] = adcBuffer[1];
      motbufindex++;
      if( motbufindex >= motbufsize ) motbufindex = 0;
      adcDataReadyFlag = 1;      
      HAL_ADC_Start_IT(hadc);
    }

}




/**
  * @brief  Function implementing the temperature calc and control thread.
  * @param  argument: Not used 
  * @retval None
  */
int mintempdelta = 8;
#pragma optimize = none
void temperatureAndPwmControlTaskFunc( void const *argument ) 
{

  if (ADC1->CR & ADC_CR_ADEN) {
    ADC1->CR |= ADC_CR_ADDIS;
    while (ADC1->CR & ADC_CR_ADEN) {}
  }
  
  /* calibrate ADC */
  ADC1->CR |= ADC_CR_ADCAL;
  while(ADC1->CR & ADC_CR_ADCAL) {}
  
  /* reset configuration */
  ADC1->CFGR2 = 0;
  /* enable device */
  ADC1->CR = ADC_CR_ADEN;
  while(!(ADC1->ISR & ADC_ISR_ADRDY));
  
  HAL_TIM_PWM_Start( &htim3, HEATER_CHANNEL ); 
  disableHeater();
  
  HAL_ADC_Start_IT( &hadc );
  
  dcs.workSetting.targetBlower = 20;
  dcs.workSetting.targetSpeed = 200;
  dcs.workSetting.targetTemp = 107;
 
  
  //pid = new PIDControl( pidSettings.p, pidSettings.i, pidSettings.d, pidSettings.dt, 
  //                     0, HEATER_TIMER_DEF_PERIOD, AUTOMATIC, REVERSE );
  PIDInit( &pid, DEF_PID_P, DEF_PID_I, DEF_PID_D, 
             0.1, 0.0, HEATER_TIMER_DEF_PERIOD, 
             AUTOMATIC, DIRECT);   	
//  for( int i = 50; i > -30; i-- ) {
//     volatile int pwm = calcHeaterPwm( i );
//     setHeaterPwm( pwm );
//     osDelay(20);
//  }
//    for( int i = -30; i > 50; i++ ) {
//     volatile int pwm = calcHeaterPwm( i );
//     setHeaterPwm( pwm );
//     osDelay(20);
//  }
//  do {
//    setHeaterPwm( HEATER_TIMER_DEF_PERIOD / 2 );
//    osDelay(2000);
//    disableHeater();
//    while( 1 ) {
//        float temp1 = getSensorTemperature();
//        osDelay( 100 );
//        float temp2 = getSensorTemperature(); 
//        if( temp2 < temp1 ) break;
//    }
//  } while( 0 );
  float prevMotTargetSpd = 0;
  int timeleft = 1200;
  while( 1 ) {
    
    // get values:
    dcs.temperature = (int)getSensorTemperature();
    dcs.motorControlVoltage = getMotorControlVoltage();
    
    // calc target motor pwm from input voltage
    dcs.workSetting.targetSpeed = (int)(TIM_DEFAULT_PERIOD - ( TIM_DEFAULT_PERIOD - (TIM_DEFAULT_PERIOD * (3.26 -  dcs.motorControlVoltage ) / 3.26) ) + 0.5f);
    if( dcs.workSetting.targetSpeed < 0 ) dcs.workSetting.targetSpeed = 0;
    if( dcs.menuState != EDIT_TIME ) {
      if( fabs( prevMotTargetSpd - dcs.workSetting.targetSpeed ) > 50 ) {
        if( dcs.menuState != SHOW_SPEED ) {
          dcs.menuState = SHOW_SPEED;
          timeleft = 1200;
        }
      }
      else {
        if( timeleft <= 0 ) {
          dcs.menuState = SHOW_TEMP;
        }
        if( timeleft > 0 ) timeleft -= 100;
      }
    }
    prevMotTargetSpd = dcs.workSetting.targetSpeed;
    
    float dt = dcs.workSetting.targetTemp - dcs.temperature;
    currentDT = dt;
    if( dt > mintempdelta ) { //dcs.workSetting.targetTemp / 100 * 12 ) {
      dcs.minTempAchieved = 0;
    }
    else {
      dcs.minTempAchieved = 1;
    }
    switch( dcs.state )
    {
    case STATE_WORKING: {
       int pwm = 0;
     // if( dt > 0 ) {
        PIDSetpointSet( &pid, (float)dcs.workSetting.targetTemp );
        PIDInputSet( &pid, (float)dcs.temperature );
        PIDCompute( &pid );
        pwm = (int)PIDOutputGet( &pid );
        //pwm = calcHeaterPwm( dt );
        setHeaterPwm( pwm );
//      }
//      else {
//        setHeaterPwm( 0 );
//      }
     
//      if( dcs.minTempAchieved == 1 ) {
//        //dcs.motorPwm = dcs.workSetting.targetSpeed;
//      }
    }
    break;
    
    case STATE_STOPPED:
      disableHeater();
      resetPid( &pid );
      break;
    }
    

    
//    // set flag edit if changing target speed, 
//    if( fabs(dcs.workSetting.targetSpeed - prevSpeed) > 2 ) { 
//      deviceMenuState.flagEdit = 1;
//      editDelay = 14;
//    }
//    else {
//      if( editDelay == 0 ) {
//        deviceMenuState.flagEdit = 0;
//      }
//    }    
//    if( editDelay == 0 ) {
//      deviceMenuState.flagEdit = 0;
//      editDelay = 14;
//    }
    
    if( dcs.temperature > RESET_TEMPERATURE ) {
        HAL_NVIC_SystemReset();
    }
    
    prevMotTargetSpd = dcs.workSetting.targetSpeed;
    
    osDelay( 100 );
   
  }
}