


#include "temp_sensor.h"
#include "math.h"
#include "string.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "motor.h"
#include "indicator.h"

extern tDeviceMenuState deviceMenuState;
extern tDeviceCurrentState  deviceCurrentState;

extern ADC_HandleTypeDef hadc;
extern TIM_HandleTypeDef htim3;

#define HEATER_CHANNEL  TIM_CHANNEL_1

#define DEF_PID_P   1.94
#define DEF_PID_I   2.0
#define DEF_PID_D   1.44

float currentDT = 0;
uint16_t currentPwm = 0;

uint16_t adcBuffer[2];
int adcIndex = 0;
uint8_t adcDataReadyFlag = 0;

#pragma optimize = none
void setHeaterPwm( int value ) {

  uint16_t ccr = HEATER_TIMER_DEF_PERIOD / 101 * value;
  if( ccr > (HEATER_TIMER_DEF_PERIOD - (HEATER_TIMER_DEF_PERIOD / 100 * 2) ) ) {
    ccr = HEATER_TIMER_DEF_PERIOD - (HEATER_TIMER_DEF_PERIOD / 100 * 2 );
  }
  
  if( ccr > 0 ) deviceCurrentState.heaterEnabled = 1;
  
 TIM3->CCR1 = ccr;
  deviceCurrentState.heaterPwm = ccr;
}
void disableHeater( void ) {
  if( deviceCurrentState.heaterEnabled != 0 ) {
    deviceCurrentState.heaterEnabled = 0;
    setHeaterPwm( 0 );
  }
}
/*
void Temperature_autotunePID(float temp,  int maxCycles, bool storeValues, int method)
{

  if(method < 0)
    method = 0;
  if(method > 5)
    method = 5;

  int32_t pidMax = 100;
  float* temperatures[1] = { &CurTemp };

  float currentTemp = CurTemp;
  
  while( 1 ) {
    currentTemp = CurTemp;
    osDelay(100);
  }

  int cycles = 0;
  bool autotuneState = true;

  uint32_t temp_millis = HAL_GetTick();
  uint32_t t1 = temp_millis;
  uint32_t t2 = temp_millis;
  int32_t t_high = 0;
  int32_t t_low;

  int32_t bias = pidMax / 2; /// ???????? ?????
  int32_t d = pidMax / 2; /// ?????? (?????? ??????????? ?? ????? ?????????)
  float Ku = 0, Tu = 0, Ku_summ = 0, Tu_summ = 0;
  float Kp = 0, Ki = 0, Kd = 0;
  float maxTemp = 20, minTemp = 20;
  if(maxCycles < 8)
    maxCycles = 8;
  if(maxCycles > 20)
    maxCycles = 20;

 

  /// ???????? ??? ???????????
  disableHeater();

  int progress = 0;
  int currentProgress = 0;
  autotune_state[heat_chan] = hasNone;

  /// ??????? ??????
  set_heat_pwm(heat_chan, 100);
  
  TPid *tmp;
  if( heat_chan == hchExtrLeft ) tmp = &leftCurrent;
  else if( heat_chan == hchExtrRight ) tmp = &rightCurrent;
  else if( heat_chan == hchTable ) tmp = &tableCurrent;

  for(;;)
  {
    IWDG_ReloadCounter();   // update iwdg

    /// ?????, ????? ?????? ?????? ???? ??????????
    tn_task_sleep(10);
    /// ??????????? ??????????? ? ???????? ????????? (????? ?? ?????????? ?? ???????)
    //GCode::keepAlive(WaitHeater);
    /// ??????? ???????????
    currentTemp = *(temperatures[temp_chan]);

    millis_t time = HAL::timeInMilliseconds();

    ///?????????? ????????? ???????? ??????? ???????????
    maxTemp = RMath::max(maxTemp, currentTemp);
    minTemp = RMath::min(minTemp, currentTemp);

    /// ???? ??????? ??????????? ?????????? ????????????? ?????
    if(heating == true && currentTemp > temp)
    { // switch heating -> off
      //if(time - t2 > (heat_chan < NUM_EXTRUDER ? 2500 : 1500)) {
      if(time - t2 > 2500)
      {
        heating = false;
        set_heat_pwm(heat_chan, bias - d); /// ?????????? ???????? ?? ?????? pwm_pos[pwmIndex] = (bias - d);
        t1 = time;
        t_high = t1 - t2;
        maxTemp = temp;
      }
    }

    /// ???? ??????? ??????????? ?????????? ????????????? ????
    if(heating == false && currentTemp < temp)
    {
      if(time - t1 > 5000)
      {
        heating = true;
        t2 = time;
        t_low = t2 - t1; // half wave length
        if(cycles > 0)
        {
          bias += (d * (t_high - t_low)) / (t_low + t_high);
          bias = constrain(bias, 8, pidMax - 8); /// ??????????? ???????? ????????? ?? 8 ?? 92%
          if(bias > pidMax / 2)
            d = pidMax - 1 - bias;	/// ?????? ?????????????? ???, ????? bias+d ???? ?????? 100%
          else
            d = bias;
          
          debug_direct("bias %d delta %d min %f, max %f, t_high %d, t_low %d\r\n", bias, d, minTemp, maxTemp, t_high, t_low);
          
          if(cycles > 4)
          {
            // Parameter according Ziegler–Nichols method: http://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
            Ku = (4.0 * d) / (3.14159 * (maxTemp - minTemp));
            Tu = static_cast<float>(t_low + t_high) / 1000.0;
            
            debug_direct("Ku %f Tu %f\r\n", Ku, Tu);
            
            if(cycles >= 8)
            {
              Ku_summ += Ku;
              Tu_summ += Tu;
              debug_direct("Ku_aver %f, Tu_aver %f\r\n", Ku_summ/(cycles-7), Tu_summ/(cycles-7));
            }
            
            if(method == 0)
            {
              Kp = 0.6 * Ku;
              Ki = Kp * 2.0 / Tu;
              Kd = Kp * Tu * 0.125;
              debug_direct("PIDClassic\r\n");
            }
            if(method == 1)
            {
              Kp = 0.33 * Ku;
              Ki = Kp * 2.0 / Tu;
              Kd = Kp * Tu / 3.0;
              debug_direct("PIDSome\r\n");
            }
            if(method == 2)
            {
              Kp = 0.2 * Ku;
              Ki = Kp * 2.0 / Tu;
              Kd = Kp * Tu / 3;
              debug_direct("APIDNone\r\n");
            }
            if(method == 3)
            {
              Kp = 0.7 * Ku;
              Ki = Kp * 2.5 / Tu;
              Kd = Kp * Tu * 3.0 / 20.0;
              debug_direct("PIDPessen\r\n");
            }
            if(method == 4)
            { //Tyreus-Lyben
              Kp = 0.4545f * Ku;      //1/2.2 KRkrit
              Ki = Kp / Tu / 2.2f;        //2.2 Tkrit
              Kd = Kp * Tu / 6.3f;      //1/6.3 Tkrit[/code]
              debug_direct("PIDTyreusLyben\r\n");
            }
            
            if(method == 5)
            {
              Kp = 0.653 * Ku_summ / (cycles - 7);
              Ki = 1.3 * Tu_summ / (cycles - 7);
              Kd = Ki * 0.0488;
              debug_direct("PIDPicaso\r\n");
            }
            
            debug_direct("Kp %f, Ki %f, Kd %f, %d/%d\r\n", Kp, Ki, Kd, cycles, maxCycles);
          }
          
        }
        if(cycles < maxCycles)	/// ????? ??? ?? ?????????? 100% ????????
        {
          autotune_progress_prc[heat_chan] = 100 * cycles / maxCycles;
          autotune_progress_prc_current = 100 * cycles / maxCycles;
        }
        set_heat_pwm(heat_chan, bias + d);
        cycles++;
        minTemp = temp;
      }
    }
    if(currentTemp > (temp + 40))
    {
      debug_direct("PIDFailedHigh\r\n");
      autotune_state[heat_chan] = hasErrOverHeat;
      /// ???????? ??? ???????????
      set_heat_pwm(hchExtrLeft, 0);
      set_heat_pwm(hchExtrRight, 0);
      set_heat_pwm(hchTable, 0);
      is_service_fl = 0;
      return;
    }

    if(0)
    {
      //if(time - temp_millis > 1000) {
      temp_millis = time;

      debug_direct("%d\t%f\t%e\t%f\t%f\t%e\t%d\r\n",
	  -1, currentTemp,
	  Temperature_get_I_PID(heat_chan), Temperature_get_D_PID(heat_chan),
	  Temperature_get_Err_PID(heat_chan), Temperature_get_Integral_PID(heat_chan),
	  Temperature_get_heat_pwm(heat_chan));
    }

    if(((time - t1) + (time - t2)) > (10L * 60L * 1000L * 2L))
    { // 20 Minutes
      debug_direct("PIDFailedTimeout\r\n");
      autotune_state[heat_chan] = hasErrTimeout;
      /// ???????? ??? ???????????
      set_heat_pwm(hchExtrLeft, 0);
      set_heat_pwm(hchExtrRight, 0);
      set_heat_pwm(hchTable, 0);
      is_service_fl = 0;
      return;
    }
    if(cycles > maxCycles)
    {
      debug_direct("PIDFinished\r\n");
      /// ???????? ??? ???????????
      set_heat_pwm(hchExtrLeft, 0);
      set_heat_pwm(hchExtrRight, 0);
      set_heat_pwm(hchTable, 0);
      if(storeValues)
      {
        tmp->K = Kp;
        tmp->Ti = Ki;
        tmp->Td = Kd;
        
        Temperature_set_pid_fram(heat_chan, *tmp);
        
        debug_direct("Result: Kp %f, Ki %f, Kd %f\r\n", Kp, Ki, Kd);
      }
      is_service_fl = 0;
      autotune_state[heat_chan] = hasOk;
      autotune_progress_prc[heat_chan] = 100;	/// 100% ???????? ??????????? ?????? ?????
      autotune_progress_prc_current = 100;
      return;
    }
  } // loop

  is_service_fl = 0;
}

 */ 
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
#pragma optimize = none
float getMotorControlVoltage( void  )
{
  uint16_t adcValue = adcBuffer[1];
  float volt = (float)adcValue/4096.0*BASE_VOLTAGE;
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
      adcDataReadyFlag = 1;      
      HAL_ADC_Start_IT(hadc);
    }

}
typedef struct {
  float p;
  float i;
  float d;
  float dt;
} tPidSettings;

typedef struct {
	float err;
	float ierr;	///???????? ?? ?????? (?????)
	float i;
	float d;
} tPidState;

float curTempError = 0;
tPidState pidState = { 0 };
tPidSettings pidSettings = { DEF_PID_P, DEF_PID_I, DEF_PID_D, 0.1 };


void resetPid()
{
  memset( &pidState, 0, sizeof( pidState ) );
}

#pragma optimize = none
uint16_t calcHeaterPwm( float delta ) 
{
  uint16_t res = 0;
  
  pidState.ierr += delta;
  float I = 1.0 / (pidSettings.i * pidState.ierr * pidSettings.dt);
  pidState.i = I;
  float D = pidSettings.d * (delta - pidState.err) / pidSettings.dt;
  pidState.err = delta;
  pidState.d = D;
  float U = pidSettings.p * ( delta + I + D );
  res = (int)( U + 0.5f );
  return res;
}
/**
  * @brief  Function implementing the temperature calc and control thread.
  * @param  argument: Not used 
  * @retval None
  */

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
  float prevSpeed = 0;
  int editDelay = 14;
  
  deviceCurrentState.workSetting.targetBlower = 20;
  deviceCurrentState.workSetting.targetSpeed = 200;
  deviceCurrentState.workSetting.targetTemp = 70;
  deviceCurrentState.workSetting.time = 1;
  
  while( 1 ) {
    
    // get values:
    deviceCurrentState.temperature = (int)getSensorTemperature();
    deviceCurrentState.motorControlVoltage = getMotorControlVoltage();
    
    // calc target motor pwm from input voltage
    deviceCurrentState.workSetting.targetSpeed = (int)(TIM_DEFAULT_PERIOD - ( TIM_DEFAULT_PERIOD - (TIM_DEFAULT_PERIOD * (3.26 -  deviceCurrentState.motorControlVoltage ) / 3.26) ) + 0.5f);
    if( deviceCurrentState.workSetting.targetSpeed < 0 ) deviceCurrentState.workSetting.targetSpeed = 0;
    
    float dt = deviceCurrentState.workSetting.targetTemp - deviceCurrentState.temperature;
    currentDT = dt;
    if( dt > 10 ) { //deviceCurrentState.workSetting.targetTemp / 100 * 12 ) {
      deviceCurrentState.minTempAchieved = 0;
    }
    else {
      deviceCurrentState.minTempAchieved = 1;
    }
    switch( deviceCurrentState.state )
    {
    case STATE_WORKING: {
       int16_t pwm = 0;
      if( dt > 0 ) {
        pwm = calcHeaterPwm( dt );
        setHeaterPwm( pwm );
      }
      else {
        setHeaterPwm( 0 );
      }
     
      if( deviceCurrentState.minTempAchieved == 1 ) {
        deviceCurrentState.motorPwm = deviceCurrentState.workSetting.targetSpeed;
      }
    }
    break;
    
    case STATE_WAITING:
      disableHeater();
      resetPid();
      break;
    }
    

    
    // set flag edit if changing target speed, 
    if( fabs(deviceCurrentState.workSetting.targetSpeed - prevSpeed) > 2 ) { 
      deviceMenuState.flagEdit = 1;
      editDelay = 14;
    }
    else {
      if( editDelay == 0 ) {
        deviceMenuState.flagEdit = 0;
      }
    }    
    if( editDelay == 0 ) {
      deviceMenuState.flagEdit = 0;
      editDelay = 14;
    }
    
    
    prevSpeed = deviceCurrentState.workSetting.targetSpeed;
    editDelay--;
    osDelay( 100 );
   
  }
}