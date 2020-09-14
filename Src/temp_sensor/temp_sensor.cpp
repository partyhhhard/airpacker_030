
#include "temp_sensor.h"
#include "math.h"


float CurTemp = 0;
extern ADC_HandleTypeDef hadc;
#pragma optimize = none
static float calcTempByV( float adcV, float r0, float t0, float betaT0, float R1 )
{
  t0+= 273.15;			
  float i = ( BASE_VOLTAGE - adcV ) / R1;
  float r = adcV/i;
  float _t=logf(r/r0)/betaT0 + 1/t0;
  return (1/_t - 273.15);
}
float getSensorTemperature( ADC_HandleTypeDef *hadc )
{
  uint16_t adcValue = 0;
  HAL_ADC_Start( hadc );
  osDelay(10);
  HAL_ADC_Stop( hadc );
  adcValue = HAL_ADC_GetValue( hadc );
  float vTable = (float)adcValue/4096.0*BASE_VOLTAGE;
  float tTable = calcTempByV( vTable, 100e3/*TABLE_BASE_R*/, 25, 4334, 10.0e3/*4.64e3*/ );
  return tTable;
}

/**
  * @brief  Function implementing the temperature calc and control thread.
  * @param  argument: Not used 
  * @retval None
  */
void temperatureControlTaskFunc( void const *argument ) 
{
  HAL_ADC_Stop( &hadc );
  HAL_ADCEx_Calibration_Start( &hadc );
  HAL_ADC_Start( &hadc );
  
  while( 1 ) {
    
    CurTemp = getSensorTemperature( &hadc );
    osDelay( 20 );
  }
}