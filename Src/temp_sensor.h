
#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H
#include "stm32f030x8.h"

#define MAX_TEMPERATURE         125
#define RESET_TEMPERATURE       135

#define BASE_VOLTAGE 3.27
#define HEATER_TIMER_DEF_PERIOD 2000
void temperatureAndPwmControlTaskFunc( void const *argument );

#define HEATER_TIMER TIM3

#define fiberheaterChannel TIM3->CCR1



#endif