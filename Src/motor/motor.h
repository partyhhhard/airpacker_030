#ifndef MOTOR_H
#define MOTOR_H

#define TIM_DEFAULT_PERIOD          10000
#define TIM_ONE_PERCENT             TIM_DEFAULT_PERIOD / 100
extern void deviceControlTaskFunc( void const *argument ) ;

typedef struct {
  int motorSpeed;
  int blowerSpeed;
} tMotorAndBlowerSettings;

extern tMotorAndBlowerSettings speedSettings;
#endif