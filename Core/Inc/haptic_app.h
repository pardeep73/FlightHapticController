/* STM32 hardware adapter for the independently tested haptic policy. */
#ifndef HAPTIC_APP_H
#define HAPTIC_APP_H

#include "main.h"
#include "haptic_controller.h"

/* Inspect this structure in the debugger to view measured angles and FSMs. */
extern HapticController haptic_controller;

/* Call once after generated GPIO/I2C/TIM setup, then poll from the main loop. */
void HapticApp_Init(I2C_HandleTypeDef *i2c, TIM_HandleTypeDef *timer);
void HapticApp_Poll(void);

#endif
