/* STM32 hardware adapter for the independently tested haptic policy. */
#ifndef HAPTIC_APP_H
#define HAPTIC_APP_H

#include "main.h"
#include "haptic_controller.h"

/* Inspect this structure in the debugger to view measured angles and FSMs. */
extern HapticController haptic_controller;

/* Keep sensor faults separate from a healthy, latched autopilot mode. */
typedef enum
{
  HAPTIC_SENSOR_NONE = 0,
  HAPTIC_SENSOR_WHO_READ,
  HAPTIC_SENSOR_IDENTITY,
  HAPTIC_SENSOR_WAKE,
  HAPTIC_SENSOR_CONFIG,
  HAPTIC_SENSOR_ACCEL_READ,
  HAPTIC_SENSOR_ACCEL_INVALID,
  HAPTIC_SENSOR_RECOVERY
} HapticSensorFault;

/* Last bus result is available in both the debugger and serial diagnostics. */
typedef struct
{
  uint8_t identity;
  uint8_t register_address;
  uint32_t i2c_error;
  HAL_StatusTypeDef hal_status;
  HapticSensorFault fault;
} HapticAppStatus;

extern HapticAppStatus haptic_app_status;

/* Call once after generated GPIO/I2C/TIM setup, then poll from the main loop. */
void HapticApp_Init(I2C_HandleTypeDef *i2c, TIM_HandleTypeDef *timer);
void HapticApp_Poll(void);

#endif
