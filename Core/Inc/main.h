/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Host tests compile selected sections of this same implementation. */
#if defined(HAPTIC_TEST_CONTROLLER) || defined(HAPTIC_TEST_APP) \
 || defined(HAPTIC_TEST_DIAGNOSTICS) || defined(HAPTIC_TEST_SOFT_I2C) \
 || defined(HAPTIC_TEST_GPIO_I2C)
#define HAPTIC_HOST_TEST
#endif

#ifndef HAPTIC_HOST_TEST
#include "stm32c0xx_hal.h"
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* Flight-envelope state and measured angles. */
/* Each signed axis uses one region plus the direction in which it was entered. */
typedef enum
{
  HAPTIC_FREE = 0,
  HAPTIC_COMPLIANT,
  HAPTIC_CLAMPED
} HapticZone;

/* Autopilot is a latched mode; returning to level alone does not release it. */
typedef enum
{
  HAPTIC_MANUAL = 0,
  HAPTIC_AUTOPILOT
} HapticMode;

/*
 * All measured angles are degrees. Positive roll/pitch mean anticlockwise as
 * viewed in Wokwi; their servo commands are 90 minus the signed angle. Tilt is
 * sqrt(pitch^2 + roll^2), not a signed angle, and its servo command is 2*tilt.
 */
typedef struct
{
  float roll_deg;
  float pitch_deg;
  float tilt_deg;
} HapticAngles;

/* Direction is -1, 0 or +1, so hysteresis never persists across a sign reversal. */
typedef struct
{
  HapticZone zone;
  int8_t direction;
} HapticAxis;

/* Public state also provides convenient watch expressions in the debugger. */
typedef struct
{
  HapticAxis roll;
  HapticAxis pitch;
  HapticZone tilt_zone;
  HapticMode mode;
  HapticAngles angles;
  bool sensor_valid;
  float roll_servo_deg;
  float pitch_servo_deg;
  float tilt_servo_deg;
} HapticController;


/* Software-I2C bus contract. */
typedef enum
{
  SOFT_I2C_OK = 0,
  SOFT_I2C_NACK,
  SOFT_I2C_TIMEOUT,
  SOFT_I2C_BUS_STUCK,
  SOFT_I2C_INVALID_ARGUMENT
} SoftI2CResult;

/*
 * set_scl/set_sda(false) actively pull low; true releases an open-drain line.
 * Reads return the actual bus level, including the slave's ACK/data/stretching.
 * delay_us receives only 1 or 5 us. get_tick returns a wrapping millisecond
 * counter. Both callbacks must return; an independent poll limit also bounds
 * clock stretching if that counter stops advancing.
 */
typedef struct
{
  void *context;
  void (*set_scl)(void *context, bool high);
  void (*set_sda)(void *context, bool high);
  bool (*read_scl)(void *context);
  bool (*read_sda)(void *context);
  void (*delay_us)(void *context, uint32_t microseconds);
  uint32_t (*get_tick)(void *context);
} SoftI2CBus;

/* Preserve the actual recovery result separately from a failed delay clock. */
typedef enum
{
  HAPTIC_TIMING_OK = 0,
  HAPTIC_TIMING_DISABLED,
  HAPTIC_TIMING_CONFIG,
  HAPTIC_TIMING_NO_PROGRESS,
  HAPTIC_TIMING_TICK_GUARD,
  HAPTIC_TIMING_POLL_GUARD
} HapticTimingFault;

typedef struct
{
  SoftI2CResult recover_result;
  HapticTimingFault timing_fault;
  uint32_t systick_ctrl;
  uint32_t systick_load;
  uint32_t val_first;
  uint32_t val_last;
  uint32_t elapsed_cycles;
  uint32_t polls;
  uint8_t lines_before; /* bit 0 = raw SCL; bit 1 = raw SDA */
  uint8_t lines_after;
} HapticSoftI2CStatus;

extern HapticSoftI2CStatus haptic_soft_i2c_status;


#if !defined(HAPTIC_HOST_TEST) || defined(HAPTIC_TEST_APP) \
 || defined(HAPTIC_TEST_DIAGNOSTICS) || defined(HAPTIC_TEST_GPIO_I2C)
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
  bool software_i2c;
  uint32_t hardware_error;
} HapticAppStatus;

/* First hardware failure, frozen before fallback can reset I2C or its pins. */
typedef struct
{
  bool captured;
  uint32_t error;
  uint32_t flag;
  uint32_t isr;
  uint32_t cr1;
  uint32_t cr2;
  uint32_t timingr;
  uint32_t gpio_idr;
  uint32_t moder;
  uint32_t otyper;
  uint32_t pupdr;
  uint32_t afr;
} HapticHardwareI2CStatus;

#endif

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* Only the I2C driver compilation unit enables this read-only observer. */
#ifdef HAPTIC_I2C_TRACE
#undef __HAL_I2C_GET_FLAG
#define __HAL_I2C_GET_FLAG(handle, flag) HapticTraceI2CFlag((handle), (flag))
#endif

/* USER CODE END EM */

#if !defined(HAPTIC_HOST_TEST) || defined(HAPTIC_TEST_APP) \
 || defined(HAPTIC_TEST_DIAGNOSTICS) || defined(HAPTIC_TEST_GPIO_I2C)
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
#endif

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* Start in manual mode, with free regions and neutral servo positions. */
void Haptic_Init(HapticController *controller);

/*
 * Calculate assignment equations from acceleration components in any common
 * unit (g or raw ADC counts). Reject a zero/nonfinite vector and leave *out
 * unchanged on failure. A nonzero static gravity vector is required.
 */
bool Haptic_CalculateAngles(float ax, float ay, float az, HapticAngles *out);

/*
 * Advance all FSMs once per sample. button_pressed is one debounced rising-edge
 * event, not a raw GPIO level. A sensor failure latches autopilot, keeps the last
 * valid angles/region history, and neutralises all outputs until valid readings
 * resume. The operator must subsequently press the button at safe angles to
 * return to manual. Valid feedback continues in both operating modes because
 * the assignment does not model autopilot flight control or aircraft dynamics.
 */
void Haptic_Update(HapticController *controller, const HapticAngles *angles,
                   bool sensor_valid, bool button_pressed);

/* Release the bus, issue up to nine recovery clocks if SDA is low, then STOP. */
SoftI2CResult SoftI2C_Recover(SoftI2CBus *bus);

/*
 * Use a seven-bit device address and an eight-bit register address.
 * Reads write the register pointer, issue a repeated START, and NACK the final
 * byte. All real transfers attempt STOP on success and failure; invalid
 * arguments leave the bus untouched. Output bytes are valid only on success.
 */
SoftI2CResult SoftI2C_ReadRegister(SoftI2CBus *bus, uint8_t address,
                                uint8_t reg, uint8_t *data, uint16_t length);
SoftI2CResult SoftI2C_WriteRegister(SoftI2CBus *bus, uint8_t address,
                                 uint8_t reg, const uint8_t *data,
                                 uint16_t length);

/*
 * The application must disable/deinitialize hardware I2C1 before calling Init.
 * Configure open-drain GPIO, release both lines, then perform bus recovery.
 */
bool HapticSoftI2C_Init(void);

/* Read or write real MPU6050 registers at its 7-bit address 0x68. */
SoftI2CResult HapticSoftI2C_Read(uint8_t reg, uint8_t *data, uint16_t length);
SoftI2CResult HapticSoftI2C_Write(uint8_t reg, uint8_t value);

/* Initialize once after the 48 MHz system clock and generated GPIO setup. */
void HapticDiagnostics_Init(void);

/* Call from the main loop; emits at most one status record per second. */
void HapticDiagnostics_Poll(void);

#if !defined(HAPTIC_HOST_TEST) || defined(HAPTIC_TEST_APP) \
 || defined(HAPTIC_TEST_DIAGNOSTICS) || defined(HAPTIC_TEST_GPIO_I2C)
extern HapticController haptic_controller;
extern HapticAppStatus haptic_app_status;
extern HapticHardwareI2CStatus haptic_hw_i2c_status;

/* Return exactly the HAL flag predicate while observing an active transfer. */
uint32_t HapticTraceI2CFlag(I2C_HandleTypeDef *handle, uint32_t flag);

/* Call once after generated GPIO/I2C/TIM setup, then poll from the main loop. */
void HapticApp_Init(I2C_HandleTypeDef *i2c, TIM_HandleTypeDef *timer);
void HapticApp_Poll(void);
#endif

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RCC_OSCX_IN_Pin GPIO_PIN_14
#define RCC_OSCX_IN_GPIO_Port GPIOC
#define RCC_OSCX_OUT_Pin GPIO_PIN_15
#define RCC_OSCX_OUT_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define VCP_USART2_RX_Pin GPIO_PIN_3
#define VCP_USART2_RX_GPIO_Port GPIOA
#define DEBUG_SWDIO_Pin GPIO_PIN_13
#define DEBUG_SWDIO_GPIO_Port GPIOA
#define DEBUG_SWCLK_Pin GPIO_PIN_14
#define DEBUG_SWCLK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
