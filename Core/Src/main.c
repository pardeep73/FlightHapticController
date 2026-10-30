/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#ifndef HAPTIC_HOST_TEST

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#endif /* !HAPTIC_HOST_TEST: generated variables/prototypes */

/* ===== Flight-envelope controller ===== */
#if !defined(HAPTIC_HOST_TEST) || defined(HAPTIC_TEST_CONTROLLER) || defined(HAPTIC_TEST_APP)
/* Haptic regions, 5-degree release hysteresis and latched flight-mode policy. */


#define RAD_TO_DEG (57.29577951308232f)
#define AXIS_COMPLIANT_ENTER (30.0f)
#define AXIS_CLAMP_ENTER (50.0f)
#define TILT_COMPLIANT_ENTER (20.0f)
#define TILT_CLAMP_ENTER (70.0f)
#define RELEASE_DEADBAND (5.0f)

/*
 * Shared magnitude FSM. Outward transitions use the stated limits and inward
 * transitions require a 5-degree retreat. Exact upper limits enter clamping;
 * this resolves the brief's unspecified equality at +/-50 and at tilt 20.
 * Direct jumps across multiple regions settle in the correct state immediately.
 */
static HapticZone UpdateZone(HapticZone previous, float magnitude,
                              float compliant_enter, float clamp_enter)
{
  const float free_release = compliant_enter - RELEASE_DEADBAND;
  const float clamp_release = clamp_enter - RELEASE_DEADBAND;

  /* A large outward jump must clamp even when the previous state was free. */
  if (magnitude >= clamp_enter)
  {
    return HAPTIC_CLAMPED;
  }

  switch (previous)
  {
    case HAPTIC_CLAMPED:
      /* A sampled jump back to level may cross both release limits at once. */
      if (magnitude <= free_release)
      {
        return HAPTIC_FREE;
      }
      if (magnitude <= clamp_release)
      {
        return HAPTIC_COMPLIANT;
      }
      return HAPTIC_CLAMPED;

    case HAPTIC_COMPLIANT:
      return magnitude <= free_release ? HAPTIC_FREE : HAPTIC_COMPLIANT;

    case HAPTIC_FREE:
    default:
      return magnitude >= compliant_enter ? HAPTIC_COMPLIANT : HAPTIC_FREE;
  }
}

/*
 * Reuse the same FSM for positive/negative roll and pitch. If a sampled command
 * reverses sign, it has crossed the free region; evaluate the new side from
 * free rather than carrying the opposite side's clamped state through zero.
 */
static void UpdateAxis(HapticAxis *axis, float angle)
{
  const int8_t direction = angle > 0.0f ? 1 : (angle < 0.0f ? -1 : 0);

  if (direction != axis->direction)
  {
    axis->zone = HAPTIC_FREE;
  }
  axis->zone = UpdateZone(axis->zone, fabsf(angle),
                          AXIS_COMPLIANT_ENTER, AXIS_CLAMP_ENTER);
  axis->direction = direction;
}

/*
 * Wokwi draws increasing servo angles clockwise (positive SVG rotation).
 * The assignment defines clockwise as negative, so subtract the signed command
 * from the 90-degree neutral position. Positive roll/pitch then move anticlockwise.
 */
static float AxisServoAngle(const HapticAxis *axis, float angle)
{
  if (axis->zone == HAPTIC_FREE)
  {
    return 90.0f;
  }
  if (axis->zone == HAPTIC_CLAMPED)
  {
    /* Hold the hard boundary throughout its inward hysteresis band. */
    return 90.0f - (float)axis->direction * AXIS_CLAMP_ENTER;
  }
  return 90.0f - angle;
}

void Haptic_Init(HapticController *controller)
{
  if (controller == NULL)
  {
    return;
  }

  /* Zero-valued enums deliberately represent free regions and manual mode. */
  *controller = (HapticController){0};
  controller->roll_servo_deg = 90.0f;
  controller->pitch_servo_deg = 90.0f;
  controller->tilt_servo_deg = 0.0f;
}

bool Haptic_CalculateAngles(float ax, float ay, float az, HapticAngles *out)
{
  float scale;
  HapticAngles calculated;

  if (out == NULL || !isfinite(ax) || !isfinite(ay) || !isfinite(az))
  {
    return false;
  }

  /*
   * The equations are independent of common scale. Normalising by the largest
   * component avoids overflow/underflow in squaring without changing angles.
   */
  scale = fmaxf(fabsf(ax), fmaxf(fabsf(ay), fabsf(az)));
  if (scale == 0.0f)
  {
    return false;
  }
  ax /= scale;
  ay /= scale;
  az /= scale;

  calculated.roll_deg = atan2f(ay, az) * RAD_TO_DEG;
  calculated.pitch_deg = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
  calculated.tilt_deg = sqrtf(calculated.pitch_deg * calculated.pitch_deg
                              + calculated.roll_deg * calculated.roll_deg);
  *out = calculated;
  return true;
}

void Haptic_Update(HapticController *controller, const HapticAngles *angles,
                   bool sensor_valid, bool button_pressed)
{
  if (controller == NULL)
  {
    return;
  }

  /* Invalid data must never flow into comparisons or PWM conversion. */
  controller->sensor_valid = sensor_valid && angles != NULL
      && isfinite(angles->roll_deg) && isfinite(angles->pitch_deg)
      && isfinite(angles->tilt_deg) && angles->tilt_deg >= 0.0f;
  if (!controller->sensor_valid)
  {
    controller->mode = HAPTIC_AUTOPILOT;
    controller->roll_servo_deg = 90.0f;
    controller->pitch_servo_deg = 90.0f;
    controller->tilt_servo_deg = 0.0f;
    return;
  }

  controller->angles = *angles;
  UpdateAxis(&controller->roll, angles->roll_deg);
  UpdateAxis(&controller->pitch, angles->pitch_deg);
  controller->tilt_zone = UpdateZone(controller->tilt_zone, angles->tilt_deg,
                                     TILT_COMPLIANT_ENTER, TILT_CLAMP_ENTER);

  /* Automatic entry has priority over a button event in the same sample. */
  if (controller->roll.zone == HAPTIC_CLAMPED
      && controller->pitch.zone == HAPTIC_CLAMPED)
  {
    controller->mode = HAPTIC_AUTOPILOT;
  }
  else if (button_pressed)
  {
    if (controller->mode == HAPTIC_MANUAL)
    {
      controller->mode = HAPTIC_AUTOPILOT;
    }
    else if (controller->roll.zone == HAPTIC_FREE
             && controller->pitch.zone == HAPTIC_FREE
             && fabsf(angles->roll_deg) < AXIS_COMPLIANT_ENTER
             && fabsf(angles->pitch_deg) < AXIS_COMPLIANT_ENTER)
    {
      /* A rejected press is consumed; a later safe sample needs a new press. */
      controller->mode = HAPTIC_MANUAL;
    }
  }

  controller->roll_servo_deg = AxisServoAngle(&controller->roll, angles->roll_deg);
  controller->pitch_servo_deg = AxisServoAngle(&controller->pitch, angles->pitch_deg);

  /* Tilt has no sign and uses twice the detected angle to fill the servo range. */
  switch (controller->tilt_zone)
  {
    case HAPTIC_CLAMPED:
      controller->tilt_servo_deg = 2.0f * TILT_CLAMP_ENTER;
      break;
    case HAPTIC_COMPLIANT:
      controller->tilt_servo_deg = 2.0f * angles->tilt_deg;
      break;
    case HAPTIC_FREE:
    default:
      controller->tilt_servo_deg = 0.0f;
      break;
  }
}
#endif

/* ===== Software-I2C protocol ===== */
#if !defined(HAPTIC_HOST_TEST) || defined(HAPTIC_TEST_SOFT_I2C) || defined(HAPTIC_TEST_GPIO_I2C)
/*
 * Standard-mode I2C using real GPIO levels. No sensor samples are synthesized:
 * every address and written byte requires a slave ACK, and all received data
 * comes from SDA. This transport avoids reliance on simulated I2C status flags.
 */


#define HALF_PERIOD_US 5U
#define STRETCH_TIMEOUT_MS 2U
#define STRETCH_POLL_LIMIT 2000U

static bool ValidBus(const SoftI2CBus *bus)
{
  return bus != NULL && bus->set_scl != NULL && bus->set_sda != NULL
      && bus->read_scl != NULL && bus->read_sda != NULL
      && bus->delay_us != NULL && bus->get_tick != NULL;
}

static void HalfPeriod(SoftI2CBus *bus)
{
  bus->delay_us(bus->context, HALF_PERIOD_US);
}

/* Respect slave clock stretching, with a second bound if the tick freezes. */
static SoftI2CResult RaiseClock(SoftI2CBus *bus)
{
  const uint32_t start = bus->get_tick(bus->context);
  uint32_t polls = 0;
  bus->set_scl(bus->context, true);
  while (!bus->read_scl(bus->context))
  {
    if ((uint32_t)(bus->get_tick(bus->context) - start) >= STRETCH_TIMEOUT_MS
        || polls++ >= STRETCH_POLL_LIMIT)
    {
      return SOFT_I2C_TIMEOUT;
    }
    bus->delay_us(bus->context, 1U);
  }
  return SOFT_I2C_OK;
}

/* SDA falling while SCL is high is START, including a repeated START. */
static SoftI2CResult Start(SoftI2CBus *bus)
{
  bus->set_sda(bus->context, true);
  HalfPeriod(bus);
  if (RaiseClock(bus) != SOFT_I2C_OK) return SOFT_I2C_TIMEOUT;
  HalfPeriod(bus);
  if (!bus->read_sda(bus->context)) return SOFT_I2C_BUS_STUCK;
  bus->set_sda(bus->context, false);
  HalfPeriod(bus);
  bus->set_scl(bus->context, false);
  return SOFT_I2C_OK;
}

/* Release SDA even when a stuck clock makes a complete STOP impossible. */
static SoftI2CResult Stop(SoftI2CBus *bus)
{
  bus->set_scl(bus->context, false);
  bus->set_sda(bus->context, false);
  HalfPeriod(bus);
  const SoftI2CResult result = RaiseClock(bus);
  HalfPeriod(bus);
  bus->set_sda(bus->context, true);
  HalfPeriod(bus);
  if (result != SOFT_I2C_OK) return result;
  return bus->read_sda(bus->context) ? SOFT_I2C_OK : SOFT_I2C_BUS_STUCK;
}

/* Preserve the first transfer error while still trying to release the bus. */
static SoftI2CResult Finish(SoftI2CBus *bus, SoftI2CResult result)
{
  const SoftI2CResult stop_result = Stop(bus);
  return result == SOFT_I2C_OK ? stop_result : result;
}

/* Send MSB first, then release SDA for the slave's ninth-clock ACK. */
static SoftI2CResult WriteByte(SoftI2CBus *bus, uint8_t byte)
{
  for (unsigned bit = 0; bit < 8U; ++bit)
  {
    bus->set_sda(bus->context, (byte & 0x80U) != 0U);
    HalfPeriod(bus);
    if (RaiseClock(bus) != SOFT_I2C_OK) return SOFT_I2C_TIMEOUT;
    HalfPeriod(bus);
    bus->set_scl(bus->context, false);
    byte <<= 1;
  }
  bus->set_sda(bus->context, true);
  HalfPeriod(bus);
  if (RaiseClock(bus) != SOFT_I2C_OK) return SOFT_I2C_TIMEOUT;
  HalfPeriod(bus);
  const bool acknowledged = !bus->read_sda(bus->context);
  bus->set_scl(bus->context, false);
  return acknowledged ? SOFT_I2C_OK : SOFT_I2C_NACK;
}

/* A master ACK requests another byte; its final NACK terminates the read. */
static SoftI2CResult ReadByte(SoftI2CBus *bus, uint8_t *byte, bool acknowledge)
{
  uint8_t received = 0;
  bus->set_sda(bus->context, true);
  for (unsigned bit = 0; bit < 8U; ++bit)
  {
    HalfPeriod(bus);
    if (RaiseClock(bus) != SOFT_I2C_OK) return SOFT_I2C_TIMEOUT;
    HalfPeriod(bus);
    received = (uint8_t)((received << 1)
                    | (bus->read_sda(bus->context) ? 1U : 0U));
    bus->set_scl(bus->context, false);
  }
  bus->set_sda(bus->context, !acknowledge);
  HalfPeriod(bus);
  if (RaiseClock(bus) != SOFT_I2C_OK) return SOFT_I2C_TIMEOUT;
  HalfPeriod(bus);
  bus->set_scl(bus->context, false);
  bus->set_sda(bus->context, true);
  *byte = received;
  return SOFT_I2C_OK;
}

SoftI2CResult SoftI2C_Recover(SoftI2CBus *bus)
{
  if (!ValidBus(bus)) return SOFT_I2C_INVALID_ARGUMENT;
  bus->set_sda(bus->context, true);
  if (RaiseClock(bus) != SOFT_I2C_OK)
  {
    return SOFT_I2C_TIMEOUT;
  }
  HalfPeriod(bus);
  /* A slave waiting to finish an interrupted byte can release within 9 clocks. */
  for (unsigned pulse = 0; pulse < 9U && !bus->read_sda(bus->context); ++pulse)
  {
    bus->set_scl(bus->context, false);
    HalfPeriod(bus);
    if (RaiseClock(bus) != SOFT_I2C_OK) return Finish(bus, SOFT_I2C_TIMEOUT);
    HalfPeriod(bus);
  }
  const SoftI2CResult result = bus->read_sda(bus->context)
      ? SOFT_I2C_OK : SOFT_I2C_BUS_STUCK;
  return Finish(bus, result);
}

SoftI2CResult SoftI2C_ReadRegister(SoftI2CBus *bus, uint8_t address,
                                uint8_t reg, uint8_t *data, uint16_t length)
{
  if (!ValidBus(bus) || address > 0x7FU || data == NULL || length == 0U)
  {
    return SOFT_I2C_INVALID_ARGUMENT;
  }
  SoftI2CResult result = Start(bus);
  if (result == SOFT_I2C_OK) result = WriteByte(bus, (uint8_t)(address << 1));
  if (result == SOFT_I2C_OK) result = WriteByte(bus, reg);
  if (result == SOFT_I2C_OK) result = Start(bus);
  if (result == SOFT_I2C_OK)
  {
    result = WriteByte(bus, (uint8_t)((address << 1) | 1U));
  }
  for (uint16_t index = 0; result == SOFT_I2C_OK && index < length; ++index)
  {
    result = ReadByte(bus, &data[index], index + 1U < length);
  }
  return Finish(bus, result);
}

SoftI2CResult SoftI2C_WriteRegister(SoftI2CBus *bus, uint8_t address,
                                 uint8_t reg, const uint8_t *data,
                                 uint16_t length)
{
  if (!ValidBus(bus) || address > 0x7FU || data == NULL || length == 0U)
  {
    return SOFT_I2C_INVALID_ARGUMENT;
  }
  SoftI2CResult result = Start(bus);
  if (result == SOFT_I2C_OK) result = WriteByte(bus, (uint8_t)(address << 1));
  if (result == SOFT_I2C_OK) result = WriteByte(bus, reg);
  for (uint16_t index = 0; result == SOFT_I2C_OK && index < length; ++index)
  {
    result = WriteByte(bus, data[index]);
  }
  return Finish(bus, result);
}
#endif

/* ===== STM32 GPIO and timing adapter ===== */
#if !defined(HAPTIC_HOST_TEST) || defined(HAPTIC_TEST_GPIO_I2C)
/*
 * GPIO adapter for standards-based software I2C. This transport observes actual
 * SDA acknowledgements and register bytes; it does not supply simulated samples.
 * PB8/PB9 remain the assignment's SCL/SDA pins, independent of I2C1 emulation.
 */


#define MPU_ADDRESS_7BIT 0x68U
#define SCL_PIN GPIO_PIN_8
#define SDA_PIN GPIO_PIN_9
#define DELAY_GUARD_MS 2U
#define DELAY_POLL_LIMIT 4096U

/* A timing failure makes SCL unavailable instead of producing unclocked data. */
static bool timing_fault;
HapticSoftI2CStatus haptic_soft_i2c_status;

/* These readings bypass the timing guard in ReadScl: record electrical levels. */
static uint8_t RawBusLines(void)
{
  return (HAL_GPIO_ReadPin(GPIOB, SCL_PIN) == GPIO_PIN_SET ? 1U : 0U)
       | (HAL_GPIO_ReadPin(GPIOB, SDA_PIN) == GPIO_PIN_SET ? 2U : 0U);
}

/* In open-drain mode SET releases the pin; only RESET actively drives it low. */
static void SetScl(void *context, bool high)
{
  (void)context;
  HAL_GPIO_WritePin(GPIOB, SCL_PIN, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void SetSda(void *context, bool high)
{
  (void)context;
  HAL_GPIO_WritePin(GPIOB, SDA_PIN, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Read physical line states so pullups, slave ACKs and clock stretching matter. */
static bool ReadScl(void *context)
{
  (void)context;
  return !timing_fault && HAL_GPIO_ReadPin(GPIOB, SCL_PIN) == GPIO_PIN_SET;
}

static bool ReadSda(void *context)
{
  (void)context;
  return HAL_GPIO_ReadPin(GPIOB, SDA_PIN) == GPIO_PIN_SET;
}

/*
 * Use the Cortex-M SysTick down-counter for microsecond bus timing. Account for
 * its reload across a millisecond boundary. IRQ/tick and iteration guards keep
 * an absent or stopped simulated counter from hanging the application.
 */
static void DelayUs(void *context, uint32_t microseconds)
{
  (void)context;
  const uint32_t control = SysTick->CTRL;
  const uint32_t reload = SysTick->LOAD + 1U;
  uint32_t clock_hz = SystemCoreClock;

  if ((control & SysTick_CTRL_CLKSOURCE_Msk) == 0U)
  {
    clock_hz /= 8U;
  }
  if (microseconds == 0U || timing_fault)
  {
    return;
  }
  /* Save the first failed delay; subsequent STOP/recovery cannot overwrite it. */
  haptic_soft_i2c_status.systick_ctrl = control;
  haptic_soft_i2c_status.systick_load = reload - 1U;
  haptic_soft_i2c_status.val_first = SysTick->VAL;
  haptic_soft_i2c_status.val_last = haptic_soft_i2c_status.val_first;
  haptic_soft_i2c_status.elapsed_cycles = 0U;
  haptic_soft_i2c_status.polls = 0U;
  if ((control & SysTick_CTRL_ENABLE_Msk) == 0U || reload <= 1U
      || clock_hz < 1000000U || microseconds > 1000U)
  {
    haptic_soft_i2c_status.timing_fault =
        (control & SysTick_CTRL_ENABLE_Msk) == 0U
        ? HAPTIC_TIMING_DISABLED : HAPTIC_TIMING_CONFIG;
    timing_fault = true;
    return;
  }

  uint32_t remaining = ((clock_hz + 999999U) / 1000000U) * microseconds;
  uint32_t previous = haptic_soft_i2c_status.val_first;
  bool tick_guard = false;
  const uint32_t started_ms = HAL_GetTick();
  for (uint32_t polls = 0U; polls < DELAY_POLL_LIMIT; ++polls)
  {
    const uint32_t current = SysTick->VAL;
    const uint32_t elapsed = previous >= current ? previous - current
                                               : previous + reload - current;
    haptic_soft_i2c_status.val_last = current;
    haptic_soft_i2c_status.polls = polls + 1U;
    haptic_soft_i2c_status.elapsed_cycles += elapsed;
    if (elapsed >= remaining)
    {
      return;
    }
    remaining -= elapsed;
    previous = current;
    if ((uint32_t)(HAL_GetTick() - started_ms) >= DELAY_GUARD_MS)
    {
      tick_guard = true;
      break;
    }
  }
  haptic_soft_i2c_status.timing_fault =
      haptic_soft_i2c_status.elapsed_cycles == 0U ? HAPTIC_TIMING_NO_PROGRESS
      : (tick_guard ? HAPTIC_TIMING_TICK_GUARD : HAPTIC_TIMING_POLL_GUARD);
  timing_fault = true;
}

static uint32_t GetTick(void *context)
{
  (void)context;
  return HAL_GetTick();
}

static SoftI2CBus bus = {
  .context = NULL,
  .set_scl = SetScl,
  .set_sda = SetSda,
  .read_scl = ReadScl,
  .read_sda = ReadSda,
  .delay_us = DelayUs,
  .get_tick = GetTick
};

bool HapticSoftI2C_Init(void)
{
  GPIO_InitTypeDef pins = {0};
  timing_fault = false;
  haptic_soft_i2c_status = (HapticSoftI2CStatus){0};
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Preload released levels before switching the pins out of alternate mode. */
  HAL_GPIO_WritePin(GPIOB, SCL_PIN | SDA_PIN, GPIO_PIN_SET);
  pins.Pin = SCL_PIN | SDA_PIN;
  pins.Mode = GPIO_MODE_OUTPUT_OD;
  pins.Pull = GPIO_PULLUP;
  pins.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &pins);

  haptic_soft_i2c_status.lines_before = RawBusLines();
  haptic_soft_i2c_status.recover_result = SoftI2C_Recover(&bus);
  haptic_soft_i2c_status.lines_after = RawBusLines();
  haptic_soft_i2c_status.gpio_odr = GPIOB->ODR;
  haptic_soft_i2c_status.gpio_moder = GPIOB->MODER;
  haptic_soft_i2c_status.gpio_pupdr = GPIOB->PUPDR;
  return haptic_soft_i2c_status.recover_result == SOFT_I2C_OK && !timing_fault;
}

SoftI2CResult HapticSoftI2C_Read(uint8_t reg, uint8_t *data, uint16_t length)
{
  const SoftI2CResult result = SoftI2C_ReadRegister(&bus, MPU_ADDRESS_7BIT,
                                                reg, data, length);
  return timing_fault ? SOFT_I2C_TIMEOUT : result;
}

SoftI2CResult HapticSoftI2C_Write(uint8_t reg, uint8_t value)
{
  const SoftI2CResult result = SoftI2C_WriteRegister(&bus, MPU_ADDRESS_7BIT,
                                                 reg, &value, 1U);
  return timing_fault ? SOFT_I2C_TIMEOUT : result;
}
#endif

/* ===== Sensor sampling, servos, LEDs and button ===== */
#if !defined(HAPTIC_HOST_TEST) || defined(HAPTIC_TEST_APP)
/*
 * Assignment hardware adapter. MPU6050 acceleration and PWM are updated together
 * every 20 ms. Flight-envelope decisions use the controller section above.
 * STM32Cube-generated peripheral initialization follows below and in MSP code.
 */

#define MPU_ADDRESS (0x68U << 1)
#define MPU_ACCEL_XOUT_H 0x3BU
#define MPU_WHO_AM_I 0x75U
#define MPU_PWR_MGMT_1 0x6BU
#define MPU_ACCEL_CONFIG 0x1CU
#define SAMPLE_PERIOD_MS 20U
#define BUTTON_DEBOUNCE_MS 30U
#define SENSOR_RETRY_MS 1000U
#define I2C_TIMEOUT_MS 5U
#define I2C_ERROR_NO_ACK 0x00000004U
#define I2C_ERROR_TIMEOUT 0x00000020U
#define I2C_ERROR_GPIO_BUS 0x00010000U

HapticController haptic_controller;
HapticAppStatus haptic_app_status;
HapticHardwareI2CStatus haptic_hw_i2c_status;
static I2C_HandleTypeDef *sensor_i2c;
static TIM_HandleTypeDef *servo_timer;
static uint32_t last_sample_ms;
static uint32_t last_retry_ms;
static uint32_t button_changed_ms;
static GPIO_PinState button_candidate;
static GPIO_PinState button_stable;
static bool sensor_ready;
static bool pending_press;

static bool hw_trace_active;
static uint32_t hw_trace_flag;
static uint32_t hw_trace_isr;
static uint32_t hw_trace_cr2;

/*
 * The instrumented driver reads ISR once, just as the original HAL macro does.
 * For a pure timeout, HAL rechecks the awaited flag immediately before returning;
 * that last observation identifies the actual BUSY/TXIS/TC/RXNE/STOPF wait.
 * No registers are written and no time or serial calls occur in this observer.
 */
uint32_t HapticTraceI2CFlag(I2C_HandleTypeDef *handle, uint32_t flag)
{
  const uint32_t isr = handle->Instance->ISR;
  if (hw_trace_active)
  {
    hw_trace_flag = flag;
    hw_trace_isr = isr;
    hw_trace_cr2 = handle->Instance->CR2;
  }
  return (isr & flag) == flag ? 1U : 0U;
}

static void BeginHardwareTrace(void)
{
  hw_trace_flag = 0U;
  hw_trace_isr = sensor_i2c->Instance->ISR;
  hw_trace_cr2 = sensor_i2c->Instance->CR2;
  hw_trace_active = true;
}

/* Save the first failure before deinitialization or GPIO ownership changes. */
static void EndHardwareTrace(void)
{
  hw_trace_active = false;
  haptic_app_status.i2c_error = HAL_I2C_GetError(sensor_i2c);
  if (haptic_app_status.hal_status == HAL_OK || haptic_hw_i2c_status.captured)
  {
    return;
  }
  haptic_hw_i2c_status.error = haptic_app_status.i2c_error;
  haptic_hw_i2c_status.flag = hw_trace_flag;
  haptic_hw_i2c_status.isr = hw_trace_isr;
  haptic_hw_i2c_status.cr1 = sensor_i2c->Instance->CR1;
  haptic_hw_i2c_status.cr2 = hw_trace_cr2;
  haptic_hw_i2c_status.timingr = sensor_i2c->Instance->TIMINGR;
  haptic_hw_i2c_status.gpio_idr = GPIOB->IDR;
  haptic_hw_i2c_status.moder = GPIOB->MODER;
  haptic_hw_i2c_status.otyper = GPIOB->OTYPER;
  haptic_hw_i2c_status.pupdr = GPIOB->PUPDR;
  haptic_hw_i2c_status.afr = GPIOB->AFR[1];
  haptic_hw_i2c_status.captured = true;
}

/* Preserve real bus errors when translating the GPIO transport's result. */
static bool SoftwareResult(SoftI2CResult result)
{
  haptic_app_status.hal_status = result == SOFT_I2C_OK ? HAL_OK : HAL_ERROR;
  haptic_app_status.i2c_error = result == SOFT_I2C_OK ? 0U
      : (result == SOFT_I2C_NACK ? I2C_ERROR_NO_ACK
      : (result == SOFT_I2C_TIMEOUT ? I2C_ERROR_TIMEOUT : I2C_ERROR_GPIO_BUS));
  return result == SOFT_I2C_OK;
}

/*
 * A simulator peripheral may fail to advance a hardware status flag. Switch
 * the same PB8/PB9 bus to open-drain GPIO only after a confirmed HAL timeout.
 * This still requires a real ACK, identity byte and accelerometer data from
 * the MPU6050; it never substitutes synthetic sensor values.
 */
static bool StartSoftwareFallback(void)
{
  if ((haptic_app_status.i2c_error & I2C_ERROR_TIMEOUT) == 0U) return false;
  haptic_app_status.hardware_error = haptic_app_status.i2c_error;
  if (HAL_I2C_DeInit(sensor_i2c) != HAL_OK) return false;
  /* Once GPIO owns these pins, recovery must keep using that transport. */
  haptic_app_status.software_i2c = true;
  if (!HapticSoftI2C_Init())
  {
    return SoftwareResult(SOFT_I2C_BUS_STUCK);
  }
  return true;
}

/*
 * Read a device register with a repeated START. The memory API issues START
 * before writing TXDR, avoiding the newer generic master API's preload path.
 */
static bool Sensor_Read(uint8_t reg, uint8_t *data, uint16_t length)
{
  haptic_app_status.register_address = reg;
  if (haptic_app_status.software_i2c)
  {
    return SoftwareResult(HapticSoftI2C_Read(reg, data, length));
  }
  BeginHardwareTrace();
  haptic_app_status.hal_status = HAL_I2C_Mem_Read(sensor_i2c, MPU_ADDRESS,
      reg, I2C_MEMADD_SIZE_8BIT, data, length, I2C_TIMEOUT_MS);
  EndHardwareTrace();
  if (haptic_app_status.hal_status != HAL_OK && StartSoftwareFallback())
  {
    return SoftwareResult(HapticSoftI2C_Read(reg, data, length));
  }
  return haptic_app_status.hal_status == HAL_OK;
}

/* Write the register address followed by its value through the memory API. */
static bool Sensor_Write(uint8_t reg, uint8_t value)
{
  haptic_app_status.register_address = reg;
  if (haptic_app_status.software_i2c)
  {
    return SoftwareResult(HapticSoftI2C_Write(reg, value));
  }
  BeginHardwareTrace();
  haptic_app_status.hal_status = HAL_I2C_Mem_Write(sensor_i2c, MPU_ADDRESS,
      reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2C_TIMEOUT_MS);
  EndHardwareTrace();
  if (haptic_app_status.hal_status != HAL_OK && StartSoftwareFallback())
  {
    return SoftwareResult(HapticSoftI2C_Write(reg, value));
  }
  return haptic_app_status.hal_status == HAL_OK;
}

/* Confirm the expected device, wake it, and select +/-2g (16384 counts/g). */
static bool Sensor_Init(void)
{
  haptic_app_status.identity = 0;
  haptic_app_status.fault = HAPTIC_SENSOR_WHO_READ;
  if (!Sensor_Read(MPU_WHO_AM_I, &haptic_app_status.identity, 1)) return false;
  haptic_app_status.fault = HAPTIC_SENSOR_IDENTITY;
  if (haptic_app_status.identity != 0x68U) return false;
  haptic_app_status.fault = HAPTIC_SENSOR_WAKE;
  if (!Sensor_Write(MPU_PWR_MGMT_1, 0x00U)) return false;
  haptic_app_status.fault = HAPTIC_SENSOR_CONFIG;
  if (!Sensor_Write(MPU_ACCEL_CONFIG, 0x00U)) return false;
  /* Wait AFTER clearing sleep so a first zero sample cannot be a wake artifact. */
  HAL_Delay(100);
  haptic_app_status.fault = HAPTIC_SENSOR_NONE;
  return true;
}

/* Decode signed big-endian values without implementation-defined signed casts. */
static float Acceleration(const uint8_t *bytes)
{
  const uint32_t raw = ((uint32_t)bytes[0] << 8) | bytes[1];
  const int32_t signed_raw = raw >= 32768U ? (int32_t)raw - 65536 : (int32_t)raw;
  return (float)signed_raw / 16384.0f;
}

/*
 * Use the Arduino Servo pulse range, 544..2400 us for 0..180 degrees.
 * TIM3 counts microseconds: 48 MHz / (47+1), with a 20000-count PWM period.
 * Confirm the final visual positions in Wokwi before recording results.
 */
static uint32_t ServoPulse(float degrees)
{
  if (degrees < 0.0f) degrees = 0.0f;
  if (degrees > 180.0f) degrees = 180.0f;
  return (uint32_t)(544.0f + degrees * (1856.0f / 180.0f) + 0.5f);
}

/* PB4=roll, PB5=pitch, PB0=tilt; PA2/PB11 indicate the latched mode. */
static void ApplyOutputs(void)
{
  __HAL_TIM_SET_COMPARE(servo_timer, TIM_CHANNEL_1,
                        ServoPulse(haptic_controller.roll_servo_deg));
  __HAL_TIM_SET_COMPARE(servo_timer, TIM_CHANNEL_2,
                        ServoPulse(haptic_controller.pitch_servo_deg));
  __HAL_TIM_SET_COMPARE(servo_timer, TIM_CHANNEL_3,
                        ServoPulse(haptic_controller.tilt_servo_deg));
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2,
      haptic_controller.mode == HAPTIC_AUTOPILOT ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11,
      haptic_controller.mode == HAPTIC_MANUAL ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Debounce both edges; only a stable rising edge creates one consumable event. */
static void PollButton(uint32_t now)
{
  const GPIO_PinState raw = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
  if (raw != button_candidate)
  {
    button_candidate = raw;
    button_changed_ms = now;
  }
  if (raw != button_stable
      && (uint32_t)(now - button_changed_ms) >= BUTTON_DEBOUNCE_MS)
  {
    button_stable = raw;
    if (raw == GPIO_PIN_SET) pending_press = true;
  }
}

void HapticApp_Init(I2C_HandleTypeDef *i2c, TIM_HandleTypeDef *timer)
{
  sensor_i2c = i2c;
  servo_timer = timer;
  haptic_app_status = (HapticAppStatus){0};
  haptic_hw_i2c_status = (HapticHardwareI2CStatus){0};
  /* Before the first transaction, observe whether the released bus is high. */
  haptic_hw_i2c_status.boot_idr = GPIOB->IDR;
  hw_trace_active = false;
  Haptic_Init(&haptic_controller);
  /* A button held at boot must be released before it can generate an edge. */
  button_candidate = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
  button_stable = button_candidate;
  pending_press = false;
  ApplyOutputs();
  if (HAL_TIM_PWM_Start(timer, TIM_CHANNEL_1) != HAL_OK
      || HAL_TIM_PWM_Start(timer, TIM_CHANNEL_2) != HAL_OK
      || HAL_TIM_PWM_Start(timer, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* MPU6050 requires startup time; PWM is already at neutral while waiting. */
  HAL_Delay(100);
  sensor_ready = Sensor_Init();
  last_sample_ms = HAL_GetTick();
  last_retry_ms = last_sample_ms;
  button_changed_ms = last_sample_ms;
  if (!sensor_ready)
  {
    Haptic_Update(&haptic_controller, NULL, false, false);
    ApplyOutputs();
  }
}

void HapticApp_Poll(void)
{
  const uint32_t now = HAL_GetTick();
  uint8_t bytes[6];
  HapticAngles angles;
  bool valid = false;

  PollButton(now);
  /* Unsigned subtraction also works when the millisecond counter wraps. */
  if ((uint32_t)(now - last_sample_ms) < SAMPLE_PERIOD_MS) return;
  last_sample_ms = now;

  if (!sensor_ready && (uint32_t)(now - last_retry_ms) >= SENSOR_RETRY_MS)
  {
    haptic_app_status.fault = HAPTIC_SENSOR_RECOVERY;
    if (haptic_app_status.software_i2c)
    {
      sensor_ready = HapticSoftI2C_Init();
      if (!sensor_ready) (void)SoftwareResult(SOFT_I2C_BUS_STUCK);
    }
    else
    {
      /* Reset the HAL/peripheral state before retrying a failed transaction. */
      haptic_app_status.hal_status = HAL_I2C_DeInit(sensor_i2c);
      if (haptic_app_status.hal_status == HAL_OK)
      {
        haptic_app_status.hal_status = HAL_I2C_Init(sensor_i2c);
      }
      haptic_app_status.i2c_error = HAL_I2C_GetError(sensor_i2c);
      sensor_ready = haptic_app_status.hal_status == HAL_OK;
    }
    sensor_ready = sensor_ready && Sensor_Init();
    last_retry_ms = HAL_GetTick();
    last_sample_ms = last_retry_ms;
    /* Allow at least one sample period after waking the sensor. */
    Haptic_Update(&haptic_controller, NULL, false, false);
    pending_press = false;
    ApplyOutputs();
    return;
  }
  if (sensor_ready)
  {
    const bool read_ok = Sensor_Read(MPU_ACCEL_XOUT_H, bytes, sizeof bytes);
    valid = read_ok && Haptic_CalculateAngles(Acceleration(&bytes[0]),
                 Acceleration(&bytes[2]), Acceleration(&bytes[4]), &angles);
    haptic_app_status.fault = !read_ok ? HAPTIC_SENSOR_ACCEL_READ
        : (valid ? HAPTIC_SENSOR_NONE : HAPTIC_SENSOR_ACCEL_INVALID);
    /* A transfer failure triggers periodic re-initialization, without blocking. */
    if (!read_ok)
    {
      sensor_ready = false;
      last_retry_ms = now;
    }
  }
  Haptic_Update(&haptic_controller, valid ? &angles : NULL, valid, pending_press);
  pending_press = false;
  ApplyOutputs();
}
#endif

/* ===== UART diagnostics ===== */
#if !defined(HAPTIC_HOST_TEST) || defined(HAPTIC_TEST_DIAGNOSTICS)
/* Keep the expanded trace within 32 KB flash; bus/control code retains -O0. */
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("Os")
#endif
/*
 * Diagnostic-only USART1 transmitter. It does not change the flight-envelope
 * policy or substitute serial observations for Wokwi servo calibration tests.
 * PA9 is already wired to the serial monitor RX in diagram.json.
 */


#define DIAGNOSTIC_BAUD_RATE 115200U
#define DIAGNOSTIC_PERIOD_MS 1000U
#define TX_WAIT_TIMEOUT_MS 2U

static uint32_t last_report_ms;
static uint32_t tx_last_progress_ms;
static char tx_buffer[640];
static uint16_t tx_position;

/*
 * Drain at most one byte per main-loop pass. A full 115200-baud record takes
 * milliseconds on the wire, so waiting for all of it here would disturb the
 * 20 ms sensor cadence. An unavailable transmitter drops the record after 2 ms
 * without progress; no diagnostic code waits for a peripheral flag.
 */
static void DrainTx(uint32_t now)
{
  if (tx_buffer[tx_position] == '\0')
  {
    return;
  }
  if ((USART1->ISR & USART_ISR_TXE_TXFNF) != 0U)
  {
    USART1->TDR = (uint8_t)tx_buffer[tx_position++];
    tx_last_progress_ms = now;
  }
  else if ((uint32_t)(now - tx_last_progress_ms) >= TX_WAIT_TIMEOUT_MS)
  {
    tx_buffer[0] = '\0';
    tx_position = 0U;
  }
}

/* Keep fault names explicit so an autopilot latch is distinct from an IMU fault. */
static const char *FaultDescription(HapticSensorFault fault)
{
  switch (fault)
  {
    case HAPTIC_SENSOR_NONE:          return "none";
    case HAPTIC_SENSOR_WHO_READ:      return "WHO_AM_I read failed";
    case HAPTIC_SENSOR_IDENTITY:      return "unexpected WHO_AM_I";
    case HAPTIC_SENSOR_WAKE:          return "sensor wake write failed";
    case HAPTIC_SENSOR_CONFIG:        return "accelerometer config failed";
    case HAPTIC_SENSOR_ACCEL_READ:    return "acceleration read failed";
    case HAPTIC_SENSOR_ACCEL_INVALID: return "zero/invalid acceleration";
    case HAPTIC_SENSOR_RECOVERY:      return "I2C recovery failed";
    default:                         return "unknown sensor fault";
  }
}

/* Numeric trace fields map directly to the enums in main.h; no diagnosis is
 * inferred from the generic 0x10000 application error. Hardware fields describe
 * the first failed transfer before deinitialization. SOFT lines are raw levels
 * before/after the latest recovery, independent of the timing-fault guard. */
static void AppendBusTrace(void)
{
  const HapticHardwareI2CStatus *h = &haptic_hw_i2c_status;
  const HapticSoftI2CStatus *s = &haptic_soft_i2c_status;
  const size_t used = strlen(tx_buffer);
  (void)snprintf(tx_buffer + used, sizeof tx_buffer - used,
      "HW captured=%u err=%08lX flag=%08lX ISR=%08lX CR1=%08lX CR2=%08lX TIMINGR=%08lX\r\n"
      "PINS IDR=%08lX BOOT=%08lX MODER=%08lX OTYPER=%08lX PUPDR=%08lX AFRH=%08lX\r\n"
      "SOFT recover=%u timing=%u lines=%u/%u ODR=%08lX MODER=%08lX PUPDR=%08lX\r\n"
      "CLOCK CTRL=%08lX LOAD=%lu VAL=%lu/%lu cycles=%lu polls=%lu\r\n",
      (unsigned)h->captured, (unsigned long)h->error,
      (unsigned long)h->flag, (unsigned long)h->isr,
      (unsigned long)h->cr1, (unsigned long)h->cr2,
      (unsigned long)h->timingr, (unsigned long)h->gpio_idr,
      (unsigned long)h->boot_idr,
      (unsigned long)h->moder, (unsigned long)h->otyper,
      (unsigned long)h->pupdr, (unsigned long)h->afr,
      (unsigned)s->recover_result, (unsigned)s->timing_fault,
      (unsigned)s->lines_before, (unsigned)s->lines_after,
      (unsigned long)s->gpio_odr, (unsigned long)s->gpio_moder,
      (unsigned long)s->gpio_pupdr,
      (unsigned long)s->systick_ctrl, (unsigned long)s->systick_load,
      (unsigned long)s->val_first, (unsigned long)s->val_last,
      (unsigned long)s->elapsed_cycles, (unsigned long)s->polls);
}

/*
 * Use integer formatting to avoid newlib's optional floating-point printf code.
 * Retain a minus sign for values such as -0.1, whose integer degree part is zero.
 * Inputs are the finite, bounded orientation angles accepted by the controller.
 */
static void FormatAngle(char text[16], float degrees)
{
  const int32_t tenths = (int32_t)(degrees * 10.0f
                                  + (degrees < 0.0f ? -0.5f : 0.5f));
  const uint32_t magnitude = tenths < 0 ? (uint32_t)-tenths : (uint32_t)tenths;
  (void)snprintf(text, 16, "%s%lu.%lu", tenths < 0 ? "-" : "",
                 (unsigned long)(magnitude / 10U),
                 (unsigned long)(magnitude % 10U));
}

void HapticDiagnostics_Init(void)
{
  GPIO_InitTypeDef pin = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  /* PA9 AF1 is USART1_TX; only transmission is needed by the serial monitor. */
  pin.Pin = GPIO_PIN_9;
  pin.Mode = GPIO_MODE_AF_PP;
  pin.Pull = GPIO_NOPULL;
  pin.Speed = GPIO_SPEED_FREQ_LOW;
  pin.Alternate = GPIO_AF1_USART1;
  HAL_GPIO_Init(GPIOA, &pin);

  /* Select PCLK explicitly; the project configures it to 48 MHz. */
  CLEAR_BIT(RCC->CCIPR, RCC_CCIPR_USART1SEL);
  USART1->CR1 = 0U;
  USART1->CR2 = 0U;
  USART1->CR3 = 0U;
  USART1->BRR = (HAL_RCC_GetPCLK1Freq() + DIAGNOSTIC_BAUD_RATE / 2U)
                / DIAGNOSTIC_BAUD_RATE;
  USART1->CR1 = USART_CR1_UE | USART_CR1_TE;
  last_report_ms = HAL_GetTick();
  tx_last_progress_ms = last_report_ms;
  tx_position = 0U;
  tx_buffer[0] = '\0';
}

void HapticDiagnostics_Poll(void)
{
  const uint32_t now = HAL_GetTick();

  DrainTx(now);
  if ((uint32_t)(now - last_report_ms) < DIAGNOSTIC_PERIOD_MS)
  {
    return;
  }
  /* Finish a previous record before reusing its bounded buffer. */
  if (tx_buffer[tx_position] != '\0')
  {
    return;
  }
  last_report_ms = now;

  /* Sample the application-owned status without changing its recovery state. */
  const HapticAppStatus status = haptic_app_status;
  const char *mode = haptic_controller.mode == HAPTIC_AUTOPILOT ? "AUTO" : "MANUAL";
  const char *bus = status.software_i2c ? "GPIO" : "HW";
  if (haptic_controller.sensor_valid)
  {
    char roll[16];
    char pitch[16];
    char tilt[16];
    FormatAngle(roll, haptic_controller.angles.roll_deg);
    FormatAngle(pitch, haptic_controller.angles.pitch_deg);
    FormatAngle(tilt, haptic_controller.angles.tilt_deg);
    (void)snprintf(tx_buffer, sizeof tx_buffer,
        "mode=%s IMU=OK who=0x%02X reg=0x%02X HAL=%u err=0x%08lX bus=%s hwerr=0x%08lX roll=%s pitch=%s tilt=%s\r\n",
        mode, (unsigned)status.identity,
        (unsigned)status.register_address, (unsigned)status.hal_status,
        (unsigned long)status.i2c_error, bus,
        (unsigned long)status.hardware_error, roll, pitch, tilt);
  }
  else
  {
    (void)snprintf(tx_buffer, sizeof tx_buffer,
        "mode=%s IMU=FAULT fault=%s who=0x%02X reg=0x%02X HAL=%u err=0x%08lX bus=%s hwerr=0x%08lX\r\n",
        mode, FaultDescription(status.fault), (unsigned)status.identity,
        (unsigned)status.register_address, (unsigned)status.hal_status,
        (unsigned long)status.i2c_error, bus,
        (unsigned long)status.hardware_error);
    if (haptic_hw_i2c_status.captured || status.software_i2c)
    {
      AppendBusTrace();
    }
  }
  tx_position = 0U;
  tx_last_progress_ms = now;
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
#endif

#ifndef HAPTIC_HOST_TEST
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* Start the 50 Hz sensor/servo controller after generated peripheral setup. */
  HapticDiagnostics_Init();
  HapticApp_Init(&hi2c1, &htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HapticApp_Poll();
    HapticDiagnostics_Poll();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10805D88;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1500 ;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : VCP_USART2_RX_Pin */
  GPIO_InitStruct.Pin = VCP_USART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
  HAL_GPIO_Init(VCP_USART2_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

#endif /* !HAPTIC_HOST_TEST: generated firmware entry and peripheral setup */
