/*
 * Assignment hardware adapter. MPU6050 acceleration and PWM are updated together
 * every 20 ms. All flight-envelope decisions live in haptic_controller.c.
 * STM32Cube-generated peripheral initialization remains in main.c and MSP code.
 */
#include "haptic_app.h"
#include "haptic_soft_i2c.h"

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
static I2C_HandleTypeDef *sensor_i2c;
static TIM_HandleTypeDef *servo_timer;
static uint32_t last_sample_ms;
static uint32_t last_retry_ms;
static uint32_t button_changed_ms;
static GPIO_PinState button_candidate;
static GPIO_PinState button_stable;
static bool sensor_ready;
static bool pending_press;

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
  haptic_app_status.hal_status = HAL_I2C_Mem_Read(sensor_i2c, MPU_ADDRESS,
      reg, I2C_MEMADD_SIZE_8BIT, data, length, I2C_TIMEOUT_MS);
  haptic_app_status.i2c_error = HAL_I2C_GetError(sensor_i2c);
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
  haptic_app_status.hal_status = HAL_I2C_Mem_Write(sensor_i2c, MPU_ADDRESS,
      reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2C_TIMEOUT_MS);
  haptic_app_status.i2c_error = HAL_I2C_GetError(sensor_i2c);
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
