/*
 * Assignment hardware adapter. MPU6050 acceleration and PWM are updated together
 * every 20 ms. All flight-envelope decisions live in haptic_controller.c.
 * STM32Cube-generated peripheral initialization remains in main.c and MSP code.
 */
#include "haptic_app.h"

#define MPU_ADDRESS (0x68U << 1)
#define MPU_ACCEL_XOUT_H 0x3BU
#define MPU_WHO_AM_I 0x75U
#define MPU_PWR_MGMT_1 0x6BU
#define MPU_ACCEL_CONFIG 0x1CU
#define SAMPLE_PERIOD_MS 20U
#define BUTTON_DEBOUNCE_MS 30U
#define SENSOR_RETRY_MS 1000U
#define I2C_TIMEOUT_MS 5U

HapticController haptic_controller;
static I2C_HandleTypeDef *sensor_i2c;
static TIM_HandleTypeDef *servo_timer;
static uint32_t last_sample_ms;
static uint32_t last_retry_ms;
static uint32_t button_changed_ms;
static GPIO_PinState button_candidate;
static GPIO_PinState button_stable;
static bool sensor_ready;
static bool pending_press;

/* Read a burst through a register-selection write and subsequent read. */
static bool Sensor_Read(uint8_t reg, uint8_t *data, uint16_t length)
{
  return HAL_I2C_Master_Transmit(sensor_i2c, MPU_ADDRESS, &reg, 1,
                                  I2C_TIMEOUT_MS) == HAL_OK
      && HAL_I2C_Master_Receive(sensor_i2c, MPU_ADDRESS, data, length,
                                  I2C_TIMEOUT_MS) == HAL_OK;
}

/* Writes include register address and value in one I2C transaction. */
static bool Sensor_Write(uint8_t reg, uint8_t value)
{
  uint8_t data[2] = {reg, value};
  return HAL_I2C_Master_Transmit(sensor_i2c, MPU_ADDRESS, data, 2,
                                  I2C_TIMEOUT_MS) == HAL_OK;
}

/* Confirm the expected device, wake it, and select +/-2g (16384 counts/g). */
static bool Sensor_Init(void)
{
  uint8_t identity = 0;
  return Sensor_Read(MPU_WHO_AM_I, &identity, 1) && identity == 0x68U
      && Sensor_Write(MPU_PWR_MGMT_1, 0x00U)
      && Sensor_Write(MPU_ACCEL_CONFIG, 0x00U);
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
    last_retry_ms = now;
    sensor_ready = Sensor_Init();
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
