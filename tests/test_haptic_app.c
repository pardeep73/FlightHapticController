/* Exercise the real hardware adapter against register-aware HAL doubles. */
#include "main.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks;
static unsigned cases;
#define CHECK(condition) do { \
  ++checks; \
  if (!(condition)) { \
    fprintf(stderr, "FAIL adapter line %d: %s\n", __LINE__, #condition); \
    exit(EXIT_FAILURE); \
  } \
} while (0)
#define NEAR(actual, expected) CHECK(fabsf((actual) - (expected)) < 0.02f)

GPIO_TypeDef mock_gpio_a;
GPIO_TypeDef mock_gpio_b;
static I2C_HandleTypeDef i2c;
static I2C_TypeDef i2c_registers;
static TIM_HandleTypeDef timer;
static uint32_t tick;
static GPIO_PinState button;
static GPIO_PinState auto_led;
static GPIO_PinState manual_led;
static uint32_t pulses[3];
static unsigned started[3];
static unsigned read_count;
static unsigned sample_count;
static unsigned identity_count;
static unsigned deinit_count;
static unsigned init_count;
static uint8_t sample_bytes[6];
static uint8_t identity;
static int fail_read_register;
static int fail_write_register;
static uint32_t read_error_code;
static bool soft_init_success;
static unsigned soft_init_count;
static unsigned soft_read_count;
static unsigned soft_write_count;
static int soft_fail_read_register;
static int soft_fail_write_register;
static SoftI2CResult soft_failure;
static bool fail_recovery;
static uint32_t last_error;
static uint8_t write_registers[16];
static uint8_t write_values[16];
static unsigned write_count;
static bool sleeping;
static uint32_t wake_tick;
static bool trace_flag_checks;
static uint32_t trace_last_flag;
static bool deinit_clears_registers;

/* Encode signed raw counts into the MPU6050's high-byte-first wire format. */
static void acceleration(int x, int y, int z)
{
  const int components[] = {x, y, z};
  for (unsigned i = 0; i < 3; ++i)
  {
    const unsigned raw = (unsigned)(components[i] < 0
        ? components[i] + 65536 : components[i]);
    sample_bytes[2 * i] = (uint8_t)(raw >> 8);
    sample_bytes[2 * i + 1] = (uint8_t)raw;
  }
}

/* Model register-addressed reads and preserve the 8-bit-address bus contract. */
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *handle,
    uint16_t address, uint16_t reg, uint16_t reg_size,
    uint8_t *data, uint16_t length, uint32_t timeout)
{
  CHECK(handle == &i2c && address == (0x68U << 1));
  CHECK(reg_size == I2C_MEMADD_SIZE_8BIT && timeout == 5U);
  ++read_count;
  if ((int)reg == fail_read_register)
  {
    if (trace_flag_checks)
    {
      i2c_registers.ISR = 0x00008003U;
      CHECK(HapticTraceI2CFlag(handle, 0x02U) == 1U);
      i2c_registers.ISR = 0x00008002U;
      CHECK(HapticTraceI2CFlag(handle, trace_last_flag) == 0U);
    }
    last_error = read_error_code; /* Inject an actual HAL error bitmask. */
    return HAL_ERROR;
  }
  last_error = 0;
  if (reg == 0x75U)
  {
    CHECK(length == 1);
    ++identity_count;
    *data = identity;
  }
  else
  {
    CHECK(reg == 0x3BU && length == sizeof sample_bytes);
    ++sample_count;
    /* A sleeping or newly woken sensor cannot provide valid gravity yet. */
    if (sleeping || (uint32_t)(tick - wake_tick) < 100U)
      memset(data, 0, length);
    else
      memcpy(data, sample_bytes, sizeof sample_bytes);
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *handle,
    uint16_t address, uint16_t reg, uint16_t reg_size,
    uint8_t *data, uint16_t length, uint32_t timeout)
{
  CHECK(handle == &i2c && address == (0x68U << 1));
  CHECK(reg_size == I2C_MEMADD_SIZE_8BIT && timeout == 5U && length == 1);
  CHECK(reg == 0x6BU || reg == 0x1CU);
  if ((int)reg == fail_write_register)
  {
    last_error = 0x20U; /* HAL_I2C_ERROR_TIMEOUT. */
    return HAL_TIMEOUT;
  }
  last_error = 0;
  CHECK(write_count < sizeof write_values);
  write_registers[write_count] = (uint8_t)reg;
  write_values[write_count++] = *data;
  if (reg == 0x6BU)
  {
    sleeping = (*data & 0x40U) != 0;
    wake_tick = tick;
  }
  return HAL_OK;
}

uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *handle)
{
  CHECK(handle == &i2c);
  return last_error;
}

HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *handle)
{
  CHECK(handle == &i2c);
  ++deinit_count;
  if (deinit_clears_registers)
  {
    memset(&i2c_registers, 0, sizeof i2c_registers);
    memset(&mock_gpio_b, 0, sizeof mock_gpio_b);
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *handle)
{
  CHECK(handle == &i2c);
  ++init_count;
  last_error = fail_recovery ? 0x20U : 0U;
  return fail_recovery ? HAL_ERROR : HAL_OK;
}

/* Software-bus doubles test adapter selection, not electrical bus waveforms. */
bool HapticSoftI2C_Init(void)
{
  ++soft_init_count;
  return soft_init_success;
}

SoftI2CResult HapticSoftI2C_Read(uint8_t reg, uint8_t *data, uint16_t length)
{
  ++soft_read_count;
  if ((int)reg == soft_fail_read_register) return soft_failure;
  if (reg == 0x75U)
  {
    CHECK(length == 1);
    *data = identity;
  }
  else
  {
    CHECK(reg == 0x3BU && length == sizeof sample_bytes);
    if (sleeping || (uint32_t)(tick - wake_tick) < 100U)
      memset(data, 0, length);
    else
      memcpy(data, sample_bytes, sizeof sample_bytes);
  }
  return SOFT_I2C_OK;
}

SoftI2CResult HapticSoftI2C_Write(uint8_t reg, uint8_t value)
{
  ++soft_write_count;
  CHECK(reg == 0x6BU || reg == 0x1CU);
  if ((int)reg == soft_fail_write_register) return soft_failure;
  CHECK(write_count < sizeof write_values);
  write_registers[write_count] = reg;
  write_values[write_count++] = value;
  if (reg == 0x6BU)
  {
    sleeping = (value & 0x40U) != 0;
    wake_tick = tick;
  }
  return SOFT_I2C_OK;
}

static unsigned channel_index(uint32_t channel)
{
  CHECK(channel == TIM_CHANNEL_1 || channel == TIM_CHANNEL_2
        || channel == TIM_CHANNEL_3);
  return channel / 4U;
}

void Mock_SetCompare(TIM_HandleTypeDef *handle, uint32_t channel, uint32_t pulse)
{
  CHECK(handle == &timer);
  CHECK(pulse >= 544U && pulse <= 2400U);
  pulses[channel_index(channel)] = pulse;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *handle, uint32_t channel)
{
  CHECK(handle == &timer);
  ++started[channel_index(channel)];
  return HAL_OK;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
  CHECK(port == GPIOA && pin == GPIO_PIN_12);
  return button;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
  if (port == GPIOA && pin == GPIO_PIN_2) auto_led = state;
  else if (port == GPIOB && pin == GPIO_PIN_11) manual_led = state;
  else CHECK(false);
}

uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t delay) { tick += delay; }
void Error_Handler(void) { CHECK(false); }

/* Reset the doubles; HapticApp_Init resets the adapter's own static state. */
static void reset(GPIO_PinState initial_button)
{
  tick = 0;
  memset(&i2c_registers, 0, sizeof i2c_registers);
  i2c.Instance = &i2c_registers;
  memset(&mock_gpio_b, 0, sizeof mock_gpio_b);
  button = initial_button;
  auto_led = manual_led = GPIO_PIN_RESET;
  memset(pulses, 0, sizeof pulses);
  memset(started, 0, sizeof started);
  read_count = sample_count = identity_count = 0;
  deinit_count = init_count = 0;
  identity = 0x68U;
  fail_read_register = fail_write_register = -1;
  read_error_code = 0x04U;
  soft_init_success = false;
  soft_init_count = soft_read_count = soft_write_count = 0;
  soft_fail_read_register = soft_fail_write_register = -1;
  soft_failure = SOFT_I2C_NACK;
  fail_recovery = false;
  last_error = 0;
  write_count = 0;
  sleeping = true;
  wake_tick = 0;
  trace_flag_checks = false;
  trace_last_flag = 0x40U;
  deinit_clears_registers = false;
  acceleration(0, 0, 16384);
}

/* Advance a monotonic uint32_t clock, including natural wraparound. */
static void after(uint32_t elapsed)
{
  tick += elapsed;
  HapticApp_Poll();
}

static void check_manual(void)
{
  CHECK(haptic_controller.mode == HAPTIC_MANUAL);
  CHECK(auto_led == GPIO_PIN_RESET && manual_led == GPIO_PIN_SET);
}

static void check_auto(void)
{
  CHECK(haptic_controller.mode == HAPTIC_AUTOPILOT);
  CHECK(auto_led == GPIO_PIN_SET && manual_led == GPIO_PIN_RESET);
}

static void test_initialization_and_wake(void)
{
  reset(GPIO_PIN_RESET);
  HapticApp_Init(&i2c, &timer);
  CHECK(started[0] == 1 && started[1] == 1 && started[2] == 1);
  CHECK(identity_count == 1 && sample_count == 0);
  CHECK(read_count == 1 && write_count == 2 && !sleeping);
  CHECK(write_registers[0] == 0x6BU && write_values[0] == 0x00U);
  CHECK(write_registers[1] == 0x1CU && write_values[1] == 0x00U);
  CHECK((uint32_t)(tick - wake_tick) >= 100U);
  CHECK(pulses[0] == 1472U && pulses[1] == 1472U && pulses[2] == 544U);
  check_manual();
  after(20);
  CHECK(haptic_controller.sensor_valid);
  check_manual();
  ++cases;
}

static void test_sample_rate(void)
{
  reset(GPIO_PIN_RESET);
  HapticApp_Init(&i2c, &timer);
  after(19); CHECK(sample_count == 0);
  after(1); CHECK(sample_count == 1 && haptic_controller.sensor_valid);
  after(19); CHECK(sample_count == 1);
  after(1); CHECK(sample_count == 2);
  /* A delayed poll samples once instead of issuing a stale catch-up burst. */
  after(60); CHECK(sample_count == 3);
  after(0); CHECK(sample_count == 3);
  ++cases;
}

static void test_signed_samples_and_outputs(void)
{
  reset(GPIO_PIN_RESET);
  HapticApp_Init(&i2c, &timer);
  acceleration(0, 11585, 11585); after(20);
  NEAR(haptic_controller.angles.roll_deg, 45.0f);
  NEAR(haptic_controller.angles.pitch_deg, 0.0f);
  CHECK(pulses[0] == 1008U && pulses[1] == 1472U && pulses[2] == 1472U);
  acceleration(0, -11585, 11585); after(20);
  NEAR(haptic_controller.angles.roll_deg, -45.0f);
  CHECK(pulses[0] == 1936U);
  acceleration(-11585, 0, 11585); after(20);
  NEAR(haptic_controller.angles.pitch_deg, 45.0f);
  CHECK(pulses[0] == 1472U && pulses[1] == 1008U);
  acceleration(11585, 0, 11585); after(20);
  NEAR(haptic_controller.angles.pitch_deg, -45.0f);
  CHECK(pulses[1] == 1936U);
  /* Approximate 1g vector: roll=+60, pitch=-60; both axes clamp. */
  acceleration(14189, 7094, 4096); after(20);
  NEAR(haptic_controller.angles.roll_deg, 60.0f);
  NEAR(haptic_controller.angles.pitch_deg, -60.0f);
  CHECK(pulses[0] == 956U && pulses[1] == 1988U && pulses[2] == 1988U);
  check_auto();
  ++cases;
}

static void test_button_bounce_hold_and_release(void)
{
  reset(GPIO_PIN_RESET);
  HapticApp_Init(&i2c, &timer);
  button = GPIO_PIN_SET; after(1);
  button = GPIO_PIN_RESET; after(9);
  button = GPIO_PIN_SET; after(10);
  button = GPIO_PIN_RESET; after(5);
  button = GPIO_PIN_SET; after(5);
  after(29); check_manual();
  after(1); check_manual(); /* Event waits for the next sensor update. */
  after(19); check_auto();
  after(21); check_auto();
  button = GPIO_PIN_RESET; after(1);
  after(30);
  button = GPIO_PIN_SET; after(1);
  after(30); check_manual();
  after(40); check_manual();
  ++cases;
}

static void test_button_held_at_boot(void)
{
  reset(GPIO_PIN_SET);
  HapticApp_Init(&i2c, &timer);
  after(40); check_manual();
  button = GPIO_PIN_RESET; after(1);
  after(30);
  button = GPIO_PIN_SET; after(1);
  after(30); check_auto();
  ++cases;
}

static void test_unsafe_press_is_consumed(void)
{
  reset(GPIO_PIN_RESET);
  HapticApp_Init(&i2c, &timer);
  acceleration(0, 10531, 12551); /* Approximately 40-degree roll, 1g. */
  button = GPIO_PIN_SET; after(1);
  after(30); check_auto();
  button = GPIO_PIN_RESET; after(1);
  after(30);
  button = GPIO_PIN_SET; after(1);
  after(30); check_auto();
  button = GPIO_PIN_RESET; after(1);
  after(30);
  acceleration(0, 0, 16384);
  after(20); check_auto();
  button = GPIO_PIN_SET; after(1);
  after(30); check_manual();
  ++cases;
}

static void test_transfer_failure_retry_and_manual_recovery(void)
{
  reset(GPIO_PIN_RESET);
  HapticApp_Init(&i2c, &timer);
  fail_read_register = 0x3BU;
  after(20);
  CHECK(!haptic_controller.sensor_valid);
  check_auto();
  CHECK(pulses[0] == 1472U && pulses[1] == 1472U && pulses[2] == 544U);
  fail_read_register = -1;
  after(980); CHECK(identity_count == 1 && sample_count == 0);
  after(20);
  CHECK(deinit_count == 1 && init_count == 1);
  CHECK(identity_count == 2 && sample_count == 0);
  CHECK((uint32_t)(tick - wake_tick) >= 100U);
  CHECK(!haptic_controller.sensor_valid);
  after(20);
  CHECK(sample_count == 1 && haptic_controller.sensor_valid);
  check_auto(); /* Sensor recovery alone must not clear the mode latch. */
  button = GPIO_PIN_SET; after(1);
  after(30); check_manual();
  ++cases;
}

static void test_zero_sample_latch_and_safe_release(void)
{
  reset(GPIO_PIN_RESET);
  HapticApp_Init(&i2c, &timer);
  acceleration(0, 0, 0); after(20);
  CHECK(!haptic_controller.sensor_valid); check_auto();
  acceleration(0, 0, 16384); after(20);
  CHECK(haptic_controller.sensor_valid); check_auto();
  button = GPIO_PIN_SET; after(1);
  after(30); check_manual();
  ++cases;
}

static void test_tick_wrap(void)
{
  reset(GPIO_PIN_RESET);
  HapticApp_Init(&i2c, &timer);
  /* Move forward to just before wrap; the adapter takes one current sample. */
  tick = UINT32_MAX - 10U; HapticApp_Poll();
  const unsigned previous_samples = sample_count;
  after(19); CHECK(sample_count == previous_samples);
  after(1); CHECK(sample_count == previous_samples + 1);
  /* Separate run verifies the 30ms debounce subtraction across the boundary. */
  reset(GPIO_PIN_RESET);
  HapticApp_Init(&i2c, &timer);
  tick = UINT32_MAX - 10U; HapticApp_Poll();
  button = GPIO_PIN_SET; after(1);
  after(30); check_auto();
  ++cases;
}

/* Diagnostic stage checks are below; they share the same register-aware doubles. */
static void test_identity_read_failure_diagnostics(void)
{
  reset(GPIO_PIN_RESET);
  fail_read_register = 0x75U;
  HapticApp_Init(&i2c, &timer);
  CHECK(haptic_app_status.fault == HAPTIC_SENSOR_WHO_READ);
  CHECK(haptic_app_status.register_address == 0x75U);
  CHECK(haptic_app_status.identity == 0);
  CHECK(haptic_app_status.hal_status == HAL_ERROR);
  CHECK(haptic_app_status.i2c_error == 0x04U);
  CHECK(write_count == 0 && !haptic_controller.sensor_valid);
  check_auto();
  after(20);
  CHECK(haptic_app_status.fault == HAPTIC_SENSOR_WHO_READ);
  ++cases;
}

static void test_wrong_identity_diagnostics(void)
{
  reset(GPIO_PIN_RESET);
  identity = 0x22U;
  HapticApp_Init(&i2c, &timer);
  CHECK(haptic_app_status.fault == HAPTIC_SENSOR_IDENTITY);
  CHECK(haptic_app_status.register_address == 0x75U);
  CHECK(haptic_app_status.identity == 0x22U);
  CHECK(haptic_app_status.hal_status == HAL_OK);
  CHECK(haptic_app_status.i2c_error == 0);
  CHECK(write_count == 0);
  check_auto();
  ++cases;
}

static void test_wake_failure_diagnostics(void)
{
  reset(GPIO_PIN_RESET);
  fail_write_register = 0x6BU;
  HapticApp_Init(&i2c, &timer);
  CHECK(haptic_app_status.fault == HAPTIC_SENSOR_WAKE);
  CHECK(haptic_app_status.register_address == 0x6BU);
  CHECK(haptic_app_status.identity == 0x68U);
  CHECK(haptic_app_status.hal_status == HAL_ERROR);
  CHECK(haptic_app_status.i2c_error == 0x10000U);
  CHECK(haptic_app_status.hardware_error == 0x20U);
  CHECK(haptic_app_status.software_i2c && soft_init_count == 1);
  CHECK(write_count == 0 && sleeping);
  check_auto();
  ++cases;
}

static void test_config_failure_diagnostics(void)
{
  reset(GPIO_PIN_RESET);
  fail_write_register = 0x1CU;
  HapticApp_Init(&i2c, &timer);
  CHECK(haptic_app_status.fault == HAPTIC_SENSOR_CONFIG);
  CHECK(haptic_app_status.register_address == 0x1CU);
  CHECK(haptic_app_status.identity == 0x68U);
  CHECK(haptic_app_status.hal_status == HAL_ERROR);
  CHECK(haptic_app_status.i2c_error == 0x10000U);
  CHECK(haptic_app_status.hardware_error == 0x20U);
  CHECK(haptic_app_status.software_i2c && soft_init_count == 1);
  CHECK(write_count == 1);
  check_auto();
  ++cases;
}

static void test_accel_read_and_invalid_diagnostics(void)
{
  reset(GPIO_PIN_RESET);
  HapticApp_Init(&i2c, &timer);
  CHECK(haptic_app_status.fault == HAPTIC_SENSOR_NONE);
  CHECK(haptic_app_status.identity == 0x68U);
  fail_read_register = 0x3BU;
  after(20);
  CHECK(haptic_app_status.fault == HAPTIC_SENSOR_ACCEL_READ);
  CHECK(haptic_app_status.register_address == 0x3BU);
  CHECK(haptic_app_status.hal_status == HAL_ERROR);
  CHECK(haptic_app_status.i2c_error == 0x04U);
  check_auto();
  reset(GPIO_PIN_RESET);
  HapticApp_Init(&i2c, &timer);
  acceleration(0, 0, 0); after(20);
  CHECK(haptic_app_status.fault == HAPTIC_SENSOR_ACCEL_INVALID);
  CHECK(haptic_app_status.register_address == 0x3BU);
  CHECK(haptic_app_status.hal_status == HAL_OK);
  CHECK(haptic_app_status.i2c_error == 0);
  acceleration(0, 0, 16384); after(20);
  CHECK(haptic_app_status.fault == HAPTIC_SENSOR_NONE);
  CHECK(haptic_controller.sensor_valid);
  check_auto(); /* Diagnostic health and mode latch are independent. */
  ++cases;
}

static void test_recovery_failure_and_next_retry(void)
{
  reset(GPIO_PIN_RESET);
  HapticApp_Init(&i2c, &timer);
  fail_read_register = 0x3BU; after(20);
  fail_read_register = -1;
  fail_recovery = true; after(1000);
  CHECK(deinit_count == 1 && init_count == 1 && identity_count == 1);
  CHECK(haptic_app_status.fault == HAPTIC_SENSOR_RECOVERY);
  CHECK(haptic_app_status.hal_status == HAL_ERROR);
  CHECK(haptic_app_status.i2c_error == 0x20U);
  CHECK(!haptic_controller.sensor_valid);
  check_auto();
  fail_recovery = false; after(1000);
  CHECK(deinit_count == 2 && init_count == 2 && identity_count == 2);
  CHECK(haptic_app_status.fault == HAPTIC_SENSOR_NONE);
  after(20);
  CHECK(haptic_controller.sensor_valid);
  check_auto();
  ++cases;
}

/* A hardware progress timeout may use the same PB8/PB9 bus through GPIO. */
static void test_timeout_fallback_reads_real_device(void)
{
  reset(GPIO_PIN_RESET);
  fail_read_register = 0x75U;
  read_error_code = 0x20U;
  soft_init_success = true;
  HapticApp_Init(&i2c, &timer);
  CHECK(haptic_app_status.software_i2c);
  CHECK(haptic_app_status.hardware_error == 0x20U);
  CHECK(haptic_app_status.fault == HAPTIC_SENSOR_NONE);
  CHECK(haptic_app_status.identity == 0x68U);
  CHECK(haptic_app_status.hal_status == HAL_OK);
  CHECK(haptic_app_status.i2c_error == 0U);
  CHECK(deinit_count == 1 && init_count == 0);
  CHECK(soft_init_count == 1 && soft_read_count == 1 && soft_write_count == 2);
  CHECK(read_count == 1 && write_count == 2 && !sleeping);
  CHECK((uint32_t)(tick - wake_tick) >= 100U);
  after(20);
  CHECK(haptic_controller.sensor_valid);
  check_manual();
  acceleration(0, 11585, 11585); after(20);
  NEAR(haptic_controller.angles.roll_deg, 45.0f);
  CHECK(pulses[0] == 1008U && pulses[1] == 1472U);
  CHECK(read_count == 1 && soft_read_count == 3);
  ++cases;
}

/* A normal address NACK is a device error, not an emulator progress failure. */
static void test_nack_does_not_enable_fallback(void)
{
  reset(GPIO_PIN_RESET);
  fail_read_register = 0x75U;
  read_error_code = 0x04U;
  soft_init_success = true;
  HapticApp_Init(&i2c, &timer);
  CHECK(!haptic_app_status.software_i2c);
  CHECK(soft_init_count == 0 && soft_read_count == 0);
  CHECK(haptic_app_status.fault == HAPTIC_SENSOR_WHO_READ);
  CHECK(haptic_app_status.i2c_error == 0x04U);
  CHECK(!haptic_controller.sensor_valid);
  check_auto();
  ++cases;
}

/* An absent sensor on the fallback bus must still cause a genuine safe fault. */
static void test_software_bus_failure_and_safe_recovery(void)
{
  reset(GPIO_PIN_RESET);
  fail_read_register = 0x75U;
  read_error_code = 0x20U;
  soft_init_success = true;
  HapticApp_Init(&i2c, &timer);
  after(20);
  CHECK(haptic_controller.sensor_valid);
  soft_fail_read_register = 0x3BU;
  soft_failure = SOFT_I2C_NACK;
  after(20);
  CHECK(haptic_app_status.fault == HAPTIC_SENSOR_ACCEL_READ);
  CHECK(haptic_app_status.i2c_error == 0x04U);
  CHECK(!haptic_controller.sensor_valid);
  check_auto();
  CHECK(pulses[0] == 1472U && pulses[1] == 1472U && pulses[2] == 544U);
  soft_fail_read_register = -1;
  after(1000);
  CHECK(soft_init_count == 2 && deinit_count == 1 && init_count == 0);
  after(20);
  CHECK(haptic_controller.sensor_valid);
  check_auto();
  button = GPIO_PIN_SET; after(1);
  after(30); check_manual();
  ++cases;
}

static void test_failed_fallback_remains_faulted_until_bus_recovers(void)
{
  reset(GPIO_PIN_RESET);
  fail_read_register = 0x75U;
  read_error_code = 0x20U;
  HapticApp_Init(&i2c, &timer);
  CHECK(haptic_app_status.software_i2c);
  CHECK(haptic_app_status.hardware_error == 0x20U);
  CHECK(haptic_app_status.fault == HAPTIC_SENSOR_WHO_READ);
  CHECK(haptic_app_status.i2c_error == 0x10000U);
  CHECK(haptic_app_status.identity == 0U);
  CHECK(!haptic_controller.sensor_valid && soft_read_count == 0);
  check_auto();
  soft_init_success = true;
  after(1000);
  CHECK(soft_init_count == 2 && deinit_count == 1 && init_count == 0);
  CHECK(haptic_app_status.identity == 0x68U);
  after(20); CHECK(haptic_controller.sensor_valid);
  check_auto();
  button = GPIO_PIN_SET; after(1);
  after(30); check_manual();
  ++cases;
}

static void test_fallback_requires_correct_identity(void)
{
  reset(GPIO_PIN_RESET);
  fail_read_register = 0x75U;
  read_error_code = 0x20U;
  soft_init_success = true;
  identity = 0x12U;
  HapticApp_Init(&i2c, &timer);
  CHECK(haptic_app_status.software_i2c);
  CHECK(haptic_app_status.fault == HAPTIC_SENSOR_IDENTITY);
  CHECK(haptic_app_status.identity == 0x12U);
  CHECK(soft_read_count == 1 && soft_write_count == 0);
  CHECK(!haptic_controller.sensor_valid);
  check_auto();
  ++cases;
}

static void test_software_error_translation(void)
{
  const SoftI2CResult failures[] = {SOFT_I2C_TIMEOUT, SOFT_I2C_BUS_STUCK};
  const uint32_t expected[] = {0x20U, 0x10000U};
  for (unsigned i = 0; i < sizeof failures / sizeof failures[0]; ++i)
  {
    reset(GPIO_PIN_RESET);
    fail_read_register = 0x75U;
    read_error_code = 0x20U;
    soft_init_success = true;
    HapticApp_Init(&i2c, &timer);
    soft_fail_read_register = 0x3BU;
    soft_failure = failures[i];
    after(20);
    CHECK(haptic_app_status.fault == HAPTIC_SENSOR_ACCEL_READ);
    CHECK(haptic_app_status.i2c_error == expected[i]);
    CHECK(haptic_app_status.hal_status == HAL_ERROR);
    CHECK(!haptic_controller.sensor_valid);
    check_auto();
  }
  ++cases;
}

static void test_write_timeout_can_switch_to_software_bus(void)
{
  const int registers[] = {0x6BU, 0x1CU};
  for (unsigned i = 0; i < sizeof registers / sizeof registers[0]; ++i)
  {
    reset(GPIO_PIN_RESET);
    fail_write_register = registers[i];
    soft_init_success = true;
    HapticApp_Init(&i2c, &timer);
    CHECK(haptic_app_status.software_i2c);
    CHECK(haptic_app_status.fault == HAPTIC_SENSOR_NONE);
    CHECK(haptic_app_status.hardware_error == 0x20U);
    CHECK(haptic_app_status.identity == 0x68U);
    CHECK(write_count == 2 && soft_write_count == 2U - i);
    CHECK(identity_count == 1 && soft_read_count == 0);
    after(20); CHECK(haptic_controller.sensor_valid);
    check_manual();
  }
  ++cases;
}

static void test_hardware_snapshot_precedes_fallback_deinit(void)
{
  reset(GPIO_PIN_RESET);
  i2c_registers.CR1 = 0x00001001U;
  i2c_registers.CR2 = 0x000120D0U;
  i2c_registers.TIMINGR = 0x10805D88U;
  mock_gpio_b.IDR = 0x00000300U;
  mock_gpio_b.MODER = 0x000A0000U;
  mock_gpio_b.OTYPER = 0x00000300U;
  mock_gpio_b.PUPDR = 0x00050000U;
  mock_gpio_b.AFR[1] = 0x00000066U;
  fail_read_register = 0x75U;
  read_error_code = 0x20U;
  trace_flag_checks = true;
  trace_last_flag = 0x40U;
  deinit_clears_registers = true;
  soft_init_success = true;
  HapticApp_Init(&i2c, &timer);
  CHECK(haptic_hw_i2c_status.captured);
  CHECK(haptic_hw_i2c_status.error == 0x20U);
  CHECK(haptic_hw_i2c_status.flag == 0x40U);
  CHECK(haptic_hw_i2c_status.isr == 0x00008002U);
  CHECK(haptic_hw_i2c_status.cr1 == 0x00001001U);
  CHECK(haptic_hw_i2c_status.cr2 == 0x000120D0U);
  CHECK(haptic_hw_i2c_status.timingr == 0x10805D88U);
  CHECK(haptic_hw_i2c_status.gpio_idr == 0x00000300U);
  CHECK(haptic_hw_i2c_status.boot_idr == 0x00000300U);
  CHECK(haptic_hw_i2c_status.moder == 0x000A0000U);
  CHECK(haptic_hw_i2c_status.otyper == 0x00000300U);
  CHECK(haptic_hw_i2c_status.pupdr == 0x00050000U);
  CHECK(haptic_hw_i2c_status.afr == 0x00000066U);
  CHECK(i2c_registers.CR1 == 0 && mock_gpio_b.MODER == 0);
  CHECK(haptic_app_status.software_i2c);
  ++cases;
}

static void test_hardware_snapshot_is_first_failure_and_init_resets(void)
{
  reset(GPIO_PIN_RESET);
  fail_read_register = 0x75U;
  trace_flag_checks = true;
  trace_last_flag = 0x40U;
  HapticApp_Init(&i2c, &timer);
  CHECK(haptic_hw_i2c_status.captured);
  CHECK(haptic_hw_i2c_status.error == 0x04U);
  CHECK(haptic_hw_i2c_status.flag == 0x40U);
  trace_last_flag = 0x04U;
  i2c_registers.CR1 = 0x77U;
  after(1000);
  CHECK(haptic_hw_i2c_status.flag == 0x40U);
  CHECK(haptic_hw_i2c_status.cr1 == 0U);
  /* Outside an active hardware call, the observer only preserves HAL predicates. */
  i2c_registers.ISR = 0x43U;
  CHECK(HapticTraceI2CFlag(&i2c, 0x41U) == 1U);
  CHECK(HapticTraceI2CFlag(&i2c, 0x44U) == 0U);
  CHECK(haptic_hw_i2c_status.flag == 0x40U);
  reset(GPIO_PIN_RESET);
  HapticApp_Init(&i2c, &timer);
  CHECK(!haptic_hw_i2c_status.captured);
  after(20);
  CHECK(!haptic_hw_i2c_status.captured);
  ++cases;
}

int main(void)
{
  test_initialization_and_wake();
  test_sample_rate();
  test_signed_samples_and_outputs();
  test_button_bounce_hold_and_release();
  test_button_held_at_boot();
  test_unsafe_press_is_consumed();
  test_transfer_failure_retry_and_manual_recovery();
  test_zero_sample_latch_and_safe_release();
  test_tick_wrap();
  test_identity_read_failure_diagnostics();
  test_wrong_identity_diagnostics();
  test_wake_failure_diagnostics();
  test_config_failure_diagnostics();
  test_accel_read_and_invalid_diagnostics();
  test_recovery_failure_and_next_retry();
  test_timeout_fallback_reads_real_device();
  test_nack_does_not_enable_fallback();
  test_software_bus_failure_and_safe_recovery();
  test_failed_fallback_remains_faulted_until_bus_recovers();
  test_fallback_requires_correct_identity();
  test_software_error_translation();
  test_write_timeout_can_switch_to_software_bus();
  test_hardware_snapshot_precedes_fallback_deinit();
  test_hardware_snapshot_is_first_failure_and_init_resets();
  printf("PASS: %u adapter cases, %u checks (register I2C, wake, PWM, debounce, recovery, diagnostics)\n",
         cases, checks);
  return EXIT_SUCCESS;
}
