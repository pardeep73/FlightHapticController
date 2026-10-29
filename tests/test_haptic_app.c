/* Exercise the real hardware adapter against controlled I2C/GPIO/timer doubles. */
#include "haptic_app.h"
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
static TIM_HandleTypeDef timer;
static uint32_t tick;
static GPIO_PinState button;
static GPIO_PinState auto_led;
static GPIO_PinState manual_led;
static uint32_t pulses[3];
static unsigned started[3];
static unsigned transmit_count;
static unsigned receive_count;
static unsigned sample_count;
static unsigned identity_count;
static uint8_t selected_register;
static uint8_t sample_bytes[6];
static uint8_t identity;
static bool fail_transmit;
static bool fail_receive;
static uint8_t write_registers[16];
static uint8_t write_values[16];
static unsigned write_count;

/* Encode a signed raw count into the MPU6050's high-byte-first wire format. */
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

/* Each mock asserts the hardware contract instead of accepting arbitrary calls. */
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *handle,
    uint16_t address, uint8_t *data, uint16_t length, uint32_t timeout)
{
  CHECK(handle == &i2c && address == (0x68U << 1));
  CHECK(timeout == 5U && (length == 1 || length == 2));
  ++transmit_count;
  if (fail_transmit) return HAL_ERROR;
  selected_register = data[0];
  if (length == 2)
  {
    CHECK(write_count < sizeof write_values);
    write_registers[write_count] = data[0];
    write_values[write_count++] = data[1];
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *handle,
    uint16_t address, uint8_t *data, uint16_t length, uint32_t timeout)
{
  CHECK(handle == &i2c && address == (0x68U << 1));
  CHECK(timeout == 5U);
  ++receive_count;
  if (fail_receive) return HAL_ERROR;
  if (selected_register == 0x75U)
  {
    CHECK(length == 1);
    ++identity_count;
    *data = identity;
  }
  else
  {
    CHECK(selected_register == 0x3BU && length == sizeof sample_bytes);
    ++sample_count;
    memcpy(data, sample_bytes, sizeof sample_bytes);
  }
  return HAL_OK;
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

/* Reset the doubles; HapticApp_Init reinitializes the adapter's own static state. */
static void reset(uint32_t initial_tick, GPIO_PinState initial_button)
{
  tick = initial_tick;
  button = initial_button;
  auto_led = manual_led = GPIO_PIN_RESET;
  memset(pulses, 0, sizeof pulses);
  memset(started, 0, sizeof started);
  transmit_count = receive_count = sample_count = identity_count = 0;
  selected_register = 0;
  identity = 0x68U;
  fail_transmit = fail_receive = false;
  write_count = 0;
  acceleration(0, 0, 16384);
}

static void poll_at(uint32_t now)
{
  tick = now;
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

static void test_initialization(void)
{
  reset(0, GPIO_PIN_RESET);
  HapticApp_Init(&i2c, &timer);
  CHECK(tick == 100U);
  CHECK(started[0] == 1 && started[1] == 1 && started[2] == 1);
  CHECK(identity_count == 1 && sample_count == 0);
  CHECK(transmit_count == 3 && receive_count == 1 && write_count == 2);
  CHECK(write_registers[0] == 0x6BU && write_values[0] == 0x00U);
  CHECK(write_registers[1] == 0x1CU && write_values[1] == 0x00U);
  CHECK(pulses[0] == 1472U && pulses[1] == 1472U && pulses[2] == 544U);
  check_manual();
  ++cases;
}

static void test_sample_rate(void)
{
  reset(0, GPIO_PIN_RESET);
  HapticApp_Init(&i2c, &timer);
  poll_at(119);
  CHECK(sample_count == 0);
  poll_at(120);
  CHECK(sample_count == 1 && haptic_controller.sensor_valid);
  poll_at(139);
  CHECK(sample_count == 1);
  poll_at(140);
  CHECK(sample_count == 2);
  /* A delayed poll samples once rather than issuing a burst of stale updates. */
  poll_at(200);
  CHECK(sample_count == 3);
  poll_at(200);
  CHECK(sample_count == 3);
  ++cases;
}

static void test_signed_samples_and_outputs(void)
{
  reset(0, GPIO_PIN_RESET);
  HapticApp_Init(&i2c, &timer);
  acceleration(0, 11585, 11585);
  poll_at(120);
  NEAR(haptic_controller.angles.roll_deg, 45.0f);
  NEAR(haptic_controller.angles.pitch_deg, 0.0f);
  CHECK(pulses[0] == 1008U && pulses[1] == 1472U && pulses[2] == 1472U);
  acceleration(0, -11585, 11585);
  poll_at(140);
  NEAR(haptic_controller.angles.roll_deg, -45.0f);
  CHECK(pulses[0] == 1936U);
  acceleration(-11585, 0, 11585);
  poll_at(160);
  NEAR(haptic_controller.angles.pitch_deg, 45.0f);
  CHECK(pulses[0] == 1472U && pulses[1] == 1008U);
  acceleration(11585, 0, 11585);
  poll_at(180);
  NEAR(haptic_controller.angles.pitch_deg, -45.0f);
  CHECK(pulses[1] == 1936U);
  /* Approximate 1g vector for roll=+60 and pitch=-60: both axes clamp. */
  acceleration(14189, 7094, 4096);
  poll_at(200);
  NEAR(haptic_controller.angles.roll_deg, 60.0f);
  NEAR(haptic_controller.angles.pitch_deg, -60.0f);
  CHECK(pulses[0] == 956U && pulses[1] == 1988U && pulses[2] == 1988U);
  check_auto();
  ++cases;
}

static void test_button_bounce_hold_and_release(void)
{
  reset(0, GPIO_PIN_RESET);
  HapticApp_Init(&i2c, &timer);
  button = GPIO_PIN_SET; poll_at(101);
  button = GPIO_PIN_RESET; poll_at(110);
  button = GPIO_PIN_SET; poll_at(120);
  button = GPIO_PIN_RESET; poll_at(125);
  button = GPIO_PIN_SET; poll_at(130);
  poll_at(159);
  check_manual();
  poll_at(160);
  /* The debounced event waits until the next 20ms sensor/output update. */
  check_manual();
  poll_at(179);
  check_auto();
  poll_at(200);
  check_auto();
  button = GPIO_PIN_RESET; poll_at(201);
  poll_at(231);
  button = GPIO_PIN_SET; poll_at(232);
  poll_at(262);
  check_manual();
  poll_at(302);
  check_manual();
  ++cases;
}

static void test_button_held_at_boot(void)
{
  reset(0, GPIO_PIN_SET);
  HapticApp_Init(&i2c, &timer);
  poll_at(140);
  check_manual();
  button = GPIO_PIN_RESET; poll_at(141);
  poll_at(171);
  button = GPIO_PIN_SET; poll_at(172);
  poll_at(202);
  check_auto();
  ++cases;
}

static void test_unsafe_press_is_consumed(void)
{
  reset(0, GPIO_PIN_RESET);
  HapticApp_Init(&i2c, &timer);
  acceleration(0, 10531, 12551); /* Approximately 40-degree roll, 1g. */
  button = GPIO_PIN_SET; poll_at(101);
  poll_at(131);
  check_auto();
  button = GPIO_PIN_RESET; poll_at(132);
  poll_at(162);
  button = GPIO_PIN_SET; poll_at(163);
  poll_at(193);
  check_auto();
  button = GPIO_PIN_RESET; poll_at(194);
  poll_at(224);
  acceleration(0, 0, 16384);
  poll_at(244);
  check_auto();
  button = GPIO_PIN_SET; poll_at(245);
  poll_at(275);
  check_manual();
  ++cases;
}

static void test_transfer_failure_and_retry(void)
{
  reset(0, GPIO_PIN_RESET);
  HapticApp_Init(&i2c, &timer);
  fail_receive = true;
  poll_at(120);
  CHECK(!haptic_controller.sensor_valid);
  check_auto();
  CHECK(pulses[0] == 1472U && pulses[1] == 1472U && pulses[2] == 544U);
  fail_receive = false;
  poll_at(1100);
  CHECK(identity_count == 1 && sample_count == 0);
  poll_at(1120);
  CHECK(identity_count == 2 && sample_count == 0);
  CHECK(!haptic_controller.sensor_valid);
  poll_at(1139);
  CHECK(sample_count == 0);
  poll_at(1140);
  CHECK(sample_count == 1 && haptic_controller.sensor_valid);
  check_auto();
  ++cases;
}

static void test_bad_identity_and_zero_sample(void)
{
  reset(0, GPIO_PIN_RESET);
  identity = 0x00U;
  HapticApp_Init(&i2c, &timer);
  CHECK(write_count == 0 && !haptic_controller.sensor_valid);
  check_auto();
  poll_at(1080);
  CHECK(identity_count == 1);
  identity = 0x68U;
  poll_at(1100);
  CHECK(identity_count == 2 && write_count == 2);
  acceleration(0, 0, 0);
  poll_at(1120);
  CHECK(!haptic_controller.sensor_valid);
  check_auto();
  acceleration(0, 0, 16384);
  poll_at(1140);
  CHECK(haptic_controller.sensor_valid);
  check_auto();
  ++cases;
}

static void test_transmit_short_circuit(void)
{
  reset(0, GPIO_PIN_RESET);
  fail_transmit = true;
  HapticApp_Init(&i2c, &timer);
  CHECK(transmit_count == 1 && receive_count == 0);
  CHECK(!haptic_controller.sensor_valid);
  check_auto();
  ++cases;
}

static void test_tick_wrap(void)
{
  /* Initialization finishes shortly before UINT32_MAX, then sampling crosses it. */
  reset(UINT32_MAX - 110U, GPIO_PIN_RESET);
  HapticApp_Init(&i2c, &timer);
  CHECK(tick == UINT32_MAX - 10U);
  poll_at(8);
  CHECK(sample_count == 0);
  poll_at(9);
  CHECK(sample_count == 1);
  reset(UINT32_MAX - 110U, GPIO_PIN_RESET);
  HapticApp_Init(&i2c, &timer);
  button = GPIO_PIN_SET; poll_at(UINT32_MAX - 9U);
  poll_at(20);
  check_auto();
  ++cases;
}

int main(void)
{
  test_initialization();
  test_sample_rate();
  test_signed_samples_and_outputs();
  test_button_bounce_hold_and_release();
  test_button_held_at_boot();
  test_unsafe_press_is_consumed();
  test_transfer_failure_and_retry();
  test_bad_identity_and_zero_sample();
  test_transmit_short_circuit();
  test_tick_wrap();
  printf("PASS: %u adapter cases, %u checks (I2C, PWM, GPIO, debounce, retries, ticks)\n",
         cases, checks);
  return EXIT_SUCCESS;
}
