/*
 * Run the production GPIO/timing adapter with controlled register observations.
 * These doubles validate fault classification; they do not reproduce Wokwi.
 */
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned cases;
static unsigned checks;
#define CHECK(condition) do { \
  ++checks; \
  if (!(condition)) { \
    fprintf(stderr, "FAIL GPIO adapter line %d: %s\n", __LINE__, #condition); \
    exit(EXIT_FAILURE); \
  } \
} while (0)

GPIO_TypeDef mock_gpio_a;
GPIO_TypeDef mock_gpio_b;
MockSysTick mock_systick;
uint32_t SystemCoreClock;
static GPIO_InitTypeDef configured_pin;
static unsigned gpio_clocks;
static unsigned pin_initializations;
static bool scl_released;
static bool sda_released;
static bool scl_held_low;
static bool sda_held_low;
static uint32_t counter_step;
static unsigned counter_interval;
static uint32_t tick_step;
static uint64_t cycles;
static uint32_t tick;
static unsigned tick_calls;
static unsigned line_writes;

void Mock_EnableGPIOBClock(void) { ++gpio_clocks; }

void HAL_GPIO_Init(GPIO_TypeDef *port, const GPIO_InitTypeDef *pins)
{
  CHECK(port == GPIOB);
  ++pin_initializations;
  configured_pin = *pins;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pins, GPIO_PinState state)
{
  CHECK(port == GPIOB);
  CHECK(pins != 0 && (pins & ~(GPIO_PIN_8 | GPIO_PIN_9)) == 0);
  CHECK(++line_writes < 100000U);
  if ((pins & GPIO_PIN_8) != 0) scl_released = state == GPIO_PIN_SET;
  if ((pins & GPIO_PIN_9) != 0) sda_released = state == GPIO_PIN_SET;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
  CHECK(port == GPIOB && (pin == GPIO_PIN_8 || pin == GPIO_PIN_9));
  const bool high = pin == GPIO_PIN_8 ? scl_released && !scl_held_low
                                    : sda_released && !sda_held_low;
  return high ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

/*
 * Counter advancement is explicit test input. It is independent of GPIO levels
 * and may be disabled while the mock millisecond tick continues, or vice versa.
 */
uint32_t HAL_GetTick(void)
{
  CHECK(++tick_calls < 100000U);
  if (counter_step != 0 && tick_calls % counter_interval == 0U)
  {
    const uint32_t reload = mock_systick.LOAD + 1U;
    if (reload != 0)
    {
      const uint32_t step = counter_step % reload;
      mock_systick.VAL = mock_systick.VAL >= step
          ? mock_systick.VAL - step : mock_systick.VAL + reload - step;
    }
    cycles += counter_step;
  }
  tick += tick_step;
  return tick + (uint32_t)(cycles / 48000U);
}

static void reset(void)
{
  memset(&mock_gpio_a, 0, sizeof mock_gpio_a);
  memset(&mock_gpio_b, 0, sizeof mock_gpio_b);
  memset(&configured_pin, 0, sizeof configured_pin);
  mock_systick.CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
  mock_systick.LOAD = 47999U;
  mock_systick.VAL = 12345U;
  SystemCoreClock = 48000000U;
  gpio_clocks = pin_initializations = tick_calls = line_writes = 0;
  scl_released = sda_released = true;
  scl_held_low = sda_held_low = false;
  counter_step = 48U;
  counter_interval = 1U;
  tick_step = 0;
  cycles = 0;
  tick = 0;
}

static void test_healthy_initialization(void)
{
  reset();
  CHECK(HapticSoftI2C_Init());
  CHECK(gpio_clocks == 1 && pin_initializations == 1);
  CHECK(configured_pin.Pin == (GPIO_PIN_8 | GPIO_PIN_9));
  CHECK(configured_pin.Mode == GPIO_MODE_OUTPUT_OD);
  CHECK(configured_pin.Pull == GPIO_PULLUP);
  CHECK(configured_pin.Speed == GPIO_SPEED_FREQ_LOW);
  CHECK(scl_released && sda_released);
  CHECK(haptic_soft_i2c_status.recover_result == SOFT_I2C_OK);
  CHECK(haptic_soft_i2c_status.timing_fault == HAPTIC_TIMING_OK);
  CHECK(haptic_soft_i2c_status.lines_before == 3U);
  CHECK(haptic_soft_i2c_status.lines_after == 3U);
  ++cases;
}

static void test_frozen_counter_with_high_physical_lines(void)
{
  reset();
  counter_step = 0;
  CHECK(!HapticSoftI2C_Init());
  CHECK(haptic_soft_i2c_status.recover_result == SOFT_I2C_TIMEOUT);
  CHECK(haptic_soft_i2c_status.timing_fault == HAPTIC_TIMING_NO_PROGRESS);
  CHECK(haptic_soft_i2c_status.lines_before == 3U);
  CHECK(haptic_soft_i2c_status.lines_after == 3U);
  CHECK(haptic_soft_i2c_status.systick_ctrl == 5U);
  CHECK(haptic_soft_i2c_status.systick_load == 47999U);
  CHECK(haptic_soft_i2c_status.val_first == 12345U);
  CHECK(haptic_soft_i2c_status.val_last == 12345U);
  CHECK(haptic_soft_i2c_status.elapsed_cycles == 0U);
  CHECK(haptic_soft_i2c_status.polls == 4096U);
  CHECK(tick_calls < 10000U);
  ++cases;
}

static void test_disabled_counter(void)
{
  reset();
  mock_systick.CTRL = SysTick_CTRL_CLKSOURCE_Msk;
  CHECK(!HapticSoftI2C_Init());
  CHECK(haptic_soft_i2c_status.timing_fault == HAPTIC_TIMING_DISABLED);
  CHECK(haptic_soft_i2c_status.systick_ctrl == 4U);
  CHECK(haptic_soft_i2c_status.lines_before == 3U);
  CHECK(haptic_soft_i2c_status.lines_after == 3U);
  ++cases;
}

static void test_invalid_timing_configuration(void)
{
  reset();
  mock_systick.LOAD = 0U;
  CHECK(!HapticSoftI2C_Init());
  CHECK(haptic_soft_i2c_status.timing_fault == HAPTIC_TIMING_CONFIG);
  CHECK(haptic_soft_i2c_status.systick_load == 0U);
  reset();
  SystemCoreClock = 500000U;
  CHECK(!HapticSoftI2C_Init());
  CHECK(haptic_soft_i2c_status.timing_fault == HAPTIC_TIMING_CONFIG);
  ++cases;
}

static void test_actual_data_line_low(void)
{
  reset();
  sda_held_low = true;
  CHECK(!HapticSoftI2C_Init());
  CHECK(haptic_soft_i2c_status.recover_result == SOFT_I2C_BUS_STUCK);
  CHECK(haptic_soft_i2c_status.timing_fault == HAPTIC_TIMING_OK);
  CHECK(haptic_soft_i2c_status.lines_before == 1U);
  CHECK(haptic_soft_i2c_status.lines_after == 1U);
  ++cases;
}

static void test_actual_clock_line_low(void)
{
  reset();
  scl_held_low = true;
  CHECK(!HapticSoftI2C_Init());
  CHECK(haptic_soft_i2c_status.recover_result == SOFT_I2C_TIMEOUT);
  CHECK(haptic_soft_i2c_status.timing_fault == HAPTIC_TIMING_OK);
  CHECK(haptic_soft_i2c_status.lines_before == 2U);
  CHECK(haptic_soft_i2c_status.lines_after == 2U);
  CHECK(tick_calls < 10000U);
  ++cases;
}

static void test_fault_snapshot_preserved_then_reset(void)
{
  reset();
  counter_step = 0;
  CHECK(!HapticSoftI2C_Init());
  const HapticSoftI2CStatus saved = haptic_soft_i2c_status;
  mock_systick.CTRL = 0U;
  mock_systick.LOAD = 0U;
  mock_systick.VAL = 2U;
  uint8_t data = 0;
  CHECK(HapticSoftI2C_Read(0x75U, &data, 1) == SOFT_I2C_TIMEOUT);
  CHECK(haptic_soft_i2c_status.timing_fault == saved.timing_fault);
  CHECK(haptic_soft_i2c_status.systick_ctrl == saved.systick_ctrl);
  CHECK(haptic_soft_i2c_status.systick_load == saved.systick_load);
  CHECK(haptic_soft_i2c_status.val_first == saved.val_first);
  CHECK(haptic_soft_i2c_status.val_last == saved.val_last);
  CHECK(haptic_soft_i2c_status.polls == saved.polls);
  reset();
  CHECK(HapticSoftI2C_Init());
  CHECK(haptic_soft_i2c_status.timing_fault == HAPTIC_TIMING_OK);
  CHECK(haptic_soft_i2c_status.recover_result == SOFT_I2C_OK);
  ++cases;
}

static void test_tick_guard_and_counter_wrap(void)
{
  reset();
  /* Counter moves, but a five-us delay cannot complete before the two-ms guard. */
  counter_step = 1U;
  tick_step = 1U;
  CHECK(!HapticSoftI2C_Init());
  CHECK(haptic_soft_i2c_status.timing_fault == HAPTIC_TIMING_TICK_GUARD);
  CHECK(haptic_soft_i2c_status.elapsed_cycles > 0U);
  CHECK(haptic_soft_i2c_status.polls < 4096U);
  reset();
  /* A valid down-counter reload during an ordinary delay must not fail. */
  mock_systick.VAL = 1U;
  CHECK(HapticSoftI2C_Init());
  CHECK(haptic_soft_i2c_status.timing_fault == HAPTIC_TIMING_OK);
  ++cases;
}

static void test_counter_makes_insufficient_progress(void)
{
  reset();
  counter_step = 1U;
  counter_interval = 100U;
  CHECK(!HapticSoftI2C_Init());
  CHECK(haptic_soft_i2c_status.timing_fault == HAPTIC_TIMING_POLL_GUARD);
  CHECK(haptic_soft_i2c_status.elapsed_cycles > 0U);
  CHECK(haptic_soft_i2c_status.elapsed_cycles < 240U);
  CHECK(haptic_soft_i2c_status.polls == 4096U);
  CHECK(haptic_soft_i2c_status.lines_after == 3U);
  ++cases;
}

int main(void)
{
  test_healthy_initialization();
  test_frozen_counter_with_high_physical_lines();
  test_disabled_counter();
  test_invalid_timing_configuration();
  test_actual_data_line_low();
  test_actual_clock_line_low();
  test_fault_snapshot_preserved_then_reset();
  test_tick_guard_and_counter_wrap();
  test_counter_makes_insufficient_progress();
  printf("PASS: %u GPIO adapter cases, %u checks (raw lines, SysTick observations, bounded timing)\n",
         cases, checks);
  return EXIT_SUCCESS;
}
