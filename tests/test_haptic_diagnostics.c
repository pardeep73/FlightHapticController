/* Compile the real diagnostic module against bounded register/tick doubles. */
#include "haptic_diagnostics.h"
#include "haptic_app.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned cases;
static unsigned checks;
#define CHECK(condition) do { \
  ++checks; \
  if (!(condition)) { \
    fprintf(stderr, "FAIL diagnostics line %d: %s\n", __LINE__, #condition); \
    exit(EXIT_FAILURE); \
  } \
} while (0)

GPIO_TypeDef mock_gpio_a;
GPIO_TypeDef mock_gpio_b;
MockUSART mock_usart1;
MockRCC mock_rcc;
HapticController haptic_controller;
HapticAppStatus haptic_app_status;

static uint32_t tick;
static unsigned tick_reads;
static unsigned gpio_clock_count;
static unsigned uart_clock_count;
static unsigned pin_init_count;
static GPIO_TypeDef *configured_port;
static GPIO_InitTypeDef configured_pin;
static char captured[512];
static size_t captured_length;

void Mock_EnableGPIOAClock(void) { ++gpio_clock_count; }
void Mock_EnableUSART1Clock(void) { ++uart_clock_count; }

void HAL_GPIO_Init(GPIO_TypeDef *port, const GPIO_InitTypeDef *pin)
{
  ++pin_init_count;
  configured_port = port;
  configured_pin = *pin;
}

uint32_t HAL_RCC_GetPCLK1Freq(void) { return 48000000U; }

uint32_t HAL_GetTick(void)
{
  /* Fail quickly if a regression adds a blocking flag-wait loop. */
  CHECK(++tick_reads <= 2U);
  return tick;
}

/* A sentinel lets every call reveal a TDR write, including repeated letters. */
static bool poll(void)
{
  const uint32_t before = tick;
  USART1->TDR = UINT32_MAX;
  tick_reads = 0;
  HapticDiagnostics_Poll();
  CHECK(tick == before && tick_reads == 1U);
  if (USART1->TDR == UINT32_MAX)
  {
    return false;
  }
  CHECK(USART1->TDR <= 0xFFU);
  CHECK(captured_length + 1U < sizeof captured);
  captured[captured_length++] = (char)USART1->TDR;
  captured[captured_length] = '\0';
  return true;
}

static void clear_capture(void)
{
  captured_length = 0U;
  captured[0] = '\0';
}

/* Reset only external doubles; Init must reset the real module's static state. */
static void reset(uint32_t initial_tick)
{
  tick = initial_tick;
  tick_reads = 0U;
  gpio_clock_count = uart_clock_count = pin_init_count = 0U;
  configured_port = NULL;
  memset(&configured_pin, 0, sizeof configured_pin);
  memset(&mock_usart1, 0xFF, sizeof mock_usart1);
  mock_rcc.CCIPR = 0xA5A5A5A5U;
  haptic_controller = (HapticController){0};
  haptic_controller.sensor_valid = true;
  haptic_app_status = (HapticAppStatus){0};
  haptic_app_status.identity = 0x68U;
  haptic_app_status.register_address = 0x3BU;
  clear_capture();
  HapticDiagnostics_Init();
  USART1->ISR = USART_ISR_TXE_TXFNF;
}

/* Drain at a fixed timestamp: one call must correspond to exactly one byte. */
static void drain_line(void)
{
  for (unsigned i = 0; i < 160U; ++i)
  {
    const bool sent = poll();
    if (!sent)
    {
      CHECK(captured_length >= 2U);
      CHECK(captured[captured_length - 2U] == '\r');
      CHECK(captured[captured_length - 1U] == '\n');
      return;
    }
  }
  CHECK(false); /* A missing terminator or oversized record must fail. */
}

static void test_uart_initialization(void)
{
  reset(0U);
  CHECK(gpio_clock_count == 1U && uart_clock_count == 1U);
  CHECK(pin_init_count == 1U && configured_port == GPIOA);
  CHECK(configured_pin.Pin == (1U << 9));
  CHECK(configured_pin.Mode == GPIO_MODE_AF_PP);
  CHECK(configured_pin.Pull == GPIO_NOPULL);
  CHECK(configured_pin.Speed == GPIO_SPEED_FREQ_LOW);
  CHECK(configured_pin.Alternate == 1U);
  CHECK(USART1->BRR == 417U);
  CHECK(USART1->CR1 == (USART_CR1_UE | USART_CR1_TE));
  CHECK(USART1->CR2 == 0U && USART1->CR3 == 0U);
  CHECK(RCC->CCIPR == (0xA5A5A5A5U & ~3U));
  CHECK(!poll());
  tick = 999U; CHECK(!poll());
  CHECK(captured_length == 0U);
  ++cases;
}

static void test_healthy_output_and_rate(void)
{
  reset(0U);
  haptic_controller.angles = (HapticAngles){-0.06f, 35.24f, 35.25f};
  tick = 1000U;
  CHECK(!poll()); /* Formatting queues the record without a blocking write. */
  CHECK(poll() && captured_length == 1U && captured[0] == 'm');
  CHECK(poll() && captured_length == 2U && captured[1] == 'o');
  drain_line();
  CHECK(strcmp(captured,
      "mode=MANUAL IMU=OK who=0x68 reg=0x3B HAL=0 err=0x00000000 bus=HW hwerr=0x00000000 roll=-0.1 pitch=35.2 tilt=35.3\r\n") == 0);
  const size_t length = captured_length;
  tick = 1999U; CHECK(!poll() && captured_length == length);
  tick = 2000U; CHECK(!poll() && captured_length == length);
  clear_capture();
  drain_line();
  CHECK(strstr(captured, "mode=MANUAL IMU=OK") != NULL);
  ++cases;
}

static void test_healthy_autopilot_and_extreme_angles(void)
{
  reset(0U);
  haptic_controller.mode = HAPTIC_AUTOPILOT;
  haptic_controller.angles = (HapticAngles){-180.0f, -90.0f, 201.246f};
  tick = 1000U; CHECK(!poll());
  drain_line();
  CHECK(strstr(captured, "mode=AUTO IMU=OK") != NULL);
  CHECK(strstr(captured, "IMU=FAULT") == NULL);
  CHECK(strstr(captured, "roll=-180.0 pitch=-90.0 tilt=201.2\r\n") != NULL);
  CHECK(captured_length < 160U);
  ++cases;
}

static void test_fault_operations_and_hex_fields(void)
{
  const struct
  {
    HapticSensorFault fault;
    const char *description;
  } faults[] = {
    {HAPTIC_SENSOR_NONE, "none"},
    {HAPTIC_SENSOR_WHO_READ, "WHO_AM_I read failed"},
    {HAPTIC_SENSOR_IDENTITY, "unexpected WHO_AM_I"},
    {HAPTIC_SENSOR_WAKE, "sensor wake write failed"},
    {HAPTIC_SENSOR_CONFIG, "accelerometer config failed"},
    {HAPTIC_SENSOR_ACCEL_READ, "acceleration read failed"},
    {HAPTIC_SENSOR_ACCEL_INVALID, "zero/invalid acceleration"},
    {HAPTIC_SENSOR_RECOVERY, "I2C recovery failed"},
    {(HapticSensorFault)255, "unknown sensor fault"}
  };

  for (size_t i = 0; i < sizeof faults / sizeof faults[0]; ++i)
  {
    reset(0U);
    haptic_controller.sensor_valid = false;
    haptic_controller.mode = HAPTIC_AUTOPILOT;
    haptic_app_status.fault = faults[i].fault;
    haptic_app_status.identity = 0x22U;
    haptic_app_status.register_address = 0x6BU;
    haptic_app_status.hal_status = HAL_TIMEOUT;
    haptic_app_status.i2c_error = 0xDEADBEEFU;
    tick = 1000U; CHECK(!poll());
    drain_line();
    CHECK(strstr(captured, "mode=AUTO IMU=FAULT fault=") != NULL);
    CHECK(strstr(captured, faults[i].description) != NULL);
    CHECK(strstr(captured, "who=0x22 reg=0x6B HAL=3 err=0xDEADBEEF bus=HW hwerr=0x00000000\r\n") != NULL);
    CHECK(strstr(captured, "roll=") == NULL);
    CHECK(captured_length < 160U);
    ++cases;
  }
}

static void test_nonblocking_tx_stall_and_recovery(void)
{
  reset(0U);
  tick = 1000U; CHECK(!poll());
  USART1->ISR = 0U;
  CHECK(!poll());
  tick = 1001U; CHECK(!poll());
  /* Readiness before 2ms retains the record; transmit precisely its first byte. */
  USART1->ISR = USART_ISR_TXE_TXFNF;
  CHECK(poll() && captured_length == 1U && captured[0] == 'm');
  USART1->ISR = 0U;
  tick = 1002U; CHECK(!poll());
  tick = 1003U; CHECK(!poll()); /* Drop after 2ms without progress. */
  USART1->ISR = USART_ISR_TXE_TXFNF;
  CHECK(!poll() && captured_length == 1U);
  clear_capture();
  tick = 2000U; CHECK(!poll());
  drain_line();
  CHECK(strstr(captured, "mode=MANUAL IMU=OK") == captured);
  ++cases;
}

static void test_tick_wrap(void)
{
  const uint32_t started = UINT32_MAX - 500U;
  reset(started);
  tick = started + 999U; CHECK(!poll());
  tick = started + 1000U; CHECK(!poll());
  drain_line();
  CHECK(strstr(captured, "mode=MANUAL IMU=OK") == captured);
  CHECK(!poll());
  clear_capture();

  /* Also exercise the separate TX-stall subtraction over the wrap boundary. */
  const uint32_t stall_started = UINT32_MAX - 1001U;
  reset(stall_started);
  tick = stall_started + 1000U; CHECK(!poll());
  USART1->ISR = 0U;
  tick = stall_started + 1001U; CHECK(!poll());
  tick = stall_started + 1002U; CHECK(!poll());
  USART1->ISR = USART_ISR_TXE_TXFNF;
  CHECK(!poll() && captured_length == 0U);
  tick = stall_started + 2000U; CHECK(!poll());
  drain_line();
  CHECK(strstr(captured, "mode=MANUAL IMU=OK") == captured);
  ++cases;
}

/* Preserve evidence of the hardware timeout after genuine GPIO recovery. */
static void test_software_transport_status(void)
{
  reset(0U);
  haptic_app_status.software_i2c = true;
  haptic_app_status.hardware_error = 0x20U;
  tick = 1000U; CHECK(!poll());
  drain_line();
  CHECK(strstr(captured, "IMU=OK") != NULL);
  CHECK(strstr(captured, "bus=GPIO hwerr=0x00000020") != NULL);
  CHECK(strstr(captured, "err=0x00000000 bus=") != NULL);
  ++cases;
}

int main(void)
{
  test_uart_initialization();
  test_healthy_output_and_rate();
  test_healthy_autopilot_and_extreme_angles();
  test_fault_operations_and_hex_fields();
  test_nonblocking_tx_stall_and_recovery();
  test_tick_wrap();
  test_software_transport_status();
  printf("PASS: %u diagnostics cases, %u checks (PA9 USART, readable status, queued TX, wrap)\n",
          cases, checks);
  return EXIT_SUCCESS;
}
