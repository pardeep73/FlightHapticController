/* Compile the real diagnostic module against bounded register/tick doubles. */
#include "main.h"

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
HapticSoftI2CStatus haptic_soft_i2c_status;
HapticHardwareI2CStatus haptic_hw_i2c_status;

static uint32_t tick;
static unsigned tick_reads;
static unsigned gpio_clock_count;
static unsigned uart_clock_count;
static unsigned pin_init_count;
static GPIO_TypeDef *configured_port;
static GPIO_InitTypeDef configured_pin;
static char captured[1024];
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
  haptic_soft_i2c_status = (HapticSoftI2CStatus){0};
  haptic_hw_i2c_status = (HapticHardwareI2CStatus){0};
  haptic_app_status.identity = 0x68U;
  haptic_app_status.register_address = 0x3BU;
  clear_capture();
  HapticDiagnostics_Init();
  USART1->ISR = USART_ISR_TXE_TXFNF;
}

/* Drain at a fixed timestamp: one call must correspond to exactly one byte. */
static void drain_line(void)
{
  for (unsigned i = 0; i < 640U; ++i)
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
  CHECK(strstr(captured, "HW captured=") == NULL);
  ++cases;
}

static void test_fault_trace_preserves_observations(void)
{
  reset(0U);
  haptic_controller.sensor_valid = false;
  haptic_controller.mode = HAPTIC_AUTOPILOT;
  haptic_app_status.software_i2c = true;
  haptic_app_status.hardware_error = 0x20U;
  haptic_app_status.fault = HAPTIC_SENSOR_RECOVERY;
  haptic_app_status.i2c_error = 0x10000U;
  haptic_hw_i2c_status = (HapticHardwareI2CStatus){
    .captured = true, .error = 0x20U, .flag = 0x40U, .isr = 0x8002U,
    .cr1 = 1U, .cr2 = 0x120D0U, .timingr = 0x10805D88U,
    .gpio_idr = 0x300U, .boot_idr = 0x300U, .moder = 0xA0000U, .otyper = 0x300U,
    .pupdr = 0x50000U, .afr = 0x66U
  };
  haptic_soft_i2c_status = (HapticSoftI2CStatus){
    .recover_result = SOFT_I2C_TIMEOUT, .timing_fault = HAPTIC_TIMING_NO_PROGRESS,
    .systick_ctrl = 5U, .systick_load = 47999U,
    .val_first = 12345U, .val_last = 12345U,
    .elapsed_cycles = 0U, .polls = 4096U,
    .lines_before = 3U, .lines_after = 3U,
    .gpio_odr = 0x300U, .gpio_moder = 0x50000U, .gpio_pupdr = 0x50000U
  };
  tick = 1000U; CHECK(!poll());
  /* Queued observations must remain intact when the next recovery changes state. */
  haptic_hw_i2c_status = (HapticHardwareI2CStatus){0};
  haptic_soft_i2c_status = (HapticSoftI2CStatus){0};
  drain_line();
  CHECK(strstr(captured, "mode=AUTO IMU=FAULT fault=I2C recovery failed") == captured);
  CHECK(strstr(captured, "HW captured=1 err=00000020 flag=00000040 ISR=00008002 CR1=00000001 CR2=000120D0 TIMINGR=10805D88\r\n") != NULL);
  CHECK(strstr(captured, "PINS IDR=00000300 BOOT=00000300 MODER=000A0000 OTYPER=00000300 PUPDR=00050000 AFRH=00000066\r\n") != NULL);
  CHECK(strstr(captured, "SOFT recover=2 timing=3 lines=3/3 ODR=00000300 MODER=00050000 PUPDR=00050000 split=0 serr=00000000\r\n") != NULL);
  CHECK(strstr(captured, "CLOCK CTRL=00000005 LOAD=47999 VAL=12345/12345 cycles=0 polls=4096\r\n") != NULL);
  CHECK(captured_length > 160U && captured_length < 640U);
  ++cases;
}

static void test_hardware_trace_without_fallback(void)
{
  reset(0U);
  haptic_controller.sensor_valid = false;
  haptic_app_status.fault = HAPTIC_SENSOR_WHO_READ;
  haptic_hw_i2c_status.captured = true;
  haptic_hw_i2c_status.flag = 0x02U;
  haptic_hw_i2c_status.isr = 0x8000U;
  tick = 1000U; CHECK(!poll());
  drain_line();
  CHECK(strstr(captured, "bus=HW") != NULL);
  CHECK(strstr(captured, "HW captured=1") != NULL);
  CHECK(strstr(captured, "flag=00000002 ISR=00008000") != NULL);
  ++cases;
}

static void test_fault_trace_maximum_numeric_fields_fit(void)
{
  reset(0U);
  haptic_controller.sensor_valid = false;
  haptic_controller.mode = HAPTIC_AUTOPILOT;
  haptic_app_status.fault = HAPTIC_SENSOR_CONFIG;
  haptic_app_status.software_i2c = true;
  haptic_app_status.identity = haptic_app_status.register_address = UINT8_MAX;
  haptic_app_status.hal_status = HAL_TIMEOUT;
  haptic_app_status.i2c_error = haptic_app_status.hardware_error = UINT32_MAX;
  haptic_hw_i2c_status = (HapticHardwareI2CStatus){
    .captured = true, .error = UINT32_MAX, .flag = UINT32_MAX, .isr = UINT32_MAX,
    .cr1 = UINT32_MAX, .cr2 = UINT32_MAX, .timingr = UINT32_MAX,
    .gpio_idr = UINT32_MAX, .boot_idr = UINT32_MAX,
    .moder = UINT32_MAX, .otyper = UINT32_MAX,
    .pupdr = UINT32_MAX, .afr = UINT32_MAX
  };
  haptic_soft_i2c_status = (HapticSoftI2CStatus){
    .recover_result = SOFT_I2C_INVALID_ARGUMENT, .timing_fault = HAPTIC_TIMING_POLL_GUARD,
    .systick_ctrl = UINT32_MAX, .systick_load = UINT32_MAX,
    .val_first = UINT32_MAX, .val_last = UINT32_MAX,
    .elapsed_cycles = UINT32_MAX, .polls = UINT32_MAX,
    .lines_before = UINT8_MAX, .lines_after = UINT8_MAX,
    .gpio_odr = UINT32_MAX, .gpio_moder = UINT32_MAX,
    .gpio_pupdr = UINT32_MAX
  };
  tick = 1000U; CHECK(!poll());
  drain_line();
  CHECK(strstr(captured, "CLOCK CTRL=FFFFFFFF LOAD=4294967295 VAL=4294967295/4294967295 cycles=4294967295 polls=4294967295\r\n") != NULL);
  CHECK(strstr(captured, "SOFT recover=4 timing=5 lines=255/255 ODR=FFFFFFFF MODER=FFFFFFFF PUPDR=FFFFFFFF split=0 serr=00000000\r\n") != NULL);
  CHECK(captured_length < 640U);
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
  test_fault_trace_preserves_observations();
  test_hardware_trace_without_fallback();
  test_fault_trace_maximum_numeric_fields_fit();
  printf("PASS: %u diagnostics cases, %u checks (PA9 USART, readable status, queued TX, wrap)\n",
          cases, checks);
  return EXIT_SUCCESS;
}
