/*
 * Diagnostic-only USART1 transmitter. It does not change the flight-envelope
 * policy or substitute serial observations for Wokwi servo calibration tests.
 * PA9 is already wired to the serial monitor RX in diagram.json.
 */
#include "haptic_diagnostics.h"

#include "haptic_app.h"
#include <stdio.h>

#define DIAGNOSTIC_BAUD_RATE 115200U
#define DIAGNOSTIC_PERIOD_MS 1000U
#define TX_WAIT_TIMEOUT_MS 2U

static uint32_t last_report_ms;
static uint32_t tx_last_progress_ms;
static char tx_buffer[160];
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
  }
  tx_position = 0U;
  tx_last_progress_ms = now;
}
