/*
 * GPIO adapter for standards-based software I2C. This transport observes actual
 * SDA acknowledgements and register bytes; it does not supply simulated samples.
 * PB8/PB9 remain the assignment's SCL/SDA pins, independent of I2C1 emulation.
 */
#include "haptic_soft_i2c.h"

#include "main.h"
#include <stddef.h>

#define MPU_ADDRESS_7BIT 0x68U
#define SCL_PIN GPIO_PIN_8
#define SDA_PIN GPIO_PIN_9
#define DELAY_GUARD_MS 2U
#define DELAY_POLL_LIMIT 4096U

/* A timing failure makes SCL unavailable instead of producing unclocked data. */
static bool timing_fault;

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
  if ((control & SysTick_CTRL_ENABLE_Msk) == 0U || reload <= 1U
      || clock_hz < 1000000U || microseconds > 1000U)
  {
    timing_fault = true;
    return;
  }

  uint32_t remaining = ((clock_hz + 999999U) / 1000000U) * microseconds;
  uint32_t previous = SysTick->VAL;
  const uint32_t started_ms = HAL_GetTick();
  for (uint32_t polls = 0U; polls < DELAY_POLL_LIMIT; ++polls)
  {
    const uint32_t current = SysTick->VAL;
    const uint32_t elapsed = previous >= current ? previous - current
                                               : previous + reload - current;
    if (elapsed >= remaining)
    {
      return;
    }
    remaining -= elapsed;
    previous = current;
    if ((uint32_t)(HAL_GetTick() - started_ms) >= DELAY_GUARD_MS)
    {
      break;
    }
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Preload released levels before switching the pins out of alternate mode. */
  HAL_GPIO_WritePin(GPIOB, SCL_PIN | SDA_PIN, GPIO_PIN_SET);
  pins.Pin = SCL_PIN | SDA_PIN;
  pins.Mode = GPIO_MODE_OUTPUT_OD;
  pins.Pull = GPIO_PULLUP;
  pins.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &pins);

  return SoftI2C_Recover(&bus) == SOFT_I2C_OK && !timing_fault;
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
