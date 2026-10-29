/*
 * Standard-mode I2C using real GPIO levels. No sensor samples are synthesized:
 * every address and written byte requires a slave ACK, and all received data
 * comes from SDA. This transport avoids reliance on simulated I2C status flags.
 */
#include "soft_i2c.h"

#include <stddef.h>

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
