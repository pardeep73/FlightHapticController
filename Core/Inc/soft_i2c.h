/* Open-drain software I2C; platform callbacks supply GPIO and bounded timing. */
#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  SOFT_I2C_OK = 0,
  SOFT_I2C_NACK,
  SOFT_I2C_TIMEOUT,
  SOFT_I2C_BUS_STUCK,
  SOFT_I2C_INVALID_ARGUMENT
} SoftI2CResult;

/*
 * set_scl/set_sda(false) actively pull low; true releases an open-drain line.
 * Reads return the actual bus level, including the slave's ACK/data/stretching.
 * delay_us receives only 1 or 5 us. get_tick returns a wrapping millisecond
 * counter. Both callbacks must return; an independent poll limit also bounds
 * clock stretching if that counter stops advancing.
 */
typedef struct
{
  void *context;
  void (*set_scl)(void *context, bool high);
  void (*set_sda)(void *context, bool high);
  bool (*read_scl)(void *context);
  bool (*read_sda)(void *context);
  void (*delay_us)(void *context, uint32_t microseconds);
  uint32_t (*get_tick)(void *context);
} SoftI2CBus;

/* Release the bus, issue up to nine recovery clocks if SDA is low, then STOP. */
SoftI2CResult SoftI2C_Recover(SoftI2CBus *bus);

/*
 * Use a seven-bit device address and an eight-bit register address.
 * Reads write the register pointer, issue a repeated START, and NACK the final
 * byte. All real transfers attempt STOP on success and failure; invalid
 * arguments leave the bus untouched. Output bytes are valid only on success.
 */
SoftI2CResult SoftI2C_ReadRegister(SoftI2CBus *bus, uint8_t address,
                                uint8_t reg, uint8_t *data, uint16_t length);
SoftI2CResult SoftI2C_WriteRegister(SoftI2CBus *bus, uint8_t address,
                                 uint8_t reg, const uint8_t *data,
                                 uint16_t length);

#endif /* SOFT_I2C_H */
