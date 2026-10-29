/* STM32 GPIO transport for the MPU6050 on the assignment's PB8/PB9 pins. */
#ifndef HAPTIC_SOFT_I2C_H
#define HAPTIC_SOFT_I2C_H

#include "soft_i2c.h"

/*
 * The application must disable/deinitialize hardware I2C1 before calling Init.
 * Configure open-drain GPIO, release both lines, then perform bus recovery.
 */
bool HapticSoftI2C_Init(void);

/* Read or write real MPU6050 registers at its 7-bit address 0x68. */
SoftI2CResult HapticSoftI2C_Read(uint8_t reg, uint8_t *data, uint16_t length);
SoftI2CResult HapticSoftI2C_Write(uint8_t reg, uint8_t value);

#endif /* HAPTIC_SOFT_I2C_H */
