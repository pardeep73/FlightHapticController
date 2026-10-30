/* CMSIS/GPIO inputs for the real main.c software-I2C platform adapter. */
#ifndef TEST_GPIO_HAL_H
#define TEST_GPIO_HAL_H
#include "diagnostics_hal.h"

typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t LOAD;
  volatile uint32_t VAL;
} MockSysTick;
extern MockSysTick mock_systick;
extern uint32_t SystemCoreClock;
#define SysTick (&mock_systick)
#define SysTick_CTRL_ENABLE_Msk (1U << 0)
#define SysTick_CTRL_CLKSOURCE_Msk (1U << 2)
#define GPIO_PIN_8 (1U << 8)
#define GPIO_MODE_OUTPUT_OD 17U
#define GPIO_PULLUP 1U
#define __HAL_RCC_GPIOB_CLK_ENABLE() Mock_EnableGPIOBClock()
void Mock_EnableGPIOBClock(void);
#endif
