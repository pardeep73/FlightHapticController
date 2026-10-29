/* Minimal HAL contract used only to run hardware-adapter tests on a host. */
#ifndef TEST_MOCK_MAIN_H
#define TEST_MOCK_MAIN_H
/* Suppress the generated main.h, whose STM32 headers need an ARM compiler. */
#define __MAIN_H
#include <stddef.h>
#include <stdint.h>

typedef struct { unsigned marker; } I2C_HandleTypeDef;
typedef struct { unsigned marker; } TIM_HandleTypeDef;
typedef struct { unsigned marker; } GPIO_TypeDef;
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
extern GPIO_TypeDef mock_gpio_a;
extern GPIO_TypeDef mock_gpio_b;
#define GPIOA (&mock_gpio_a)
#define GPIOB (&mock_gpio_b)
#define GPIO_PIN_2 (1U << 2)
#define GPIO_PIN_11 (1U << 11)
#define GPIO_PIN_12 (1U << 12)
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define __HAL_TIM_SET_COMPARE(timer, channel, value) \
    Mock_SetCompare((timer), (channel), (value))
void Mock_SetCompare(TIM_HandleTypeDef *, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *, uint16_t,
                                        uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *, uint16_t,
                                       uint8_t *, uint16_t, uint32_t);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, uint16_t);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t);
void Error_Handler(void);
#endif
