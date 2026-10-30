/* Minimal HAL contract used only to run hardware-adapter tests on a host. */
#ifndef TEST_MOCK_MAIN_H
#define TEST_MOCK_MAIN_H
/* main.h selects these HAL doubles through the explicit host-test build flag. */
#include <stddef.h>
#include <stdint.h>

typedef struct
{
  volatile uint32_t ISR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t TIMINGR;
} I2C_TypeDef;
typedef struct { I2C_TypeDef *Instance; unsigned marker; } I2C_HandleTypeDef;
typedef struct { unsigned marker; } TIM_HandleTypeDef;
typedef struct
{
  volatile uint32_t IDR;
  volatile uint32_t MODER;
  volatile uint32_t OTYPER;
  volatile uint32_t PUPDR;
  volatile uint32_t ODR;
  volatile uint32_t AFR[2];
} GPIO_TypeDef;
typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
extern GPIO_TypeDef mock_gpio_a;
extern GPIO_TypeDef mock_gpio_b;
#define GPIOA (&mock_gpio_a)
#define GPIOB (&mock_gpio_b)
#define GPIO_PIN_2 (1U << 2)
#define GPIO_PIN_11 (1U << 11)
#define GPIO_PIN_12 (1U << 12)
#define I2C_MEMADD_SIZE_8BIT 1U
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define __HAL_TIM_SET_COMPARE(timer, channel, value) \
    Mock_SetCompare((timer), (channel), (value))
void Mock_SetCompare(TIM_HandleTypeDef *, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *, uint16_t, uint16_t,
    uint16_t, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *, uint16_t, uint16_t,
    uint16_t, uint8_t *, uint16_t, uint32_t);
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *);
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, uint16_t);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t);
void Error_Handler(void);
#endif
