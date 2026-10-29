/* Extend the shared adapter HAL double with only the USART diagnostic contract. */
#ifndef TEST_DIAGNOSTICS_HAL_H
#define TEST_DIAGNOSTICS_HAL_H

#include "main.h"

typedef struct
{
  uint32_t Pin;
  uint32_t Mode;
  uint32_t Pull;
  uint32_t Speed;
  uint32_t Alternate;
} GPIO_InitTypeDef;

typedef struct
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t CR3;
  volatile uint32_t BRR;
  volatile uint32_t ISR;
  volatile uint32_t TDR;
} MockUSART;

typedef struct { volatile uint32_t CCIPR; } MockRCC;

extern MockUSART mock_usart1;
extern MockRCC mock_rcc;
#define USART1 (&mock_usart1)
#define RCC (&mock_rcc)
#define GPIO_PIN_9 (1U << 9)
#define GPIO_MODE_AF_PP 2U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_LOW 0U
#define GPIO_AF1_USART1 1U
#define RCC_CCIPR_USART1SEL 3U
#define USART_CR1_UE (1U << 0)
#define USART_CR1_TE (1U << 3)
#define USART_ISR_TXE_TXFNF (1U << 7)
#define CLEAR_BIT(register_value, bits) ((register_value) &= ~(bits))
#define __HAL_RCC_GPIOA_CLK_ENABLE() Mock_EnableGPIOAClock()
#define __HAL_RCC_USART1_CLK_ENABLE() Mock_EnableUSART1Clock()

void Mock_EnableGPIOAClock(void);
void Mock_EnableUSART1Clock(void);
void HAL_GPIO_Init(GPIO_TypeDef *, const GPIO_InitTypeDef *);
uint32_t HAL_RCC_GetPCLK1Freq(void);

#endif /* TEST_DIAGNOSTICS_HAL_H */
