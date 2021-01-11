#include <stm32f103.h>

// ---- GPIO/AFIO ---- //
volatile gpio_t GPIOA __attribute__((section(".gpioa")));
volatile gpio_t GPIOB __attribute__((section(".gpiob")));
volatile gpio_t GPIOC __attribute__((section(".gpioc")));
volatile gpio_t GPIOD __attribute__((section(".gpiod")));
volatile gpio_t GPIOE __attribute__((section(".gpioe")));

volatile afio_t AFIO __attribute__((section(".afio")));

// ---- RCC ---- //
volatile rcc_t RCC __attribute__((section(".rcc")));

// ---- USART ---- //
volatile usart_t USART1 __attribute__((section(".usart1")));
volatile usart_t USART2 __attribute__((section(".usart2")));
volatile usart_t USART3 __attribute__((section(".usart3")));

// ---- I2C ---- //
volatile i2c_t I2C1 __attribute__((section(".i2c1")));
volatile i2c_t I2C2 __attribute__((section(".i2c2")));

// ---- SPI ---- //
volatile spi_t SPI1 __attribute__((section(".spi1")));
volatile spi_t SPI2 __attribute__((section(".spi2")));
