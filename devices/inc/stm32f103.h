/***********************************************************************************************************************
*                                                                                                                      *
* STM32-CPP v0.1                                                                                                       *
*                                                                                                                      *
* Copyright (c) 2021 David Lenfesty                                                                                    *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

// TODO copyright??

#ifndef stm32f103_h
#define stm32f103_h

#include <stdint.h>

// ---- GPIO/AFIO ---- //

enum gpio_cr_cnf_in
{
    GPIO_CR_CNF_IN_ANALOG   = 0b00,
    GPIO_CR_CNF_IN_FLOATING = 0b01,
    GPIO_CR_CNF_IN_PULL     = 0b10,
};

enum gpio_cr_cnf_out
{
    GPIO_CR_CNF_OUT_PP  = 0b00,
    GPIO_CR_CNF_OUT_OD  = 0b01,
    GPIO_CR_CNF_AF_PP   = 0b10,
    GPIO_CR_CNF_AF_OD   = 0b11,
};

enum gpio_cr_mode
{
    GPIO_CR_MODE_INPUT          = 0b00,
    GPIO_CR_MODE_OUTPUT_10MHz   = 0b01,
    GPIO_CR_MODE_OUTPUT_2MHz    = 0b10,
    GPIO_CR_MODE_OUTPUT_50MHz   = 0b11,
};

typedef struct
{
    uint32_t CRL;
    uint32_t CRH;
    uint32_t IDR;
    uint32_t ODR;
    uint32_t BSRR;
    uint32_t BRR;
    uint32_t LCKR;
} gpio_t;

// See datasheet for complete details on these mappings.
enum afio_mapr
{
    AFIO_MAPR_SWJ_CFG               = 0x07000000,
    AFIO_MAPR_ADC2_ETRGREG_REMAP    = 0x00100000,
    AFIO_MAPR_ADC2_ETRGINJ_REMAP    = 0x00080000,
    AFIO_MAPR_ADC1_ETRGREG_REMAP    = 0x00040000,
    AFIO_MAPR_ADC1_ETRGINJ_REMAP    = 0x00020000,
    AFIO_TIM5CH4_IREMAP             = 0x00010000,
    AFIO_PD01_REMAP                 = 0x00008000,
    AFIO_CAN_REMAP                  = 0x00006000,
    AFIO_TIM4_REMAP                 = 0x00001000,
    AFIO_TIM3_REMAP                 = 0x00000C00,
    AFIO_TIM2_REMAP                 = 0x00000300,
    AFIO_TIM1_REMAP                 = 0x000000C0,
    AFIO_USART3_REMAP               = 0x00000030,
    AFIO_USART2_REMAP               = 0x00000008,
    AFIO_USART1_REMAP               = 0x00000004,
    AFIO_I2C1_REMAP                 = 0x00000002,
    AFIO_SPI1_REMAP                 = 0x00000001,
};

typedef struct
{
    uint32_t EVCR;
    uint32_t MAPR;
    uint32_t EXTICR1;
    uint32_t EXTICR2;
    uint32_t EXTICR3;
    uint32_t EXTICR4;
    uint32_t MAPR2;
} afio_t;

extern volatile gpio_t GPIOA;
extern volatile gpio_t GPIOB;
extern volatile gpio_t GPIOC;
extern volatile gpio_t GPIOD;
extern volatile gpio_t GPIOE;

// ---- RCC ---- //

enum rcc_cr_bits
{
	RCC_PLL_READY	= 0x02000000,
	RCC_PLL_ON		= 0x01000000,
	RCC_CSS_ON		= 0x00080000,
	RCC_HSE_BYP		= 0x00040000,
	RCC_HSE_READY	= 0x00020000,
	RCC_HSE_ON		= 0x00010000,
	RCC_HSI_READY	= 0x00000002,
	RCC_HSI_ON		= 0x00000001
};

typedef struct
{
    uint32_t CR;
    uint32_t CFGR;
    uint32_t CIR;
	uint32_t APB2RSTR;
	uint32_t APB1RSTR;
	uint32_t AHBENR;
	uint32_t APB2ENR;
	uint32_t APB1ENR;
	uint32_t BDCR;
	uint32_t CSR;
} rcc_t;

extern volatile rcc_t RCC;

// ---- USART ---- //

typedef struct
{
    uint32_t SR;
    uint32_t DR;
	uint32_t BRR;
	uint32_t CR1;
	uint32_t CR2;
	uint32_t CR3;
	uint32_t GTPR;
} usart_t;

extern volatile usart_t USART1;
extern volatile usart_t USART2;
extern volatile usart_t USART3;

// ---- I2C ---- //

typedef struct
{
	uint32_t CR1;
	uint32_t CR2;
	uint32_t OAR1;
	uint32_t OAR2;
    uint32_t DR;
    uint32_t SR1;
    uint32_t SR2;
    uint32_t CCR;
    uint32_t TRISE;
} i2c_t;

// TODO not everything has all these I2C's
extern volatile i2c_t I2C1;
extern volatile i2c_t I2C2;

// ---- SPI ---- //

typedef struct
{
    uint32_t CR1;
    uint32_t CR2;
    uint32_t SR;
    uint32_t DR;
    uint32_t CRCPR;
    uint32_t RXCRCR;
    uint32_t TXCRCR;
    uint32_t I2SCFGR;
    uint32_t I2SPR;
} spi_t;

extern volatile spi_t SPI1;
extern volatile spi_t SPI2;

// ---- TIM ---- //

// TODO advanced vs. gen. purpose timers?
// for now just assume you won't use "advanced" features in a general timer
typedef struct
{
    uint32_t CR1;
    uint32_t CR2;
    uint32_t SMCR;
    uint32_t DIER;
    uint32_t SR;
    uint32_t EGR;
    uint32_t CCMR1;
    uint32_t CCMR2;
    uint32_t CCER;
    uint32_t CNT;
    uint32_t PSC;
	uint32_t ARR;
	uint32_t RCR;
	uint32_t CCR1;
	uint32_t CCR2;
	uint32_t CCR3;
	uint32_t CCR4;
	uint32_t BDTR;
	uint32_t DCR;
	uint32_t DMAR;
} tim_t;

extern volatile tim_t TIM1;
extern volatile tim_t TIM2;
extern volatile tim_t TIM3;
extern volatile tim_t TIM4;
extern volatile tim_t TIM5;
extern volatile tim_t TIM6;
extern volatile tim_t TIM7;
extern volatile tim_t TIM8;
extern volatile tim_t TIM9;
extern volatile tim_t TIM10;
extern volatile tim_t TIM11;
extern volatile tim_t TIM12;
extern volatile tim_t TIM13;
extern volatile tim_t TIM14;

#endif
