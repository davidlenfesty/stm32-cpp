/***********************************************************************************************************************
*                                                                                                                      *
* STM32-CPP v0.1                                                                                                       *
*                                                                                                                      *
* Copyright (c) 2020 Andrew D. Zonenberg                                                                               *
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

#ifndef stm32f031_h
#define stm32f031_h

#include <stdint.h>

typedef struct
{
	uint32_t MODER;
	uint32_t OTYPER;
	uint32_t OSPEEDR;
	uint32_t PUPDR;
	uint32_t IDR;
	uint32_t ODR;
	uint32_t BSRR;
	uint32_t LCKR;
	uint32_t AFRL;
	uint32_t AFRH;
} gpio_t;

extern volatile gpio_t GPIOA;
extern volatile gpio_t GPIOB;
extern volatile gpio_t GPIOC;

enum rcc_ahb
{
	RCC_AHB_GPIOA	= 0x020000,
	RCC_AHB_GPIOB	= 0x040000,
	RCC_AHB_GPIOC	= 0x080000,
	RCC_AHB_GPIOD	= 0x100000,
	RCC_AHB_GPIOE	= 0x200000,
	RCC_AHB_GPIOF	= 0x400000
};

enum rcc_apb2
{
	RCC_APB2_USART1 = 0x4000,
	RCC_APB2_SPI1	= 0x1000,
	RCC_APB2_TIM1	= 0x0800
};

enum rcc_apb1
{
	RCC_APB1_TIM3	= 0x0002,
	RCC_APB1_TIM2	= 0x0001
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
	uint32_t AHBRSTR;
	uint32_t CFGR2;
	uint32_t CFGR3;
	uint32_t CR2;
} rcc_t;

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

extern volatile rcc_t RCC;

typedef struct
{
	uint32_t CR1;
	uint32_t CR2;
	uint32_t CR3;
	uint32_t BRR;
	uint32_t GTPR;
	uint32_t RTOR;
	uint32_t RQR;
	uint32_t ISR;
	uint32_t ICR;
	uint32_t RDR;
	uint32_t TDR;
} usart_t;

enum usart_bits
{
	USART_ISR_TXE = 0x80,
	USART_ISR_RXNE = 0x20
};

extern volatile usart_t USART1;

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
	uint32_t field_30;
	uint32_t CCR1;
	uint32_t CCR2;
	uint32_t CCR3;
	uint32_t CCR4;
	uint32_t field_44;
	uint32_t DCR;
	uint32_t DMAR;
} tim_t;

extern volatile tim_t TIM2;
extern volatile tim_t TIM3;

typedef struct
{
	uint32_t CFGR1;
	uint32_t EXTICR1;
	uint32_t EXTICR2;
	uint32_t EXTICR3;
	uint32_t EXTICR4;
	uint32_t CFGR2;
} syscfg_t;

extern volatile syscfg_t SYSCFG;

typedef struct
{
	uint32_t CR1;
	uint32_t CR2;
	uint32_t SR;
	uint8_t DR;			//STM32f0x1 datasheet page 807, 28.9.4 says the register is 16-bits wide, and that
						//"Unused bits are ignored when writing to the register". This is untrue.
						//If you access DR as a 16-bit write, you get *two* bytes of data sent.
	uint8_t padding1;
	uint16_t padding2;
	uint32_t CRCPR;
	uint32_t RXCRCR;
	uint32_t TXCRCR;
	uint32_t I2SCFGR;
	uint32_t I2SPR;
} spi_t;

enum spi_cr1_bits
{
	SPI_BIDI_MODE	= 0x8000,
	SPI_BIDI_OE		= 0x4000,
	SPI_RX_ONLY		= 0x0400,
	SPI_SOFT_CS		= 0x0200,
	SPI_INTERNAL_CS	= 0x0100,
	SPI_LSB_FIRST	= 0x0080,
	SPI_ENABLE		= 0x0040,
	SPI_MASTER		= 0x0004
};

extern volatile spi_t SPI1;

#endif
