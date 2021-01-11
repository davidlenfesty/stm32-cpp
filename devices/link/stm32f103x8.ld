MEMORY
{
	FLASH(RX):	ORIGIN = 0x08000000, LENGTH = 64K
	SRAM(RW):	ORIGIN = 0x20000000, LENGTH = 20K

	/* SFRs */
	AHB(RW):	ORIGIN = 0x40020000, LENGTH = 32K
	APB2(RW):	ORIGIN = 0x40010000, LENGTH = 32K
	APB1(RW):	ORIGIN = 0x40000000, LENGTH = 32K
}

SECTIONS
{
	/* Code comes right after the vector tables */
	.text :
	{
		*(.vector)
		*(.text.*)
		*(.rodata)
		*(.ctors)
		. = ALIGN(4);
		__ctor_start = .;
		*(.init_array)
		__ctor_end = .;
		__dtor_start = .;
		*(.fini_array)
		__dtor_end = .;
	} > FLASH

	/* Initialized data needs special handling because it lives in two places */
	.data :
	{
		__data_romstart = LOADADDR(.data);
		__data_start = .;
		*(.data)
		__data_end = .;
	} > SRAM AT> FLASH

	/* BSS is pretty straightforward. Use all remaining space for the stack */
	.bss :
	{
		__bss_start__ = .;
		*(.bss)
		__bss_end__ = .;
		__end = .;
		. = ORIGIN(SRAM) + LENGTH(SRAM) - 4;
		__stack = .;

	} > SRAM

	/* SFRs */
	.sfr_apb1 :
	{
		. = ALIGN(1024);
		*(.tim2)
		. = ALIGN(1024);
		*(.tim3)
		. = ALIGN(1024);
		*(.tim4)
		. += 7168;		/* reserved */
		. = ALIGN(1024);
		*(.rtc)
		. = ALIGN(1024);
		*(.wwdg)
		. = ALIGN(1024);
		*(.iwdg)
		. = ALIGN(1024);
		. += 1024;      /* reserved */
		. = ALIGN(1024);
		*(.spi2)
		. = ALIGN(1024);
		. += 2048;      /* reserved */
		. = ALIGN(1024);
		*(.usart2)
		. = ALIGN(1024);
		*(.usart3)
		. += 2048;      /* reserved */
		. = ALIGN(1024);
		*(.i2c1)
		. = ALIGN(1024);
		*(.i2c2)
		. = ALIGN(1024);
		*(.usb)
		. = ALIGN(1024);
		*(.usb_can_sram)
		. = ALIGN(1024);
		*(.bxcan)
		. += 1024;      /* reserved */
		. = ALIGN(1024);
		*(.bkp)
		. = ALIGN(1024);
		*(.pwr)

	} > APB1

	.sfr_apb2 :
	{
		. = ALIGN(1024);
		*(.afio)
		. = ALIGN(1024);
		*(.exti)
		. = ALIGN(1024);
		*(.gpioa)
		. = ALIGN(1024);
		*(.gpiob)
		. = ALIGN(1024);
		*(.gpioc)
		. = ALIGN(1024);
		*(.gpiod)
		. = ALIGN(1024);
		*(.gpioe)
		. += 2048;      /* reserved */
		. = ALIGN(1024);
		*(.adc1)
		. = ALIGN(1024);
		*(.adc2)
		. = ALIGN(1024);
		*(.tim1)
		. = ALIGN(1024);
		*(.spi1)
		. += 1024;      /* reserved */
		. = ALIGN(1024);
		*(.usart1)

	} > APB2

	.sfr_ahb :
	{
		. = ALIGN(1024);
		*(.dma)
		. += 2048;      /* reserved */
		. = ALIGN(1024);
		*(.rcc)
		. += 2048;      /* reserved */
		. = ALIGN(1024);
		*(.flash)
		. += 2048;      /* reserved */
		. = ALIGN(1024);
		*(.crc)

	} > AHB

}
