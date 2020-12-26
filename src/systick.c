#include "delay.h"
#include "systick.h"
#include <stm32l0xx.h>

static volatile uint32_t ms_count = 0;

void SysTick_Handler(void) {
	ms_count++;
}

void systick_setup(void) {
	SysTick_Config(16000000 / 1000);
}

void delay_ms(uint32_t ms) {
	const uint32_t end = ms_count + ms;
	while (ms_count < end) {
		// busy wait
	}
}

static uint32_t us_count(void) {
	// the systick counts down from its reload value
	const uint32_t us_since_last_interrupt = 1000
			- ((SysTick->VAL & SysTick_VAL_CURRENT_Msk) / 16u);
	return (ms_count * 1000) + us_since_last_interrupt;
}

void delay_us(uint32_t us) {
	uint32_t start_us = us_count();
	while ((us_count() - start_us) < us) {
		// busy wait
	}
}
