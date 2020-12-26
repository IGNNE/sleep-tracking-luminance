#include "clock.h"
#include <stm32l073xx.h>

void clock_setup_16MHz(void) {
	// turn on HSI
	RCC->CR |= RCC_CR_HSION;
	while (!(RCC->CR & RCC_CR_HSIRDY)) {
		// wait for HSI ready
	}

	// set prescale factors for peripheral clocks
	RCC->CFGR = (RCC->CFGR
			& ~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk))
			| (RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_PPRE2_DIV1);

	// power settings
	RCC->APB1ENR |= RCC_APB1ENR_PWREN; // enable PWR clock
	PWR->CR = (PWR->CR & ~PWR_CR_VOS)
			| ((2u << PWR_CR_VOS_Pos) & PWR_CR_VOS_Msk); // voltage scale 2: medium power
	FLASH->ACR |= FLASH_ACR_LATENCY; // 1 flash wait state

	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;

	SystemCoreClockUpdate(); // updates cmsis global state
}
