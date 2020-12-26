#include "i2c.h"
#include <stm32l0xx.h>
#include <stddef.h>

// sets up I2C1 on PB8 (SCL) and PB9 (SDA)
// connected to LCD shield
// standard mode (100kHz), 7 bit addressing
void i2c_setup(void) {
	RCC->IOPENR |= RCC_IOPENR_IOPBEN; // enable peripheral clock for port B
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // enable peripheral clock for I2C1
	RCC->CCIPR &= ~RCC_CCIPR_I2C1SEL_Msk; // select I2C1 clock source 0 = APB

	// set alternate functions for PB8 and PB9 to AF4 (I2C1)
	GPIOB->AFR[1] = (GPIOB->AFR[1]
			& ~(GPIO_AFRH_AFSEL8_Msk | GPIO_AFRH_AFSEL9_Msk))
			| ((4 << GPIO_AFRH_AFSEL8_Pos) & GPIO_AFRH_AFSEL8_Msk)
			| ((4 << GPIO_AFRH_AFSEL9_Pos) & GPIO_AFRH_AFSEL9_Msk);

	// set GPIO mode for PB8 and PB9 to alternate function (mode 2)
	GPIOB->MODER = (GPIOB->MODER
			& ~(GPIO_MODER_MODE8_Msk | GPIO_MODER_MODE9_Msk))
			| ((2 << GPIO_MODER_MODE8_Pos) & GPIO_MODER_MODE8_Msk)
			| ((2 << GPIO_MODER_MODE9_Pos) & GPIO_MODER_MODE9_Msk);

	GPIOB->OTYPER |= GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9; // open drain

	// pull up
	GPIOB->PUPDR = (GPIOB->PUPDR
			& ~(GPIO_PUPDR_PUPD8_Msk | GPIO_PUPDR_PUPD9_Msk))
			| ((1 << GPIO_PUPDR_PUPD8_Pos) & GPIO_PUPDR_PUPD8_Msk)
			| ((1 << GPIO_PUPDR_PUPD9_Pos) & GPIO_PUPDR_PUPD9_Msk);

	// I2C timing setup for 16MHz clock (Pg. 720, Table 125):
	// timings prescaler: 3
	// SCL low period: 5 µs
	// SCL high period: 4 µs
	// data hold time: 500 ns
	// data setup time: 1250 ns
	I2C1->TIMINGR = ((3 << I2C_TIMINGR_PRESC_Pos) & I2C_TIMINGR_PRESC_Msk)
			| ((0x13 << I2C_TIMINGR_SCLL_Pos) & I2C_TIMINGR_SCLL_Msk)
			| ((0x0f << I2C_TIMINGR_SCLH_Pos) & I2C_TIMINGR_SCLH_Msk)
			| ((0x02 << I2C_TIMINGR_SDADEL_Pos) & I2C_TIMINGR_SDADEL_Msk)
			| ((0x04 << I2C_TIMINGR_SCLDEL_Pos) & I2C_TIMINGR_SCLDEL_Msk);

	// enable I2C peripheral
	I2C1->CR1 |= I2C_CR1_PE;
}

bool i2c_transfer(uint8_t slave_address, const uint8_t *write_buffer,
		uint8_t bytes_to_write, uint8_t *read_buffer, uint8_t bytes_to_read) {

	const bool read_pending = (read_buffer != NULL) && (bytes_to_read > 0);

	while ((I2C1->ISR & I2C_ISR_BUSY) != 0) {
		// wait for any communication in progress to finish
	}

	// set 7 bit slave address
	I2C1->CR2 = (I2C1->CR2 & ~I2C_CR2_SADD_Msk)
			| ((slave_address & 0x7fu) << 1u);

	if ((write_buffer != NULL) && (bytes_to_write > 0)) {
		// set transfer direction to write
		I2C1->CR2 &= ~I2C_CR2_RD_WRN;

		// set number of bytes to write
		I2C1->CR2 = (I2C1->CR2 & ~I2C_CR2_NBYTES_Msk)
				| ((uint32_t) (bytes_to_write << I2C_CR2_NBYTES_Pos)
						& I2C_CR2_NBYTES_Msk);

		if (read_pending) {
			// disable autoend
			I2C1->CR2 &= ~I2C_CR2_AUTOEND;
		} else {
			// enable autoend
			I2C1->CR2 |= I2C_CR2_AUTOEND;
		}

		// start transfer
		I2C1->CR2 |= I2C_CR2_START;

		while (bytes_to_write--) {
			while ((I2C1->ISR & I2C_ISR_TXIS) == 0) {
				// wait until TXDR is empty and new data can be written

				if ((I2C1->ISR & I2C_ISR_NACKF) != 0) {
					// NACK received
					I2C1->ICR &= ~I2C_ICR_NACKCF; // clear NACKF
					return false;
				}

			}

			// send data
			I2C1->TXDR = *write_buffer++;
		}

		if (read_pending) {
			while ((I2C1->ISR & I2C_ISR_TC) == 0) {
				// wait until transfer is complete
			}
		}
	}

	if (read_pending) {
		// set transfer direction to read
		I2C1->CR2 |= I2C_CR2_RD_WRN;

		// set number of bytes to read
		I2C1->CR2 = (I2C1->CR2 & ~I2C_CR2_NBYTES_Msk)
				| ((uint32_t) (bytes_to_read << I2C_CR2_NBYTES_Pos)
						& I2C_CR2_NBYTES_Msk);

		// start transfer
		I2C1->CR2 |= I2C_CR2_START;

		// enable autoend
		I2C1->CR2 |= I2C_CR2_AUTOEND;

		for (uint8_t i = 0; i < bytes_to_read; i++) {
			while ((I2C1->ISR & I2C_ISR_RXNE) == 0) {
				// wait until data is ready to be read from RXDR
			}
			read_buffer[i] = I2C1->RXDR & I2C_RXDR_RXDATA_Msk; // read data received
		}
	}

	return true;
}
