#include "clock.h"
#include "systick.h"
#include "i2c.h"
#include "lcd.h"
#include "custom_adc_lib.h"
#include <delay.h>
#include <stdio.h>
#include <stm32l073xx.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

float VoutArray[] = { 0.0011498, 0.0033908, 0.011498, 0.041803, 0.15199,
		0.53367, 1.3689, 1.9068, 2.3 };
float LuxArray[] = { 1.0108, 3.1201, 9.8051, 27.43, 69.545, 232.67, 645.11,
		73.52, 1000 };

#define DATA_ARRAY_SIZE 500
float * data_base_ptr = (float *) (DATA_EEPROM_BASE);
size_t * data_array_pos_ptr = (size_t *) (DATA_EEPROM_BASE
		+ (DATA_ARRAY_SIZE * sizeof(float)));
bool * buffer_overflow_ptr = (bool *) (DATA_EEPROM_BASE
		+ (DATA_ARRAY_SIZE * sizeof(float)) + sizeof(size_t));

const int NUM_AVERAGE_VALUES = 6;

/**
 * Slightly modified from https://wiki.seeedstudio.com/Grove-Luminance_Sensor/
 *
 * This code uses MultiMap implementation from http://playground.arduino.cc/Main/MultiMap
 */
float FmultiMap(float val, float * _in, float * _out, uint8_t size) {
	// take care the value is within range
	// val = constrain(val, _in[0], _in[size-1]);
	if (val <= _in[0])
		return _out[0];
	if (val >= _in[size - 1])
		return _out[size - 1];

	// search right interval
	uint8_t pos = 1;  // _in[0] allready tested
	while (val > _in[pos])
		pos++;

	// this will handle all exact "points" in the _in array
	if (val == _in[pos])
		return _out[pos];

	// interpolate in the right segment for the rest
	return (val - _in[pos - 1]) * (_out[pos] - _out[pos - 1])
			/ (_in[pos] - _in[pos - 1]) + _out[pos - 1];
}

static void usart_setup(void) {
	// Enable peripheral clock for port A
	RCC->IOPENR |= RCC_IOPENR_IOPAEN;

	// Set alternate functions for PA2 and PA3 to AF4 (USART2)
	GPIOA->AFR[0] = (GPIOA->AFR[0]
			& ~(GPIO_AFRL_AFSEL2_Msk | GPIO_AFRL_AFSEL3_Msk))
			| ((4 << GPIO_AFRL_AFSEL2_Pos) & GPIO_AFRL_AFSEL2_Msk)
			| ((4 << GPIO_AFRL_AFSEL3_Pos) & GPIO_AFRL_AFSEL3_Msk);

	// Set GPIO mode for PA2 and PA3 to alternate function (mode 2)
	GPIOA->MODER = (GPIOA->MODER
			& ~(GPIO_MODER_MODE2_Msk | GPIO_MODER_MODE3_Msk))
			| ((2 << GPIO_MODER_MODE2_Pos) & GPIO_MODER_MODE2_Msk)
			| ((2 << GPIO_MODER_MODE3_Pos) & GPIO_MODER_MODE3_Msk);

	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // enable peripheral clock for USART2

	// Set Baudrate to 115200
	USART2->BRR = 16000000 / 115200;
	// Enable USART2 receiver, transmitter, RX interrupt and USART
	USART2->CR1 |= (USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE
			| USART_CR1_UE);

	NVIC_EnableIRQ(USART2_IRQn);
}

/**
 * Lässt Schreib-/Lesezugriff auf Flash zu
 */
void flash_init() {
	while ((FLASH->SR & FLASH_SR_BSY) != 0)
		;
	if ((FLASH->PECR & FLASH_PECR_PELOCK) != 0) {
		FLASH->PEKEYR = FLASH_PEKEY1;
		FLASH->PEKEYR = FLASH_PEKEY2;
	}
}

/**
 * Speichert einen Wert in einer Art Ringbuffer
 * @param lum Wert, der gespeichert werden soll
 */
void flash_save_value(float lum) {
	if ((*data_array_pos_ptr) >= DATA_ARRAY_SIZE) {
		(*data_array_pos_ptr) = 0;
		(*buffer_overflow_ptr) = true;
	}
	data_base_ptr[(*data_array_pos_ptr)] = lum;
	(*data_array_pos_ptr)++;
}

/**
 * Verwirft alle Werte im Flash
 */
void flash_reset() {
	(*data_array_pos_ptr) = 0;
	(*buffer_overflow_ptr) = false;
}

/**
 * Gibt alle Werte als ein zusammenhängendes Array zurück
 *
 * @param[in,out] val_array Array mit min. ::DATA_ARRAY_SIZE Einträgen
 *
 * @return Die tatsächlich geschriebenen Array-Einträge
 *
 */
size_t flash_get_values(float * val_array) {
	if (!*buffer_overflow_ptr) {
		// no overflow, just copy the values
		size_t values_in_bytes = (*data_array_pos_ptr) * sizeof(float);
		memcpy(val_array, data_base_ptr, values_in_bytes);
		return *data_array_pos_ptr;
	} else {
		// overflow happened, we need to get a bit more creative to get all values
		size_t num_old_values = DATA_ARRAY_SIZE - (*data_array_pos_ptr);
		size_t index_old_value = (*data_array_pos_ptr);
		memcpy(val_array, &data_base_ptr[index_old_value], num_old_values * sizeof(float)); // copy old values
		memcpy(&val_array[num_old_values], data_base_ptr, index_old_value * sizeof(float)); // copy new values
		return DATA_ARRAY_SIZE;
	}
}
/**
 * @brief Initialisiert die Pins PB3/PB5 um als Buttons genutzt zu werden
 *
 */
void setup_buttons(void) {
	//Setup Button PB3/PB5 -> D3/D4
	RCC->IOPENR |= RCC_IOPENR_GPIOBEN; // Set of RCC Clock IO port enable register bits for clock for GPIO B
	GPIOB->MODER &= ~GPIO_MODER_MODE3; // Clear of GPIO port mode register bits --> input mode is set
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD3; // Clear of GPIO port pull-up/pull-down register bits --> No pull-up pull-down mode
	GPIOB->MODER &= ~GPIO_MODER_MODE5;
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD5;
}

/**
 * Gibt zurück ob der Start Button gedrückt wird
 *
 * @return aktueller Wert des start Button
 *
 */
bool startButton_pressed(void) {
	return !(GPIOB->IDR & GPIO_IDR_ID3);
}

/**
 * Gibt zurück ob der Ende Button gedrückt wird
 *
 * @return aktueller Wert des ende Button
 *
 */
bool stopButton_pressed(void) {
	return !(GPIOB->IDR & GPIO_IDR_ID5);
}

/**
 * @brief Sendet die gespeicherten EEPROM lux-werte seriell über USART
 * @param values_counter Anzahl der gespeicherten Werte im EEPORM dieser session
 * @param val_array Array mit ::DATA_ARRAY_SIZE und den gespeicherten Lux-werten
 */
void send_data(uint32_t values_counter, float * val_array) {
	for (uint32_t i = 0; i < values_counter; i++) {
		char lux_buffer[(sizeof(float) * 8 + 1)];
		sprintf(lux_buffer, "%f ", val_array[i]);
		// Send Lux-values over USART
		for (int j = 0; lux_buffer[j] != '\0'; j++) {
			while (!(USART2->ISR & USART_ISR_TXE))
				; // wait until the TDR register has been read
			USART2->TDR = lux_buffer[j] & USART_TDR_TDR_Msk;
		}
	}
}

/**
 * @brief Schreibt den lux-wert auf das LC-Display
 * @param luminance Wert zum schreiben
 */
void write_lux_to_lcd(float luminance) {
	char buffer[10];
	snprintf(buffer, sizeof(buffer), "%.1f lux", luminance);
	lcd_print_string(buffer);
}

/**
 * @brief Initialisiert alle nötigen Komponenten und führt das Hauptprogramm aus.
 * @return nicht implementiert
 */
int main(void) {
	clock_setup_16MHz();
	systick_setup();
	i2c_setup();
	lcd_init();
	flash_init();
	usart_setup();

	// set up gpios
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	GPIOA->MODER &= ~(GPIO_MODER_MODE0);

	adc_init();
	setup_buttons();

	// Infinite Loop
	while (1) {
		lcd_clear_display();
		lcd_print_string("Gruen: Ausgabe");
		lcd_set_cursor(1, 0);
		lcd_print_string("Rot: Aufnahme");
		delay_ms(500);

		// Check which button is pressed at Startup
		bool record_values = false;
		bool print_values = false;

		uint8_t read_values = 0;	// read values since last sending

		// Waiting loop until recording is started
		while (!record_values && !print_values) {
			if (startButton_pressed()) {
				while (startButton_pressed())
					; // wait until button released
				record_values = true;
			}
			if (stopButton_pressed()) {
				while (stopButton_pressed())
					; // wait until button released
				print_values = true;
			}
		}

		// this saves the averaged value
		float luminance_average = 0;
		// Record data loop
		while (record_values) {
			float sensor_voltage = (float) (read_vdda()
					* read_adc_raw_blocking(ADC_CHANNEL_0)) / (4095 * 1000);

			float luminance = FmultiMap(sensor_voltage, VoutArray, LuxArray, 9);
			luminance_average += luminance;

			lcd_clear_display();
			lcd_print_string("Aufnahme...");
			lcd_set_cursor(1, 0);
			write_lux_to_lcd(luminance);

			// Only send values all 30 seconds
			if (read_values == NUM_AVERAGE_VALUES) {
				flash_save_value(luminance_average / NUM_AVERAGE_VALUES);
				luminance_average = 0;
				read_values = 0;
			}
			read_values++;
			delay_ms(500);

			// Stop recording when start button is pressed again
			if (startButton_pressed()) {
				while (startButton_pressed())
					; // wait until button released
				record_values = false;
			}
		}

		// Send data over serial port after pressing green button
		if (print_values) {
			lcd_clear_display();
			lcd_print_string("Sende...");

			float all_values[DATA_ARRAY_SIZE];
			size_t values_counter = flash_get_values(all_values);
			send_data(values_counter, all_values);
			lcd_print_string("ok");
			delay_ms(2000);

			lcd_clear_display();
			lcd_print_string("Gruen: Weiter");
			lcd_set_cursor(1, 0);
			lcd_print_string("Rot: Loeschen");

			bool delete_or_continue = false;
			while (!delete_or_continue) {
				if (startButton_pressed()) {
					while (startButton_pressed())
						; // wait until button released

					// delete values
					flash_reset();
					lcd_clear_display();
					lcd_print_string("Geloescht!");
					delay_ms(2000);
					delete_or_continue = true;
				}
				if (stopButton_pressed()) {
					while (stopButton_pressed())
						; // wait until button released
					delete_or_continue = true;
				}
			}

			// done printing values
			print_values = false;
		}
	}
}
