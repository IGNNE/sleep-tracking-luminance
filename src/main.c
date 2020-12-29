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

#define DATA_ARRAY_SIZE 3000
float * data_base_ptr = (float *) (DATA_EEPROM_BASE);
size_t * data_array_pos_ptr = (size_t *) (DATA_EEPROM_BASE
		+ (DATA_ARRAY_SIZE * sizeof(float)));
bool * buffer_overflow_ptr = (bool *) (DATA_EEPROM_BASE + DATA_ARRAY_SIZE
		+ sizeof(size_t));

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
	if (*data_array_pos_ptr >= DATA_ARRAY_SIZE) {
		*data_array_pos_ptr = 0;
		*buffer_overflow_ptr = true;
	}
	data_base_ptr[(*data_array_pos_ptr)++] = lum;
}

/**
 * Verwirft alle Werte im Flash
 */
void flash_reset() {
	*data_array_pos_ptr = 0;
	*buffer_overflow_ptr = false;
}

/**
 * Gibt alle Werte als ein zusammenhängendes Array zurück
 *
 * @param[in,out] val_array Array mit min. ::DATA_ARRAY_SIZE Einträgen zeigen
 *
 * @return Die tatsächlich geschriebenen Array-Einträge
 *
 * @todo pls test!!1!
 */
size_t flash_get_values(float * val_array) {
	if (!*buffer_overflow_ptr) {
		// no overflow, just copy the values
		memcpy(val_array, data_base_ptr, *data_array_pos_ptr);
		return *data_array_pos_ptr;
	} else {
		// overflow happened, we need to get a bit more creative to get all values
		size_t values_before_overflow = DATA_ARRAY_SIZE - (*data_array_pos_ptr);
		size_t values_after_overflow = (*data_array_pos_ptr);
		memcpy(val_array + (*data_array_pos_ptr), data_base_ptr,
				values_before_overflow);
		memcpy(val_array, data_base_ptr + values_before_overflow,
				values_after_overflow);
		return DATA_ARRAY_SIZE;
	}
}


void setup_buttons(void){
	//Setup Button PB3/PB5 -> D3/D4
	RCC->IOPENR |= RCC_IOPENR_GPIOBEN; 		// Set of RCC Clock IO port enable register bits for clock for GPIO B
	GPIOB->MODER &= ~GPIO_MODER_MODE3; 	// Clear of GPIO port mode register bits --> input mode is set
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD3; 	// Clear of GPIO port pull-up/pull-down register bits --> No pull-up pull-down mode
    GPIOB->MODER &= ~GPIO_MODER_MODE5;
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD5;
}

/*void EXTI4_15_IRQHandler(void) {
	if ((EXTI->PR & EXTI_PR_PIF13_Msk) != 0){		//If selected trigger request occured in Pin5
		       EXTI->PR |= EXTI_PR_PIF13;			//Reset Bit to look if trigger occured by writing 1
		       stop_reading = 1;					//Stop reading
		   }
	if ((EXTI->PR & EXTI_PR_PIF5_Msk) != 0){		//If selected trigger request occured in Pin5
			       EXTI->PR |= EXTI_PR_PIF5;			//Reset Bit to look if trigger occured by writing 1
			       stop_reading = 1;					//Stop reading
			   }
}*/

void send_data(){
	//todo
}

int main(void) {
	clock_setup_16MHz();
	systick_setup();
	i2c_setup();
	lcd_init();
	flash_init();
	lcd_print_string("    Embedded    "
			"    Systems!    ");

	// set up gpios
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	GPIOA->MODER &= ~(GPIO_MODER_MODE0);

	adc_init();
	setup_buttons();
	int button_pressed = 0;
	int keep_reading = 1;

	while(button_pressed != 1){
	if (GPIOB->IDR & GPIO_IDR_ID3) { 	// Value of IDR_ID3 (Pin 3) -> high/True, low/False
		}else{
			button_pressed = 1;			// Activate reading
		}
	}
/*
	int * data_ptr = (int *) (DATA_EEPROM_BASE);

	// create random number
	srand(read_adc_raw_blocking(ADC_CHANNEL_0));
	int current = rand();
	int last = *data_ptr;
	*data_ptr = current;
	char buffer[10];

	lcd_print_string("current: ");
	snprintf(buffer, sizeof(buffer), "%x", current);
	lcd_print_string(buffer);
	lcd_set_cursor(1, 0);

	lcd_print_string("last: ");
	snprintf(buffer, sizeof(buffer), "%x", last);
	lcd_print_string(buffer);
*/
	while (keep_reading != 0) {
		float sensor_voltage = (float) (read_vdda()
				* read_adc_raw_blocking(ADC_CHANNEL_0)) / (4095 * 1000);

		float luminance = FmultiMap(sensor_voltage, VoutArray, LuxArray, 9);

		char buffer[10];
		snprintf(buffer, sizeof(buffer), "%.1f lux", luminance);
		lcd_clear_display();
		lcd_print_string(buffer);
		//flash_save_value(luminance);
		delay_ms(500);
		if (GPIOB->IDR & GPIO_IDR_ID5) { 	// Value of IDR_ID3 (Pin 3) -> high/True, low/False
			}else{
				keep_reading = 0;			// Stop reading
			}
		}
	lcd_clear_display();
	lcd_print_string("    STOP    "
				"    Systems!    ");
	}
