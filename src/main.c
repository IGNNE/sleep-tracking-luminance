#include "clock.h"
#include "systick.h"
#include "i2c.h"
#include "lcd.h"
#include "custom_adc_lib.h"
#include <delay.h>
#include <stdio.h>
#include <stm32l073xx.h>

float VoutArray[] =  { 0.0011498,  0.0033908,   0.011498, 0.041803,0.15199,     0.53367, 1.3689,   1.9068,  2.3};
float  LuxArray[] =  { 1.0108,     3.1201,  9.8051,   27.43,   69.545,   232.67,  645.11,   73.52,  1000};

/**
 * Slightly modified from https://wiki.seeedstudio.com/Grove-Luminance_Sensor/
 *
 * This code uses MultiMap implementation from http://playground.arduino.cc/Main/MultiMap
 */
float FmultiMap(float val, float * _in, float * _out, uint8_t size)
{
    // take care the value is within range
    // val = constrain(val, _in[0], _in[size-1]);
    if (val <= _in[0]) return _out[0];
    if (val >= _in[size-1]) return _out[size-1];

    // search right interval
    uint8_t pos = 1;  // _in[0] allready tested
    while(val > _in[pos]) pos++;

    // this will handle all exact "points" in the _in array
    if (val == _in[pos]) return _out[pos];

    // interpolate in the right segment for the rest
    return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}

int main(void) {
	clock_setup_16MHz();
	systick_setup();
	i2c_setup();
	lcd_init();
	lcd_print_string("    Embedded    "
	                 "    Systems!    ");

	// set up gpios
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	GPIOA->MODER &=~(GPIO_MODER_MODE0);

	adc_init();

	while (1) {
		float sensor_voltage = (float)(read_vdda() * read_adc_raw_blocking(ADC_CHANNEL_0)) / (4095 * 1000);

		float luminance = FmultiMap(sensor_voltage, VoutArray, LuxArray, 9);

		char buffer[10];
		snprintf(buffer, sizeof(buffer), "%.1f lux", luminance);
		lcd_clear_display();
		lcd_print_string(buffer);
		delay_ms(500);
	}
}
