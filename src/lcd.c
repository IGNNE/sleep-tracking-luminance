#include "lcd.h"
#include "delay.h"
#include "clock.h"
#include "i2c.h"
#include "systick.h"
#include <stdbool.h>
#include <stddef.h>

#define LCD_I2C_ADDRESS 0x3e

// lcd_instructions
#define CLEAR_DISPLAY			0x01
#define RETURN_HOME				0x02
#define ENTRY_MODE_SET			0x04
#define DISPLAY_CONTROL			0x08
#define CURSOR_DISPLAY_SHIFT	0x10
#define FUNCTION_SET			0x20
#define SET_CGRAM_ADDRESS		0x40
#define SET_DDRAM_ADDRESS		0x80

// lcd_entry_mode_options
#define DECREMENT					0x00
#define INCREMENT					0x02
#define ACCOMPANIES_DISPLAY_SHIFT	0x01

// lcd_display_control_options
#define CURSOR_BLINK	0x01
#define CURSOR_ON		0x02
#define DISPLAY_ON		0x04

// lcd_cursor_display_shift_options
#define CURSOR_SHIFT	0x00
#define DISPLAY_SHIFT	0x08
#define SHIFT_LEFT		0x00
#define SHIFT_RIGHT		0x04

// lcd_function_set_options
#define FONT_5X8	0x00
#define FONT_5X10	0x04
#define ONE_LINE	0x00
#define TWO_LINES	0x08
#define DATA_4BIT	0x00
#define DATA_8BIT	0x10

#define LCD_LINES               2
#define LCD_CHARACTERS_PER_LINE 16
#define LCD_FIRST_LINE_BEGIN    0x00
#define LCD_FIRST_LINE_END      (LCD_FIRST_LINE_BEGIN + LCD_CHARACTERS_PER_LINE)
#define LCD_SECOND_LINE_BEGIN   0x40 // the display can scroll horizontally through 64 characters
#define LCD_SECOND_LINE_END     (LCD_SECOND_LINE_BEGIN + LCD_CHARACTERS_PER_LINE)

static uint8_t cursor_position = LCD_FIRST_LINE_BEGIN;

/* internal functions */

static void lcd_write(uint8_t value, bool instruction) {
	uint8_t write_buffer[] = { instruction ? 0x80 : 0x40, value };
	i2c_transfer(LCD_I2C_ADDRESS, write_buffer, 2, NULL, 0);
}

static void lcd_instruction(uint8_t value) {
	lcd_write(value, true);
}

static void lcd_write_data(uint8_t value) {
	lcd_write(value, false);
}

/* public functions */

void lcd_init(void) {
	// lcd initialization sequence
	delay_ms(15);
	lcd_instruction(FUNCTION_SET | TWO_LINES | FONT_5X8);
	delay_us(39);
	lcd_instruction(DISPLAY_CONTROL | DISPLAY_ON | CURSOR_ON | CURSOR_BLINK);
	delay_us(39);
	lcd_instruction(CLEAR_DISPLAY);
	delay_ms(2);
}

void lcd_print_char(char character) {
	if (cursor_position == LCD_FIRST_LINE_BEGIN) {
		lcd_clear_display();
	}

	lcd_write_data(character);

	cursor_position++;
	if (cursor_position == LCD_FIRST_LINE_END) {
		// set cursor to beginning of next line
		cursor_position = LCD_SECOND_LINE_BEGIN;
		lcd_instruction(SET_DDRAM_ADDRESS | cursor_position);
	} else if (cursor_position == LCD_SECOND_LINE_END) {
		// set cursor to beginning of first line
		cursor_position = LCD_FIRST_LINE_BEGIN;
	}

}

void lcd_print_string(const char *zstr) {
	if (zstr == NULL) {
		return;
	}
	char character = 0;
	while ((character = *zstr++) != '\0') {
		lcd_print_char(character);
	}
}

void lcd_clear_display(void) {
	lcd_instruction(CLEAR_DISPLAY);
	cursor_position = 0;
	delay_ms(2);
}

void lcd_return_home(void) {
	lcd_instruction(RETURN_HOME);
	cursor_position = 0;
	delay_ms(2);
}

void lcd_set_cursor(uint8_t row, uint8_t column) {
	cursor_position = (((row % LCD_LINES) * LCD_SECOND_LINE_BEGIN)
			+ (column % LCD_CHARACTERS_PER_LINE));
	lcd_instruction(SET_DDRAM_ADDRESS | cursor_position);
}
