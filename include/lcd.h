#ifndef LCD_H_
#define LCD_H_

#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdint.h>
#endif

// initializes the lcd
// depends on clock, systick and i2c being initialized
// must be called once before using the other lcd functions
void lcd_init(void);

// prints a single character on the lcd
void lcd_print_char(char character);

// prints a null-terminated string on the lcd
void lcd_print_string(const char *zstr);

// clears the display and returns cursor to the first character
void lcd_clear_display(void);

// returns cursor to the first character
void lcd_return_home(void);

// set cursor position (zero-indexed)
void lcd_set_cursor(uint8_t row, uint8_t column);

#ifdef __cplusplus
}
#endif

#endif
