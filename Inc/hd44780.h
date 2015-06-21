/**
* @file    hd44780.h
* @author  Artem Borisovskiy (bytefu@gmail.com)
* @date    2012
* @brief   Cross-platform library for LCDs with HD44780-compatible controllers

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software
is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*********************************************************************************/

#ifndef HC44780_H_
#define HC44780_H_
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
typedef enum { HD44780_RESULT_OK, HD44780_RESULT_ERROR } HD44780_Result;
typedef enum { HD44780_PINMODE_INPUT, HD44780_PINMODE_OUTPUT } HD44780_PinMode;
typedef enum { HD44780_PINSTATE_LOW, HD44780_PINSTATE_HIGH } HD44780_PinState;

/* Abstract HD44780 pin definition */
typedef enum {
	HD44780_PIN_RS, // LOW: command. HIGH: character.
	HD44780_PIN_ENABLE, // latch, activated by a HIGH pulse.
	HD44780_PIN_RW, // optional; LOW: write to LCD, HIGH: read from LCD
	HD44780_PIN_BACKLIGHT, // optional; should be connected to base/gate of transistor/FET
	/* Data pins DP0..DP7; in 4-bit mode DP0..DP3 are not needed. */
	HD44780_PIN_DP0,
	HD44780_PIN_DP1,
	HD44780_PIN_DP2,
	HD44780_PIN_DP3,
	HD44780_PIN_DP4,
	HD44780_PIN_DP5,
	HD44780_PIN_DP6,
	HD44780_PIN_DP7,
	HD44780_PINS_AMOUNT // enum member counter, must be last
} HD44780_Pin;

/* Hardware-independent pin control interface.
* configure() function is optional if you want to configure
* the display pins manually.
*/
struct HD44780_GPIO_Interface_Struct;
typedef struct HD44780_GPIO_Interface_Struct HD44780_GPIO_Interface;

struct HD44780_GPIO_Interface_Struct {
	HD44780_Result (*configure)(HD44780_GPIO_Interface *interface, HD44780_Pin pin, HD44780_PinMode mode);
	HD44780_Result (*write)(HD44780_GPIO_Interface *interface, HD44780_Pin pin, HD44780_PinState value);
	HD44780_Result (*read)(HD44780_GPIO_Interface *interface, HD44780_Pin pin, HD44780_PinState *value);
};

typedef void (*HD44780_AssertFn)(const char *filename, unsigned long line);
typedef void (*HD44780_DelayMicrosecondsFn)(uint16_t us);

typedef enum {
	HD44780_OPT_USE_RW = 0x01,
	HD44780_OPT_USE_BACKLIGHT = 0x02,
} HD44780_Options;

/* Hardware abstraction layer */
typedef struct {
	HD44780_GPIO_Interface *gpios;
	HD44780_DelayMicrosecondsFn delay_microseconds;
	HD44780_AssertFn assert_failure_handler;
	HD44780_Options options;
} HD44780_Config;

typedef enum { HD44780_MODE_4BIT, HD44780_MODE_8BIT } HD44780_Mode;
typedef enum { HD44780_CHARSIZE_5x8, HD44780_CHARSIZE_5x10 } HD44780_CharSize;

/* HD44780 control structure */
typedef struct {
	HD44780_Config cfg;
	uint8_t displayfunction;
	uint8_t displaycontrol;
	uint8_t displaymode;
	uint8_t initialized;
	uint8_t columns_amount;
	uint8_t lines_amount;
	uint8_t currline;
	HD44780_Pin dp_first;
	unsigned dp_amount;
} HD44780;

HD44780_Result hd44780_init(HD44780 *display, HD44780_Mode mode,
const HD44780_Config *config, uint8_t columns, uint8_t rows, HD44780_CharSize charsize);
HD44780_Result hd44780_write_char(HD44780 *display, char c);
HD44780_Result hd44780_write_string(HD44780 *display, const char *s);
HD44780_Result hd44780_clear(HD44780 *display);
HD44780_Result hd44780_home(HD44780 *display);
HD44780_Result hd44780_scroll_left(HD44780 *display);
HD44780_Result hd44780_scroll_right(HD44780 *display);
HD44780_Result hd44780_left_to_right(HD44780 *display);
HD44780_Result hd44780_right_to_left(HD44780 *display);
HD44780_Result hd44780_create_char(HD44780 *display, uint8_t code, const uint8_t *charmap);
HD44780_Result hd44780_move_cursor(HD44780 *display, uint8_t column, uint8_t row);
HD44780_Result hd44780_display_on(HD44780 *display);
HD44780_Result hd44780_display_off(HD44780 *display);
HD44780_Result hd44780_blink_on(HD44780 *display);
HD44780_Result hd44780_blink_off(HD44780 *display);
HD44780_Result hd44780_cursor_on(HD44780 *display);
HD44780_Result hd44780_cursor_off(HD44780 *display);
HD44780_Result hd44780_autoscroll_on(HD44780 *display);
HD44780_Result hd44780_autoscroll_off(HD44780 *display);
HD44780_Result hd44780_backlight_on(HD44780 *display);
HD44780_Result hd44780_backlight_off(HD44780 *display);

#define HD44780_MAKE_5BITS(b4,b3,b2,b1,b0) \
(((b0) & 1) | \
((b1) & 1) << 1 | \
((b2) & 1) << 2 | \
((b3) & 1) << 3 | \
((b4) & 1) << 4)
#ifdef __cplusplus
}
#endif
#endif /* HC44780_H_ */

