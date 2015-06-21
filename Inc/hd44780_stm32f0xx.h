/**
* @file    hd44780_stm32f0xx.h 
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

#ifndef HD44780_STM32F0XX_H_
#define HD44780_STM32F0XX_H_
#ifdef __cplusplus
extern "C" {
#endif
//#include <stm32f0xx_rcc.h>
#include "gpio.h"
#include <stdint.h>
#include "hd44780.h"
	
typedef struct {
	GPIO_TypeDef *gpio;
	uint16_t pinmask;
} HD44780_STM32F0xx_Pin;

typedef struct {
  HD44780_STM32F0xx_Pin pins[HD44780_PINS_AMOUNT];
} HD44780_STM32F0xx_Pinout;

typedef struct {
  HD44780_GPIO_Interface interface;
  HD44780_STM32F0xx_Pinout pinout;
  HD44780_AssertFn assert_failure_handler;
} HD44780_STM32F0xx_GPIO_Driver;

extern const HD44780_GPIO_Interface HD44780_STM32F0XX_PINDRIVER_INTERFACE;
#ifdef __cplusplus
}
#endif
#endif /* HD44780_STM32F0XX_H_ */

