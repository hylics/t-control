/**
* @file    hd44780_stm32f0xx.c 
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

//#include <stm32f0xx_rcc.h>
#include "gpio.h"
#include <stdint.h>
#include <stdlib.h>
#include "hd44780_stm32f0xx.h"
//#ifdef USE_FULL_ASSERT
#ifndef NDEBUG
#define HD44780_STM32F0XX_ASSERT(x) \
{ \
if (!(x)) \
{ \
HD44780_STM32F0xx_GPIO_Driver *driver = ((HD44780_STM32F0xx_GPIO_Driver*)interface); \
if (driver->assert_failure_handler != NULL) \
driver->assert_failure_handler(__FILE__, __LINE__); \
} \
}
#define HD44780_STM32F0XX_RETURN_ASSERT(x,ret) \
do { \
int condition = (x); \
HD44780_STM32F0XX_ASSERT(condition) \
if (!condition) \
return (ret); \
} while (0)
#else
#define HD44780_STM32F0XX_ASSERT(x)
#define HD44780_STM32F0XX_RETURN_ASSERT(x,ret)
#endif


/**
* @brief Fills each GPIO_InitStruct member with its default value.
* @param GPIO_InitStruct : pointer to a GPIO_InitTypeDef structure which will
* be initialized.
* @retval None
*/
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct) {
	/* Reset GPIO init structure parameters values */
	GPIO_InitStruct->Pin = GPIO_PIN_All;
	GPIO_InitStruct->Speed = GPIO_SPEED_LOW;
	GPIO_InitStruct->Mode = GPIO_MODE_INPUT;
}

static HD44780_Result stm32f0xx_default_pin_configure(
	HD44780_GPIO_Interface *interface, HD44780_Pin pin, HD44780_PinMode mode)
{
	HD44780_STM32F0XX_RETURN_ASSERT(interface != NULL, HD44780_RESULT_ERROR);
	HD44780_STM32F0xx_GPIO_Driver *driver = (HD44780_STM32F0xx_GPIO_Driver*)interface;
	HD44780_STM32F0xx_Pin *hw_pin = &driver->pinout.pins[pin];
	HD44780_STM32F0XX_RETURN_ASSERT(hw_pin != NULL, HD44780_RESULT_ERROR);
	HD44780_STM32F0XX_RETURN_ASSERT(hw_pin->gpio != NULL, HD44780_RESULT_ERROR);
	GPIO_InitTypeDef gpio_config;
	GPIO_StructInit(&gpio_config);
	switch (mode)
	{
		case HD44780_PINMODE_OUTPUT:
		gpio_config.Mode = GPIO_MODE_OUTPUT_PP;
		break;
		case HD44780_PINMODE_INPUT:
		gpio_config.Mode = GPIO_MODE_INPUT;
		break;
		default:
		HD44780_STM32F0XX_ASSERT(0);
		break;
	}
	gpio_config.Pin = hw_pin->pinmask;
	if (hw_pin->gpio != NULL)
	HAL_GPIO_Init(hw_pin->gpio, &gpio_config);
	return HD44780_RESULT_OK;
}


static HD44780_Result stm32f0xx_default_pin_write(
HD44780_GPIO_Interface *interface, HD44780_Pin pin, HD44780_PinState value)
{
	HD44780_STM32F0XX_RETURN_ASSERT(interface != NULL, HD44780_RESULT_ERROR);
	HD44780_STM32F0xx_GPIO_Driver *driver = (HD44780_STM32F0xx_GPIO_Driver*)interface;
	HD44780_STM32F0xx_Pin *hw_pin = &driver->pinout.pins[pin];
	HD44780_STM32F0XX_RETURN_ASSERT(hw_pin != NULL, HD44780_RESULT_ERROR);
	HD44780_STM32F0XX_RETURN_ASSERT(hw_pin->gpio != NULL, HD44780_RESULT_ERROR);
	HAL_GPIO_WritePin(hw_pin->gpio, hw_pin->pinmask,
	(value == HD44780_PINSTATE_LOW ? GPIO_PIN_RESET : GPIO_PIN_SET));
	return HD44780_RESULT_OK;
}


static HD44780_Result stm32f0xx_default_pin_read(
HD44780_GPIO_Interface *interface, HD44780_Pin pin, HD44780_PinState *value)
{
	HD44780_STM32F0XX_RETURN_ASSERT(interface != NULL, HD44780_RESULT_ERROR);
	HD44780_STM32F0XX_RETURN_ASSERT(value != NULL, HD44780_RESULT_ERROR);
	HD44780_STM32F0xx_GPIO_Driver *driver = (HD44780_STM32F0xx_GPIO_Driver*)interface;
	HD44780_STM32F0xx_Pin *hw_pin = &driver->pinout.pins[pin];
	HD44780_STM32F0XX_RETURN_ASSERT(hw_pin != NULL, HD44780_RESULT_ERROR);
	HD44780_STM32F0XX_RETURN_ASSERT(hw_pin->gpio != NULL, HD44780_RESULT_ERROR);
	GPIO_PinState out_bit = HAL_GPIO_ReadPin(hw_pin->gpio, hw_pin->pinmask);
	*value = (out_bit == GPIO_PIN_RESET ? HD44780_PINSTATE_LOW : HD44780_PINSTATE_HIGH);
	return HD44780_RESULT_OK;
}


const HD44780_GPIO_Interface HD44780_STM32F0XX_PINDRIVER_INTERFACE =
{
	stm32f0xx_default_pin_configure,
	stm32f0xx_default_pin_write,
	stm32f0xx_default_pin_read
};

