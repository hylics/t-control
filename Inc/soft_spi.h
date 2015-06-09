/*



*/

#ifndef __SOFT_SPI_H
#define __SOFT_SPI_H

#define MOSI_HIGH (GPIOB->BSRR = (uint32_t)GPIO_PIN_15)
#define MOSI_LOW (GPIOB->BSRR = (uint32_t)GPIO_PIN_15 << 16)

#define SCK_HIGH (GPIOB->BSRR = (uint32_t)GPIO_PIN_13)
#define SCK_LOW (GPIOB->BSRR = (uint32_t)GPIO_PIN_13 << 16)

#define MISO_STATE (GPIOB->IDR & GPIO_PIN_14)

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"


void spi_send_byte(uint8_t data);
uint8_t spi_read_byte(void);

#endif /* __SOFT_SPI_H */

