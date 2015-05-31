/***************************************************************************//**
 *   @file   Communication.h
 *   @brief  Header file of Communication Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 501
*******************************************************************************/
#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

#include "gpio.h"
#include "spi.h"
#include "stm32f0xx_hal.h"
//#include "AD7792.h"

/******************************************************************************/
/* SPI-AD7792 timeout definitions                                             */
/******************************************************************************/

//#define SPI_TIMEOUT			20    //20 milliseconds for one SPI transmit or receive from ad7792
#define TIMEOUT_COMMAND 2     //timeout address <-> command
#define TIMEOUT_PACKET  50    //timeout address,command <-> address,command
#define TIMEOUT_LH      1     //timeout CS LOW or HIGH <-> transmit or receive
#define TIMEOUT_RESET   2000  //timeout after reset
#define TIMEOUT_WRGL    300    //timeout after Rdy pin go to Low state
#define ADI_DELAY(x)    (HAL_Delay(x))

/******************************************************************************/
/* GPIO Definitions                                                           */
/******************************************************************************/

#define ADI_PAR_CS_PIN        		// Add code here
#define ADI_PART_CS_PIN_OUT   		// Add code here
#define ADI_PART_CS_LOW        		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define ADI_PART_CS_HIGH       		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define GPIO1_PIN              		// Add code here
#define GPIO1_STATE            		HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) // Add code here
#define SPI_HW                    &hspi2

/******************************************************************************/
/* Functions Prototypes                                                       */
/******************************************************************************/
/* Initializes the SPI communication peripheral. */
uint8_t SPI_Init(uint8_t lsbFirst, uint32_t clockFreq, uint8_t clockPol, uint8_t clockPha);
/* Writes data to SPI. */
uint8_t SPI_Write(uint8_t *data, uint8_t bytesNumber);
/* Reads data from SPI. */
uint8_t SPI_Read(uint8_t *data, uint8_t bytesNumber);

#endif	// _COMMUNICATION_H_
