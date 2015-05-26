/***************************************************************************//**
 *   @file   Communication.c
 *   @brief  Implementation of Communication Driver.
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

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include "Communication.h"

extern __IO uint8_t dma_t_cplt, dma_r_cplt;


//#define DMA_T_CPLT while(dma_t_cplt) {}, dma_t_cplt=1
__STATIC_INLINE void dma_tr_cplt() {
	while(dma_t_cplt) {
		//
	}
	dma_t_cplt = 1;
}

__STATIC_INLINE void dma_rc_cplt() {
	while(dma_r_cplt) {
		//
	}
	dma_r_cplt = 1;
}


/***************************************************************************//**
 * @brief Initializes the SPI communication peripheral. By default calls SPI_Init(0, 1000000, 1, 1);
 *
 * @param lsbFirst - Transfer format (0 or 1).
 *                   Example: 0x0 - MSB first.
 *                            0x1 - LSB first.
 * @param clockFreq - SPI clock frequency (Hz).
 *                    Example: 1000 - SPI clock frequency is 1 kHz.
 * @param clockPol - SPI clock polarity (0 or 1).
 *                   Example: 0x0 - idle state for SPI clock is low.
 *	                          0x1 - idle state for SPI clock is high.
 * @param clockPha - SPI clock phase (0 or 1).
 *                   Example: 0x0 - data is latched on the leading edge of SPI
 *                                  clock and data changes on trailing edge.
 *                            0x1 - data is latched on the trailing edge of SPI
 *                                  clock and data changes on the leading edge.
 *
 * @return 0 - Initialization failed, 1 - Initialization succeeded.
*******************************************************************************/
uint8_t SPI_Init(uint8_t lsbFirst,
                       uint32_t clockFreq,
                       uint8_t clockPol,
                       uint8_t clockPha)
{
	// Add your code here.
	
    return(1);
}

/***************************************************************************//**
 * @brief Writes data to SPI.
 *
 * @param data - Write data buffer:
 *               - first byte is the chip select number;
 *               - from the second byte onwards are located data bytes to write.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
uint8_t SPI_Write(uint8_t *data, uint8_t bytesNumber) {
	uint8_t CS_flag = data[0];
	
  //Modifying CS by this function or external
	if(CS_flag) {
		ADI_PART_CS_LOW;
		ADI_DELAY(TIMEOUT_LH);
	}
	
	//write to address register
	HAL_SPI_Transmit_DMA(SPI_HW, &data[1], (uint16_t)1);
	
	dma_tr_cplt();
	
	ADI_DELAY(TIMEOUT_COMMAND);
	
	//write to ADC
	HAL_SPI_Transmit_DMA(SPI_HW, &data[2], (uint16_t)bytesNumber);
	
	dma_tr_cplt();
	
	//Modifying CS by this function or external
	if(CS_flag) {
		ADI_DELAY(TIMEOUT_LH);
		ADI_PART_CS_HIGH;
	}
	
	ADI_DELAY(TIMEOUT_PACKET);

	return(bytesNumber);
}

/***************************************************************************//**
 * @brief Reads data from SPI.
 *
 * @param data - As an input parameter, data represents the write buffer:
 *               - first byte is the chip select number;
 *               - from the second byte onwards are located data bytes to write.
 *               As an output parameter, data represents the read buffer:
 *               - from the first byte onwards are located the read data bytes. 
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
uint8_t SPI_Read(uint8_t *data, uint8_t bytesNumber) {
	// Add your code here.
	uint8_t CS_flag = data[0];

	if(CS_flag) {
		ADI_PART_CS_LOW;
		ADI_DELAY(TIMEOUT_LH);
	}
	//write to address register
	HAL_SPI_Transmit_DMA(SPI_HW, &data[1], (uint16_t)1);
	//HAL_SPI_Transmit(SPI_HW, &data[1], (uint16_t)1, SPI_TIMEOUT);
	
	dma_tr_cplt();
	
	ADI_DELAY(TIMEOUT_COMMAND);
	
	//read from ADC
	HAL_SPI_Receive_DMA(SPI_HW, &data[0], (uint16_t)bytesNumber);
	//HAL_SPI_Receive(SPI_HW, &data[0], (uint16_t)bytesNumber, SPI_TIMEOUT);
	
	dma_rc_cplt();
	
	if(CS_flag) {
		ADI_DELAY(TIMEOUT_LH);
		ADI_PART_CS_HIGH;
	}
	
	ADI_DELAY(TIMEOUT_PACKET);

	return(bytesNumber);
}
