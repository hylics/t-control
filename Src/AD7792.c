/***************************************************************************//**
 *   @file   AD7792.c
 *   @brief  Implementation of AD7792 Driver.
 *   @author Bancisor MIhai
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
#include "AD7792.h"				// AD7792 definitions.
#include "Communication.h"		// Communication definitions.

#define TIMEOUT_COMMAND 2     //address <-> command
#define TIMEOUT_PACKET  50    //address,command <-> address,command
#define TIMEOUT_LH      1     //CS LOW or HIGH <-> transmit or receive
#define TIMEOUT_RESET   2000  //after reset



/***************************************************************************//**
 * @brief Initializes the AD7792 and checks if the device is present.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful (ID is 0x0B).
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/
unsigned char AD7792_Init(void)
{ 
	unsigned char status = 0x1;
    
  //SPI_Init(0, 1000000, 1, 1);
	ADI_PART_CS_HIGH;
  if((AD7792_GetRegisterValue(AD7792_REG_ID, 1, 1) & 0x0F) != AD7792_ID)
		{
			status = 0x0;
		}
    
	return(status);
}

/***************************************************************************//**
 * @brief Sends 32 consecutive 1's on SPI in order to reset the part.
 *
 * @return  None.    
*******************************************************************************/
void AD7792_Reset(void)
{
	unsigned char dataToSend[6] = {0x03, 0xff, 0xff, 0xff, 0xff, 0xff};
	
  ADI_PART_CS_LOW;
  ADI_DELAY(TIMEOUT_LH);
	SPI_Write(dataToSend, 5);
	ADI_DELAY(TIMEOUT_LH);
	ADI_PART_CS_HIGH;
	ADI_DELAY(TIMEOUT_RESET);
}
/***************************************************************************//**
 * @brief Reads the value of the selected register
 *
 * @param regAddress - The address of the register to read.
 * @param size - The size of the register to read.
 *
 * @return data - The value of the selected register register.
*******************************************************************************/
unsigned long AD7792_GetRegisterValue(unsigned char regAddress, 
                                      unsigned char size,
                                      unsigned char modifyCS)
{	
	unsigned char data[5]      = {0x00, 0x00, 0x00, 0x00, 0x00};
	unsigned long receivedData = 0x00;
  unsigned char i            = 0x00; 
    
	data[0] = 0x01 * modifyCS;
	data[1] = AD7792_COMM_READ |  AD7792_COMM_ADDR(regAddress);
	SPI_Read(data, (size));
	for(i = 0; i < size; i++)
    {
        receivedData = (receivedData << 8) + data[i];
    }
    
    return (receivedData);
}
/***************************************************************************//**
 * @brief Writes the value to the register
 *
 * @param -  regAddress - The address of the register to write to.
 * @param -  regValue - The value to write to the register.
 * @param -  size - The size of the register to write.
 *
 * @return  None.    
*******************************************************************************/
void AD7792_SetRegisterValue(unsigned char regAddress,
                             unsigned long regValue, 
                             unsigned char size,
                             unsigned char modifyCS)
{
	unsigned char data[5]      = {0x00, 0x00, 0x00, 0x00, 0x00};	
	unsigned char* dataPointer = (unsigned char*)&regValue;
  unsigned char bytesNr      = size + 1;
    
    data[0] = 0x01 * modifyCS;
    data[1] = AD7792_COMM_WRITE |  AD7792_COMM_ADDR(regAddress);
    while(bytesNr > 1)
    {
        data[bytesNr] = *dataPointer;
        dataPointer++;
        bytesNr--;
    }	    
	SPI_Write(data, (size));
}
/***************************************************************************//**
 * @brief  Waits for RDY pin to go low.
 *
 * @return None.
*******************************************************************************/
void AD7792_WaitRdyGoLow(void)
{
    while( AD7792_RDY_STATE )
    {
        ;
    }
}

/***************************************************************************//**
 * @brief Sets the operating mode of AD7792.
 *
 * @param mode - Mode of operation.
 *
 * @return  None.    
*******************************************************************************/
void AD7792_SetMode(unsigned long mode)
{
    unsigned long command;
    
    command = AD7792_GetRegisterValue(AD7792_REG_MODE,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7792_MODE_SEL(0xFF);
    command |= AD7792_MODE_SEL(mode);
    AD7792_SetRegisterValue(
            AD7792_REG_MODE,
            command,
            2, 
            1); // CS is modified by SPI read/write functions.
}

/*my func*/
void AD7792_SetCLCS(unsigned long clc)
{
	unsigned long command;
    
    command = AD7792_GetRegisterValue(AD7792_REG_MODE,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7792_MODE_CLKSRC(0xFF);
    command |= AD7792_MODE_CLKSRC(clc);
    AD7792_SetRegisterValue(
            AD7792_REG_MODE,
            command,
            2, 
            1); // CS is modified by SPI read/write functions.
}

/*my func*/
void AD7792_SetRate(unsigned long rate)
{
	unsigned long command;
    
    command = AD7792_GetRegisterValue(AD7792_REG_MODE,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7792_MODE_RATE(0xFF);
    command |= AD7792_MODE_RATE(rate);
    AD7792_SetRegisterValue(
            AD7792_REG_MODE,
            command,
            2, 
            1); // CS is modified by SPI read/write functions.
}




/*my func*/
void AD7792_EnableBuf(void)
{
	unsigned long command;
	
	command = AD7792_GetRegisterValue(AD7792_REG_CONF,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
	command &= ~AD7792_CONF_BUF;
  command |= AD7792_CONF_BUF;
  AD7792_SetRegisterValue(
            AD7792_REG_CONF,
            command,
            2, 
            1); // CS is modified by SPI read/write functions.
	
}

/*my func*/
void AD7792_IexDir(unsigned long direction)
{
	unsigned long command;
	
	command = AD7792_GetRegisterValue(AD7792_REG_IO,
                                      1,
                                      1); // CS is modified by SPI read/write functions.
	command &= ~AD7792_IEXCDIR(direction);
  command |= AD7792_IEXCDIR(direction);
  AD7792_SetRegisterValue(
            AD7792_REG_IO,
            command,
            1, 
            1); // CS is modified by SPI read/write functions.
	
}

/*my func*/
void AD7792_IexEn(unsigned long current)
{
	unsigned long command;
	
	command = AD7792_GetRegisterValue(AD7792_REG_IO,
                                      1,
                                      1); // CS is modified by SPI read/write functions.
	command &= ~AD7792_IEXCEN(current);
  command |= AD7792_IEXCEN(current);
  AD7792_SetRegisterValue(
            AD7792_REG_IO,
            command,
            1, 
            1); // CS is modified by SPI read/write functions.
	
}

/*my func*/
void AD7792_SetUnipolar(void)
{
	unsigned long command;
	
	command = AD7792_GetRegisterValue(AD7792_REG_CONF,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
	command &= ~AD7792_CONF_UNIPOLAR;
  command |= AD7792_CONF_UNIPOLAR;
  AD7792_SetRegisterValue(
            AD7792_REG_CONF,
            command,
            2, 
            1); // CS is modified by SPI read/write functions.
	
}

/*my func*/

int AD7792_conf(unsigned long gain, unsigned long channel, unsigned long current) {
	unsigned long command;
	int retval = 0;
	
	command = AD7792_GetRegisterValue(AD7792_REG_CONF, 2, 1); // CS is modified by SPI read/write functions.
	command &= ~AD7792_CONF_GAIN(0xFF);
  command |= AD7792_CONF_GAIN(gain);
	
	command &= ~AD7792_CONF_CHAN(0xFF);
  command |= AD7792_CONF_CHAN(channel);
	
	command |= AD7792_CONF_UNIPOLAR;
	
	
	AD7792_SetRegisterValue(AD7792_REG_CONF, command, 2, 1);
	/*if( (AD7792_GetRegisterValue(AD7792_REG_CONF, 2, 1) & command) != command ) {
		  //retval = 1;
	}*/
	
	command &= ~AD7702_FS_1_120ms;
	command |= AD7702_FS_1_120ms;
  AD7792_SetRegisterValue(
            AD7792_REG_MODE,
            command,
            2, 
            1); // CS is modified by SPI read/write functions.
	/*if( (AD7792_GetRegisterValue(AD7792_REG_MODE, 2, 1) & command) != command) {
      //retval = 2;
	}*/
	

	command = 0x0;
	command |= AD7792_IEXCEN(current);
	AD7792_SetRegisterValue(AD7792_REG_IO, command, 1, 1);
	/*if( (AD7792_GetRegisterValue(AD7792_REG_IO, 1, 1) & command) != command) {
		  //retval = 3;
	}*/
	
	return retval;
}



/***************************************************************************//**
 * @brief Selects the channel of AD7792.
 *
 * @param  channel - ADC channel selection.
 *
 * @return  None.    
*******************************************************************************/
void AD7792_SetChannel(unsigned long channel)
{
    unsigned long command;
    
    command = AD7792_GetRegisterValue(AD7792_REG_CONF,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7792_CONF_CHAN(0xFF);
    command |= AD7792_CONF_CHAN(channel);
    AD7792_SetRegisterValue(
            AD7792_REG_CONF,
            command,
            2,
            1); // CS is modified by SPI read/write functions.
}

/***************************************************************************//**
 * @brief  Sets the gain of the In-Amp.
 *
 * @param  gain - Gain.
 *
 * @return  None.    
*******************************************************************************/
void AD7792_SetGain(unsigned long gain)
{
    unsigned long command;
    
    command = AD7792_GetRegisterValue(AD7792_REG_CONF,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7792_CONF_GAIN(0xFF);
    command |= AD7792_CONF_GAIN(gain);
    AD7792_SetRegisterValue(
            AD7792_REG_CONF,
            command,
            2,
            1); // CS is modified by SPI read/write functions.
}
/***************************************************************************//**
 * @brief Sets the reference source for the ADC.
 *
 * @param type - Type of the reference.
 *               Example: AD7792_REFSEL_EXT	- External Reference Selected
 *                        AD7792_REFSEL_INT	- Internal Reference Selected.
 *
 * @return None.    
*******************************************************************************/
void AD7792_SetIntReference(unsigned char type)
{
    unsigned long command = 0;
    
    command = AD7792_GetRegisterValue(AD7792_REG_CONF,
                                      2,
                                      1); // CS is modified by SPI read/write functions.
    command &= ~AD7792_CONF_REFSEL(AD7792_REFSEL_INT);
    command |= AD7792_CONF_REFSEL(type);
    AD7792_SetRegisterValue(AD7792_REG_CONF,
							command,
							2,
                            1); // CS is modified by SPI read/write functions.
}

/***************************************************************************//**
 * @brief Performs the given calibration to the specified channel.
 *
 * @param mode - Calibration type.
 * @param channel - Channel to be calibrated.
 *
 * @return none.
*******************************************************************************/
void AD7792_Calibrate(unsigned char mode, unsigned char channel)
{
    unsigned short oldRegValue = 0x0;
    unsigned short newRegValue = 0x0;
    
    AD7792_SetChannel(channel);
    oldRegValue &= AD7792_GetRegisterValue(AD7792_REG_MODE, 2, 1); // CS is modified by SPI read/write functions.
    oldRegValue &= ~AD7792_MODE_SEL(0x7);
    newRegValue = oldRegValue | AD7792_MODE_SEL(mode);
    ADI_PART_CS_LOW; 
    AD7792_SetRegisterValue(AD7792_REG_MODE, newRegValue, 2, 0); // CS is not modified by SPI read/write functions.
    AD7792_WaitRdyGoLow();
    ADI_PART_CS_HIGH;
    
}

/***************************************************************************//**
 * @brief Returns the result of a single conversion.
 *
 * @return regData - Result of a single analog-to-digital conversion.
*******************************************************************************/
unsigned long AD7792_SingleConversion(void)
{
    unsigned long command = 0x0;
    unsigned long regData = 0x0;
    
    command  = AD7792_MODE_SEL(AD7792_MODE_SINGLE);
    ADI_PART_CS_LOW;
	  ADI_DELAY(TIMEOUT_LH);
    AD7792_SetRegisterValue(AD7792_REG_MODE, 
                            command,
                            2,
                            0);// CS is not modified by SPI read/write functions.
	  //ADI_DELAY(30);
    AD7792_WaitRdyGoLow();
	  ADI_DELAY(TIMEOUT_WRGL);
	  ADI_DELAY(5);
    regData = AD7792_GetRegisterValue(AD7792_REG_DATA, 2, 0); // CS is not modified by SPI read/write functions.
    ADI_DELAY(TIMEOUT_LH);
	  ADI_PART_CS_HIGH;

    return(regData);
}

/***************************************************************************//**
 * @brief Returns the average of several conversion results.
 *
 * @return samplesAverage - The average of the conversion results.
*******************************************************************************/
unsigned long AD7792_ContinuousReadAvg(unsigned char sampleNumber)
{
    unsigned long samplesAverage = 0x0;
    unsigned long command        = 0x0;
    unsigned char count          = 0x0;
    
    command = AD7792_MODE_SEL(AD7792_MODE_CONT);
    ADI_PART_CS_LOW;
	  ADI_DELAY(TIMEOUT_LH);
    AD7792_SetRegisterValue(AD7792_REG_MODE,
                            command, 
                            2,
                            0);// CS is not modified by SPI read/write functions.
    for(count = 0;count < sampleNumber;count ++)
    {
        AD7792_WaitRdyGoLow();
        samplesAverage += AD7792_GetRegisterValue(AD7792_REG_DATA, 2, 0);  // CS is not modified by SPI read/write functions.
    }
		ADI_DELAY(TIMEOUT_LH);
    ADI_PART_CS_HIGH;
    samplesAverage = samplesAverage / sampleNumber;
    
    return(samplesAverage);
}
