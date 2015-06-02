/***************************************************************************//**
 *   @file   AD7792.h
 *   @brief  Header file of AD7792 Driver.
 *   @author Bancisor Mihai
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
#ifndef __AD7792_H__
#define __AD7792_H__

/******************************************************************************/
/* AD7792                                                                     */
/******************************************************************************/

#include <stdint.h>
#include "gpio.h"

/* AD7792 GPIO */
#define AD7792_RDY_STATE       GPIO1_STATE

/** 
  * @brief  HAL Lock structures definition  
  */
typedef enum 
{
  ADI_UNLOCKED = 0x00,
  ADI_LOCKED   = 0x01  
} ADI_LockTypeDef;

/** 
  * @brief  HAL ADI Status structures definition  
  */
typedef enum 
{
  ADI_OK       = 0x00,
  ADI_ERROR    = 0x01,
  ADI_BUSY     = 0x02,
  ADI_TIMEOUT  = 0x03
} ADI_StatusTypeDef;

/** 
  * @brief  HAL pin CS structures definition  
  */
typedef struct __ADI_PIN_HandleTypeDef
{
	GPIO_TypeDef* gpio;
	uint16_t pin;
}ADI_PIN_HandleTypeDef;

/** 
  * @brief  AD7792 handle Structure definition  
  */ 
typedef struct __AD7792_HandleTypeDef
{
	uint32_t mode;              /*16 bit*/
	uint32_t conf;              /*16 bit*/
	uint32_t io;                /*8 bit*/
	uint32_t offset[3];            /*16/24 bit*/
	uint32_t fullscale[3];         /*16/24 bit*/
	ADI_PIN_HandleTypeDef cs;
	ADI_PIN_HandleTypeDef rdy;
	ADI_LockTypeDef lock;
	ADI_StatusTypeDef state;
} AD7792_HandleTypeDef;

/** @brief  type arguments for manipulating ad7792 registers*/
typedef enum {reg_all, reg_mode, reg_conf, reg_io, reg_offset, reg_full_scale} op_mode_TypeDef;

/*AD7792 Registers*/
#define AD7792_REG_COMM		  0 /* Communications Register(WO, 8-bit) */
#define AD7792_REG_STAT	    0 /* Status Register	    (RO, 8-bit) */
#define AD7792_REG_MODE	    1 /* Mode Register	     	(RW, 16-bit */
#define AD7792_REG_CONF	    2 /* Configuration Register (RW, 16-bit)*/
#define AD7792_REG_DATA	    3 /* Data Register	     	(RO, 16-/24-bit) */
#define AD7792_REG_ID	      4 /* ID Register	     	  (RO, 8-bit) */
#define AD7792_REG_IO	      5 /* IO Register	     	  (RO, 8-bit) */
#define AD7792_REG_OFFSET   6 /* Offset Register	    (RW, 24-bit */
#define AD7792_REG_FULLSCALE	7 /* Full-Scale Register	(RW, 24-bit */

/* Communications Register Bit Designations (AD7792_REG_COMM) */
#define AD7792_COMM_WEN		  (1 << 7) 			/* Write Enable */
#define AD7792_COMM_WRITE	  (0 << 6) 			/* Write Operation */
#define AD7792_COMM_READ    (1 << 6) 			/* Read Operation */
#define AD7792_COMM_ADDR(x)	(((x) & 0x7) << 3)	/* Register Address */
#define AD7792_COMM_CREAD	  (1 << 2) 			/* Continuous Read of Data Register */

/* Status Register Bit Designations (AD7792_REG_STAT) */
#define AD7792_STAT_RDY		(1 << 7) /* Ready */
#define AD7792_STAT_ERR		(1 << 6) /* Error (Overrange, Underrange) */
#define AD7792_STAT_CH3		(1 << 2) /* Channel 3 */
#define AD7792_STAT_CH2		(1 << 1) /* Channel 2 */
#define AD7792_STAT_CH1		(1 << 0) /* Channel 1 */

/****************************************************************************************/
/* Mode Register Bit Designations (AD7792_REG_MODE)                                     */
/****************************************************************************************/
#define AD7792_MODE_SEL(x)		(((x) & 0x7) << 13)	/* Operation Mode Select */
#define AD7792_MODE_CLKSRC(x)	(((x) & 0x3) << 6) 	/* ADC Clock Source Select */
#define AD7792_MODE_RATE(x)		((x) & 0xF) 		    /* Filter Update Rate Select */

/* AD7792_MODE_SEL(x) options */
#define AD7792_MODE_CONT		     0 /* Continuous Conversion Mode */
#define AD7792_MODE_SINGLE	   	 1 /* Single Conversion Mode */
#define AD7792_MODE_IDLE		     2 /* Idle Mode */
#define AD7792_MODE_PWRDN		     3 /* Power-Down Mode */
#define AD7792_MODE_CAL_INT_ZERO 4 /* Internal Zero-Scale Calibration */
#define AD7792_MODE_CAL_INT_FULL 5 /* Internal Full-Scale Calibration */
#define AD7792_MODE_CAL_SYS_ZERO 6 /* System Zero-Scale Calibration */
#define AD7792_MODE_CAL_SYS_FULL 7 /* System Full-Scale Calibration */

/* AD7792_MODE_CLKSRC(x) options */
#define AD7792_CLK_INT		  0 /* Internal 64 kHz Clk not available at the CLK pin */
#define AD7792_CLK_INT_CO	  1 /* Internal 64 kHz Clk available at the CLK pin */
#define AD7792_CLK_EXT		  2 /* External 64 kHz Clock */
#define AD7792_CLK_EXT_DIV2	3 /* External Clock divided by 2 */

/* AD7792_MODE_RATE(x) */
#define AD7792_RATE_4ms             1  /*470 Hz*/
#define AD7792_RATE_8ms             2  /*242 Hz*/
#define AD7792_RATE_16ms            3  /*123 Hz*/
#define AD7792_RATE_32ms            4  /*62 Hz*/
#define AD7792_RATE_40ms            5  /*50 Hz*/
#define AD7792_RATE_48ms            6  /*39 Hz*/
#define AD7792_RATE_60ms            7  /*33.2 Hz*/
#define AD7792_RATE_101ms           8  /*19.6 Hz*/
#define AD7702_RATE_1_120ms         9  /*16.7 Hz reject 50Hz only*/
#define AD7792_RATE_2_120ms         10 /*16.7 Hz reject 50 and 60Hz*/
#define AD7792_RATE_160ms           11 /*12.5 Hz*/
#define AD7792_RATE_200ms           12 /*10 Hz*/
#define AD7792_RATE_240ms           13 /*8.33 Hz*/
#define AD7792_RATE_320ms           14 /*6.25 Hz*/
#define AD7792_RATE_480ms           15 /*4.17 Hz*/

/****************************************************************************************/
/* Configuration Register Bit Designations (AD7792_REG_CONF)                            */
/****************************************************************************************/
#define AD7792_CONF_VBIAS(x)  (((x) & 0x3) << 14) /* Bias Voltage Generator Enable */
#define AD7792_CONF_BO_EN	    (1 << 13) 		    	/* Burnout Current Enable */
#define AD7792_CONF_UNIPOLAR  (1 << 12) 			    /* Unipolar/Bipolar Enable */
#define AD7792_CONF_BOOST	    (1 << 11) 		     	/* Boost Enable */
#define AD7792_CONF_GAIN(x)	  (((x) & 0x7) << 8) 	/* Gain Select */
#define AD7792_CONF_REFSEL(x) (((x) & 0x1) << 7) 	/* INT/EXT Reference Select */
#define AD7792_CONF_BUF		    (1 << 4) 			    	/* Buffered Mode Enable */
#define AD7792_CONF_CHAN(x)	  ((x) & 0x7) 		    /* Channel select */

/* AD7792_CONF_VBIAS(x) options*/
#define AD7792_VBIAS_DISABLE   0
#define AD7792_VBIAS_AIN1M     1
#define AD7792_VBIAS_AIN2M     2

/* AD7792_CONF_GAIN(x) options */
#define AD7792_GAIN_1       0
#define AD7792_GAIN_2       1
#define AD7792_GAIN_4       2
#define AD7792_GAIN_8       3
#define AD7792_GAIN_16      4
#define AD7792_GAIN_32      5
#define AD7792_GAIN_64      6
#define AD7792_GAIN_128     7

/* AD7792_CONF_REFSEL(x) options */
#define AD7792_REFSEL_INT   1	/* Internal Reference Selected. */
#define AD7792_REFSEL_EXT   0	/* External Reference Applied between REFIN(+) and REFIN(–). */

/* AD7792_CONF_CHAN(x) options */
#define AD7792_CH_AIN1P_AIN1M	  0 /* AIN1(+) - AIN1(-) */
#define AD7792_CH_AIN2P_AIN2M	  1 /* AIN2(+) - AIN2(-) */
#define AD7792_CH_AIN3P_AIN3M  	2 /* AIN3(+) - AIN3(-) */
#define AD7792_CH_AIN1M_AIN1M	  3 /* AIN1(-) - AIN1(-) */
#define AD7792_CH_TEMP		    	6 /* Temp Sensor */
#define AD7792_CH_AVDD_MONITOR	7 /* AVDD Monitor */

/* ID Register Bit Designations (AD7792_REG_ID) */
#define AD7792_ID			    0xA
#define AD7792_ID_MASK		0xF

/****************************************************************************************/
/* IO (Excitation Current Sources) Register Bit Designations (AD7792_REG_IO)            */
/****************************************************************************************/
#define AD7792_IEXCDIR(x)	(((x) & 0x3) << 2)
#define AD7792_IEXCEN(x)	(((x) & 0x3) << 0)

/* AD7792_IEXCDIR(x) options*/
#define AD7792_DIR_IEXC1_IOUT1_IEXC2_IOUT2	0  /* IEXC1 connect to IOUT1, IEXC2 connect to IOUT2 */
#define AD7792_DIR_IEXC1_IOUT2_IEXC2_IOUT1	1  /* IEXC1 connect to IOUT2, IEXC2 connect to IOUT1 */
#define AD7792_DIR_IEXC1_IEXC2_IOUT1		    2  /* Both current sources IEXC1,2 connect to IOUT1  */
#define AD7792_DIR_IEXC1_IEXC2_IOUT2		    3  /* Both current sources IEXC1,2 connect to IOUT2  */

/* AD7792_IEXCEN(x) options*/
#define AD7792_EN_IXCEN_10uA				1  /* Excitation Current 10uA */
#define AD7792_EN_IXCEN_210uA				2  /* Excitation Current 210uA */
#define AD7792_EN_IXCEN_1mA					3  /* Excitation Current 1mA */

/******************************************************************************/
/* Public Functions Prototypes                                                */
/******************************************************************************/

/* Initialize AD7792 and check if the device is present*/
uint8_t AD7792_Init(void);

uint32_t AD7792_GetRegisterValue(uint8_t regAddress, uint8_t size, uint8_t modifyCS);
void AD7792_SetRegisterValue(uint8_t regAddress, uint32_t regValue, uint8_t size, uint8_t modifyCS);

/* Sends 32 consecutive 1's on SPI in order to reset the part. */
void AD7792_Reset(void);

/* Selects the channel of AD7792. */
void AD7792_SetChannel(uint32_t channel);

/* Performs the given calibration to the specified channel. */
void AD7792_Calibrate(uint8_t mode, uint8_t channel);

/* Returns the result of a single conversion. */
uint32_t AD7792_SingleConversion(AD7792_HandleTypeDef *adc_instance);

/* Returns the average of several conversion results. */
uint32_t AD7792_ContinuousReadAvg(uint8_t sampleNumber);

/*my func*/
ADI_StatusTypeDef AD7792_conf(AD7792_HandleTypeDef *adc_instance, op_mode_TypeDef type);

//int AD7792_conf(uint32_t gain, uint32_t channel, uint32_t current);

#endif	// _AD7792_H_
