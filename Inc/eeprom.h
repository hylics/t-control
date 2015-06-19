/**
  ******************************************************************************
  * @file    STM32F0xx_EEPROM_Emulation/inc/eeprom.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    29-May-2012
  * @brief   This file contains all the functions prototypes for the EEPROM 
  *          emulation firmware library.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
//#include "stm32f0xx_hal_flash.h"
#include "ct_assert.h"

/* Exported constants --------------------------------------------------------*/


/* Define the size of the sectors to be used */
#define PAGE_SIZE             ((uint32_t)0x0400)  /* Page size = 1KByte */

/* EEPROM start address in Flash */

#define EEPROM_START_ADDRESS  ((uint32_t)(FLASH_BASE+126*1024)) /* EEPROM emulation start address:
                                                        from sector2, after 125KByte of used 
                                                        Flash memory */

/* Pages 0 and 1 base and end addresses */
#define PAGE0_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + 0x0000))
#define PAGE0_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (PAGE_SIZE - 1)))

#define PAGE1_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + 0x0400))
#define PAGE1_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (2 * PAGE_SIZE - 1)))

/* Used Flash pages for EEPROM emulation */
#define PAGE0                 ((uint16_t)0x0000)
#define PAGE1                 ((uint16_t)0x0001)

/* No valid page define */
#define NO_VALID_PAGE         ((uint16_t)0x00AB)

/* Page status definitions */
#define ERASED                ((uint16_t)0xFFFF)     /* Page is empty */
#define RECEIVE_DATA          ((uint16_t)0xEEEE)     /* Page is marked to receive data */
#define VALID_PAGE            ((uint16_t)0x0000)     /* Page containing valid data */

/* Valid pages in read and write defines */
#define READ_FROM_VALID_PAGE  ((uint8_t)0x00)
#define WRITE_IN_VALID_PAGE   ((uint8_t)0x01)

/* Page full define */
#define PAGE_FULL             ((uint8_t)0x80)

/* Variables' number */
#define NB_OF_VAR             ((uint8_t)0x03)

#define FLASH_N_END      10000 //stm32f072 10k cycles
/* Exported types ------------------------------------------------------------*/
typedef float float32_t;
typedef double float64_t;
typedef enum {in_rtd, in_thermocouple} input_t;
typedef enum {pwm_jitter, pwm_simple, pwm_bresenham} out_pf_t;

typedef __packed struct __SavedDomain_t{
	uint16_t header;
	uint16_t offset[3];
	uint16_t fullscale[3];
	float32_t Kp;
	float32_t Ki;
	float32_t Kd;
	uint16_t pwm_period;
	//float32_t pwm_scale_f;
	input_t input; // define used temperature sensor
	out_pf_t pf_out; //used function to set output
	uint32_t cnt_fw; //flash write counter
	uint32_t crc;
	uint32_t padding:16;
} SavedDomain_t; //size 48???? bytes, non packed

/*size of SavedDomain_t may be aligned to uint16_t size*/
STATIC_ASSERT(!(sizeof(SavedDomain_t) % sizeof(uint16_t))); //for flash operations
STATIC_ASSERT(!(sizeof(SavedDomain_t) % sizeof(uint32_t))); //for crc32
//STATIC_ASSERT((sizeof(SavedDomain_t) == 52)); //with __packed not working
STATIC_ASSERT(sizeof(SavedDomain_t) <= 1024);

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
HAL_StatusTypeDef ee_format(SavedDomain_t* page);
HAL_StatusTypeDef SaveOptToFlash(SavedDomain_t* Src, SavedDomain_t* Dst);

#endif /* __EEPROM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
