/**
  ******************************************************************************
  * @file    STM32F0xx_EEPROM_Emulation/src/eeprom.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    29-May-2012
  * @brief   This file provides all the EEPROM emulation firmware functions.
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
	The provided implementation of the EEPROM emulation firmware runs from the internal 
Flash, thus the access to the Flash will be stalled during operations requiring Flash erase or 
programming (EEPROM initialization, variable update or page erase). As a consequence, 
the application code is not executed and the interrupt cannot be serviced.
This behavior may be acceptable for many applications; however, for applications with real-
time constraints, you need to run the critical processes from the internal RAM.
In this case:
1.     Relocate the vector table in the internal RAM.
2.     Execute all critical code and interrupt service routines from the internal RAM. the 
compiler provides a keyword to declare func
tions as a RAM function; the function is 
copied from the Flash to the RAM at system st
artup just like any initialized variable. It is 
important to note that, for a RAM function, all used variable(s) and called function(s) 
should be within the RAM
  */ 

/** @addtogroup STM32F0xx_EEPROM_Emulation
  * @{
  */ 

/* Includes ------------------------------------------------------------------*/
#include "eeprom.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FLASH_TIMEOUT  200
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern FLASH_ProcessTypeDef pFlash;

/* Global variable used to store variable value in read sequence */
const SavedDomain_t SavedDomain __attribute__ ((at(PAGE0_BASE_ADDRESS + 1024*14)));

/* Virtual address defined by the user: 0xFFFF value is prohibited */
//extern uint16_t VirtAddVarTab[NB_OF_VAR];

/* Private function prototypes -----------------------------------------------*/
extern void FLASH_PageErase(uint32_t PageAddress);
extern HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout);
/* Private functions ---------------------------------------------------------*/

HAL_StatusTypeDef ee_format() {
	HAL_StatusTypeDef status;
	
	status = HAL_FLASH_Unlock();
	if(status != HAL_OK) {
		return status;
	}
	
	FLASH_PageErase(PAGE0_BASE_ADDRESS);
	
	status = FLASH_WaitForLastOperation(FLASH_TIMEOUT);
	if(status != HAL_OK) {
		return status;
	}
	
	CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
	HAL_FLASH_Lock();
	
	return status;
}





