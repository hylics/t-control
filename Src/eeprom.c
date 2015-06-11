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
#include "float.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FLASH_TIMEOUT  200
#define PWM_PERIOD 20000
/* Private macro -------------------------------------------------------------*/
/* Extern variables ---------------------------------------------------------*/
//extern FLASH_ProcessTypeDef pFlash;

/* Global variable used to store variable value in read sequence */
//const SavedDomain_t SavedDomain __attribute__ ((at(PAGE0_BASE_ADDRESS))) = {};

const SavedDomain_t SavedDomain __attribute__ ((aligned(1024))) = {
	0xABAB,                 //header
  0x8000, 0x8000, 0x8000, //ADC offset
  0x54A3, 0x54A3, 0x54A3, //ADC fullscale
  0.0f, 0.0f, 0.0f,       //PID coeff KP Ki Kd
	PWM_PERIOD,             // pwm Init.Period ms
  FLT_MAX/(float32_t)PWM_PERIOD //scaling factor
};

/* Virtual address defined by the user: 0xFFFF value is prohibited */
//extern uint16_t VirtAddVarTab[NB_OF_VAR];

/* Extern function prototypes -----------------------------------------------*/
extern void FLASH_PageErase(uint32_t PageAddress);
extern HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout);
/* Private functions ---------------------------------------------------------*/

HAL_StatusTypeDef ee_format(SavedDomain_t* page) {
	HAL_StatusTypeDef status;
	
	status = HAL_FLASH_Unlock();
	if(status != HAL_OK) {
		return status;
	}
	
	//FLASH_PageErase(PAGE0_BASE_ADDRESS);
	FLASH_PageErase((uint32_t)page);
	
	status = FLASH_WaitForLastOperation(FLASH_TIMEOUT);
	if(status != HAL_OK) {
		return status;
	}
	
	CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
	HAL_FLASH_Lock();
	
	return status;
}

//void WriteFlash(void* Src, void* Dst, int Len)
//{
//  uint16_t* SrcW = (uint16_t*)Src;
//  volatile uint16_t* DstW = (uint16_t*)Dst;

//  FLASH->CR |= FLASH_CR_PG; /* Programm the flash */
//  while (Len)
//  {
//    *DstW = *SrcW;
//    while ((FLASH->SR & FLASH_SR_BSY) != 0 )
//      ;
//    if (*DstW != *SrcW )
//    {
//      goto EndPrg;
//    }
//    DstW++;
//    SrcW++;
//    Len = Len - sizeof(uint16_t);
//  }
//EndPrg:
//  FLASH->CR &= ~FLASH_CR_PG; /* Reset the flag back !!!! */
//}

/*HAL_StatusTypeDef SaveOptToFlash(SavedDomain_t* Src, SavedDomain_t* Dst) {
	uint64_t* SrcW = (uint64_t*)Src;
	volatile uint32_t* DstW = (uint32_t*)Dst;
	uint32_t len = sizeof(SavedDomain_t)/4;
	HAL_StatusTypeDef status = HAL_ERROR;
	
	status = HAL_FLASH_Unlock();
	if(status != HAL_OK) {
		return status;
	}
	
	while(len) {
		//params: uint32_t TypeProgram, uint32_t Address, uint64_t Data
	  status = HAL_FLASH_Program(TYPEPROGRAM_WORD, (uint32_t)DstW, (uint64_t)*SrcW);
	  if(status != HAL_OK) {
		  return status;
	  }
	  DstW++;
    SrcW++;
		len -= sizeof(uint32_t);
	}

	status = HAL_FLASH_Lock();
	if(status != HAL_OK) {
		return status;
	}
	
	return status;
}*/

HAL_StatusTypeDef SaveOptToFlash(SavedDomain_t* Src, SavedDomain_t* Dst) {
	uint16_t* SrcW = (uint16_t*)Src;
	__IO uint16_t* DstW = (uint16_t*)Dst;
	uint32_t len = sizeof(SavedDomain_t)/sizeof(uint16_t);
	
	HAL_StatusTypeDef status = HAL_ERROR;
	
	status = HAL_FLASH_Unlock();
	if(status != HAL_OK) {
		return status;
	}
	
	SET_BIT(FLASH->CR, FLASH_CR_PG); /* Programm the flash */
	
	while(len) {
		*DstW = *SrcW;
	  status = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE);
		if(status != HAL_OK) {
		  return status;
	  }
    if (*DstW != *SrcW )
    {
      return HAL_ERROR;
    }
		
	  DstW++;
    SrcW++;
		len -= sizeof(uint32_t);
	}
	CLEAR_BIT(FLASH->CR, FLASH_CR_PG);

	status = HAL_FLASH_Lock();
	if(status != HAL_OK) {
		return status;
	}
	
	return status;
}



