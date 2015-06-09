/****
*
*/

#ifndef __adi_H
#define __adi_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "AD7792.h"

/* Prototypes ----------------------------------------------------------------*/
void ADI_Init(void);
uint32_t rec_filter(uint32_t data, uint8_t Nb, uint8_t k);
	 
#ifdef __cplusplus
}
#endif

#endif //__adi_H

