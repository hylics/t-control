/*******************************************************************************
 *   @file   main.h
 *   @brief  Header file of main.
 *   @author hylics
********************************************************************************/

#ifndef __MAIN_H__
#define __MAIN_H__

#define ARM_MATH_CM0
#include "float.h"
#include "fenv.h"
#include "soft_spi.h"
#include "eeprom.h"
#include "adi.h"
#include "arm_math.h"
#include "rtd_linearization.h"

typedef struct __Temperature_t {
	float32_t rtd;
	float32_t thermocouple;
	float32_t setpoint;
}Temperature_t;

void set_output(float32_t out);

#endif	// __MAIN_H__

