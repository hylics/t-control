/**
 *   @file   rtd_linearization.h
 *   @brief  Header file of RTD linearization functions.
 *   @author hylics
 
Copyright (c) 2015, hylics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of t-control nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* AN709*/
/*Constants RTD transpher function*/
/*alpha 375*/
/* A=3.81e-3  B=-6.02e-7  C=-6.0e-12 */
/*alpha 385*/
/* A=3.908e-3  B=-5.755e-7  C=-4.183e-12*/
/* Rt = R0 * (1+AT+BT^2 -100CT^3 + CT^4 ) C=0 if t>0*/
/* Z1=-A     Z2=A^2 - 4*B   Z3=(4*B)/R0     Z4=2*B*/
/* Trtd(Rx)=(Z1+sqrt(Z2+Z3*r0))/Z4   */

//#define ALPHA 375
//#define AA   (((ALPHA)<385) ? (3.81E-3) : (3.908E-3))


#ifndef _RTD_LINEARIZATION_H
#define _RTD_LINEARIZATION_H

/* configuration */
#define RTD_METHOD_MATH //RTD_METHOD_MATH  RTD_METHOD_PIECEWISE
#define USE_ARM_MATH
#define RTD_N_POLY_4

/* defines */
#define R_REF 2000
#define ADC_MAX 65536
//#define R_ZERO 32768
#define TMIN (-200)  // = minimum temperature in degC
#define TMAX (540)  // = maximum temperature in degC
#define RMIN (185.201)  // = input resistance in ohms at -200 degC
#define RMAX (2942.08)  // = input resistance in ohms at 540 degC

/* end defines*/

/* includes */
#include <stdint.h>

#if defined(RTD_METHOD_MATH)
  #if defined(USE_ARM_MATH)
    #define ARM_MATH_CM0
    #include "arm_math.h"
	#else
	  #include <math.h>
	#endif
	
// First coef for alpha 375, second for 385
//static const float AA[2] = {3.81E-3, 3.908E-3};
//static const float BB[2] = {-6.02E-7, -5.755E-7};
//static const float CC[2] = {-6.0E-12, -4.183E-12};
static const float32_t R0[2] = {100.0F, 1000.0F};
static const float32_t Z1[2] = {-3.81E-3, -3.908E-3};
static const float32_t Z2[2] = {1.69241E-5, 1.757446E-5};
// Alpha 375: R 100, 1000 Ohm; Alpha 385: R 100, 1000 Ohm
static const float32_t Z3[4] = {-2.408E-8, -2.408E-9, -2.302E-8, -2.302E-9};
static const float32_t Z4[2] = {-1.204E-6, -1.151E-6};

#elif defined(RTD_METHOD_PIECEWISE)

typedef float float32_t;
typedef double float64_t;

// definitons....
#define NSEG 128  // = number of sections in table
#define RSEG 21.5381  // = (RMAX-RMIN)/NSEG = resistance RSEG in ohms of each segment

  static const float32_t C_rtd[] = {-200.001,-195.009,-189.996,-184.963,-179.91,-174.837,-169.746,-164.635,-159.507,-154.36,
	-149.197,-144.016,-138.819,-133.605,-128.376,-123.131,-117.871,-112.597,-107.309,-102.007,-96.6908,-91.3621,-86.0207,
	-80.6669,-75.301,-69.9235,-64.5345,-59.1344,-53.7235,-48.3019,-42.8701,-37.4281,-31.9762,-26.5146,-21.0435,-15.563,
	-10.0732,-4.57438,0.933528,6.45042,11.9763,17.5113,23.0554,28.6087,34.1712,39.7429,45.3239,50.9143,56.514,62.1232,
	67.7419,73.3701,79.008,84.6554,90.3125,95.9794,101.656,107.343,113.039,118.745,124.461,130.188,135.924,141.671,
	147.428,153.195,158.972,164.76,170.558,176.367,182.186,188.015,193.856,199.707,205.569,211.442,217.325,223.22,229.126,
	235.042,240.97,246.909,252.859,258.821,264.794,270.778,276.774,282.782,288.801,294.832,300.875,306.93,312.996,319.075,
	325.165,331.268,337.383,343.511,349.65,355.803,361.967,368.145,374.335,380.537,386.753,392.981,399.223,405.477,
	411.745,418.026,424.32,430.628,436.949,443.284,449.633,455.995,462.371,468.761,475.165,481.583,488.016,494.462,
  500.924,507.399,513.89,520.395,526.915,533.449,539.999};

#else
  #error "Define method used to calculate temperature RTD_METHOD_MATH or RTD_METHOD_PIECEWISE"
#endif
/* end includes*/

typedef enum {a375, a385} alpha_t;
typedef enum {r100, r1000} r_zero_t;

/* function prototypes*/
float32_t rtd_get_temp(uint32_t Rx, alpha_t a, r_zero_t rz);

/* end function prototypes*/

#endif	// _RTD_LINEARIZATION_H
