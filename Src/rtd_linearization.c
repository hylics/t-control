/***************************************************************************//**
 *   @file   rtd_linearization.c
 *   @brief  Implementation of RTD linearization functions.
 *   @author hylics
********************************************************************************

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

*******************************************************************************/

#include "rtd_linearization.h"

/***************************************************************************//**
 * @brief Temperature of RTD Function                             rtd_get_temp
 * @param Rx = resistance of RTD
 * @return corresponding temperature of RTD
*******************************************************************************/
float32_t rtd_get_temp(uint32_t Rx, alpha_t a, r_zero_t rz) {
	float32_t t = 0.0f;
	float32_t r = R_REF*(float32_t)Rx/ADC_MAX;
	
#if defined(RTD_METHOD_MATH)
	// first determine if input resistance is within spec'd range
  if (r<RMIN) {          // if input is under-range..
    t = TMIN;           // ..then set to minimum of range
	}
  else if (r>RMAX) {      // if input is over-range..
    t = TMAX;            // ..then set to maximum of range
	}
  // if input (r) is within range, then solve for output.
  else {
		//use pow() or unroll this for reduce computation time 21688bytes pow()
		// if r < resistance at 0C, use negative transfer function
		if(r < R0[rz]) {
			float32_t rf = r / R0[rz];
		#if defined(RTD_N_POLY_2) //max_delta 0.2443 0.1711
		  t = k0[a] + k1[a]*rf + k2[a]*pow(rf,2);
		#elif defined(RTD_N_POLY_3) //max_delta 0.0150 0.0126
		  t = k0[a] + k1[a]*rf + k2[a]*pow(rf,2) + k3[a]*pow(rf,3);
		#elif defined(RTD_N_POLY_4) //max_delta 0.0041 0.0023
		  t = k0[a] + k1[a]*rf + k2[a]*pow(rf,2) + k3[a]*pow(rf,3) + k4[a]*pow(rf,4);
		#elif defined(RTD_N_POLY_5) //max_delta 7.5340e-05 5.9128e-05
		  t = k0[a] + k1[a]*rf + k2[a]*pow(rf,2) + k3[a]*pow(rf,3) + k4[a]*pow(rf,4) + k5[a]*pow(rf,5);
		#else
      #error "Define RTD_N_POLY_2-5
    #endif
    // (5th-order, 4th-order, 3rd-order, 2nd-order respectively)
		}
    // if r >= threshold, use positive transfer function
		#if defined(USE_ARM_MATH)
		else {
		  float32_t zrx = 0;
		  arm_sqrt_f32((Z2[a]+Z3[2*a+rz]*r), &zrx);
			t = (Z1[a]+zrx)/Z4[a];
		}
		#else
    else {
		  t=(Z1[a]+sqrt(Z2[a]+Z3[2*a+rz]*r))/Z4[a];
		}
    #endif
  }
	
#elif defined(RTD_METHOD_PIECEWISE)
	int32_t i;
  i=(r-RMIN)/RSEG;       // determine which coefficients to use
  if (i<0)                // if input is under-range..
    i=0;                  // ..then use lowest coefficients
  else if (i>NSEG-1)      // if input is over-range..
    i=NSEG-1;             // ..then use highest coefficients
  t = C_rtd[i]+(r-(RMIN+RSEG*i))*(C_rtd[i+1]-C_rtd[i])/RSEG;
#else
  #error "Define method used to calculate temperature RTD_METHOD_MATH or RTD_METHOD_PIECEWISE"
#endif
	
	return t;
}


