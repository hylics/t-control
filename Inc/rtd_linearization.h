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

#ifndef _RTD_LINEARIZATION_H
#define _RTD_LINEARIZATION_H

/* defines */
#define METHOD_MATH       1
#define METHOD_PIECEWISE  2
#define RTD_LIN_METHOD  METHOD_PIECEWISE

/* end defines*/

/* includes */
#if RTD_LIN_METHOD == METHOD_MATH
  #define ARM_MATH_CM0
  #include "arm_math.h"
#else //RTD_LIN_METHOD == METHOD_PIECEWISE
  #include "stdint.h"
  typedef float float32_t;
	typedef double float64_t;
#endif
/* end includes*/

//float32_t i;


/* function prototypes*/


/* end function prototypes*/

#endif	// _RTD_LINEARIZATION_H
