//**************************************************************
// File         : RTDmath.c
// Author       : Grayson King, Analog Devices
// Last Revised : 6 August 2004
// Compiler     : intended for Keil C51
// Description  : Subroutines for linearization of RTD signals
//                using direct mathematical calculation method.
// More Info    : Details in application note AN-709, available
//                at....  http://www.analog.com/MicroConverter
//**************************************************************

// include files....
#include<math.h>        // math functions

// definitions....
#define AA (3.9083E-3)      // A term of forward transfer functions
#define BB (-5.775E-7)      // B term of forward transfer functions
#define CC (-4.183E-12)     // C term of forward transfer functions
#define Z1 (-3.9083E-3)     // Z1 coef. of positive reverse transfer function
#define Z2 (17.58480889E-6) // Z2 coef. of positive reverse transfer function
#define Z3 (-23.10E-9)      // Z3 coef. of positive reverse transfer function
#define Z4 (-1.155E-6)      // Z4 coef. of positive reverse transfer function
#define TMIN (-200)     // minimum temperature [degC]
#define TMAX (850)      // maximum temperature [degC]
#define RMIN 18.52008   // minimum resistance [ohms]
#define RMAX 390.4811   // maximum resistance [ohms]

// _____________________________________________________________
// Temperature of RTD Function                             T_rtd
// input: r = resistance of RTD
// output: T_rtd() = corresponding temperature of RTD
// Calculates temperature of RTD as a function of resistance via
// a direct mathematical method.

float T_rtd (float r) {
  float t;

  // first determine if input resistance is within spec'd range
  if (r<RMIN)           // if input is under-range..
    t = TMIN;           // ..then set to minimum of range
  else if (r>RMAX)      // if input is over-range..
    t = TMAX;           // ..then set to maximum of range

  // if input (r) is within range, then solve for output.
  else {

    // if r < threshold, use negative transfer function
//    if (r<100)  t=-242.0199+2.222812*r+2.585885E-3*pow(r,2)-4.826040E-6*pow(r,3)-2.818340E-8*pow(r,4)+1.524259E-10*pow(r,5);
//    if (r<96.6)  t=-241.9610+2.216253*r+2.854064E-3*pow(r,2)-9.912120E-6*pow(r,3)+1.705183E-8*pow(r,4);
    if (r<95.1)  t=-242.0906+2.227625*r+2.517790E-3*pow(r,2)-5.861951E-6*pow(r,3);
//    if (r<72.1)  t=-242.9703+2.283841*r+1.472734E-3*pow(r,2);
    // NOTE: un-comment only one of the above four lines...
    // (5th-order, 4th-order, 3rd-order, 2nd-order respectively)

    // if r >= threshold, use positive transfer function
    else t=(Z1+sqrt(Z2+Z3*r))/Z4;

  }
  return (t);
}

// _____________________________________________________________
// Resistance of RTD Function                              R_rtd
// input: t = temperature of RTD
// output: R_rtd() = corresponding resistance of RTD
// Calculates resistance of RTD as a function of temperature via
// a direct mathematical method.

float R_rtd (float t) {
  float r;

  if (t>=0)  r = 100 * (1 + AA*t + BB*pow(t,2));
  else       r = 100 * (1 + AA*t + BB*pow(t,2) + CC*(t-100)*pow(t,3));

  return (r);
}

// _____________________________________________________________
// Minimum Temperature Function                         Tmin_rtd
// Returns minimum temperature specified by lookup table.
float Tmin_rtd () {
  return (TMIN);
}

// _____________________________________________________________
// Maximum Temperature Function                         Tmax_rtd
// Returns maximum temperature specified by lookup table.
float Tmax_rtd () {
  return (TMAX);
}

// _____________________________________________________________
// Minimum Resistance Function                          Rmin_rtd
// Returns minimum RTD resistance specified by lookup table.
float Rmin_rtd () {
  return (RMIN);
}

// _____________________________________________________________
// Maximum Resistance Function                          Rmax_rtd
// Returns maximum RTD resistance specified by lookup table.
float Rmax_rtd () {
  return (RMAX);
}
