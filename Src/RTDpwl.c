//**************************************************************
// File         : RTDpwl.c
// Author       : Automatically generated using 'coefRTD.exe'
// Compiler     : intended for Keil C51
// Description  : Subroutines for linearization of RTD signals
//                using piecewise linear approximation method.
// More Info    : Details in application note AN-709, available
//                at....  http://www.analog.com/MicroConverter
//**************************************************************

#if defined(RTD_METHOD_PIECEWISE)

// definitons....
#define TMIN (-200)  // = minimum temperature in degC
#define TMAX (540)  // = maximum temperature in degC
#define RMIN (185.201)  // = input resistance in ohms at -200 degC
#define RMAX (2942.08)  // = input resistance in ohms at 540 degC
#define NSEG 128  // = number of sections in table
#define RSEG 21.5381  // = (RMAX-RMIN)/NSEG = resistance RSEG in ohms of each segment

// lookup table....
const float C_rtd[] = {-200.001,-195.009,-189.996,-184.963,-179.91,-174.837,-169.746,-164.635,-159.507,-154.36,
	-149.197,-144.016,-138.819,-133.605,-128.376,-123.131,-117.871,-112.597,-107.309,-102.007,-96.6908,-91.3621,-86.0207,
	-80.6669,-75.301,-69.9235,-64.5345,-59.1344,-53.7235,-48.3019,-42.8701,-37.4281,-31.9762,-26.5146,-21.0435,-15.563,
	-10.0732,-4.57438,0.933528,6.45042,11.9763,17.5113,23.0554,28.6087,34.1712,39.7429,45.3239,50.9143,56.514,62.1232,
	67.7419,73.3701,79.008,84.6554,90.3125,95.9794,101.656,107.343,113.039,118.745,124.461,130.188,135.924,141.671,
	147.428,153.195,158.972,164.76,170.558,176.367,182.186,188.015,193.856,199.707,205.569,211.442,217.325,223.22,229.126,
	235.042,240.97,246.909,252.859,258.821,264.794,270.778,276.774,282.782,288.801,294.832,300.875,306.93,312.996,319.075,
	325.165,331.268,337.383,343.511,349.65,355.803,361.967,368.145,374.335,380.537,386.753,392.981,399.223,405.477,
	411.745,418.026,424.32,430.628,436.949,443.284,449.633,455.995,462.371,468.761,475.165,481.583,488.016,494.462,
  500.924,507.399,513.89,520.395,526.915,533.449,539.999};

	//const float C_rtd[] = {-200.001,-195.009,-189.996,-184.963,-179.91,-174.837,-169.746,-164.635,-159.507,-154.36};
// lookup table size:
//   = 128 linear sections
//   = 129 coefficients
//   = 516 bytes (4 bytes per floating point coefficient)

// linearization routine error band:  
//   = -0.001297degC .. 0.00130525degC
// specified over measurement range -200degC .. 540degC

// _____________________________________________________________
// Temperature of RTD Function                             T_rtd
// input: r = resistance of RTD
// output: T_rtd() = corresponding temperature of RTD
// Calculates temperature of RTD as a function of resistance via
// a piecewise linear approximation method.

float T_rtd (float r) {
  float t;
  int i;
  i=(r-RMIN)/RSEG;       // determine which coefficients to use
  if (i<0)               // if input is under-range..
    i=0;                 // ..then use lowest coefficients
  else if (i>NSEG-1)     // if input is over-range..
    i=NSEG-1;            // ..then use highest coefficients
  t = C_rtd[i]+(r-(RMIN+RSEG*i))*(C_rtd[i+1]-C_rtd[i])/RSEG;
  return (t);
}

// _____________________________________________________________
// Resistance of RTD Function                              R_rtd
// input: t = temperature of RTD
// output: R_rtd() = corresponding resistance of RTD
// Calculates resistance of RTD as a function of temperature via
// a piecewise linear approximation method.

float R_rtd (float t) {
  float r;
  int i, adder;

  // set up initial values
  i = NSEG/2;           // starting value for 'i' index
  adder = (i+1)/2;      // adder value used in do loop

  // determine if input t is within range
  if (t<C_rtd[0])           // if input is under-range..
    i=0;                    // ..then use lowest coefficients
  else if (t>C_rtd[NSEG])   // if input is over-range..
    i=NSEG-1;               // ..then use highest coefficients

  // if input within range, determine which coefficients to use
  else do {
    if (C_rtd[i]>t)   i-=adder; // either decrease i by adder..
    if (C_rtd[i+1]<t) i+=adder; // ..or increase i by adder
    if (i<0)       i=0;         // make sure i is >=0..
    if (i>NSEG-1)  i=NSEG-1;    // ..and <=NSEG-1
    adder = (adder+1)/2;        // divide adder by two (rounded)
  } while ((C_rtd[i]>t)||(C_rtd[i+1]<t));   // repeat 'til done

  // compute final result
  r = RMIN+RSEG*i + (t-C_rtd[i])*RSEG/(C_rtd[i+1]-C_rtd[i]);

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

#elif !defined(RTD_METHOD_MATH)
 #error "Define method used to calculate temperature RTD_METHOD_MATH or RTD_METHOD_PIECEWISE"
#endif

