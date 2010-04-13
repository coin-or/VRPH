////////////////////////////////////////////////////////////
//                                                        //
// This file is part of the VRPH C/C++ package for        //
// solving the Vehicle Routing Problem by Chris Groer     //
// Code is free for use by academic researchers.          //
// For other purposes, contact cgroer@gmail.com           //
//                                                        //
////////////////////////////////////////////////////////////

#ifndef _VRP_CONFIG_H
#define _VRP_CONFIG_H

#define EPS_EXE	"epstopdf"
// To convert .ps to .pdf files - only tested on WINDOWS and Cygwin

#if HAS_PLPLOT
#include "plplot.h"
#endif

// Set this to 1 if you want to have different random
// seeds that incorporate time - 
// otherwise the solution should be repeatable
#define RESEED_RNG 0

// Set this to 1 if you want to prevent "trivial"
// moves that have no effect on the total route length
#define FORBID_TINY_MOVES           1


#endif

