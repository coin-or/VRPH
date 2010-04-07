////////////////////////////////////////////////////////////
//                                                        //
// This file is part of the VRPH C/C++ package for        //
// solving the Vehicle Routing Problem by Chris Groer     //
// Code is free for use by academic researchers.          //
// For other purposes, contact cgroer@gmail.com           //
//                                                        //
////////////////////////////////////////////////////////////

#ifndef _OSMAN_H
#define _OSMAN_H

bool osman_insert(VRP *V, int k, double alpha);
int osman_perturb(VRP *V, int num, double alpha);

#endif

