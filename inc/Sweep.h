////////////////////////////////////////////////////////////
//                                                        //
// This file is part of the VRPH software package for     //
// generating solutions to vehicle routing problems.      //
// VRPH was developed by Chris Groer (cgroer@gmail.com).  //
//                                                        //
// (c) Copyright 2010 Chris Groer.                        //
// All Rights Reserved.  VRPH is licensed under the       //
// Common Public License.  See LICENSE file for details.  //
//                                                        //
////////////////////////////////////////////////////////////

#ifndef _SWEEP_H
#define _SWEEP_H

#define VRPH_UNUSED          0            // For node status use
#define VRPH_ADDED           1            // For node status use
#define VRPH_INTERIOR        2            // For node status use

class Sweep
{

public:
    Sweep();
    bool Construct(class VRP *V);
    
};

#endif


