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

#ifndef _CW_H
#define _CW_H


#define VRPH_UNUSED		0			// For node status use
#define VRPH_ADDED		1			// For node status use
#define VRPH_INTERIOR	2			// For node status use

class ClarkeWright
{
public:
    ClarkeWright(int n);
    ~ClarkeWright();
    bool Construct(class VRP *V, double lambda, bool use_neighbor_list);
    class VRPSavingsElement *s;
    void CreateSavingsMatrix(class VRP *V, double lambda, bool use_neighbor_list);	
    bool has_savings_matrix;
    int savings_matrix_size;

};

#endif


