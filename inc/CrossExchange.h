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
#ifndef _CROSS_EX_H
#define _CROSS_EX_H

class CrossExchange
{
public:
    bool route_search(class VRP *V, int r1, int r2, int criteria);

private:
    bool evaluate(class VRP *V, int i1, int i2, int k1, int k2, int j1, int j2, int l1, int l2,
        int criteria, VRPMove *M);	
    bool move(class VRP *V, VRPMove *M);	

};

#endif


