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

#ifndef _THREEPM_H
#define _THREEPM_H

class ThreePointMove
{
public:
	bool search(class VRP *V, int b, int criteria);
	bool route_search(class VRP *V, int r1, int r2, int criteria);

private:
	bool evaluate(class VRP *V, int a, int j, int k, int criteria, VRPMove *M);
	bool move(class VRP *V, VRPMove *M);//, int i, int j);
	
};

#endif
