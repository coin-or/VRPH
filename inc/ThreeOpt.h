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

#ifndef _THREE_OPT_H
#define _THREE_OPT_H

class ThreeOpt
{
public:
	bool route_search(class VRP *V, int r, int criteria);

private:
	bool evaluate(class VRP *V, int a, int b, int c, int d, int e, int f, int criteria, VRPMove *M);
	bool move(class VRP *V, VRPMove *M);
	
};

#endif

