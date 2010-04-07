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

#ifndef _OR_H
#define _OR_H


class OrOpt
{
public:
    bool search(class VRP *V, int i, int j, int rules);
    bool route_search(class VRP *V, int r1, int r2, int k, int rules);

private:
    bool evaluate(class VRP *V, int a, int len, int c, int d, int rules, VRPMove *M);
    bool move(class VRP *V, VRPMove *M);	

};


#endif



