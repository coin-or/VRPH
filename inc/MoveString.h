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

#ifndef _MOVE_STR_H
#define _MOVE_STR_H

class MoveString
{
public:
    bool evaluate(class VRP *V, int a, int b, int u, int v, VRPMove *M);
    bool move(class VRP *V, int a, int b, int u, int v);
};

#endif


