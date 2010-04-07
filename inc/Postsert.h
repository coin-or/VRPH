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

#ifndef _POSTSERT_H
#define _POSTSERT_H

class Postsert
{
public:
    bool evaluate(class VRP *V, int i, int j, VRPMove *M);
    bool move(VRP *V, int u, int i);

};

#endif


