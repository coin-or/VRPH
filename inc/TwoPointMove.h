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


#ifndef _TPM_H_
#define _TPM_H_

class TwoPointMove
{
public:
    bool search(class VRP *V, int i, int rules);
    bool route_search(class VRP *V, int r1, int r2, int rules);

private:
    bool evaluate(class VRP *V, int i, int j, int rules, VRPMove *M);
    bool move(class VRP *V, VRPMove *M);//, int i, int j);

};

#endif

