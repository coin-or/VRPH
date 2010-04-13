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
#ifndef _VRP_MOVE_H
#define _VRP_MOVE_H

#define	MAX_AFFECTED_ROUTES     3
#define MAX_ARGUMENTS			15

class VRPMove
{
	///
	/// Contains data regarding a particular "move" or
	/// solution modification.
	///

public:

	VRPMove();
    VRPMove(int n);
    ~VRPMove();

	bool is_better(class VRP *V, VRPMove *M2, int criteria);

	int    criteria;
	int    num_affected_routes;
	int    route_nums [MAX_AFFECTED_ROUTES];
	double route_lens [MAX_AFFECTED_ROUTES];
	int    route_loads[MAX_AFFECTED_ROUTES];
	int    route_custs[MAX_AFFECTED_ROUTES];

    double *arrival_times; // for time window addition

	double savings;
	int total_number_of_routes;
	double new_total_route_length;
	int move_type;
	int num_arguments;
	int move_arguments[MAX_ARGUMENTS];
	int eval_arguments[MAX_ARGUMENTS];

	bool evaluated_savings;	

};

#endif

