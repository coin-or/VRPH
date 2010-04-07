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

#ifndef _VRP_SOLUTION_H
#define _VRP_SOLUTION_H

class VRPSolution
{
	///
	/// Contains fields and methods to process solutions to the VRP.
	///

public:
	VRPSolution();
	VRPSolution(int n);
	~VRPSolution();

	bool in_IP;		// Flag to tell if the solution has been added to the IP before
	double obj;		// objective function value
	int n;			// # of non-DEPOT nodes in the solution
	int *sol;		// Place for a solution buffer
	double time;	// time at which the solution was first discovered
	int hash(int salt);	

};

class VRPSolutionWarehouse
{
public:

	VRPSolutionWarehouse();
	~VRPSolutionWarehouse();

	VRPSolutionWarehouse(int num_sols, int n);

	
	int num_sols;
	int max_size;
	double worst_obj;
	VRPSolution *sols;
	struct htable_entry* hash_table;

	int add_sol(VRPSolution *new_sol, int start_index);
	bool liquidate();
	void sort_sols();
	void show();
	

};


#endif

