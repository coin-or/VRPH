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

#ifndef _VRP_VRPH_TABU_H
#define _VRP_VRPH_TABU_H

#define NUM_VRPH_TABU_ROUTES		50

class VRPTabuList
{
public:
	VRPTabuList();
	VRPTabuList(int t);
	
	// Destructor
	~VRPTabuList();

	void update_list(VRPRoute *r);
	int max_entries;
	int num_entries;
	int start_index;
	int *hash_vals1;
	int *hash_vals2;
	// Circular buffer containing the hash values of the routes
	bool full;
	// Set to true once we have num_entries elements in the list
	void show();
	void empty();
};


#endif







