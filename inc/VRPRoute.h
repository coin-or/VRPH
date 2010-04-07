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

#ifndef _VRP_ROUTE_H
#define _VRP_ROUTE_H


#define MAX_NEIGHBORING_ROUTES		5
#define DUPLICATE_ROUTE				0
#define OVERWRITTEN_ROUTE			1
#define ADDED_ROUTE					2
#define BETTER_ROUTE				3


class VRPRoute
{
	/// 
	/// Stores information about a particular route.  The ordering
	/// field is not updated during the search and is filled in
	/// only when requested.
	///
public:
	
	VRPRoute();
	VRPRoute(int n);
	~VRPRoute();
	
	int start;
	int end;
	double length;
	int load;
	int num_customers;
	double obj_val;

	int hash_val;
	int hash_val2;
	
	double total_service_time;
	double time;
	double *x;
	double *y;

	char *name;		// Used when we add a route to an IP as a column
	
	double x_center;
	double y_center;

	double min_theta;
	double max_theta;
	
	int neighboring_routes[MAX_NEIGHBORING_ROUTES];

	int *ordering;

	void create_name();

	int hash(int salt);

};


class VRPRouteWarehouse
{
public:
	VRPRouteWarehouse();
	VRPRouteWarehouse(int h_size);
	~VRPRouteWarehouse();

	int hash_table_size;
	int num_unique_routes;
	struct htable_entry* hash_table;

	void remove_route(int hash_val, int hash_val2);
	int add_route(VRPRoute *R);
	void liquidate();


};
#endif

