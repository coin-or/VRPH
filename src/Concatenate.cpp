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

#include "VRPH.h"

bool Concatenate::evaluate(VRP *V, int i_route, int j_route, int rules, VRPMove *M)
{

    ///
    /// This function takes route i_route and j_route and attempts to 
    /// concatenate them together.
    /// Example: 
    /// 1-i-a-b-c-...-x-1
    /// and
    /// 1-aa-bb-...-zz-j-1
    /// and merges them to form
    /// 1-aa-bb-...-zz-j-i-a-b-c-...-x-1
    /// Note that the move_arguments i_route and j_route refer to the
    /// route numbers themselves and not the node numbers!
    ///
    
    
    double new_length, i_length, j_length;
    int new_load, i_load, j_load;
    int i,j;

    // First make sure that the routes are in the correct form
    // i must be after the VRPH_DEPOT and j must be before the VRPH_DEPOT

    i= V->route[i_route].start;
    j= V->route[j_route].end;


    if(VRPH_MAX(V->pred_array[i],0) != VRPH_DEPOT || VRPH_MAX(V->next_array[j],0)!=VRPH_DEPOT)
    {
        report_error("%s: Concatenate::error in the start/end nodes!\n");
    }

    i_length = V->route[i_route].length;
    j_length = V->route[j_route].length;
    i_load = V->route[i_route].load;
    j_load = V->route[j_route].load;

    new_length = j_length + i_length + V->d[j][i] - (V->d[VRPH_DEPOT][i] +
        V->d[j][VRPH_DEPOT]);
        
    new_load = i_load + j_load;

    

    M->num_affected_routes=2;
    M->route_nums[0]=i_route;
    M->route_nums[1]=j_route;
    M->savings = new_length - (i_length+j_length);//new - old
    // Lengths and loads are the same since we merged and there is really only one route now
    M->route_lens[0]=new_length;
    M->route_lens[1]=0;
    M->route_loads[0]=new_load;
    M->route_loads[1]=0;
    M->route_custs[0]= V->route[i_route].num_customers+V->route[j_route].num_customers;
    M->route_custs[1]=0;
    M->new_total_route_length= V->total_route_length+M->savings;
    M->total_number_of_routes= V->total_number_of_routes-1;// We destroyed a route!
    M->move_type=CONCATENATE;
    M->num_arguments=2;
    M->move_arguments[0]=i_route;
    M->move_arguments[1]=j_route;

    if(V->is_feasible(M, rules)==true)
        return true;
    else
        return false;
    
}

bool Concatenate::move(VRP *V, int i_route, int j_route)
{
    ///
    /// Attempts to merge the two routes i_route and j_route into a single route
    ///
    
    VRPMove M;
    int i,j, pre_i, pre_j, post_i, post_j, end_i, start_j, route_after_i,
        route_after_j, route_before_i, route_before_j, current_node, next_node;

    i= V->route[i_route].start;
    j= V->route[j_route].end;
    // First, evaluate the move

    int rules=0; 
    if(evaluate(V,i_route,j_route, rules, &M)==false)
        return false;    // the move is not allowed

    // Otherwise, it's feasible - make the move

    V->update(&M);

    // Modify the arrays
    
    // pre_i is what used to be before i
    pre_i= V->pred_array[i];
    // post_i is what used to be after i
    post_i= V->next_array[i];
    // pre_j is what used to be before j
    pre_j= V->pred_array[j];
    // post_j is what used to be after j
    post_j= V->next_array[j];

    // i is the start of its own route!
    end_i= V->route[i_route].end;

    // j is the end of its own route!
    start_j= V->route[j_route].start;
    
    // This is the first node in the route that used to follow i
    route_after_i= V->next_array[end_i];
    // This is the last node in the route that used to precede i;
    route_before_i=pre_i;
    
    // This is the first node in the route that used to precede j
    route_before_j= V->pred_array[start_j];
    // This is the first node in the route that used to follow j
    route_after_j=post_j;

    // 1-i-a-b-c-...-x-1
    // and
    // 1-aa-bb-...-zz-j-1
    // becomes
    // 1-aa-bb-...-zz-j-i-a-b-c-...-x-1

    V->next_array[j]=i;
    V->pred_array[i]=j;

    if(VRPH_ABS(route_after_j)==i)
    {
        current_node=start_j;
        while( (next_node= V->next_array[current_node]) > 0)
        {
            V->route_num[current_node]=i_route;

            current_node=next_node;
        }

        V->route_num[current_node]=i_route;
        // Update i_route information
        V->route[i_route].end=end_i;
        V->route[i_route].start=start_j;
        
#if CONCATENATE_VERIFY
        V->verify_routes("Concatenate 1\n");
#endif
        
        return true;        
    }

    if(VRPH_ABS(route_after_i)!=start_j)
    {
        V->next_array[VRPH_ABS(route_before_i)]=route_after_i;
        V->pred_array[VRPH_ABS(route_after_i)]=route_before_i;
        V->next_array[VRPH_ABS(end_i)]=route_after_j;
        V->pred_array[VRPH_ABS(route_after_j)]=-end_i;
    }
    else
    {
        // Must have i in first position of its route
        // j is the very next route with j in last position
        V->next_array[VRPH_ABS(route_before_i)]=-start_j;
        V->pred_array[VRPH_ABS(start_j)]=route_before_i;
        V->next_array[VRPH_ABS(end_i)]=post_j;
        // changed!
        V->pred_array[VRPH_ABS(post_j)]=-end_i;
    }

    // So new start is start(j) and new end is end(i)
    // Now update the rest of the start and end arrays
    current_node=start_j;
    while( (next_node= V->next_array[current_node]) > 0)
    {
        V->route_num[current_node]=i_route;
        current_node=next_node;
    }

    V->route_num[current_node]=i_route;
    // Update route information
    V->route[i_route].end=end_i;
    V->route[i_route].start=start_j;
    

    
#if CONCATENATE_VERIFY
    V->verify_routes("CWConcatenate 2\n");
#endif
    
    return true;

}


