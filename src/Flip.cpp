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



bool Flip::evaluate(class VRP *V, int start_point, int end_point, VRPMove *M)
{    
    ///
    /// Evaluates the move of reversing a portion of a route in between nodes
    /// start and end.  
    /// Example:  0-a-b-start-d-e-f-g-h-end-x-y-z-0 becomes
    ///           0-a-b-start-h-g-f-e-d-end-x-y-z-0.
    /// If the move is feasible, the information
    /// regarding the move is stored in the VRPMove data structure M.
    /// start_point must be before end_point in the current route orientation.  
    ///

    int post_start, pre_end, route_num;
    double old_cost, new_cost;
    double savings, route_len;

    //Check for VRPH_DEPOT nodes!
    if(start_point==VRPH_DEPOT || end_point==VRPH_DEPOT)
        report_error("%s: flip::called with VRPH_DEPOT node-ambiguous\n");

    route_num= V->route_num[start_point];

    if(V->route_num[end_point]!=route_num)
    {
        fprintf(stderr,"%d(route #=%d), %d(route #=%d)\n",start_point, route_num,
            end_point, V->route_num[end_point]);

        report_error("%s: flip attempted using different routes!\n");
    }

    if(V->next_array[start_point]==end_point)
        return false;

    post_start= VRPH_MAX(V->next_array[start_point],VRPH_DEPOT);
    pre_end= VRPH_MAX(V->pred_array[end_point],VRPH_DEPOT);

    if(post_start == pre_end || post_start==end_point || pre_end==start_point)
        return false;    // Nothing to reverse!


    // Need to compute the old costs and the new cost - 

    old_cost=(V->d[start_point][post_start]-1*V->nodes[post_start].service_time) + 
        (V->d[pre_end][end_point] -  1*V->nodes[end_point].service_time) ;
    new_cost=(V->d[start_point][pre_end]-1*V->nodes[pre_end].service_time) + 
        (V->d[post_start][end_point] - 1*V->nodes[end_point].service_time);

    savings=new_cost - old_cost;
    
    // The move satisfies the savings rules - now check feasibility...

    route_len= V->route[route_num].length;

    // Check feasibility
    route_len=route_len+savings;

    if(route_len>V->max_route_length)
        return false;    // infeasible

    // Otherwise it's feasible since no load change occurs
    M->num_affected_routes=1;
    M->savings=savings;
    M->route_nums[0]=route_num;
    M->route_lens[0]=route_len;
    M->route_loads[0]=V->route[route_num].load;
    M->route_custs[0]= V->route[route_num].num_customers; // no change
    M->new_total_route_length= V->total_route_length+savings;
    M->total_number_of_routes = V->total_number_of_routes;
    M->move_type=FLIP;
    M->num_arguments=2;
    M->move_arguments[0]=start_point;
    M->move_arguments[1]=end_point;


    return true;

}

bool Flip::move(VRP *V, int start_point, int end_point)
{
    ///
    /// This reverses the portion of the route between start_point and end_point
    /// if the proposed move is feasible.  Returns true and makes all relevant
    /// solution modifications if the move is made and false otherwise.
    ///

    int current, old_next, cnt;
    int start, end, i, route_num;
    VRPMove M;

    if(start_point<=VRPH_DEPOT || end_point<=VRPH_DEPOT )
        report_error("%s: flip::reverse_partial_route called with VRPH_DEPOT or negative index\n");

    if(start_point==end_point)
        report_error("%s: flip::reverse_partial_route called with start==end\n");

    route_num= V->route_num[start_point];

    if(route_num!= V->route_num[end_point])
        report_error("%s: flip::not in the same route\n");

    // evaluate the move
    if(evaluate(V,start_point,end_point, &M)==false)
        return false;

    V->update(&M);

    // Now update the arrays

    i=0;

    start = start_point;
    end = end_point;
    // Assume the orientation is correct to avoid checking again!!

    // Special case!  next(start) = end
    if(VRPH_MAX(VRPH_DEPOT,V->next_array[start])==end)
    {
        int pre_start= V->pred_array[start];

        if(pre_start<0)
            report_error("%s: flip::pre_start <0 in special case!!\n");

        old_next= V->next_array[end];

        V->next_array[end]=start;
        V->pred_array[start]=end;

        V->next_array[start]=old_next;
        V->pred_array[old_next]=start;

        V->next_array[pre_start]=end;
        V->pred_array[end]=pre_start;

#if FLIP_VERIFY
        V->verify_routes("flip 1\n");
#endif

        return true;

    }

    // Otherwise we have only a single case to handle
    current= V->next_array[start];        //n1
    old_next= V->next_array[current];    //n2


    V->next_array[current]=end;        //next[n1]=end
    V->pred_array[end]=current;        //pred[end]=n1;
    V->pred_array[current]=old_next;    //pred[n1]=n2;
    current=old_next;                            //current=n2
    old_next= V->next_array[current];    //old_next=n3

    cnt=0;

    while(old_next != end)
    {

        V->next_array[current] = V->pred_array[current];
        V->pred_array[current]=old_next;
        current = old_next;
        old_next = V->next_array[current];
        cnt++;
        if(cnt>V->num_nodes)
            report_error("%s: flip::Impossible loop encountered\n");

    }
    V->next_array[current]= V->pred_array[current];
    V->pred_array[current]=start;
    V->next_array[start]=current;

    return true;
}
