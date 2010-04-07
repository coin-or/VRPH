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

VRPMove::VRPMove()
{
    this->evaluated_savings=false;
    this->move_type=-1;
    this->new_total_route_length=VRP_INFINITY;
    this->num_affected_routes=-1;
    this->num_arguments=-1;
    this->savings=-1;
    this->total_number_of_routes=-1;

    this->arrival_times=NULL;
    
}

VRPMove::VRPMove(int n)
{
    this->evaluated_savings=false;
    this->move_type=-1;
    this->new_total_route_length=VRP_INFINITY;
    this->num_affected_routes=-1;
    this->num_arguments=-1;
    this->savings=-1;
    this->total_number_of_routes=-1;

    this->arrival_times=new double[n]; // Set up for time windows for each customer
}

VRPMove::~VRPMove()
{
    if(this->arrival_times)
        delete [] this->arrival_times;
}


bool VRPMove::is_better(VRP *V, VRPMove *M2, int rules)
{
    ///
    /// Evaluates this move versus M2 in terms of the provided
    /// rules.  Returns true of this move is superior to M2
    /// and false otherwise.  
    /// 

    if(M2->num_affected_routes==-1)
    {
        // M2 does not have meaningful information, so return true
        // Probably has savings=VRP_INFINITY
        return true;
    }


    // We are only concerned about the savings (increase/decrese) in total length
    // This is the default approach
    if(rules & VRPH_SAVINGS_ONLY)
    {
        // Decide in terms of total length only

        if(this->savings <= M2->savings)
            return true;
        else
            return false;
        
    }


    // We will try to maximize the sum of the squares of the # of customers on a route
    if(rules & VRPH_MINIMIZE_NUM_ROUTES)
    {
        // First check the # of routes in the solution produced by the two moves
        if(this->total_number_of_routes<M2->total_number_of_routes)
            return true;

        if(this->total_number_of_routes>M2->total_number_of_routes)
            return false;

        // Otherwise the # of routes remains the same
        // If the two moves affect diff. #'s of routes, then just use the total length
        if(this->num_affected_routes != M2->num_affected_routes)
        {
            if(this->savings < M2->savings)
                return true;
            else
                return false;    
        }

        // If they affect the same # of routes, minimize the sum of the squares
        // of the customers in the route

        int i,sq, sq2;

        sq=0; sq2=0;
        for(i=0;i<this->num_affected_routes;i++)
            sq+= (this->route_custs[i])*(this->route_custs[i]);

        for(i=0;i<M2->num_affected_routes;i++)
            sq2+= (M2->route_custs[i])*(M2->route_custs[i]);

        if(sq>sq2)
            // this move is better
            return true;
        if(sq2<sq)
            // M2 is better
            return false;

        // Otherwise, sq==sq2, use the savings

        if(this->savings <= M2->savings)
            return true;
        else
            return false;    
    }


    // Shouldn't get here!
    report_error("%s: Reached bizarre place with rules=%08x\n",__FUNCTION__,rules);
    return false;

}

