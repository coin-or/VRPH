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


bool OrOpt::search(class VRP *V, int a, int len, int rules)
{
    ///
    /// Looks for string insertions of length len beginning at a that 
    /// meet the provided rules.  Makes move if one is found.
    ///


    VRPMove M, BestM;
    BestM.savings=VRP_INFINITY;
    int i,b,c,d;
    int string_end;
    int str[10];
        
    int accept_type;

    //default setting
    accept_type=VRPH_FIRST_ACCEPT;

    if( (rules & VRPH_FIRST_ACCEPT) > 0)
        accept_type=VRPH_FIRST_ACCEPT;
    if( (rules & VRPH_BEST_ACCEPT) > 0)
        accept_type=VRPH_BEST_ACCEPT;
    if( (rules & VRPH_LI_ACCEPT) > 0)
        accept_type=VRPH_LI_ACCEPT;
    
    string_end = V->get_string_end(a,len);

    if(string_end==-1)
    {
        // We couldn't get a string of the right length
        return false;
    }

    if(rules & VRPH_FIXED_EDGES)
    {
        // Make sure we aren't disturbing fixed edges
        i=VRPH_MAX(V->pred_array[a],VRPH_DEPOT);

        if( V->fixed[i][a] ) 
            return false;

        i=VRPH_MAX(V->next_array[string_end],VRPH_DEPOT);
        if( V->fixed[string_end][i])
            return false;
    }


    // Load the string into the str array to look for the VRPH_DEPOT
    str[0]=a;
    for(i=1;i<len;i++)
    {
        str[i]= V->next_array[str[i-1]];
        if(str[i]<=VRPH_DEPOT)
            return false;
    }

    V->create_search_neighborhood(a, rules);

    int *old_sol=NULL;
    if(rules & VRPH_TABU)
    {
        // Remember the original solution 
        old_sol=new int[V->num_original_nodes+2];
        V->export_solution_buff(old_sol);
    }
        
    for(i=0;i<V->search_size;i++)
    {
        c=V->search_space[i];
    
        if(c!=VRPH_DEPOT)                
        {
            
            d=VRPH_MAX(V->next_array[c],0);
            b=VRPH_MAX(V->pred_array[c],0);

            int flag=0;
            if(d==VRPH_DEPOT)
                flag=1;

            // Look for overlap
            
            for(int j=0;j<len;j++)
            {
                if(str[j]==c || str[j]==d)
                    // Either c or d is already in the string that starts at a
                    flag=1;
            }
            if(flag==0)
            {

                // Try the move
                if(evaluate(V,a,len,c,d,rules, &M)==true)
                {
                    // The move is good--make it
                    if(accept_type == VRPH_FIRST_ACCEPT || (accept_type==VRPH_LI_ACCEPT && M.savings<-VRPH_EPSILON))
                    {
                        if(move(V, &M)==false)
                            report_error("%s: move error 1\n",__FUNCTION__);
                        else
                        {
                            if(!(rules & VRPH_TABU))
                                return true;
                            else
                            {
                                // Check VRPH_TABU status of move - return true if its ok
                                // or revert to old_sol if not and continue to search.
                                if(V->check_tabu_status(&M, old_sol))
                                {
                                    delete [] old_sol;
                                    return true; // The move was ok
                                }
                                // else we reverted back - continue the search for a move
                            }
                        }                        
                    }

                    if(accept_type == VRPH_BEST_ACCEPT || accept_type==VRPH_LI_ACCEPT)
                    {
                        if(M.is_better(V, &BestM, rules))//if(M.savings<best_savings)
                            BestM=M;
                            
                    }

                    if(accept_type == VRPH_LI_ACCEPT)
                    {
                        // Move if downhill
                        if(M.savings<-VRPH_EPSILON)
                        {
                            if(move(V, &M)==false)
                                report_error("%s: move error 2\n",__FUNCTION__);
                            else
                            {
                                if(!(rules & VRPH_TABU))
                                    return true;
                                else
                                {
                                    // Check VRPH_TABU status of move - return true if its ok
                                    // or revert to old_sol if not and continue to search.
                                    if(V->check_tabu_status(&M, old_sol))
                                    {
                                        delete [] old_sol;
                                        return true; // The move was ok
                                    }
                                    // else we reverted back - continue the search for a move
                                }
                            }                        
                        }
                        else
                        {
                            // Uphill move - see if it's the best so far
                            if(M.is_better(V, &BestM, rules))//if(M.savings<BestM.savings)
                                BestM=M;
                        }
                    }
                }
            }
        }
    }

    if(accept_type == VRPH_FIRST_ACCEPT || BestM.savings==VRP_INFINITY)
    {
        if(old_sol)
            delete [] old_sol;
        return false ; // No moves found
    }

    if(move(V,&BestM)==true)
    {
        if(!(rules & VRPH_TABU))
            return true;                    
    }
    
    if(rules & VRPH_TABU)
    {    
        // Check VRPH_TABU status of move - return true if its ok
        // or revert to old_sol if not and return
        if(V->check_tabu_status(&BestM, old_sol))// was &M??
        {
            delete [] old_sol;
            return true; // The move was ok
        }
        else
        {
            delete [] old_sol;
            return false;
        }
    }
        
    // Should have already returned
    report_error("%s: move error\n",__FUNCTION__);
    return false;


}

bool OrOpt::route_search(VRP *V, int r1, int r2, int len, int rules)
{
    ///
    /// Searches for the best OrOpt move where we take a string of length len
    /// from route r1 and try to move the string into route r2 (and vice versa)
    /// subject to the provided rules
    ///

    VRPMove M, BestM;
    BestM.savings=VRP_INFINITY;
    int c,d;
    int accept_type;

    //default setting
    accept_type=VRPH_FIRST_ACCEPT;

    if( (rules & VRPH_FIRST_ACCEPT) > 0)
        accept_type=VRPH_FIRST_ACCEPT;
    if( (rules & VRPH_BEST_ACCEPT) > 0)
        accept_type=VRPH_BEST_ACCEPT;
    if( (rules & VRPH_LI_ACCEPT) > 0)
        accept_type=VRPH_LI_ACCEPT;

    int j;
    j= V->route[r1].start;
    while(j!=VRPH_DEPOT)
    {
        // Loop through r1 and r2
        c= V->route[r2].start;
        while(c!=VRPH_DEPOT)
        {
            d=VRPH_MAX(V->next_array[c],0);
            if(evaluate(V,j,len,c,d,rules, &M)==true)
            {
                if(accept_type==VRPH_FIRST_ACCEPT || (accept_type==VRPH_LI_ACCEPT && M.savings<-VRPH_EPSILON) )
                {    
                    // Make the move
                    if(move(V,&M)==false)
                        report_error("%s: first accept move returns false",__FUNCTION__);
                    else    
                        return true;
                }


                if(accept_type == VRPH_LI_ACCEPT || accept_type == VRPH_BEST_ACCEPT)
                {
                    if(M.is_better(V, &BestM, rules))
                        BestM=M;                

                }

            }

            // Advance r2 node
            c=d;
        }

        // Advance r1 node
        j=VRPH_MAX(V->next_array[j],0);
    }

    if(accept_type==VRPH_FIRST_ACCEPT)
        return false; // No moves found

    if(BestM.savings==VRP_INFINITY)
        return false;

    if(accept_type==VRPH_BEST_ACCEPT || accept_type==VRPH_LI_ACCEPT)
    {
        
            // Make the move
            if(move(V,&BestM)==false)
                report_error("%s: first accept move returns false\n",__FUNCTION__);
            else    
                return true;

    }

    report_error("%s: shouldn't get here...\n",__FUNCTION__);

    return false;

}

bool OrOpt::evaluate(class VRP *V, int a, int len, int c, int d, int rules, VRPMove *M)
{
    ///
    /// Evaluates the move of taking the string of length len beginning at a and 
    /// inserting it between c and d subject to the provided rules
    ///

    V->num_evaluations[OR_OPT_INDEX]++;

    M->evaluated_savings=false;

    if(rules & VRPH_FIXED_EDGES)
    {
        // Make sure we aren't disturbing fixed edges
        if( V->fixed[c][d] ) 
            return false;
    }    

    if(V->routed[a]==false || V->routed[c]==false  ||
        V->routed[d]==false )
        return false;

    int a_route, c_route;

    M->eval_arguments[0]=a;M->eval_arguments[1]=len;
    M->eval_arguments[2]=c;M->eval_arguments[3]=d;

    // First make sure the edge c-d exists
    if(c!=VRPH_DEPOT && VRPH_MAX(V->next_array[c],0)!=d )
        report_error("%s: c-d not an edge!\n",__FUNCTION__);
        

    if(c==VRPH_DEPOT && VRPH_MAX(V->pred_array[d],0)!=c)
        report_error("%s: c-d not an edge!\n",__FUNCTION__);
    

    a_route= V->route_num[a];
    if(c!=VRPH_DEPOT)
        c_route= V->route_num[c];
    else
        c_route= V->route_num[d];

    
    if( (rules & VRPH_INTER_ROUTE_ONLY) && (a_route ==c_route))
        return false;

    if((rules & VRPH_INTRA_ROUTE_ONLY) && a_route !=c_route)
        return false;
    
    // Construct the appropriate string;

    int string_end = V->get_string_end(a, len);
    if(string_end==-1)
        // The string of length len beginning at a is not contained in a single route
        return false;

    if(rules & VRPH_FIXED_EDGES)
    {
        // Make sure we aren't disturbing fixed edges
        int z,b;

        z=VRPH_MAX(V->pred_array[a],VRPH_DEPOT);
        b=VRPH_MAX(V->next_array[string_end],VRPH_DEPOT);

        if( V->fixed[z][a] || V->fixed[string_end][b] ) 
            return false;
        
        
    }

    // Now just use MoveString to evaluate

    MoveString MS;

    if(MS.evaluate(V,c,d,a,string_end, M)==true) 
    {
        if(V->check_move(M,rules)==true)
            return true;
        else
            return false;
        
    }
    

    return false;


}



bool OrOpt::move(class VRP *V, VRPMove *M)
{
    ///
    /// Modifies all solution information by taking the string of length
    /// len at and inserting between c and d if it meets the rules
    ///

    int a,len,c,d;

    a=M->eval_arguments[0];
    len=M->eval_arguments[1];
    c=M->eval_arguments[2];
    d=M->eval_arguments[3];

    int string_end = V->get_string_end(a, len);
    if(string_end==-1)
    {
        report_error("%s: no string end in move??\n",__FUNCTION__);
        
    }

    MoveString MS;

    if(    MS.move(V,c,d,a,string_end)==false)
    {
        report_error("%s: MS.move is false!\n",__FUNCTION__);
        
    }

    V->num_moves[OR_OPT_INDEX]++;
    V->capture_best_solution();
    return true;
}
