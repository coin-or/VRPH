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

// SEARCH
bool OnePointMove::search(class VRP *V, int j, int rules)
{
    ///
    /// Attempts to find an appropriate one point move involving node j using the specified
    /// rules.     If acceptable move is found, the move is made and function returns
    /// true.  Returns false if no move is found.
    ///
    
    VRPMove M;
    VRPMove BestM;
    M.savings=M.new_total_route_length=BestM.savings=BestM.new_total_route_length=VRP_INFINITY;

    int i,k;
    int best_k=0;
    int accept_type;

    i=VRPH_MAX(V->pred_array[j],VRPH_DEPOT);
    k=VRPH_MAX(V->next_array[j],VRPH_DEPOT);

    // Return false if there are two or fewer nodes in i's route
    if(V->route[V->route_num[j]].num_customers<=3)
        return false;

    if( (rules & VRPH_FIXED_EDGES)  )
    {
        // Make sure we aren't disturbing fixed edges
        if( (V->fixed[i][j]) || (V->fixed[j][k]) )
            return false;

    }    

    // Determine acceptance type
    //default setting
    accept_type=VRPH_FIRST_ACCEPT;

    if( rules & VRPH_FIRST_ACCEPT )
        accept_type=VRPH_FIRST_ACCEPT;
    if( rules & VRPH_BEST_ACCEPT )
        accept_type=VRPH_BEST_ACCEPT;
    if( rules & VRPH_LI_ACCEPT )
        accept_type=VRPH_LI_ACCEPT;

    // Create the search_space
    V->create_search_neighborhood(j, rules);    

    int *old_sol=NULL;
    if(rules & VRPH_TABU)
    {
        // Remember the original solution 
        old_sol=new int[V->num_original_nodes+2];
        V->export_solution_buff(old_sol);
    }
        
    for(i=0;i<V->search_size;i++)
    {            
        k=V->search_space[i];
        if(evaluate(V,j,k,rules,&M)==true)
        {
            // Feasible move found
            if(accept_type==VRPH_FIRST_ACCEPT || (accept_type==VRPH_LI_ACCEPT && M.savings<-VRPH_EPSILON) )
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

            if(accept_type==VRPH_BEST_ACCEPT || accept_type==VRPH_LI_ACCEPT)
            {
                // store the move

                if(M.is_better(V, &BestM, rules))
                {
                    best_k=k;
                    BestM=M;
                }
            }                
        }
    }        


    // We've considered all the possibilities now...
    if(accept_type==VRPH_FIRST_ACCEPT || BestM.savings==VRP_INFINITY)
    {
        // No moves found
        if(old_sol)
            delete [] old_sol;
        return false;
    }

    // else we found a move - try to make it


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
    report_error("%s: move error 4\n",__FUNCTION__);    

    return false;
}


bool OnePointMove::route_search(class VRP *V, int r1, int r2, int rules)
{
    ///
    /// Searches for a one point move where a node from route r1 is moved
    /// into route r2.
    ///

    // So search is now ROUTE_BASED - no neighbor_lists
    if( (rules & VRPH_USE_NEIGHBOR_LIST) > 0)
        report_error("%s: route_searches do not use neighbor list\n",__FUNCTION__);

    // Otherwise, we have a route search - look for all moves involving 
    // route r1 and route r2

    VRPMove M;
    VRPMove BestM;
    int k;
    double best_savings=VRP_INFINITY;
    int best_k=0;
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
        k= V->route[r2].start;
        while(k!=VRPH_DEPOT)
        {
            if(evaluate(V,j,k,rules,&M)==true)
            {

                // Feasible move found
                if(accept_type==VRPH_FIRST_ACCEPT)
                {
                    // This is the first move found -- make it and return if VRPH_FIRST_ACCEPT
                    
                    if(move(V,&M)==true)
                        return true;
                    else
                        report_error("%s: move is false?\n",__FUNCTION__);
                                
                }

                if(accept_type==VRPH_BEST_ACCEPT)
                {
                    // VRPH_BEST_ACCEPT - store the move

                    if(M.savings<best_savings)
                    {
                        best_savings=M.savings;
                        best_k=k;
                        BestM=M;
                    }
                }

                if(accept_type==VRPH_LI_ACCEPT)
                {
                    // move if downhill, store otherwise.
                    
                    if(M.savings<-VRPH_EPSILON)
                    {
                        // it's downhill, so make it.
                        if(move(V,&M)==true)
                            return true;
                        else
                            report_error("%s: false 7\n",__FUNCTION__);
                    }

                    if(M.savings<best_savings)
                    {
                        best_savings=M.savings;
                        best_k=k;
                        BestM=M;
                    }
                }
            }
            k=VRPH_MAX(V->next_array[k],0);
        }
        j=VRPH_MAX(V->next_array[j],0);
    }

    if(accept_type==VRPH_FIRST_ACCEPT)
    {
        // No moves found
        return false;
    }

    // Otherwise we will make the best move found
    if(best_savings==VRP_INFINITY)
        return false;

    // else we found a move - make it
    if(move(V,&BestM)==true)
        return true;
    else
        report_error("%s: false 8\n",__FUNCTION__);
    
    return false;
    

}

// EVALUATE
bool OnePointMove::evaluate(class VRP *V, int j, int b, int rules, VRPMove *M)
{
    ///
    /// This function evaluates the move of inserting j either before or after node b
    /// and places the best savings found in the VRPMove struct M if the move is feasible
    /// and returns false if no feasible move is found, true otherwise.
    ///

    V->num_evaluations[ONE_POINT_MOVE_INDEX]++;

    int a,c,i,k;

    a = VRPH_MAX(V->pred_array[b],VRPH_DEPOT);
    c = VRPH_MAX(V->next_array[b],VRPH_DEPOT);
    k = VRPH_MAX(V->next_array[j],VRPH_DEPOT);
    i = VRPH_MAX(V->pred_array[j],VRPH_DEPOT);


    if(rules & VRPH_FIXED_EDGES)
    {        
        // Make sure we aren't disturbing fixed edges

        if( V->fixed[i][j]|| V->fixed[j][k] )
            return false;

        if(b!=VRPH_DEPOT &&  (V->fixed[b][c] && V->fixed[a][b]) )
            return false;
    }

    M->evaluated_savings=false;

    if(b==j || V->routed[j]==false || V->routed[b]==false || j==VRPH_DEPOT) 
        return false;

    if(b!=VRPH_DEPOT)
    {
        if( (rules & VRPH_INTER_ROUTE_ONLY) && (V->route_num[j] ==V->route_num[b]) )
            return false;

        if((rules & VRPH_INTRA_ROUTE_ONLY) && (V->route_num[j] !=V->route_num[b]) )
            return false;

        // Can quickly check veh capacity: j added to V->route_num[b]
        if(V->route_num[j] != V->route_num[b])
        {
            if( V->nodes[j].demand + V->route[V->route_num[b]].load > V->max_veh_capacity)
                return false;
        }


    }
    
    double savings1, savings2, best_savings;
    Postsert postsert;
    Presert presert;
    best_savings=VRP_INFINITY;
    savings1=VRP_INFINITY;
    savings2=VRP_INFINITY;

    // First consider the complicated case where b is the VRPH_DEPOT.
    // We have 2*r edges to consider where r is the # of routes
    // In this case we will find the best possible insertion
    if(b==VRPH_DEPOT)
    {
        int current_start, current_end, current_route;
        current_start=abs(V->next_array[VRPH_DEPOT]);
        bool allowed, found_move;
        VRPMove CurrentM;
        found_move=false;
        
        for(;;)
        {
            // Consider the edge VRPH_DEPOT-current_start
            int t=current_start;
            allowed=true;

            if(j!=t)
            {
                // Check for fixed edges
                if((rules & VRPH_FIXED_EDGES) && V->fixed[VRPH_DEPOT][t])
                    allowed=false;

                if( (presert.evaluate(V,j,t,&CurrentM)==true)&&(V->check_move(&CurrentM,rules)==true) && allowed )
                {
                    
                    if(CurrentM.is_better(V,M,rules))
                    {
                        found_move=true;
                        *M=CurrentM;
                    }
                }
            }

            // Now try the t-VRPH_DEPOT edge                
            current_route= V->route_num[current_start];
            current_end= V->route[current_route].end;
            t=current_end;
            allowed=true;
            if(j!=t)
            {
                // Check for fixed edges
                if((rules & VRPH_FIXED_EDGES) && V->fixed[t][VRPH_DEPOT])
                    allowed=false;

                if( (postsert.evaluate(V,j,t,&CurrentM)==true)&&(V->check_move(&CurrentM,rules)==true) && allowed )
                {
                    if(CurrentM.is_better(V,M,rules))
                    {
                        found_move=true;
                        *M=CurrentM;
                    }
                }
                
            }

            // Now advance to the next node adjacent to the VRPH_DEPOT
            current_start=abs(V->next_array[current_end]);
            if(current_start==VRPH_DEPOT)    // We're done
                break;
        }

        return found_move;

        
    }
    
    // Special Case
    if(c == j)
    {
        // Only option is to insert j between a and b (b is not VRPH_DEPOT)

        if(rules & VRPH_FIXED_EDGES)
        {
            // Make sure we aren't disturbing fixed edges

            if( V->fixed[a][b] )//|| V->fixed[b][c])
                return false;
        }
        
        if( (presert.evaluate(V,j,b,M)==true)&&(V->check_move(M,rules)==true) )
            return true;
        else
            return false;
        
    }


    if(a == j)
    {
        // Only option is to insert j between b and c.  b is not VRPH_DEPOT

        if(rules & VRPH_FIXED_EDGES)
        {
            // Make sure we aren't disturbing fixed edges

            if( V->fixed[b][c] )//|| V->fixed[a][b] ) 
                return false;
        }

        if( (postsert.evaluate(V,j,b,M)==true)&&(V->check_move(M,rules)==true) )
            return true;
        else
            return false;
        
    }
    
    // No conflicts o/w    - can insert j either before or after b 
    // We will choose the better move always!

    // savings = new-old
    savings1 = (V->d[a][j]+V->d[j][b]+V->d[i][k]) - (V->d[a][b]+V->d[i][j]+V->d[j][k])  ;
    savings2 = (V->d[i][k]+V->d[b][j]+V->d[j][c]) - (V->d[b][c]+V->d[i][j]+V->d[j][k])  ;

    best_savings = VRPH_MIN(savings1, savings2);

    
    if( savings1 <= savings2 &&  (presert.evaluate(V,j,b,M)==true)
        &&(V->check_move(M,rules)==true) )
    {

        if(rules & VRPH_FIXED_EDGES)
        {
            // Make sure we aren't disturbing fixed edges

            if( V->fixed[a][b] )//|| V->fixed[b][c] ) 
                return false;
        }
        return true;
    }
    
    // Now check savings2
    if( savings2<savings1 && postsert.evaluate(V,j,b,M)==true &&
        (V->check_move(M,rules)==true) )
    {

        if(rules & VRPH_FIXED_EDGES)
        {
            // Make sure we aren't disturbing fixed edges

            if( V->fixed[b][c] )//|| V->fixed[a][b] ) 
                return false;
        }

        return true;
    }

    // Need to check savings1 again in the event that savings2 was better, but the move
    // itself was infeasible--not sure if this ever happens...

    if( (presert.evaluate(V,j,b,M)==true)&&(V->check_move(M,rules)==true))
    {
        if(rules & VRPH_FIXED_EDGES)
        {
            // Make sure we aren't disturbing fixed edges

            if( V->fixed[a][b] )//|| V->fixed[b][c]) 
                return false;
        }

        return true;
    }
    

    
    // ELSE no feasible moves found
    return false;
}

bool OnePointMove::move(class VRP *V, VRPMove *M)
{
    ///
    /// Makes the one point move determined by the VRPMove M. 
    ///        
    
        
    if(M->move_type==PRESERT)
    {
        Presert presert;
        if(presert.move(V,M->move_arguments[0],M->move_arguments[1]))
        {
            V->num_moves[ONE_POINT_MOVE_INDEX]++;
            V->capture_best_solution();

            return true;
        }
        else
            report_error("%s: presert move is false\n",__FUNCTION__);


    }
    else
    {
        if(M->move_type==POSTSERT)
        {
            Postsert postsert;
            if(postsert.move(V,M->move_arguments[0],M->move_arguments[1]))
            {
                V->num_moves[ONE_POINT_MOVE_INDEX]++;
                V->capture_best_solution();
                return true;
            }
            else
                report_error("%s: postsert move is false\n",__FUNCTION__);
        }
    }
    
    return false;
    
}
