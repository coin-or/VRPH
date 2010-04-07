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
bool TwoPointMove::search(class VRP *V, int j, int rules)
{
    ///
    /// Attempts to find the best Two-Point move involving node j using the specified
    /// search space, and rules.
    /// If an acceptable move is found, then the move is made and all relevant solution
    /// modifications are made.
    ///

    VRPMove M;
    VRPMove BestM;
    BestM.savings=VRP_INFINITY;
    int i,k;
    int accept_type;

    if(j==VRPH_DEPOT)        
        return false;

    if(rules & VRPH_FIXED_EDGES)
    {
        i=VRPH_MAX(V->pred_array[j],VRPH_DEPOT);
        k=VRPH_MAX(V->next_array[j],VRPH_DEPOT);

        // Make sure we aren't disturbing fixed edges

        if( V->fixed[i][j] || V->fixed[j][k] ) 
            return false;
    }

    accept_type=VRPH_FIRST_ACCEPT;//default

    if( (rules & VRPH_FIRST_ACCEPT) > 0)
        accept_type=VRPH_FIRST_ACCEPT;
    if( (rules & VRPH_BEST_ACCEPT) > 0)
        accept_type=VRPH_BEST_ACCEPT;
    if( (rules & VRPH_LI_ACCEPT) > 0)
        accept_type=VRPH_LI_ACCEPT;

    int *old_sol=NULL;
    if(rules & VRPH_TABU)
    {
        // Remember the original solution 
        old_sol=new int[V->num_original_nodes+2];
        V->export_solution_buff(old_sol);
    }

    // Create the search_space
    V->create_search_neighborhood(j, rules);

    for(i=0;i<V->search_size;i++)
    {
        k=V->search_space[i];

        if(k!=VRPH_DEPOT && k!=j)
        {
            // VRPH_DEPOT not allowed in TwoPointMove

            if(evaluate(V,j,k,rules,&M)==true)
            {
                // Feasible move found
                if(accept_type==VRPH_FIRST_ACCEPT || (accept_type==VRPH_LI_ACCEPT && M.savings<-VRPH_EPSILON) )
                {
                    // make the move

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

                        }
                    }
                }



                if(accept_type==VRPH_BEST_ACCEPT || accept_type==VRPH_LI_ACCEPT )
                {
                    // compare to best move so far
                    if(M.is_better(V, &BestM, rules))
                        BestM=M;

                }

            }
        }
    }



    if(accept_type==VRPH_FIRST_ACCEPT)
    {
        if(old_sol)
            delete [] old_sol;
        return false;
    }

    if(BestM.savings==VRP_INFINITY)
    {
        if(old_sol)
            delete [] old_sol;
        return false;
    }
    // else we found a move - make it

    if(move(V,&BestM)==true)
    {
        if(!(rules & VRPH_TABU))
            return true;                    
    }

    if(rules & VRPH_TABU)
    {    
        // Check VRPH_TABU status of move - return true if its ok
        // or revert to old_sol if not and return
        if(V->check_tabu_status(&M, old_sol))
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


    report_error("%s: best move false!\n",__FUNCTION__);


    return false;
}


bool TwoPointMove::route_search(class VRP *V, int r1, int r2, int rules)
{
    ///
    /// Searches for all TPM moves involving a node from route r1
    /// and the other from route r2.
    ///


    VRPMove M;
    VRPMove BestM;
    int j,k;
    int accept_type;

    BestM.savings=VRP_INFINITY;

    if(r1==r2)
    {
        fprintf(stderr,"TPM::%d==%d???\n",r1,r2);
        report_error("%s: called with r1==r2\n",__FUNCTION__);
    }

    if( (rules & VRPH_USE_NEIGHBOR_LIST) > 0)
        report_error("%s: ROUTE_BASED search does not use neighbor_list\n",__FUNCTION__);

    accept_type=VRPH_FIRST_ACCEPT;//default

    if( (rules & VRPH_FIRST_ACCEPT) > 0)
        accept_type=VRPH_FIRST_ACCEPT;
    if( (rules & VRPH_BEST_ACCEPT) > 0)
        accept_type=VRPH_BEST_ACCEPT;
    if( (rules & VRPH_LI_ACCEPT) > 0)
        accept_type=VRPH_LI_ACCEPT;


    j= V->route[r1].start;
    while(j!=VRPH_DEPOT)
    {

        k= V->route[r2].start;
        while(k!=VRPH_DEPOT)
        {
            // Try to swap nodes j and k
            if(evaluate(V,j,k,rules,&M)==true)
            {
                // valid move found
                if(accept_type == VRPH_FIRST_ACCEPT || (accept_type==VRPH_LI_ACCEPT && M.savings<-VRPH_EPSILON) )
                {
                    // make the move
                    if(move(V,&M)==false)
                        report_error("%s: first accept move is false!\n",__FUNCTION__);
                    else
                        return true;
                }

                if( (accept_type == VRPH_LI_ACCEPT) || (accept_type==VRPH_BEST_ACCEPT) )
                {

                    // See if it's the best so far...
                    if(M.is_better(V, &BestM, rules))
                        BestM=M;
                }


            }
            // Advance the route r2 node
            k=VRPH_MAX(V->next_array[k],0);
        }
        // Advance the route r1 node
        j=VRPH_MAX(V->next_array[j],0);
    }


    if(accept_type==VRPH_FIRST_ACCEPT)
        return false;    // No moves found
    if(BestM.savings == VRP_INFINITY)
        return false;

    if( (accept_type == VRPH_LI_ACCEPT) || (accept_type == VRPH_BEST_ACCEPT))
    {

        // We found a move -- make it...
        if(move(V,&BestM)==false)
            report_error("%s: best move is false!\n",__FUNCTION__);
        else
            return true;

    }

    report_error("%s: shouldn't be here...\n",__FUNCTION__);

    return false;

}

bool TwoPointMove::evaluate(class VRP *V, int j, int b, int rules, VRPMove *M)
{
    ///
    /// This function evaluates the move of swapping the positions of j 
    /// and b in the current solution.  If a satisfactory move is found subject
    /// to the provided rules, then the solution modification data is placed
    /// in the VRPMove M and the function returns true.  Returns false otherwise.
    ///

    V->num_evaluations[TWO_POINT_MOVE_INDEX]++;    

    if(V->routed[j]==false || V->routed[b]==false || j==b)
        return false;

    if(rules & VRPH_FIXED_EDGES)
    {
        // Make sure we aren't disturbing fixed edges
        int i,k,a,c;

        a=VRPH_MAX(V->pred_array[b],VRPH_DEPOT);
        c=VRPH_MAX(V->next_array[b],VRPH_DEPOT);

        i=VRPH_MAX(V->pred_array[j],VRPH_DEPOT);
        k=VRPH_MAX(V->next_array[j],VRPH_DEPOT);

        if( V->fixed[a][b] || V->fixed[b][c] || V->fixed[i][j] || V->fixed[j][k] ) 
            return false;
    }

    M->evaluated_savings=false;// false until we check


    class Swap swap;

    if(V->routed[j]==false || V->routed[b]==false)
        return false;

    if(j==VRPH_DEPOT || b==VRPH_DEPOT || j==b)
    {
        fprintf(stderr,"TPM::j=%d, b=%d\n",j,b);
        report_error("%s: evaluate() called with bad arguments??\n",__FUNCTION__);
    }

    if((rules & VRPH_INTER_ROUTE_ONLY) && (V->route_num[j]==V->route_num[b]) )
        return false;

    if((rules & VRPH_INTRA_ROUTE_ONLY) && (V->route_num[j]!=V->route_num[b]) )
        return false;


    if( (swap.evaluate(V,j,b,M)==true) && (V->check_move(M, rules)==true))
    {
        return true;
        // The move is good
    }
    else
    {
        // Not allowed
        return false;
    }

}


bool TwoPointMove::move(class VRP *V, VRPMove *M)
{

    ///
    /// Performs the actual solution modification given by the move M.
    ///

    if(M->move_type!=SWAP)
        report_error("%s: Unknown move type\n",__FUNCTION__);

    class Swap swap;

    if(swap.move(V,M->move_arguments[0], M->move_arguments[1])==false)
    {
        // This is an error
        report_error("%s: TPM::swap.move evaluates to false!!\n",__FUNCTION__);

    }

    V->capture_best_solution();
    V->num_moves[TWO_POINT_MOVE_INDEX]++;

    return true;

}

