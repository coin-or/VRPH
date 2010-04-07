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

bool ThreePointMove::search(VRP *V, int b, int rules)
{
    ///
    /// Searches for ThreePointMoves involving node b.  In this
    /// move, the position of node b is exchanged with two other 
    /// nodes in an existing edge.
    ///

    VRPMove M;
    VRPMove BestM;
    BestM.savings=VRP_INFINITY;
    int h,i,ii,j;
    int accept_type;
    
    if(b==VRPH_DEPOT)        
        return false;

    if(rules & VRPH_FIXED_EDGES)
    {
        i=VRPH_MAX(V->pred_array[b],VRPH_DEPOT);
        j=VRPH_MAX(V->next_array[b],VRPH_DEPOT);

        // Make sure we aren't disturbing fixed edges

        if( V->fixed[i][b] || V->fixed[b][j] ) 
            return false;
    }

    accept_type=VRPH_FIRST_ACCEPT;//default

    if( (rules & VRPH_FIRST_ACCEPT) > 0)
        accept_type=VRPH_FIRST_ACCEPT;
    if( (rules & VRPH_BEST_ACCEPT) > 0)
        accept_type=VRPH_BEST_ACCEPT;
    if( (rules & VRPH_LI_ACCEPT) > 0)
        accept_type=VRPH_LI_ACCEPT;


    // Create the search_space
    V->create_search_neighborhood(b, rules);

    int *old_sol=NULL;
    if(rules & VRPH_TABU)
    {
        // Remember the original solution 
        old_sol=new int[V->num_original_nodes+2];
        V->export_solution_buff(old_sol);
    }
        
    for(ii=0;ii<V->search_size;ii++)
    {
        // Get the edge i-j
        i=V->search_space[ii];
        j=VRPH_MAX(VRPH_DEPOT,V->next_array[i]);
        h=VRPH_MAX(VRPH_DEPOT,V->pred_array[i]);

        // Edge i-j
        if(i!=VRPH_DEPOT && j!=VRPH_DEPOT)
        {
            // We have the edge i-j to consider swapping with node b

            if(evaluate(V,b,i,j,rules,&M)==true)
            {
                
                if((accept_type==VRPH_FIRST_ACCEPT) || ((accept_type==VRPH_LI_ACCEPT)&&M.savings<=-VRPH_EPSILON))
                {
                    // Make the move
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

                if(accept_type==VRPH_LI_ACCEPT || accept_type==VRPH_BEST_ACCEPT)
                {
                    if(M.is_better(V, &BestM, rules))
                        BestM=M;
                }
            }
        }

        // Edge h-i
        if(h!=VRPH_DEPOT && i!=VRPH_DEPOT)
        {
            // We have the edge h-i to consider swapping with node b
            if(evaluate(V,b,h,i,rules,&M)==true)
            {
                if(accept_type==VRPH_FIRST_ACCEPT || ((accept_type==VRPH_LI_ACCEPT)&&M.savings<=-VRPH_EPSILON))
                {
                    // Make the move
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

                if(accept_type==VRPH_LI_ACCEPT || accept_type==VRPH_BEST_ACCEPT)
                {
                    if(M.is_better(V, &BestM, rules))
                        BestM=M;
                }
            }
        }
    }

    if(accept_type==VRPH_FIRST_ACCEPT || BestM.savings==VRP_INFINITY)
    {
        if(rules&VRPH_TABU)
            delete [] old_sol;
        return false;        // No moves found
    }    

    if(accept_type==VRPH_BEST_ACCEPT || accept_type==VRPH_LI_ACCEPT)
    {
        if(move(V,&BestM)==false)
        {
            report_error("%s: best move evaluates to false\n",__FUNCTION__);
        }
        else
        {
            if(!(rules & VRPH_TABU))
                return true;
            else
            {
                // Check VRPH_TABU status of move - return true if its ok
                // or revert to old_sol if not and continue to search.
                if(V->check_tabu_status(&BestM, old_sol))
                {
                    delete [] old_sol;
                    return true; // The move was ok
                }
                // else we reverted back - search over
                delete [] old_sol;
                return false;

            }
        }
    }
    report_error("%s: should have already returned\n",__FUNCTION__);
    return false;
}

bool ThreePointMove::route_search(VRP *V, int r1, int r2, int rules)
{
    ///
    /// Searches for all ThreePointMoves involving two nodes from route r1
    /// and one node from route r2.
    ///

    VRPMove M;
    VRPMove BestM;
    int j,k,l,m;
    int accept_type;
    
    BestM.savings=VRP_INFINITY;

    if(r1==r2)
    {
        fprintf(stderr,"ThreePM::%d==%d???\n",r1,r2);
        report_error("%s: search called with r1==r2\n",__FUNCTION__);
    }

    if( (rules & VRPH_USE_NEIGHBOR_LIST) > 0)
        report_error("%s: route_search does not use neighbor_list\n",__FUNCTION__);

    accept_type=VRPH_FIRST_ACCEPT;//default

    if( (rules & VRPH_FIRST_ACCEPT) > 0)
        accept_type=VRPH_FIRST_ACCEPT;
    if( (rules & VRPH_BEST_ACCEPT) > 0)
        accept_type=VRPH_BEST_ACCEPT;
    if( (rules & VRPH_LI_ACCEPT) > 0)
        accept_type=VRPH_LI_ACCEPT;


    j= V->route[r1].start;
    k=VRPH_MAX(V->next_array[j],VRPH_DEPOT);

    // We will try to move j-k from route r1 and exchange with l from route r2
    while(k!=VRPH_DEPOT)
    {

        l= V->route[r2].start;
        m= VRPH_MAX(V->next_array[l],VRPH_DEPOT);

        while(m!=VRPH_DEPOT)
        {
            // Try to swap j-k with node l
            if(evaluate(V,l,j,k,rules,&M)==true)
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

            // Try to swap l-m with node j
            if(evaluate(V,j,l,m,rules,&M)==true)
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
            l=m;
            m=VRPH_MAX(V->next_array[m],0);
        }
        // Advance the route r1 node
        j=k;
        k=VRPH_MAX(V->next_array[j],VRPH_DEPOT);

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

bool ThreePointMove::evaluate(VRP *V, int b, int i, int j, int rules, VRPMove *M)
{

    ///
    /// Evaluates the move of exchanging node b with the position of edge i-j.
    /// subject to the given rules.  Details of move placed in the VRPMove M if
    /// it meets rules.
    ///

    V->num_evaluations[THREE_POINT_MOVE_INDEX]++;

    if(V->routed[b]==false|| V->routed[i]==false|| V->routed[j]==false)
        return false;

    M->evaluated_savings=false;


    if(b==i || b==j)//conflict
        return false;

    if(b==VRPH_DEPOT || i==VRPH_DEPOT || j==VRPH_DEPOT)
        return false;

    int a,c,h,k;
    int a_route, i_route;

    a_route=V->route_num[b];// this is b's former route
    i_route=V->route_num[i];

    // Don't do it if tiny routes
    if(V->route[a_route].num_customers<=1 || V->route[i_route].num_customers<=2)
        return false;    

    a=VRPH_MAX(V->pred_array[b],0);
    c=VRPH_MAX(V->next_array[b],0);
    h=VRPH_MAX(V->pred_array[i],0);
    k=VRPH_MAX(V->next_array[j],0);

    if(rules & VRPH_FIXED_EDGES)
    {
        // Make sure we aren't disturbing fixed edges
        if( V->fixed[a][b] || V->fixed[b][c] ) 
            return false;

        if( V->fixed[h][i] || V->fixed[j][k] ) 
            return false;

    }

    double savings=0;

    if(b==h)
    {
        // a-b/h-c/i-j-k --> a-c/i-j-b/h-k - postsert(b,j)
        savings = (V->d[a][i]+V->d[j][b]+V->d[b][k]) - 
            (V->d[a][b]+V->d[b][c]+V->d[j][k]);
    }
    
    if(b==k)
    {
        // h-i-j/a-b/k-c --> h-b/k-i-j/a-c - presert(b,i)
        savings = (V->d[h][b]+V->d[b][i]+V->d[j][c]) - 
            (V->d[h][i]+V->d[a][b]+V->d[b][c]);

    }

    // else no conflicts

    if(b!=h && b!=k)
    {
        // old:  a-b-c...h-i-j-k
        // new:  a-i-j-c...h-b-k

        // No conflicts
        // savings = new - old
        savings = (V->d[a][i]+V->d[j][c]+V->d[h][b]+V->d[b][k]) - 
            (V->d[a][b]+V->d[b][c]+V->d[h][i]+V->d[j][k]);

    }

    M->savings=savings;
    if(V->check_savings(M,rules)==false)
        return false;

    if(a_route==i_route)
    {
        // Same route - check length feasibility
        if(V->route[a_route].length+savings>V->max_route_length)
            return false;
        
        // Otherwise the move is feasible - record the move...

        M->num_arguments=3;
        M->move_arguments[0]=b;
        M->move_arguments[1]=i;
        M->move_arguments[2]=j;
        M->move_type=THREE_POINT_MOVE;
        M->new_total_route_length=V->total_route_length+savings;
        M->num_affected_routes=1;
        M->route_nums[0]=a_route;
        M->route_lens[0]=V->route[a_route].length+savings;
        M->route_custs[0]=V->route[a_route].num_customers;
        M->route_loads[0]=V->route[a_route].load;
        M->savings=savings;
        M->total_number_of_routes=V->total_number_of_routes;        
        // Now check the move
        if(V->check_move(M,rules)==true)
            return true;
        else
            return false;


    }

    // else diff. routes - can't have any overlaps in this case
    int new_a_load = V->route[a_route].load+V->nodes[i].demand + V->nodes[j].demand - V->nodes[b].demand;
    int new_i_load = V->route[i_route].load-V->nodes[i].demand - V->nodes[j].demand + V->nodes[b].demand;

    if(new_a_load>V->max_veh_capacity || new_i_load>V->max_veh_capacity)
        return false;

    double new_a_len = V->route[a_route].length + V->d[a][i]+V->d[i][j]+V->d[j][c]-V->d[a][b]-V->d[b][c];
    double new_i_len = V->route[i_route].length + V->d[h][b]+V->d[b][k]-V->d[h][i]-V->d[i][j]-V->d[j][k];

    if(new_a_len > V->max_route_length || new_i_len > V->max_route_length)
        return false;

    // else the move is feasible - record it.

    M->num_arguments=3;
    M->move_arguments[0]=b;
    M->move_arguments[1]=i;
    M->move_arguments[2]=j;
    M->move_type=THREE_POINT_MOVE;
    M->new_total_route_length=V->total_route_length+savings;
    M->num_affected_routes=2;
    M->route_nums[0]=a_route;
    M->route_nums[1]=i_route;
    M->route_lens[0]=new_a_len;
    M->route_lens[1]=new_i_len;
    M->route_custs[0]=V->route[a_route].num_customers+1;
    M->route_custs[1]=V->route[i_route].num_customers-1;
    M->route_loads[0]=new_a_load;
    M->route_loads[1]=new_i_load;
    M->savings=savings;
    M->total_number_of_routes=V->total_number_of_routes;        


    // Now check the move
    if(V->check_move(M,rules)==true)
        return true;
    else
        return false;
}

bool ThreePointMove::move(VRP *V, VRPMove *M)
{
    ///
    /// Makes the ThreePointMove specified by M.
    ///

    V->num_moves[THREE_POINT_MOVE_INDEX]++;

#if THREE_PM_VERIFY
    V->verify_routes("Before ThreePM\n");
#endif


    // Easiest way to do this is by just post/preserting the nodes.

    int a,b,c,h,i,j,k;

    b=M->move_arguments[0];
    i=M->move_arguments[1];
    j=M->move_arguments[2];

    a=VRPH_MAX(V->pred_array[b],0);
    c=VRPH_MAX(V->next_array[b],0);
    h=VRPH_MAX(V->pred_array[i],0);
    k=VRPH_MAX(V->next_array[j],0);

    // First artificially inflate the constraints
    double orig_max_len=V->max_route_length;
    int orig_veh_cap=V->max_veh_capacity;

    V->max_route_length=VRP_INFINITY;
    V->max_veh_capacity=VRP_INFINITY;

    Postsert postsert;
    Presert presert;


    if(b==h)
    {
        // a-b/h-c/i-j-k --> a-c/i-j-b/h-k - postsert(b,j)
        if(postsert.move(V,b,j)==false)
            report_error("%s: postsert %d,%d is false!\n",__FUNCTION__,b,j);

        V->max_route_length=orig_max_len;
        V->max_veh_capacity=orig_veh_cap;
        V->verify_routes("After ThreePM b==h\n");
    }
    
    if(b==k)
    {
        // h-i-j/a-b/k-c --> h-b/k-i-j/a-c - presert(b,i)
        if(presert.move(V,b,i)==false)
            report_error("%s: presert %d,%d is false\n",__FUNCTION__,b,i);

        V->max_route_length=orig_max_len;
        V->max_veh_capacity=orig_veh_cap;
        V->verify_routes("After ThreePM b==k\n");

    }

    if(b!=h && b!=k)
    {
        // First put b between h and i
        if(h!=VRPH_DEPOT)
        {
            if(postsert.move(V,b,h)==false)
                report_error("%s: postsert %d,%d is false!\n",__FUNCTION__,b,h);
        }
        else
        {
            if(presert.move(V,b,i)==false)
                report_error("%s: presert %d,%d is false!\n",__FUNCTION__,b,i);
        }

        // Now put i in between a and c
        if(a!=VRPH_DEPOT)
        {
            if(postsert.move(V,i,a)==false)
            {
                fprintf(stderr,"postsert(%d,%d)\n",i,a);
                report_error("%s: postsert %d,%d is false!\n",__FUNCTION__,i,a);
            }
        }
        else
        {
            if(presert.move(V,i,c)==false)
                report_error("%s: presert %d,%d is false!\n",__FUNCTION__,i,c);
        }

        // Now put j after i
        if(postsert.move(V,j,i)==false)
            report_error("%s: postsert %d,%d is false!\n",__FUNCTION__,j,i);

    }
    // Restore constraints
    V->max_route_length=orig_max_len;
    V->max_veh_capacity=orig_veh_cap;

    V->total_number_of_routes=M->total_number_of_routes;

    V->verify_routes("After ThreePM\n");

    V->capture_best_solution();
    return true;
}

