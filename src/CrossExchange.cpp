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


bool CrossExchange::route_search(class VRP *V, int r1, int r2, int rules)
{
    ///
    /// Attempts to find a cross exchange move between routes r1 and r2.
    /// Edges i1-i2 and k1-k2 in route r1, 
    /// and edges j1-j2 and l1-l2 in route r2.
    ///

    // Make sure we have two diff. routes!
    if(r1==r2)
        return false;

#if CROSS_EXCHANGE_DEBUG > 1
    printf("Evaluating CE move b/w routes %d and %d\n",r1,r2);
#endif


    int start_1, end_1, start_2, end_2;
    int i1, i2, k1, k2; // Route r1 nodes
    int j1, j2, l1, l2;    // Route r2 nodes

    VRPMove M, BestM;

    BestM.savings=VRP_INFINITY;

    int accept_type;

    // Default
    accept_type = VRPH_FIRST_ACCEPT;

    if( (rules & VRPH_LI_ACCEPT) == VRPH_LI_ACCEPT)
        accept_type = VRPH_LI_ACCEPT;


    if( (rules & VRPH_BEST_ACCEPT) == VRPH_BEST_ACCEPT)
        accept_type = VRPH_BEST_ACCEPT;

    // Use current_1a/b and next_1a/b for route r1 and current_2a/b and next_2a/b for route r2
    if(V->route[r1].num_customers < 4 || V->route[r2].num_customers < 4)
        // Not possible to find a CE move
        return false;

    int *old_sol=NULL;
    if(rules & VRPH_TABU)
    {
        // Remember the original solution 
        old_sol=new int[V->num_original_nodes+2];
        V->export_solution_buff(old_sol);
    }

    // Get route starts and ends

    start_1= V->route[r1].start;
    end_1= V->route[r1].end;

    int end_i=V->pred_array[end_1];


    start_2= V->route[r2].start;
    end_2= V->route[r2].end;

    int end_j=V->pred_array[end_2];

    i1 = start_1; // don't allow VRPH_DEPOT edges for now...
    i2 = V->next_array[i1];

    while(i2 != end_i)
    {
        // First edge in route 1 is i1-i2

        // Now find the second edge in route r1
        k1 = V->next_array[i2];


        if(k1==end_1)
            break;    // VRPH_DEPOT edge not allowed

        k2 = V->next_array[k1];
        while(k2 != end_1)
        {
            // Second edge in route 1 is k1-k2

            // Now find edges in the second route

            j1 = start_2; 
            j2 = V->next_array[j1];

            while(j2 != end_j)
            {

                // First edge in route 2 is j1-j2

                // Now find the second edge in route r2: l1-l2
                l1 = V->next_array[j2];

                if(l1==end_2)
                    break;

                l2 = V->next_array[l1];

                while(l2 != end_2)
                {
                    // Now evaluate the cross exchange move involving
                    // i1-i2, k1-k2, and
                    // j1-j2, l1-l2

                    if(this->evaluate(V,i1,i2,k1,k2,j1,j2,l1,l2,rules,&M) == true)
                    {

                        // We have found a valid move.
                        if(accept_type == VRPH_FIRST_ACCEPT || (accept_type==VRPH_LI_ACCEPT && M.savings<-VRPH_EPSILON) )
                        {
                            // Make the move

                            if(move(V, &M)==false)
                                report_error("%s: move error 1\n");
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


                        if(accept_type == VRPH_LI_ACCEPT || accept_type == VRPH_BEST_ACCEPT)
                        {
                            // Check for new best move
                            if(M.is_better(V, &BestM, rules))
                                BestM=M;

                        }

                    }

                    // Now advance the second edge in route r2

                    l1 = l2;
                    l2 = V->next_array[l1];

                }
                // end route r2 edge 2 loop

                // Now advance the first edge in route r2

                j1 = j2;
                j2= V->next_array[j1];

            }
            // end route r2 edge 1 loop

            // Now advance the second edge in route r1
            k1 = k2;
            k2 = V->next_array[k1];

        }
        // end route r1 edge 2 loop

        // Now advance the first edge in route r1

        i1 = i2;
        i2 = V->next_array[i1];

    }

#if CROSS_EXCHANGE_DEBUG > 1
    printf("END OF LOOP: %f\n",BestM.savings);
#endif
    if(BestM.savings == VRP_INFINITY || accept_type == VRPH_FIRST_ACCEPT)
        return false;

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
            report_error("%s: best move evaluates to false\n");
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

    report_error("%s: should have returned already...\n",__FUNCTION__);
    return false;
}



// EVALUATE
bool CrossExchange::evaluate(class VRP *V, int i1, int i2, int k1, int k2, int j1, int j2, int l1, int l2,
                             int rules, VRPMove *M)
{
    ///
    /// Evaluate the move of removing the edges i1-i2 and k1-k2 in one route and j1-j2 and l1-l2 in 
    /// another route and replacing these edges with i1-j2, j1-i2, k1-l2, and l1-k2
    ///

    V->num_evaluations[CROSS_EXCHANGE_INDEX]++;

#if CROSS_EXCHANGE_DEBUG > 1
    printf("Evaluting CE move: %d-%d, %d-%d, %d-%d, %d-%d\n",i1,i2,k1,k2,j1,j2,l1,l2);
#endif
    if(V->routed[i1]==false|| V->routed[i2]==false|| V->routed[k1]==false|| V->routed[k2]==false
        || V->routed[j1]==false|| V->routed[j2]==false|| V->routed[l1]==false|| V->routed[l2]==false)
        return false;


    M->evaluated_savings=false;
    double savings;

    // savings = new - old

    M->savings = savings = (V->d[i1][j2]+V->d[j1][i2]+V->d[k1][l2]+V->d[l1][k2]) - 
        (V->d[i1][i2]+V->d[j1][j2]+V->d[k1][k2]+V->d[l1][l2]);

    // Check the savings now
    if(V->check_savings(M,rules)==false)
        return false;


    // else the move has potential -- now check feasibility
    // Assume that edge i1-i2 is before k1-k2 in one route and that j1-j2 is before l1-l2 
    // in the second route - this is not checked!!


    // Move seems to make sense
    int i_route, j_route ;

    i_route= V->route_num[i2];
    j_route= V->route_num[j2];

    if(i_route==j_route)
    {
        fprintf(stderr,"i1=%d;i2=%d;k1=%d;k2=%d;j1=%d;j2=%d;l1=%d;l2=%d\n",i1,i2,k1,k2,j1,j2,l1,l2);
        fprintf(stderr,"i_route=%d; j_route=%d\n",i_route,j_route);
        V->show_route(i_route);
        report_error("%s: i_route==j_route!!\n",__FUNCTION__);
    }

    // Otherwise, the move appears to make sense

    double new_i_len, new_j_len;

    // savings = new_i_len + new_i_len - (old_i_len + old_j_len)

    //new_i_len=    route distance to (i1) + d(i1,j2)+ route distance between j2 and l1 
    //                + d(l1,k2) + route distance from (k2) to depot at end of route

    //new_j_len=    route distance to (j1) + d(j1,i2) + route distance between i2 and k1
    //                + d(k1,l2) + route distance from (l2) to depot at end of route


    VRPSegment S_0_i1, S_j2_l1, S_k2_0;
    V->get_segment_info(VRPH_DEPOT, i1, &S_0_i1);
    V->get_segment_info(j2, l1, &S_j2_l1);
    V->get_segment_info(k2, VRPH_DEPOT, &S_k2_0);

    new_i_len = S_0_i1.len + V->d[i1][j2] + S_j2_l1.len + V->d[l1][k2] + S_k2_0.len;

    if(new_i_len>V->max_route_length)
        return false;

    new_j_len =  savings + V->route[i_route].length + V->route[j_route].length - new_i_len;

    if(new_j_len>V->max_route_length)
        return false;

    int new_i_load, new_j_load;

    new_i_load = S_0_i1.load + S_j2_l1.load + S_k2_0.load;

    if(new_i_load>V->max_veh_capacity)
        return false;


    new_j_load = V->route[i_route].load + V->route[j_route].load - new_i_load;


    if(new_j_load > V->max_veh_capacity)
        return false;

    // The move is feasible - check the rules    

    M->savings=savings;

#if CROSS_EXCHANGE_DEBUG>1
    printf("CROSS_EXCHANGE::savings is %f\n", savings);
#endif

    M->num_affected_routes=2;
    M->route_nums[0] = i_route;
    M->route_nums[1] = j_route;
    M->route_custs[0]= S_0_i1.num_custs + S_j2_l1.num_custs +  S_k2_0.num_custs;

#if CROSS_EXCHANGE_DEBUG > 1
    printf("%d + %d + %d = %d\n",S_0_i1.num_custs, S_j2_l1.num_custs,  S_k2_0.num_custs, 
        M->route_custs[0]);
#endif
    M->route_custs[1]= V->route[i_route].num_customers + V->route[j_route].num_customers-
        M->route_custs[0];
    M->route_lens[0]=new_i_len;
    M->route_lens[1]=new_j_len;
    M->route_loads[0]=new_i_load;
    M->route_loads[1]=new_j_load;
    M->savings=savings;
    M->new_total_route_length= V->total_route_length+savings;
    M->num_arguments=9;
    M->move_arguments[0]=i1; M->move_arguments[1]=i2; M->move_arguments[2]=k1; M->move_arguments[3]=k2;
    M->move_arguments[4]=j1; M->move_arguments[5]=j2; M->move_arguments[6]=l1; M->move_arguments[7]=l2;
    M->move_arguments[8]=rules;
    M->total_number_of_routes = V->total_number_of_routes;

#if CROSS_EXCHANGE_DEBUG > 1
    printf("CROSS_EXCHANGE::\nlengths: %f, %f\nloads: %d, %d\ncusts: %d, %d\n",M->route_lens[0],
        M->route_lens[1], M->route_loads[0], M->route_loads[1], M->route_custs[0], M->route_custs[1]);
#endif


    if(V->check_move(M,rules)==true)
        return true;
    else
        return false;

}


bool CrossExchange::move(class VRP *V, VRPMove *M)
{
    ///
    /// Make the move of removing the edges i1-i2 and k1-k2 in one route and j1-j2 and l1-l2 in 
    /// another route and replacing these edges with i1-j2, j1-i2, k1-l2, and l1-k2.  See
    ///

    int i1, i2, k1, k2, j1, j2, l1, l2;

    i1=M->move_arguments[0];i2=M->move_arguments[1];k1=M->move_arguments[2];k2=M->move_arguments[3];
    j1=M->move_arguments[4];j2=M->move_arguments[5];l1=M->move_arguments[6];l2=M->move_arguments[7];


#if CROSS_EXCHANGE_DEBUG
    printf("Routes before cross (%d-%d, %d-%d) (%d-%d, %d-%d)\n",i1,i2,k1,k2,j1,j2,l1,l2);
    V->show_route(V->route_num[i1]);
    V->show_route(V->route_num[j1]);
#endif

    V->next_array[j1]=i2;
    V->pred_array[i2]=j1;

    V->next_array[k1]=l2;
    V->pred_array[l2]=k1;

    V->next_array[i1]=j2;
    V->pred_array[j2]=i1;

    V->next_array[l1]=k2;
    V->pred_array[k2]=l1;



    // Now update the route_num and the start and ends
    int current_node;

    current_node= V->route[M->route_nums[0]].start;

    while(current_node!=VRPH_DEPOT)
    {
        V->route[M->route_nums[0]].end=current_node;        
        V->route_num[current_node] = M->route_nums[0];
        current_node= VRPH_MAX(VRPH_DEPOT,V->next_array[current_node]);
    }

    current_node= V->route[M->route_nums[1]].start;

    while(current_node!=VRPH_DEPOT)
    {
        V->route[M->route_nums[1]].end=current_node;        
        V->route_num[current_node] = M->route_nums[1];
        current_node= VRPH_MAX(VRPH_DEPOT, V->next_array[current_node]);
    }

    V->update(M);


#if CROSS_EXCHANGE_DEBUG
    printf("Routes after CrossExchange move (%d-%d, %d-%d), (%d-%d, %d-%d)\n",
        i1,i2,k1,k2, j1,j2,l1,l2);
    V->show_route(M->route_nums[0]);
    V->show_route(M->route_nums[1]);
#endif

#if CROSS_EXCHANGE_VERIFY
    V->verify_routes("After CrossExchange move\n");
#endif

    V->num_moves[CROSS_EXCHANGE_INDEX]++;

    V->capture_best_solution();

    return true;

}

