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
bool ThreeOpt::route_search(class VRP *V, int r, int rules)
{
    ///
    /// Searches for a Three-Opt move in route r.
    /// If a satisfactory move is found,
    /// then the move is made.  If no move is found, false is returned.
    ///

    VRPMove M, BestM;
    int ii,a,b,c,d,e,f;

    BestM.savings=VRP_INFINITY;

    int e11, e12, e21, e22, e31, e32;
    // The edges involved in the move: e1, e2, e3
    

    int accept_type=VRPH_FIRST_ACCEPT;

    // Initialize to null edges
    e11=e12=e21=e22=e31=e32=0;    

    if( (rules & VRPH_USE_NEIGHBOR_LIST) > 0)
        report_error("%s: neighbor_list not used in 3OPT search--searches all nodes in route\n",__FUNCTION__);

    accept_type = VRPH_FIRST_ACCEPT;    //default

    if( (rules & VRPH_LI_ACCEPT) == VRPH_LI_ACCEPT)
        accept_type=VRPH_LI_ACCEPT;

    if( (rules & VRPH_BEST_ACCEPT) == VRPH_BEST_ACCEPT)
        accept_type=VRPH_BEST_ACCEPT;

    ii=0;

    b= V->route[r].start;
    a=VRPH_MAX(V->pred_array[b],0);
    // edge 1 is a-b
    c=VRPH_MAX(V->next_array[b],0);    
    if(c==VRPH_DEPOT)
        return false;
    d=VRPH_MAX(V->next_array[c],0);
    if(d==VRPH_DEPOT)
        return false;
    e=VRPH_MAX(V->next_array[d],0);
    if(e==VRPH_DEPOT)
        return false;
    f=VRPH_MAX(V->next_array[e],0);
    if(f==VRPH_DEPOT)
        return false;

    int a_end, c_end, e_end;
    // Set the a_end to 3 hops back from the route's end since
    // we will have searched all possible moves by this point.
    // This is ugly...
    a_end = VRPH_MAX(VRPH_DEPOT,V->pred_array[VRPH_MAX(VRPH_DEPOT,V->pred_array[VRPH_MAX(VRPH_DEPOT, V->pred_array[V->route[r].end])])]);
    c_end = VRPH_MAX(VRPH_DEPOT, V->pred_array[V->route[r].end]);
    e_end = V->route[r].end;

    int *old_sol=NULL;
    if(rules & VRPH_TABU)
    {
        // Remember the original solution 
        old_sol=new int[V->num_original_nodes+2];
        V->export_solution_buff(old_sol);
    }



    // Evaluate the first move and then enter the loop
    // We begin with the edges a-b, c-d, and e-f
    if(evaluate(V,a,b,c,d,e,f,rules,&M)==true)
    {
        if(accept_type==VRPH_FIRST_ACCEPT || ((accept_type==VRPH_LI_ACCEPT)&&M.savings<-VRPH_EPSILON))
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

        if(accept_type==VRPH_BEST_ACCEPT || accept_type==VRPH_LI_ACCEPT)
        {
            if(M.is_better(V, &BestM, rules))//if(M.savings<BestM.savings)
                BestM=M;
        }
    }

    a=b;


    while(a!=a_end)
    {
        b=VRPH_MAX(V->next_array[a],0);
        c=VRPH_MAX(V->next_array[b],0);
        while(c!=c_end)
        {
            d=VRPH_MAX(V->next_array[c],0);
            e=VRPH_MAX(V->next_array[d],0);
            while(e!=e_end)
            {    
                f=VRPH_MAX(V->next_array[e],0);

                // Evaluate the move!
                if(evaluate(V,a,b,c,d,e,f,rules,&M)==true)
                {
                    if(accept_type==VRPH_FIRST_ACCEPT || ((accept_type==VRPH_LI_ACCEPT)&&M.savings<-VRPH_EPSILON))
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

                    if(accept_type==VRPH_BEST_ACCEPT || accept_type==VRPH_LI_ACCEPT)
                    {
                        if(M.is_better(V, &BestM, rules))
                            BestM=M;

                    }
                }
                e=f;
            }
            c=d;
        }
        a=b;
    }


    if(accept_type==VRPH_FIRST_ACCEPT || BestM.savings==VRP_INFINITY)
        return false;

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

    if(move(V,&BestM)==false)//best_e11,best_e12,best_e21,best_e22, best_e31, best_e32,rules)==false)
        report_error("%s: first move is false!\n",__FUNCTION__);
    else
        return true;


    return false;
}

bool ThreeOpt::evaluate(class VRP *V, int a, int b, int c, int d, int e, int f,
                        int rules, VRPMove *M)
{
    ///
    /// Evaluates the Three-Opt move involving the directed edges
    /// ab, cd, and ef, subject to the given rules.  The function
    /// finds the most cost effective of the possible moves and
    /// stores the relevant data in the VRPMove M and returns true.
    /// If no satisfactory move is found, the function returns false.
    ///    

    V->num_evaluations[THREE_OPT_INDEX]++;
    M->evaluated_savings=false;

    if(V->routed[a]==false || V->routed[b]==false || V->routed[c]==false
        || V->routed[d]==false|| V->routed[e]==false|| V->routed[f]==false)
        return false;

    if(rules & VRPH_FIXED_EDGES)
    {
        // Make sure we aren't disturbing fixed edges
        if( V->fixed[a][b] || V->fixed[c][d] || V->fixed[e][f]) 
            return false;

    }

    int a_route;

    if(a!=VRPH_DEPOT)
        a_route = V->route_num[a];
    else
        a_route = V->route_num[b];

    M->eval_arguments[0]=a;M->eval_arguments[1]=b;M->eval_arguments[2]=c;
    M->eval_arguments[3]=d;M->eval_arguments[4]=e;M->eval_arguments[5]=f;

    // IMPORTANT!! Assume that edges are in order and have no conflicts

    double s1, s2, s3, s4, s5, s6, s7, old;


    // savings = new-old
    double minval=VRP_INFINITY;
    int type=0;
    old= V->d[a][b]+V->d[c][d]+V->d[e][f];
    s1=(V->d[a][b]+V->d[c][e]+V->d[d][f])-old;    // 2-opt move
    minval=s1; type=1;
    s2=(V->d[a][c]+V->d[b][d]+V->d[e][f])-old;    // 2-opt move
    if(s2<minval){ minval=s2; type=2;}
    s3=(V->d[a][c]+V->d[b][e]+V->d[d][f])-old;    // 3-opt move
    if(s3<minval){ minval=s3; type=3;}
    s4=(V->d[a][d]+V->d[b][e]+V->d[c][f])-old;    // 3-opt move
    if(s4<minval){ minval=s4; type=4;}
    s5=(V->d[a][d]+V->d[c][e]+V->d[b][f])-old;    // 3-opt move
    if(s5<minval){ minval=s5; type=5;}
    s6=(V->d[a][e]+V->d[b][d]+V->d[c][f])-old;    // 3-opt move
    if(s6<minval){ minval=s6; type=6;}
    s7=(V->d[a][e]+V->d[c][d]+V->d[b][f])-old;    // 2-opt move
    if(s7<minval){ minval=s7; type=7;}

    // No need to check load here since it's INTRA only

    // Now check route feasibility of the best savings
    if(minval + V->route[a_route].length > V->max_route_length )
        // The move is infeasible
        return false;

    // else the move is feasible - store the results in the move struct

    M->savings=minval;
    M->num_affected_routes=1;
    M->route_lens[0]=minval+V->route[a_route].length;
    M->route_nums[0]=a_route;
    M->route_custs[0]=V->route[a_route].num_customers;
    M->route_loads[0]=V->route[a_route].load;
    M->total_number_of_routes=V->total_number_of_routes;
    M->new_total_route_length=V->total_route_length+M->savings;
    M->eval_arguments[0]=a;M->eval_arguments[1]=b;M->eval_arguments[2]=c;
    M->eval_arguments[3]=d;M->eval_arguments[4]=e;M->eval_arguments[5]=f;
    M->move_type=type;

    // Now check the move
    if(V->check_move(M,rules)==true)
    {
        return true;
    }
    else
        return false;

}

bool ThreeOpt::move(class VRP *V, VRPMove *M)// int a, int b, int c, int d, int e, int f, int rules)
{
    ///
    /// This function makes the actual solution modification involving the Three-Opt
    /// move with the edges V->d[a][b], V->d[c][d], and V->d[e][f].  
    ///

    int a,b,c,d,e,f;
    a=M->eval_arguments[0];b=M->eval_arguments[1];c=M->eval_arguments[2];
    d=M->eval_arguments[3];e=M->eval_arguments[4];f=M->eval_arguments[5];

    // Two cases:  1) all in one route;  2) 3 different routes (not considered!)

    int a_route,c_route,e_route;

    if(a!=VRPH_DEPOT)
        a_route= V->route_num[a];
    else
        a_route= V->route_num[b];

    if(c!=VRPH_DEPOT)
        c_route= V->route_num[c];
    else
        c_route= V->route_num[d];

    if(e!=VRPH_DEPOT) 
        e_route= V->route_num[e];
    else
        e_route= V->route_num[f];



    // INTRAROUTE CASE:
    if(a_route==c_route && c_route==e_route)
    {
        int type = M->move_type;


        //// Now find the best of these - no need to check load here since it's INTRA only

        //
        double oldlen, oldobj;
        double temp_maxlen;
        int temp_vehcap, a_route;

        // Remember the actual maximums as we may need to artificially inflate them.
        temp_maxlen= V->max_route_length;
        temp_vehcap= V->max_veh_capacity;

        Flip flip;

        a_route= V->route_num[b];// b is not VRPH_DEPOT

        oldlen= V->route[a_route].length;
        oldobj= V->total_route_length;

        if(type==1)
        {

            if(f==VRPH_DEPOT)
            {
                V->postsert_dummy(e);
                flip.move(V,c,V->dummy_index);
                V->remove_dummy();
            }
            else
                flip.move(V,c,f);


            V->num_moves[THREE_OPT_INDEX]++;
            V->capture_best_solution();
            return true;

        }

        if(type==2)
        {


            if(a==VRPH_DEPOT)
            {

                V->presert_dummy(b);
                flip.move(V,V->dummy_index,d);
                V->remove_dummy();
            }
            else
                flip.move(V,a,d);

            V->num_moves[THREE_OPT_INDEX]++;
            V->capture_best_solution();
            return true;
        }

        if(type==3)
        {
            V->max_route_length=VRP_INFINITY;    
            V->max_veh_capacity=VRP_INFINITY;

            if(a==VRPH_DEPOT)
            {

                V->presert_dummy(b);
                flip.move(V,V->dummy_index,d);
                V->remove_dummy();
            }
            else
                flip.move(V,a,d);

            if(f==VRPH_DEPOT)
            {

                V->postsert_dummy(e);
                flip.move(V,b,V->dummy_index);
                V->remove_dummy();

            }
            else
                flip.move(V,b,f);

            V->max_route_length=temp_maxlen;
            V->max_veh_capacity=temp_vehcap;

            V->num_moves[THREE_OPT_INDEX]++;
            V->capture_best_solution();
            return true;
        }


        if(type==4)
        {
            a_route= V->route_num[b]; // b not VRPH_DEPOT!

            if(a!=VRPH_DEPOT && f!=VRPH_DEPOT)
            {
                V->next_array[a]=d;
                V->pred_array[d]=a;

                V->next_array[e]=b;
                V->pred_array[b]=e;

                V->next_array[c]=f;
                V->pred_array[f]=c;
            }

            if(a==VRPH_DEPOT && f!=VRPH_DEPOT)
            {
                int prev_end=VRPH_ABS(V->pred_array[b]);
                V->next_array[prev_end] = -d;
                V->pred_array[d] = -prev_end;

                V->next_array[e]=b;
                V->pred_array[b]=e;

                V->next_array[c]=f;
                V->pred_array[f]=c;

                V->route[a_route].start=d;///!!!!
            }

            if(a!=VRPH_DEPOT && f==VRPH_DEPOT)
            {
                int prev_start=VRPH_ABS(V->next_array[e]);
                V->pred_array[prev_start] = -c;
                V->next_array[c] = -prev_start;

                V->next_array[e]=b;
                V->pred_array[b]=e;

                V->next_array[a]=d;
                V->pred_array[d]=a;
            }

            if(a==VRPH_DEPOT && f==VRPH_DEPOT)
            {
                int prev_end=VRPH_ABS(V->pred_array[b]);
                int prev_start=VRPH_ABS(V->next_array[e]);


                V->next_array[prev_end]=-b;
                V->pred_array[b]=-prev_end;

                V->next_array[e]=b;
                V->pred_array[b]=e;

                V->next_array[c]=-prev_start;
                V->pred_array[prev_start]=-c;

                V->route[a_route].start=d;///!!!!


            }

            //Now manually adjust the route_len and obj. value


            V->route[a_route].length=oldlen+M->savings;//s4;
            V->total_route_length=oldobj+M->savings;//s4;

            V->num_moves[THREE_OPT_INDEX]++;
            V->capture_best_solution();
            return true;


        }

        if(type==5)
        {
            V->max_route_length=VRP_INFINITY;    
            V->max_veh_capacity=VRP_INFINITY;

            int prev_end=-1;

            // Remember:  flip changes the obj. value!!!!
            if(a==VRPH_DEPOT)
            {
                prev_end=VRPH_ABS(V->pred_array[b]);
                V->presert_dummy(b);
                flip.move(V,V->dummy_index,d);
                V->remove_dummy();

            }
            else
                flip.move(V,a,d);

            if(a==VRPH_DEPOT)
            {

                V->next_array[prev_end]=-d;
                V->pred_array[d]=-prev_end;
                V->route[a_route].start=d; //!!!
            }
            else
            {
                V->next_array[a]=d;
                V->pred_array[d]=a;
            }

            if(f==VRPH_DEPOT)
            {
                int prev_start=VRPH_ABS(V->next_array[e]);
                V->next_array[b]=-prev_start;
                V->pred_array[prev_start]=-b;

            }
            else
            {
                V->next_array[b]=f;
                V->pred_array[f]=b;
            }

            V->next_array[e]=c;
            V->pred_array[c]=e;



            //Now manually adjust the route_len and obj. value

            V->max_route_length=temp_maxlen;
            V->max_veh_capacity=temp_vehcap;


            a_route= V->route_num[b];

            V->route[a_route].length=oldlen+M->savings;//s5;
            V->total_route_length=oldobj+M->savings;//s5;

            V->num_moves[THREE_OPT_INDEX]++;
            V->capture_best_solution();
            return true;
        }

        if(type==6)
        {
            V->max_route_length=VRP_INFINITY;    
            V->max_veh_capacity=VRP_INFINITY;

            int prev_end=VRPH_ABS(V->pred_array[b]);

            if(f==VRPH_DEPOT)
            {
                V->postsert_dummy(e);
                flip.move(V,c,V->dummy_index);
                V->remove_dummy();
            }
            else
                flip.move(V,c,f);


            if(a==VRPH_DEPOT)
            {

                V->next_array[prev_end]=-e;
                V->pred_array[e]=-prev_end;
                V->route[V->route_num[b]].start=e;
            }
            else
            {
                V->next_array[a]=e;
                V->pred_array[e]=a;
            }

            V->next_array[d]=b;
            V->pred_array[b]=d;

            if(f==VRPH_DEPOT)
            {
                int prev_start=VRPH_ABS(V->next_array[e]);
                V->next_array[c]=-prev_start;
                V->pred_array[prev_start]=-c;

            }
            else
            {
                V->next_array[c]=f;
                V->pred_array[f]=c;
            }

            //Now manually adjust the route_len and obj. value

            a_route= V->route_num[b];
            V->route[a_route].length=oldlen+M->savings;//s6;
            V->total_route_length=oldobj+M->savings;//s6;

            V->max_route_length=temp_maxlen;
            V->max_veh_capacity=temp_vehcap;

            V->num_moves[THREE_OPT_INDEX]++;
            V->capture_best_solution();
            return true;

        }

        if(type==7)
        {

            //ae,dc,bf
            if(a==VRPH_DEPOT && f==VRPH_DEPOT)
            {
                // This is just a route reversal - should have no savings!!
                report_error("%s: route reversal??\n",__FUNCTION__);
            }

            if(a==VRPH_DEPOT)
            {
                V->presert_dummy(b);
                flip.move(V,V->dummy_index,f);
                V->remove_dummy();
            }
            else
            {
                if(f==VRPH_DEPOT)
                {
                    V->postsert_dummy(e);
                    flip.move(V,a,V->dummy_index);
                    V->remove_dummy();
                }
                else
                    flip.move(V,a,f);

            }

            V->num_moves[THREE_OPT_INDEX]++;
            V->capture_best_solution();
            return true;


        }

    }


    return false;

}


