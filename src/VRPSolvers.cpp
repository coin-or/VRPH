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

double VRP::RTR_solve(int heuristics, int intensity, int max_stuck, int max_perturbs,
                      double dev, int nlist_size, int perturb_type, int accept_type, bool verbose)
{
    ///
    /// Uses the given parameters to generate a 
    /// VRP solution via record-to-record travel.
    /// Assumes that data has already been imported into V and that we have
    /// some existing solution.
    /// Returns the objective function value of the best solution found
    ///

    // Make sure accept_type is either VRPH_BEST_ACCEPT or VRPH_FIRST_ACCEPT - matters only
    // for the downhill phase as we use VRPH_LI_ACCEPT in the diversification phase

    if(accept_type!=VRPH_BEST_ACCEPT && accept_type!=VRPH_FIRST_ACCEPT)
        report_error("%s: accept_type must be VRPH_BEST_ACCEPT or VRPH_FIRST_ACCEPT\n");

    int ctr, n, j,  i,  R, random, fixed, neighbor_list, objective, tabu;

    random=fixed=neighbor_list=0;

    if(heuristics & VRPH_RANDOMIZED)
        random=VRPH_RANDOMIZED;

    if(heuristics & VRPH_FIXED_EDGES)
        fixed=VRPH_FIXED_EDGES;

    if(heuristics & VRPH_USE_NEIGHBOR_LIST)
        neighbor_list=VRPH_USE_NEIGHBOR_LIST;

    objective=VRPH_SAVINGS_ONLY;
    // default strategy

    if(heuristics & VRPH_MINIMIZE_NUM_ROUTES)
        objective=VRPH_MINIMIZE_NUM_ROUTES;


    if(heuristics & VRPH_TABU)
    {
        tabu=VRPH_TABU; // We will use a primitive Tabu Search in the uphill phase
        // Clear the tabu list
        this->tabu_list->empty();
    }
    else
        tabu=0;

    n=num_nodes;

    // Define the heuristics we will use

    OnePointMove OPM;
    TwoPointMove TPM;
    TwoOpt         TO;
    OrOpt         OR;
    ThreeOpt     ThreeO;
    CrossExchange    CE;
    ThreePointMove ThreePM;

    double start_val;
    int *perm;
    perm=new int[this->num_nodes];


    j=VRPH_ABS(this->next_array[VRPH_DEPOT]);
    for(i=0;i<this->num_nodes;i++)
    {
        perm[i]=j;
        if(!routed[j])
            report_error("%s: Unrouted node in solution!!\n");

        j=VRPH_ABS(this->next_array[j]);
    }
    if(j!=VRPH_DEPOT)
        report_error("%s: VRPH_DEPOT is not last node in solution!!\n");


    int rules;

    // Set the neighbor list size used in the improvement search
    neighbor_list_size=VRPH_MIN(nlist_size, this->num_nodes);

    // Set the deviation
    deviation=dev;

    int num_perturbs=0;

    record=this->total_route_length;
    this->best_total_route_length=this->total_route_length;
    this->export_solution_buff(this->current_sol_buff);
    this->export_solution_buff(this->best_sol_buff);

    normalize_route_numbers();

    ctr=0;


uphill:
    // Start an uphill phase using the following "rules":
    double beginning_best=this->best_total_route_length;
    rules=VRPH_LI_ACCEPT+VRPH_RECORD_TO_RECORD+objective+random+fixed+neighbor_list+tabu;

    if(verbose)
        printf("Uphill starting at %5.2f\n",this->total_route_length);
    
    for(int k=1;k<intensity;k++)
    {
        start_val=total_route_length;

        if(heuristics & ONE_POINT_MOVE)
        {
            if(random)
                random_permutation(perm, this->num_nodes);

            for(i=1;i<=n;i++)
            {
#if FIXED_DEBUG
                if(fixed && !check_fixed_edges("Before 1PM\n"))
                    fprintf(stderr,"Error before OPM search(%d)\n",perm[i-1]);
#endif
                OPM.search(this,perm[i-1],rules);

#if FIXED_DEBUG
                if(fixed && !check_fixed_edges("After 1PM\n"))
                {
                    fprintf(stderr,"Error after OPM search(%d)\n",perm[i-1]);
                    this->show_route(this->route_num[perm[i-1]]);
                }
#endif
            }
        }


        if(heuristics & TWO_POINT_MOVE)
        {
            if(random)
                random_permutation(perm, this->num_nodes);

            for(i=1;i<=n;i++)    
                TPM.search(this,perm[i-1],rules + VRPH_INTER_ROUTE_ONLY);

            //check_fixed_edges("After 2PM\n");

        }


        if(heuristics & THREE_POINT_MOVE)
        {
            if(random)
                random_permutation(perm, this->num_nodes);

            for(i=1;i<=n;i++)    
                ThreePM.search(this,perm[i-1],rules + VRPH_INTER_ROUTE_ONLY);

            //check_fixed_edges("After 3PM\n");

        }



        if(heuristics & TWO_OPT)
        {
            if(random)
                random_permutation(perm, this->num_nodes);

            for(i=1;i<=n;i++)    
                TO.search(this,perm[i-1],rules);

            //check_fixed_edges("After TO\n");


        }        


        if(heuristics & OR_OPT)
        {
            if(random)
                random_permutation(perm, this->num_nodes);

            for(i=1;i<=n;i++)    
                OR.search(this,perm[i-1],4,rules);

            for(i=1;i<=n;i++)    
                OR.search(this,perm[i-1],3,rules);

            for(i=1;i<=n;i++)    
                OR.search(this,perm[i-1],2,rules);

            //check_fixed_edges("After OR\n");

        }

        if(heuristics & THREE_OPT)
        {
            normalize_route_numbers();
            R=total_number_of_routes;

            for(i=1; i<=R; i++)    
                ThreeO.route_search(this,i,rules-neighbor_list);

            //check_fixed_edges("After 3O\n");

        }

        if(heuristics & CROSS_EXCHANGE)
        {
            normalize_route_numbers();
            this->find_neighboring_routes();
            R=total_number_of_routes;

            for(i=1; i<=R-1; i++)    
            {
                for(j=0;j<1;j++)
                    CE.route_search(this,i, route[i].neighboring_routes[j],rules-neighbor_list); 
            }

            //check_fixed_edges("After CE\n");
        }
    }

    if(total_route_length<record)
        record = total_route_length;

    if(verbose)
    {
        printf("Uphill complete\t(%d,%5.2f,%5.2f)\n",count_num_routes(),total_route_length, record);
        printf("# of recorded routes: %d[%d]\n",total_number_of_routes,count_num_routes());

    }

    if(this->best_total_route_length<beginning_best-VRPH_EPSILON)
    {
        if(verbose)
            printf("New best found in uphill!\n");
        // We found a new best solution during the uphill phase that might
        // now be "forgotten"!! I have seen this happen where it is never recovered
        // again, so we just import it and start the downhill phase with this solution...
        //this->import_solution_buff(this->best_sol_buff);

    }

downhill:

    // Now enter a downhill phase
    double orig_val=total_route_length;
    if(verbose)
        printf("Downhill starting at %f (best=%f)\n",orig_val,this->best_total_route_length);


    if((heuristics & ONE_POINT_MOVE)|| (heuristics & KITCHEN_SINK) )
    {
        rules=VRPH_DOWNHILL+objective+random+fixed+neighbor_list+accept_type;
        for(;;)
        {
            // One Point Move
            start_val=total_route_length;

            if(random)
                random_permutation(perm, this->num_nodes);

            for(i=1;i<=n;i++)
                OPM.search(this,perm[i-1],rules );


            if(VRPH_ABS(total_route_length-start_val)<VRPH_EPSILON)
                break; 

        }

    }



    if((heuristics & TWO_POINT_MOVE) || (heuristics & KITCHEN_SINK) )
    {
        rules=VRPH_DOWNHILL+VRPH_INTER_ROUTE_ONLY+objective+random+fixed+neighbor_list+accept_type;
        for(;;)
        {
            // Two Point Move
            start_val=total_route_length;

            if(random)
                random_permutation(perm, this->num_nodes);

            for(i=1;i<=n;i++)    
                TPM.search(this,perm[i-1],rules);

            if(VRPH_ABS(total_route_length-start_val)<VRPH_EPSILON)
                break; 

        }

    }



    if((heuristics & TWO_OPT)|| (heuristics & KITCHEN_SINK) )
    {
        // Do inter-route first a la Li
        rules=VRPH_DOWNHILL+VRPH_INTER_ROUTE_ONLY+objective+random+fixed+neighbor_list+accept_type;
        for(;;)
        {

            start_val=total_route_length;

            if(random)
                random_permutation(perm, this->num_nodes);

            for(i=1;i<=n;i++)    
                TO.search(this,perm[i-1],rules);

            if(VRPH_ABS(total_route_length-start_val)<VRPH_EPSILON)
                break; 
        }

        // Now do both intra and inter
        rules=VRPH_DOWNHILL+objective+random+fixed+neighbor_list+accept_type;

        for(;;)
        {

            start_val=total_route_length;

            if(random)
                random_permutation(perm, this->num_nodes);

            for(i=1;i<=n;i++)    
                TO.search(this,perm[i-1],rules);

            if(VRPH_ABS(total_route_length-start_val)<VRPH_EPSILON)
                break; 
        }
    }

    if((heuristics & THREE_POINT_MOVE) || (heuristics & KITCHEN_SINK) )
    {
        rules=VRPH_DOWNHILL+VRPH_INTER_ROUTE_ONLY+objective+random+fixed+accept_type+neighbor_list;
        for(;;)
        {
            // Three Point Move
            start_val=total_route_length;

            if(random)
                random_permutation(perm, this->num_nodes);

            for(i=1;i<=n;i++)    
                ThreePM.search(this,perm[i-1],rules);

            if(VRPH_ABS(total_route_length-start_val)<VRPH_EPSILON)
                break; 

        }
    }


    if((heuristics & OR_OPT) || (heuristics & KITCHEN_SINK))
    {

        rules=VRPH_DOWNHILL+ objective +random +fixed + accept_type + neighbor_list;

        for(;;)
        {
            // OrOpt
            start_val=total_route_length;
            if(random)
                random_permutation(perm, this->num_nodes);

            for(i=1;i<=n;i++)    
                OR.search(this,perm[i-1],4,rules);
            for(i=1;i<=n;i++)
                OR.search(this,perm[i-1],3,rules);
            for(i=1;i<=n;i++)
                OR.search(this,perm[i-1],2,rules);


            if(VRPH_ABS(total_route_length-start_val)<VRPH_EPSILON)
                break; 
        }
    }

    if((heuristics & THREE_OPT) || (heuristics & KITCHEN_SINK) )
    {
        normalize_route_numbers();
        R= total_number_of_routes;
        rules=VRPH_DOWNHILL+objective+VRPH_INTRA_ROUTE_ONLY+ random +fixed + accept_type;
        for(;;)
        {
            // 3OPT
            start_val=total_route_length;

            for(i=1;i<=R;i++)    
                ThreeO.route_search(this,i,rules);

            if(VRPH_ABS(total_route_length-start_val)<VRPH_EPSILON)
                break; 
        }
    }


    if( (heuristics & CROSS_EXCHANGE) )
    {
        normalize_route_numbers();
        this->find_neighboring_routes();
        R=total_number_of_routes;

        rules=VRPH_DOWNHILL+objective+VRPH_INTRA_ROUTE_ONLY+ random +fixed + accept_type;

        for(i=1; i<=R-1; i++)    
        {
            for(j=0;j<=1;j++)
                CE.route_search(this,i, route[i].neighboring_routes[j], rules); 
        }
    }


    // Repeat the downhill phase until we find no more improvements
    if(total_route_length<orig_val-VRPH_EPSILON)
        goto downhill;
    if(verbose)
        printf("Downhill complete: %5.2f[downhill started at %f] (%5.2f)\n",total_route_length,orig_val,
        this->best_total_route_length);


    if(total_route_length < record-VRPH_EPSILON)    
    {
        // New record - reset ctr
        ctr=1;
        record=total_route_length;
    }
    else
        ctr++;

    if(ctr<max_stuck)
        goto uphill;

    if(ctr==max_stuck)
    {
        if(num_perturbs<max_perturbs)
        {
            if(verbose)
                printf("perturbing\n");
            if(perturb_type==VRPH_LI_PERTURB)
                perturb();
            else
                osman_perturb(VRPH_MAX(20,num_nodes/10),.5+lcgrand(20));

            // Reset record
            this->record=this->total_route_length;
            if(tabu)
                this->tabu_list->empty();

            ctr=1;
            num_perturbs++;
            goto uphill;
        }
    }


    if(verbose)
    {
        if(has_service_times==false)
            printf("BEST OBJ:  %f\n",best_total_route_length);
        else
            printf("BEST OBJ:  %f\n",best_total_route_length-total_service_time);
    }

    delete [] perm; 

    // Import the best solution found
    this->import_solution_buff(best_sol_buff);

    if(has_service_times==false)
        return best_total_route_length;
    else
        return best_total_route_length-total_service_time;


}

double VRP::SA_solve(int heuristics, double start_temp, double cool_ratio,
                     int iters_per_loop, int num_loops, int nlist_size, bool verbose)    
{
    ///
    /// Uses the given parameters to generate a VRP solution using Simulated Annealing.
    /// Assumes that data has already been imported into V and that we have
    /// some existing solution.  Returns the total route length of the best solution found.
    ///

    this->temperature = start_temp;
    this->cooling_ratio = cool_ratio;

    int ctr, n, j,  i,  R, rules, random, fixed, neighbor_list, objective;

    if(heuristics & VRPH_RANDOMIZED)
        random=VRPH_RANDOMIZED;
    else
        random=0;

    if(heuristics & VRPH_FIXED_EDGES)
        fixed=VRPH_FIXED_EDGES;
    else
        fixed=0;

    if(heuristics & VRPH_USE_NEIGHBOR_LIST)
        neighbor_list=VRPH_USE_NEIGHBOR_LIST;
    else
        neighbor_list=0;

    objective=VRPH_SAVINGS_ONLY;
    // default strategy

    if(heuristics & VRPH_MINIMIZE_NUM_ROUTES)
        objective=VRPH_MINIMIZE_NUM_ROUTES;
    
    n=num_nodes;

    // The perm[] array will contain all the nodes in the current solution
    int *perm;
    perm=new int[this->num_nodes];
    j=VRPH_ABS(this->next_array[VRPH_DEPOT]);
    for(i=0;i<this->num_nodes;i++)
    {
        perm[i]=j;
        if(!routed[j])
            report_error("%s: Unrouted node in solution!!\n");

        j=VRPH_ABS(this->next_array[j]);
    }
    if(j!=VRPH_DEPOT)
        report_error("%s: VRPH_DEPOT is not last node in solution!!\n");

    // Define the heuristics we may use

    OnePointMove OPM;
    TwoPointMove TPM;
    TwoOpt         TO;
    OrOpt         OR;
    ThreeOpt     ThreeO;
    CrossExchange    CE;
    ThreePointMove ThreePM;

    double start_val;

    this->export_solution_buff(this->best_sol_buff);
    // We are assuming we have an existing solution

    // Set the neighbor list size used in the improvement search
    this->neighbor_list_size=VRPH_MIN(nlist_size, num_nodes);

    best_total_route_length=this->total_route_length;
    normalize_route_numbers();

    ctr=0;

    // The idea is to perform num_loops loops of num_iters iterations each.
    // For each iteration, run through the given heuristic operation at random
    // and perform a Simulated Annealing search.

    rules=VRPH_USE_NEIGHBOR_LIST+VRPH_FIRST_ACCEPT+VRPH_SIMULATED_ANNEALING+VRPH_SAVINGS_ONLY;

    double worst_obj=0;
    for(ctr=0;ctr<num_loops;ctr++)
    {
        if(verbose)
        {
            printf("\nctr=%d of %d, temp=%f, obj=%f (overall best=%f; worst=%f)\n",ctr,num_loops,
                this->temperature, 
                this->total_route_length,this->best_total_route_length,worst_obj);
            fflush(stdout);
        }
        // Reset worst_obj;
        worst_obj=0;

        // Cool it...
        this->temperature = this->cooling_ratio * this->temperature;


        for(int k=0; k < iters_per_loop; k++)
        {
            start_val=total_route_length;
            if(heuristics & THREE_OPT)
            {
                rules=VRPH_SIMULATED_ANNEALING+VRPH_INTRA_ROUTE_ONLY+random+fixed+objective;
                normalize_route_numbers();
                R=total_number_of_routes;
                for(i=1; i<=R; i++)    
                {
                    ThreeO.route_search(this,i,rules);
                    if(this->total_route_length > worst_obj)
                        worst_obj=this->total_route_length;
                }

            }


            if(heuristics & ONE_POINT_MOVE)
            {
                rules=VRPH_SIMULATED_ANNEALING+neighbor_list+random+fixed+objective;
                if(random)
                    random_permutation(perm, this->num_nodes);

                for(i=1;i<=n;i++)    
                {

                    OPM.search(this,perm[i-1],rules);
                    if(this->total_route_length > worst_obj)
                        worst_obj=this->total_route_length;
                }

            }



            if(heuristics & TWO_POINT_MOVE)
            {
                rules=VRPH_SIMULATED_ANNEALING+neighbor_list+random+fixed+objective;
                if(random)
                    random_permutation(perm, this->num_nodes);

                for(i=1;i<=n;i++)    
                {
                    TPM.search(this,perm[i-1],rules);
                    if(this->total_route_length > worst_obj)
                        worst_obj=this->total_route_length;
                }


            }



            if(heuristics & TWO_OPT)
            {

                rules=VRPH_SIMULATED_ANNEALING+neighbor_list+random+fixed+objective;
                if(random)
                    random_permutation(perm, this->num_nodes);

                for(i=1;i<=n;i++)    
                {
                    TO.search(this,perm[i-1],rules);
                    if(this->total_route_length > worst_obj)
                        worst_obj=this->total_route_length;
                }


            }        

            if(heuristics & THREE_POINT_MOVE)
            {
                rules=VRPH_SIMULATED_ANNEALING+VRPH_INTER_ROUTE_ONLY+neighbor_list+random+fixed+objective;
                if(random)
                    random_permutation(perm, this->num_nodes);


                for(i=1;i<=n;i++)    
                {
                    ThreePM.search(this,perm[i-1],rules);
                    if(this->total_route_length > worst_obj)
                        worst_obj=this->total_route_length;
                }
            }

            if(heuristics & OR_OPT)
            {
                rules=VRPH_SIMULATED_ANNEALING+neighbor_list+random+fixed+objective;
                if(random)
                    random_permutation(perm, this->num_nodes);

                for(i=1;i<=n;i++)    
                {
                    OR.search(this,perm[i-1],3,rules);
                    if(this->total_route_length > worst_obj)
                        worst_obj=this->total_route_length;
                }

                for(i=1;i<=n;i++)    
                {
                    OR.search(this,perm[i-1],2,rules);
                    if(this->total_route_length > worst_obj)
                        worst_obj=this->total_route_length;
                }


            }

            if(heuristics & CROSS_EXCHANGE)
            {
                normalize_route_numbers();
                this->find_neighboring_routes();
                R=total_number_of_routes;
                rules=VRPH_SIMULATED_ANNEALING+fixed+objective;
                if(random)
                    random_permutation(perm, this->num_nodes);


                for(i=1; i<=R-1; i++)    
                {
                    for(j=0;j<=1;j++)
                    {
                        CE.route_search(this,i, route[i].neighboring_routes[j],rules);
                        if(this->total_route_length > worst_obj)
                            worst_obj=this->total_route_length;
                    }
                }
            }            
        }
    }

    delete [] perm;

    // Restore the best sol
    this->import_solution_buff(this->best_sol_buff);
    // Now return the obj. function value

    if(has_service_times==false)
        return this->best_total_route_length;
    else
        return this->best_total_route_length-total_service_time;

}


