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
#include <stdarg.h>

void VRP::show_next_array()
{
    ///
    /// Debugging function that shows the current next_array[]
    ///
    int i,n;

    n= num_original_nodes;

    printf("Next Array:\n");
    for(i=0;i<=n;i++)
        printf("%03d -> %03d\n",i,next_array[i]);

    return;
}

void VRP::show_pred_array()
{
    ///
    /// Debugging function that shows the pred_array.
    ///

    int i,n;

    n= num_original_nodes;
    printf("Pred Array:\n");

    for(i=0;i<=n;i++)
        printf("%03d -> %03d\n",i,pred_array[i]);


    return;
}

bool VRP::verify_routes(const char *message)
{
    ///
    /// This debugging function will manually calculate the objective function of the current 
    /// solution and the route values, etc., and compare with the claimed 
    /// value.  Returns false if any inconsistencies are found and prints 
    /// the message.  Returns true with no output otherwise. 
    ///

    int i, n, cnt;
    double len=0;
    double rlen=0;
    double tot_len=0;
    int next_node;
    int current_node, current_route, route_start, flag, current_start, current_end;
    int total_load = 0;
    int current_load = 0;
    int num_in_route=0;
    int counted_routes=0;


    // First check the pred/next arrays for consistency
    current_node=VRPH_DEPOT;
    next_node=VRPH_ABS(this->next_array[current_node]);
    while(next_node!=VRPH_DEPOT)
    {    
        if(VRPH_ABS(this->pred_array[next_node])!=current_node)
        {
            fprintf(stderr,"%d->%d??\nNext: %d->%d\nPred:%d->%d",current_node,next_node,
                current_node,this->next_array[current_node],next_node,this->pred_array[next_node]);
            
            report_error("%s: Next/pred inconsistency\n",__FUNCTION__);
        }
        current_node=next_node;
        next_node=VRPH_ABS(this->next_array[current_node]);
    }

    if(VRPH_ABS(this->pred_array[next_node])!=current_node)
    {
        fprintf(stderr,"%d->%d??\nNext: %d->%d\nPred:%d->%d",current_node,next_node,
            current_node,this->next_array[current_node],next_node,this->pred_array[next_node]);

        report_error("%s: Next/pred inconsistency\n",__FUNCTION__);
    }

    n=num_nodes;
    // Only consider the nodes in the solution!

    flag=0;    
    i = 1;
    cnt = 0;
    route_start = -next_array[VRPH_DEPOT];
    if(route_start < 0)
    {
        fprintf(stderr,"next[VRPH_DEPOT] is incorrect\n");
        report_error(message,__FUNCTION__);

    }

    current_node = route_start;
    current_route = route_num[current_node];    
    current_start = route[current_route].start;
    current_end = route[current_route].end;
    counted_routes++;

    if(route_start != current_start)
    {
        fprintf(stderr,"Error in initial route start:  %d != %d\n",route_start, current_start);
        report_error(message,__FUNCTION__);
    }
        
    

    total_load+=nodes[current_node].demand;
    current_load+=nodes[current_node].demand;
    len+=d[VRPH_DEPOT][current_node];
    rlen+=d[VRPH_DEPOT][current_node];
    if(current_node!= dummy_index)
        num_in_route++;

    while(route_start != 0 && i<num_nodes+1)
    {
        // When we finally get a route beginning at 0, this is the last route 
        // and there is no next route, so break out

        if(next_array[current_node]==current_node)
        {
            // We've entered a loop
            fprintf(stderr,"Self loop found in next array(%d)\n",current_node);
            report_error("%s: Self loop!\n",__FUNCTION__);
        }

        if(next_array[current_node]==VRPH_DEPOT)
        {
            // We're at the end of the Solution!
            len+=d[current_node][VRPH_DEPOT];
            rlen+=d[current_node][VRPH_DEPOT];
            current_route=route_num[current_node];
            tot_len+=rlen;
            if(num_in_route != route[current_route].num_customers)
            {
                fprintf(stderr,"Customer count error!!\nCounted(%d)!=Claimed(%d) in final route %d\n",num_in_route,
                    route[current_route].num_customers,current_route);
                show_route(current_route);
                report_error(message);
            }

            // Now check the # of routes
            if(counted_routes != total_number_of_routes)
            {
                // error in # of routes
                fprintf(stderr, "Incorrect # of routes recorded %d!=%d\n",counted_routes, 
                    total_number_of_routes);
                report_error(message);

            }
            if(VRPH_ABS(len-total_route_length)<.01 && flag==0 )
            {
                // everything looks good
                return true;
            }
            else
            {
                if(VRPH_ABS(len-total_route_length)>=.01)
                {
                    fprintf(stderr,"Objective function error: calculated(%f)!=claimed(%f)\n",len,
                        total_route_length);
                    report_error(message);
                }

                
                report_error(message);

            }

        }

        if(next_array[current_node]>0)
        {
            // Next node is somewhere in the middle of a route

            next_node = next_array[current_node];
            // Make sure current_node and next_node have the same route #
            if(route_num[current_node]!=route_num[next_node])
            {
                fprintf(stderr,"Route # error for %d and %d: %d!=%d\n",current_node, next_node,
                    route_num[current_node],route_num[next_node]);
                
                report_error(message);    
            }
            len+=d[current_node][next_node];
            rlen+=d[current_node][next_node];


            current_node=next_node;

            if(current_node!= dummy_index)
                num_in_route++;

            total_load+=nodes[current_node].demand;
            current_load+=nodes[current_node].demand;
            cnt++;
        }
        else
        {
            // We must have a non-positive "next" node indicating the beginning of a new route

            len+=d[current_node][VRPH_DEPOT];

            rlen+=d[current_node][VRPH_DEPOT];
            tot_len+=rlen;
            current_route=route_num[current_node];
            current_end = route[current_route].end;

            if(num_in_route != route[current_route].num_customers)
            {
                fprintf(stderr,"%d (calculated) != %d (claimed) in route %d\n",num_in_route,
                    route[current_route].num_customers,current_route);
                show_route(current_route);
                
                // Report this now..
                report_error(message);
            }

            if(current_node != current_end)
            {
                fprintf(stderr,"Error in route ends: %d!=%d\n",current_node, current_end);
                report_error(message);
            }
            if(VRPH_ABS(rlen-route[current_route].length)>.1)
            {
                fprintf(stderr,"Route Lengths:  Calculated(%f)!=Claimed(%f)\n",rlen,route[current_route].length);

                int ii=route[current_route].start;
                int jj;
                fprintf(stderr,"0->%d = %f[%f]\n",ii, d[VRPH_DEPOT][ii], nodes[ii].service_time);
                while(ii != 0)
                {
                    jj=VRPH_MAX(VRPH_DEPOT,next_array[ii]);
                    fprintf(stderr,"%d->%d = %f[%f]\n",ii,jj,d[ii][jj], nodes[jj].service_time);
                    ii=jj;

                }

                report_error(message);
            }

            

            if(current_load != route[current_route].load)
            {
                fprintf(stderr,"Route Loads:  %d!=%d\n",current_load, route[current_route].load);
                report_error(message);
            }

            i++;
            route_start = - (next_array[current_node]);    
            current_route = route_num[route_start];
            current_start = route[current_route].start;
            current_end = route[current_route].end;
            counted_routes++;

            if(route_start != current_start)
            {
                fprintf(stderr,"Route %d:  %d != %d\n",current_route, route_start, current_start);
                report_error(message);
            }

            current_node = route_start;
            total_load+=nodes[current_node].demand;
            // reset current_load to 0
            current_load=nodes[current_node].demand;
            //len+=nodes[current_node].depot_distance;
            len+=d[VRPH_DEPOT][current_node];
            // reset rlen to 0
            //rlen=nodes[current_node].depot_distance;
            rlen=d[VRPH_DEPOT][current_node];
            if(current_node!= dummy_index)
                num_in_route=1;
            else
                num_in_route=0;
            cnt++;
        }
    }

    return true;
}    

void report_error(const char* format, ...) 
{
    ///
    /// Prints the message and function name to stderr and terminates the program.
    ///

    va_list args;

    va_start(args, format);
    vfprintf(stderr, format, args);
    va_end(args);

    fprintf(stderr,"Exiting\n");
    exit(-1);
}

