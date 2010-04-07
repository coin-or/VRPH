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
#define VRPH_MAX_CYCLES 500

void VRPH_version()
{
    printf("--------------------------------------------\n");
    printf("VRPH, version 1.0\nCopyright 2010 Chris Groer\nDistributed under Common Public License 1.0\n");
    printf("--------------------------------------------\n\n");
}


VRP::VRP(int n)
{
    /// 
    /// Constructor for an n-node problem.
    ///

    int i,j;

    num_nodes=n;
    num_original_nodes=n;
    total_demand=0;
    num_days=0;
    
    next_array = new int[n+2];
    pred_array = new int[n+2];
    route_num = new int[n+2];
    route = new VRPRoute[n+2];
    routed = new bool[n+2];
    best_sol_buff = new int[n+2];
    current_sol_buff = new int[n+2];
    search_space = new int[n+2];
    nodes = new VRPNode[n+2];    
    // Add an extra spot for the VRPH_DEPOT and for the dummy node
    
    symmetric=true;
    // Set to false only when we encounter FULL_MATRIX file

    forbid_tiny_moves=true;    
    // Default is to allow these moves

    d=NULL;
    // The distance matrix is allocated when the problem is loaded
    fixed=new bool*[n+2];
    fixed[0]=new bool[(n+2)*(n+2)];
    for(i=1;i<n+2;i++)
        fixed[i]=fixed[i-1]+(n+2);
    for(i=0;i<n+2;i++)
    {
        routed[i]=false;
        for(j=0;j<n+2;j++)
            fixed[i][j]=false;
    }

    // Set these to default values--they may change once
    // we read the file.

    min_vehicles=-1;
    has_service_times=false;
    max_route_length=VRP_INFINITY;
    orig_max_route_length=VRP_INFINITY;
    total_route_length=0.0;
    best_known=VRP_INFINITY;
    depot_normalized=false; 
    // Will be set to true if we shift nodes so VRPH_DEPOT is at origin
    
    // These are for record-to-record travel
    record = 0.0;
    deviation = VRPH_DEFAULT_DEVIATION;        

    // For keeping track of the statistics

    for(i=0;i<NUM_HEURISTICS;i++)
    {
        num_evaluations[i]=0;
        num_moves[i]=0;

    }

    total_service_time = 0.0;

    // Create the solution warehouse
    this->solution_wh=new VRPSolutionWarehouse(NUM_ELITE_SOLUTIONS,n);

    this->tabu_list=new VRPTabuList(MAX_VRPH_TABU_LIST_SIZE);
    this->route_wh=NULL;

    // Set this to true only if we have valid coordinates 
    // This is valid only for plotting the solution
    can_display=false;
    
    
}

VRP::VRP(int n, int ndays)
{
    /// 
    /// Constructor for an n-node, ndays-day problem.
    ///

    int i,j;

    num_nodes=n;
    num_original_nodes=n;
    total_demand=0;
    num_days=ndays;
    
    next_array = new int[n+2];
    pred_array = new int[n+2];
    route_num = new int[n+2];
    route = new VRPRoute[n+2];
    routed = new bool[n+2];
    best_sol_buff = new int[n+2];
    current_sol_buff = new int[n+2];
    search_space = new int[n+2];
    nodes = new VRPNode[n+2];    
    // Add an extra spot for the VRPH_DEPOT and for the dummy node
    
    forbid_tiny_moves=true;    
    // Default is to forbid these moves

    d=NULL;
    // The distance matrix is allocated when the problem is loaded
    fixed=new bool*[n+2];
    fixed[0]=new bool[(n+2)*(n+2)];
    for(i=1;i<n+2;i++)
        fixed[i]=fixed[i-1]+(n+2);
    for(i=0;i<n+2;i++)
    {
        routed[i]=false;
        for(j=0;j<n+2;j++)
            fixed[i][j]=false;
    }

    // Set these to default values--they may change once
    // we read the file.

    min_vehicles=-1;
    has_service_times=false;
    max_route_length=VRP_INFINITY;
    orig_max_route_length=VRP_INFINITY;
    total_route_length=0.0;
    best_known=VRP_INFINITY;

    depot_normalized=false; 
    // Will be set to true if we shift nodes so VRPH_DEPOT is at origin
    
    // These are for record-to-record travel
    record = 0.0;
    deviation = VRPH_DEFAULT_DEVIATION;        

    // For keeping track of the statistics

    for(i=0;i<NUM_HEURISTICS;i++)
    {
        num_evaluations[i]=0;
        num_moves[i]=0;

    }

    total_service_time = 0.0;

    // Create the solution warehouse
    this->solution_wh=new VRPSolutionWarehouse(NUM_ELITE_SOLUTIONS,n);

    this->tabu_list=new VRPTabuList(MAX_VRPH_TABU_LIST_SIZE);
    this->route_wh=NULL;

    // Now allocate d days worth of storage at each of the nodes
    for(i=0;i<=n+1;i++)
    {
        this->nodes[i].daily_demands=new int[ndays+1];
        this->nodes[i].daily_service_times=new double[ndays+1];
    }

    // Set this to true only if we have valid coordinates 
    // This is valid only for plotting the solution
    can_display=false;
    
}

VRP::~VRP()
{
    ///
    /// Destructor for the VRP.
    ///

    delete [] this->best_sol_buff;
    delete [] this->current_sol_buff;
    delete [] this->d[0];
    delete [] this->d;
    delete [] this->fixed[0];
    delete [] this->fixed;
    delete [] this->next_array;
    delete [] this->search_space;
    delete [] this->nodes;
    delete [] this->pred_array;
    delete [] this->route;
    delete [] this->route_num;
    delete [] this->routed;
    delete this->solution_wh;
    delete this->tabu_list;

}


// Accessor functions for private data 
int VRP::get_num_nodes()
{
    ///
    /// Returns the number of nodes in the instance.
    ///

    return this->num_nodes;
}

double VRP::get_total_route_length()
{
    ///
    /// Returns the total route length of the current solution.
    ///

    return this->total_route_length;
}

double VRP::get_total_service_time()
{
    ///
    /// Returns the total service time of all customers in the instance.
    ///
    
    return this->total_service_time;
}

double VRP::get_best_sol_buff(int *sol_buff)
{
    ///
    /// Copies the best solution buffer found so far into the sol_buff[] array.
    /// Assumes that sol_buff[] is of sufficient size.  Returns the total route length
    /// of this solution.
    ///

    memcpy(sol_buff,this->best_sol_buff,(this->num_nodes+1)*sizeof(int));
    return this->best_total_route_length;
}

double VRP::get_best_total_route_length()
{
    ///
    /// Returns the total route length of the best solution discovered so far.
    ///

    return this->best_total_route_length;        
}

int VRP::get_total_number_of_routes()
{
    ///
    /// Returns the number of routes in the current solution.
    ///

    return this->total_number_of_routes;
}

int VRP::get_num_original_nodes()
{
    ///
    /// Returns the number of original nodes in the instance.
    ///

    return this->num_original_nodes;
}

int VRP::get_num_days()
{
    ///
    /// Returns the number of days in the currently loaded instance.
    ///

    return this->num_days;
}

double VRP::get_best_known()
{
    return this->best_known;
    
}

int VRP::get_max_veh_capacity()
{   
    return this->max_veh_capacity;
}

double VRP::get_max_route_length()
{
    return this->max_route_length;     
}

void VRP::set_best_total_route_length(double val)
{
    this->best_total_route_length=val;
    return;

}
bool VRP::clone(VRP *W)
{ 
    ///
    /// Copy Constructor for VRP.
    ///

    this->balance_parameter=W->balance_parameter;
    this->best_known=W->best_known;
    this->best_total_route_length=W->best_total_route_length;
    // Copy the best solution buffer
    memcpy(this->best_sol_buff, W->best_sol_buff, (sizeof(int))*(W->num_nodes+2));

    // Copy the currnet solution buffer
    memcpy(this->current_sol_buff, W->current_sol_buff, (sizeof(int))*(W->num_nodes+2));

    this->coord_type=W->coord_type;
    this->d=W->d;                // OK to just copy the pointers here
    this->deviation=W->deviation;
    this->display_type=W->display_type;
    this->edge_weight_format=W->edge_weight_format;
    this->dummy_index=W->dummy_index;
    this->has_service_times=W->has_service_times;
    this->matrix_size=W->matrix_size;
    this->max_route_length=W->max_route_length;
    this->neighbor_list_size=W->neighbor_list_size;
    this->nodes = W->nodes; // OK to just copy the pointers here?

    this->num_nodes=W->num_nodes;
    this->total_route_length=W->total_route_length;
    this->orig_max_route_length=W->orig_max_route_length;
    this->orig_max_veh_capacity=W->orig_max_veh_capacity;

    this->problem_type=W->problem_type;
    this->record=W->record;


    this->search_size=W->search_size;

    this->temperature=W->temperature;
    this->total_number_of_routes=W->total_number_of_routes;
    this->total_service_time=W->total_service_time;
    this->max_veh_capacity=W->max_veh_capacity;
    this->violation=W->violation;
    this->edge_weight_type=W->edge_weight_type;

    // Assume that the current_solution has been sent to W->
    W->export_solution_buff(W->current_sol_buff);
    this->import_solution_buff(W->current_sol_buff);
    return true;    

}


void VRP::create_distance_matrix(int type)
{    
    ///
    /// Creates the O(n^2) size distance matrix for the
    /// provided data using the distance function referenced by type.
    /// If the type is EXPLICIT, then the entire distance matrix should
    /// be provided in the actual TSPLIB file.
    ///
    

    int i,j,k,n;


    if(type==VRPH_EXPLICIT)
        // We have presumably already loaded in the distance matrix!
        return;

    n= this->num_original_nodes;
    k=0;

    // Otherwise construct the matrix - we store the whole thing even though
    // it is symmetric as we found that this was quite a bit faster...

    for(i=0;i<=n+1;i++)
    {
        for(j=0;j<=n+1;j++)
            this->d[i][j]=VRPDistance(type, this->nodes[i].x,this->nodes[i].y,this->nodes[j].x,
                this->nodes[j].y) + this->nodes[j].service_time ;

    }


    return;

}

void VRP::create_neighbor_lists(int nsize)
{
    ///
    /// Creates the neighbor list of size nsize for each node
    /// including the VRPH_DEPOT. 
    ///

    if(nsize>num_nodes )
    {
        fprintf(stderr,"Requested neighbor list size is greater than num_nodes!\n%d>%d\n",
            nsize,num_nodes);
        report_error("%s: Neighbor list error!!\n",__FUNCTION__);
    }

    if(nsize>MAX_NEIGHBORLIST_SIZE )
    {
        fprintf(stderr,"Requested neighbor list size is greater than MAX_NEIGHBORLIST_SIZE!\n%d>%d\n",
            nsize,MAX_NEIGHBORLIST_SIZE);
        report_error("%s: Neighbor list error!!\n",__FUNCTION__);
    }


    int i,j,n,b;
    int k;
    double dd;
    double max;
    int maxpos;
    VRPNeighborElement *NList;

    n= num_nodes;

    // Set the neighbor_list_size value
    neighbor_list_size=nsize;

    NList=new VRPNeighborElement[nsize];

    // First, do the neighbor_list for the VRPH_DEPOT
    max=0.0;
    // NEW
    maxpos=0;

    for(i=1;i<=nsize;i++)
    {
        NList[i-1].val=d[VRPH_DEPOT][i];

        NList[i-1].position = i;

        // keep track of the largest value
        if(NList[i-1].val > max)
        {
            max=NList[i-1].val;
            maxpos=i-1;
        }

    }    

    // Now go through the rest of the nodes.
    for(i=nsize+1;i<=n;i++)
    {
        if(d[VRPH_DEPOT][i]<max)
        {
            // Replace element in position maxpos
            NList[maxpos].val=d[VRPH_DEPOT][i];
            //nodes[i].depot_distance;
            NList[maxpos].position=i;
        }
        // Now find the new max
        max=0.0;
        for(b=0;b<nsize;b++)
        {
            if(NList[b].val>max)
            {
                maxpos=b;
                max=NList[b].val;
            }
        }
    }

    qsort(NList,nsize,sizeof(VRPNeighborElement), VRPNeighborCompare);
    i=0;
    for(b=0;b<nsize;b++)
    {
        nodes[VRPH_DEPOT].neighbor_list[b].val=NList[b].val;
        nodes[VRPH_DEPOT].neighbor_list[b].position=NList[b].position;

#        if NEIGHBOR_DEBUG
        printf("(%d,%d,%f) \n",VRPH_DEPOT,nodes[VRPH_DEPOT].neighbor_list[b].position,
            nodes[VRPH_DEPOT].neighbor_list[b].val);
#endif


    }

#    if NEIGHBOR_DEBUG
    printf("\n\n");
#endif


    k=0;
    // Done with NeighborList for VRPH_DEPOT.  Now do it for the rest of the nodes
    for(i=1;i<=n;i++)
    {
        // Loop through all the nodes and create their Neighbor Lists
        // First initialize the NList
        for(j=0;j<nsize;j++)
        {
            NList[j].position=VRP_INFINITY;
            NList[j].val=VRP_INFINITY;
        }


        // VRPH_DEPOT can be in NList, so set NList[0] to depot_distance
        NList[0].position=VRPH_DEPOT;
        NList[0].val=d[i][VRPH_DEPOT];


        for(j=1;j<i;j++)
        {
#if NEIGHBOR_DEBUG
            printf("Loop 1; i=%d; in j loop(%d)\n",i,j);
#endif

            dd=d[i][j];

            if(j<nsize)
            {
                // Construct the original nsize long neighbor list
                NList[j].val=dd;
                NList[j].position=j;
                // Update max size
                if(NList[j].val>max)
                {
                    max=NList[j].val;
                    maxpos=j;

                }
            }
            else
            {
                // j >= nsize
                if(dd<max)
                {
                    NList[maxpos].val=dd;
                    NList[maxpos].position=j;
                    // Now find the new max element;
                    max=0.0;
                    for(b=0;b<nsize;b++)
                    {
                        if(NList[b].val>max)
                        {
                            max=NList[b].val;
                            maxpos=b;
                        }
                    }
                }
            }

        }


        for(j=i+1;j<=n;j++)
        {
#if NEIGHBOR_DEBUG
            printf("Loop 2; i=%d; in j loop(%d)\n",i,j);
#endif

            dd=d[i][j];

            if(j<=nsize)// was <
            {
                // Construct the original nsize long neighbor list
                NList[j-1].val=dd;
                NList[j-1].position=j;
                // Update max size
                if(NList[j-1].val>max)
                {
                    max=NList[j-1].val;
                    maxpos=j-1;

                }
            }
            else
            {
                if(dd<max)
                {
                    // Replace maxpos with this one
                    NList[maxpos].val=dd;
                    NList[maxpos].position=j;
                    // Now find the new max element;
                    max=0.0;
                    for(b=0;b<nsize;b++)
                    {
                        if(NList[b].val>max)
                        {
                            max=NList[b].val;
                            maxpos=b;
                        }
                    }
                }
            }

        }


#if NEIGHBOR_DEBUG
        printf("Node %d neighbor list before sorting\n",i);
        for(b=0;b<nsize;b++)
        {
            printf("NList[%d]=%d,%f[%f]\n",b,NList[b].position,NList[b].val,d[i][NList[b].position]);
        }
#endif
        // Now sort the NList and stick the resulting list into
        // the neighbor list for node i
        qsort (NList, nsize, sizeof(VRPNeighborElement), VRPNeighborCompare);

        for(b=0;b<nsize;b++)
        {
            nodes[i].neighbor_list[b].position=NList[b].position;
            nodes[i].neighbor_list[b].val=NList[b].val;
            if(i== nodes[i].neighbor_list[b].position)
            {
                fprintf(stderr,"ERROR:: Node %d is in it's own neighbor list!!\n", i);
                fprintf(stderr,"i=%d; %d[%d],%f[%f])\n",i,nodes[i].neighbor_list[b].position,
                    NList[b].position,nodes[i].neighbor_list[b].val,NList[b].val);

                report_error("%s: Error creating neighbor lists\n",__FUNCTION__);
            }
#if NEIGHBOR_DEBUG

            printf("NList[%d]=%d,%f[%f]",b,nodes[i].neighbor_list[b].position,nodes[i].neighbor_list[b].val,d[i][nodes[i].neighbor_list[b].position]);

#endif
        }

        // Hackk to make the VRPH_DEPOT in everyone's list
        //nodes[i].neighbor_list[nsize-1].position=VRPH_DEPOT;
        //nodes[i].neighbor_list[nsize-1].val=d[i][VRPH_DEPOT];

    }

    delete [] NList;

    return;

}


bool VRP::check_feasibility(VRPViolation *VV)
{
    ///
    /// The function returns true if the routes are all feasible
    /// and false if any of them are infeasible, placing the worst violations
    /// in the VRPViolation VV
    ///

    int i;
    bool is_feasible=true; 
    // We will set this to false if we find a violation

    VV->capacity_violation=-VRP_INFINITY;
    VV->length_violation=-VRP_INFINITY;

    this->normalize_route_numbers();

    for(i=1;i<=this->total_number_of_routes;i++)
    {
        if(this->route[i].length>this->orig_max_route_length)
        {
            // Length violation
            if(this->route[i].length - this->orig_max_route_length > VV->length_violation)
                VV->length_violation = this->route[i].length - this->orig_max_route_length ;

            is_feasible=false;
        }

        if(this->route[i].load>this->orig_max_veh_capacity)
        {
            // Load violation
            if(this->route[i].load - this->orig_max_veh_capacity > VV->capacity_violation)
                VV->capacity_violation = this->route[i].load - this->orig_max_veh_capacity ;

            is_feasible=false;

        }

    }

    return is_feasible;

}    




void VRP::refresh_routes()
{
    ///
    /// Ignores the current route length and load values and
    /// recalculates them directly from the next_array.
    ///


    int i, n, cnt;
    double len=0;
    double rlen=0;
    double tot_len=0;
    int next_node;
    int current_node, current_route, route_start, current_start, current_end;
    int total_load = 0;
    int current_load = 0;


    n=num_nodes;

    i = 1;
    cnt = 0;
    route_start = -next_array[VRPH_DEPOT];

    current_node = route_start;
    current_route = route_num[current_node];

    current_start = route[current_route].start;
    current_end = route[current_route].end;

    total_load+=nodes[current_node].demand;
    current_load+=nodes[current_node].demand;
    len+=d[VRPH_DEPOT][current_node];
    rlen+=d[VRPH_DEPOT][current_node];

    while(route_start != 0 && i<num_nodes+1)
    {
        // When we finally get a route beginning at 0, this is the last route 
        // and there is no next route, so break out
        if(next_array[current_node]==current_node)
        {
            // We've entered a loop
            fprintf(stderr,"(2)Self loop found in next array(%d)\n",current_node);
            report_error("%s: Error in refresh_routes()\n",__FUNCTION__);
        }

        if(next_array[current_node]==0)
        {
            // We're at the end of the Solution!

            len+=d[current_node][VRPH_DEPOT];
            rlen+=d[current_node][VRPH_DEPOT];
            current_route=route_num[current_node];
            route[current_route].length = rlen;
            route[current_route].load = current_load;
            total_route_length=len;

            // We're done now
            return;
        }

        if(next_array[current_node]>0)
        {
            // Next node is somewhere in the middle of a route        

            next_node = next_array[current_node];
            len+=d[current_node][next_node];
            rlen+=d[current_node][next_node];
            current_node=next_node;
            total_load+=nodes[current_node].demand;
            current_load+=nodes[current_node].demand;
            cnt++;
        }
        else
        {
            // We must have a non-positive "next" node indicating the beginning of a new route
            //len+=nodes[current_node].depot_distance;
            len+=d[current_node][VRPH_DEPOT];
            //rlen+=nodes[current_node].depot_distance;
            rlen+=d[current_node][VRPH_DEPOT];
            tot_len+=rlen;
            current_route=route_num[current_node];
            current_end = route[current_route].end;

            route[current_route].length = rlen;
            route[current_route].load = current_load;

            i++;
            route_start = - (next_array[current_node]);    
            current_route = route_num[route_start];
            current_start = route[current_route].start;
            current_end = route[current_route].end;
            if(route_start != current_start)
            {
                fprintf(stderr,"Route %d:  %d != %d\n",current_route, route_start, current_start);
                report_error("%s: Error in refresh_routes()\n",__FUNCTION__);
            }

            current_node = route_start;
            total_load+=nodes[current_node].demand;
            // reset current_load to 0
            current_load=nodes[current_node].demand;
            len+=d[VRPH_DEPOT][current_node];
            rlen=d[VRPH_DEPOT][current_node];
            cnt++;
        }
    }


    return;


}
void VRP::create_pred_array()
{
    ///
    /// This function creates a pred_array from the existing next_array
    ///
    int i,j;

    i=VRPH_DEPOT;
    j=next_array[i];
    while(j!=VRPH_DEPOT)
    {
        if(j>0)
            pred_array[j]=i;
        else
            pred_array[-j]=-i;
        
        i=VRPH_ABS(j);
        j=next_array[i];

    }
    // The VRPH_DEPOT
    pred_array[j]=-i;
    return;

    



    /*for(i=0;i<=num_nodes;i++)
    {
        j=next_array[i];
        if(j>0)
            pred_array[j] = i;
        else
            pred_array[-j] = -i;
    }
    return;*/
}





bool VRP::get_segment_info(int a, int b, struct VRPSegment *S)
{
    ///
    /// Calculates the length, load, and # of customers on the segment
    /// of the route between nodes a and b.  Assumes that a is before b
    /// in the route - this is not checked!
    /// Example:  a-i-j-b has
    /// length: d(a,i)+d(i,j)+d(j,b)
    /// load:   a + i + j + b
    /// #:      4
    /// 


    if(a==b)
    {
        // Degenerate case...
        S->segment_start=b;
        S->segment_end=a;
        S->len=0;  //nodes[a].service_time??;
        S->load=nodes[a].demand;
        S->num_custs=1;
        return true;

    }

    S->len=0;    
    S->segment_start=a;
    S->segment_end=b;
    S->num_custs=0;        


    // Check special cases
    if(a==VRPH_DEPOT)
    {
        // Safe to assume we want the segment 0-i-j-...-k-b-...

        S->segment_start=route[route_num[b]].start;
        S->segment_end=b;
        S->len+=d[VRPH_DEPOT][S->segment_start];

    }

    if(b==VRPH_DEPOT)
    {
        // Safe to assume we want the segment ...a-i-j-...-k-0

        S->segment_start=a;
        S->segment_end=route[route_num[a]].end;

    }

    // Now calculate the length, load, and # customer on this segment

    int current_node = S->segment_start;
    int next_node;

    S->load=nodes[current_node].demand;
    if(current_node!=dummy_index)
        S->num_custs++;;

    while(current_node != S->segment_end)
    {
        next_node = VRPH_MAX(next_array[current_node],0);
        S->len+=d[current_node][next_node];
        current_node=next_node;        
        S->load+=nodes[next_node].demand;
        if(current_node!= dummy_index)
            S->num_custs += 1;
    }

    if(b==VRPH_DEPOT)
        S->len+=d[S->segment_end][VRPH_DEPOT];



    return true;

}

int VRP::get_string_end(int a, int len)
{
    ///
    /// Finds the node that is (len-1) hops from a
    /// so that the string from a to the end contains len nodes.
    /// Returns -1 if not possible to get such a string.
    ///

    int ctr=1;
    int current_node;

    current_node=a;
    while(ctr<len)
    {
        current_node= next_array[current_node];
        if(current_node<0)
        {
            // The string of length len beginning at a is not contained in a single route
            return -1;
        }
        ctr++;

    }

    return current_node;
}

void VRP::reverse_route(int i)
{
    ///
    /// This function reverses route number i - does no 
    /// error checking since we don't know if the routes
    /// have normalized numbers or not...
    ///

    
    int current_node, start_node, last_node, prev_route, next_route, temp;
    int orig_start, orig_end ;

    if(i<=0)
    {
        // Illegal!
        fprintf(stderr,"Reversing route  of negative index?? i=%d\n",i);
        report_error("%s: Can't reverse route!\n",__FUNCTION__);
    }

    orig_end = route[i].end;
    orig_start = route[i].start;

    start_node=orig_start;
    current_node=start_node;

    // We begin with the first node in the route - we will change its next_array[]
    // value at the very end

    temp = next_array[current_node];
    prev_route = -pred_array[current_node];
    // So next[prev_route] should equal start_node
    // We will change this so that next[prev_route]=-last_node

    pred_array[current_node] = temp;
    current_node = temp;
    while( (temp = next_array[current_node]) >0)
    {        
        next_array[current_node] = pred_array[current_node];
        pred_array[current_node] = temp;
        current_node = temp;
    }

    // current_node is now the end of the original route, so we can finish the reversal

    temp= pred_array[current_node];
    last_node = current_node;
    next_route = -next_array[last_node];

    route[i].end=orig_start;
    route[i].start=orig_end;

#if REVERSE_DEBUG
    printf("start_node: %d; last_node: %d; prev_route: %d; next_route: %d\n",start_node,last_node, prev_route,next_route);
#endif

    // Final set of updates
    next_array[prev_route]=-last_node;
    pred_array[next_route]=-start_node;
    next_array[start_node]=-next_route;
    pred_array[last_node]= -prev_route;
    next_array[last_node]=temp;
    next_array[prev_route]=-last_node;

    // Need to update length if asymmetric
    if(!this->symmetric)
    {
        // Update length as it possibly changed!
        this->refresh_routes();
    }
    
    return;
}



bool VRP::postsert_dummy(int i)
{
    // This function inserts dummy node after i

    int pre_i, post_i, dummy;
    int start, end, start_i, end_i;
    int n, i_route;
    //double tu, uv, tv, iu, ju, ij, u_loss, i_gain, i_change, i_length, u_length;

    // Special cases first

    if(i<=VRPH_DEPOT || i>matrix_size)
        report_error("%s: input doesn't make sense\n",__FUNCTION__);

    n= num_nodes;
    dummy= dummy_index;

    i_route= route_num[i];

    start_i= route[i_route].start;
    end_i= route[i_route].end;

    start=start_i;
    if(end_i==i)
        // i is last in the route
        end=dummy;
    else 
        end=end_i;

    // pre_i is what used to be before i
    pre_i= pred_array[i];
    // post_i is what used to be after i
    post_i= next_array[i];

    next_array[i] = dummy;
    next_array[dummy] = post_i;

    // Now need to update pred_array as well!

    // u's predecessor is now i
    pred_array[dummy]=i;

    // The element who used to be after i is now preceded by u 
    if(post_i>=0)
        pred_array[post_i]=dummy;
    else
        // post_i 
        pred_array[-post_i]=-dummy;

    //start_array[dummy]=start;
    //end_array[dummy]=end;

    // Update i_route information
    route_num[dummy]=i_route;
    route[i_route].end=end;
    route[i_route].start=start;

    return true;
}

bool VRP::presert_dummy(int i)
{
    // This function inserts a dummy node BEFORE node i in 
    // whatever route node i is currently in

    int pre_i, post_i;
    int start, end, start_i, end_i, dummy;
    int n, i_route;


    n= num_nodes ;
    dummy = dummy_index; // dummy node has index (n+1)

    if(i<=VRPH_DEPOT)
        report_error("%s: bad index\n",__FUNCTION__);

    i_route = route_num[i];

    start_i= route[i_route].start;
    end_i= route[i_route].end;


    // Get the start and end of i's route
    start=start_i;
    end=end_i;

    if(start==i)
        // i is first in the route, so dummy is now the first node
        start=dummy;

    // pre_i is what used to be before i
    pre_i= pred_array[i];
    // post_i is what used to be after i
    post_i= next_array[i];


    // dummy is now followed by i
    next_array[dummy]=i;
    pred_array[i]=dummy;
    pred_array[dummy]=pre_i;    

    // The element who used to be after i is now preceded by dummy
    if(pre_i>0)// was >=!!
        next_array[pre_i]=dummy;
    else
        // post_i 
        next_array[VRPH_ABS(pre_i)]=-dummy;

    // Update i_route information
    route_num[dummy]=i_route;
    route[i_route].end=end;
    route[i_route].start=start;

    // Add in the relevant Data fields for the dummy node!!

    return true;

}
bool VRP::remove_dummy()
{
    int pre_d, post_d, d_route, dummy, d_start, d_end, n;

    n= num_nodes;
    dummy= dummy_index;

    post_d= next_array[dummy];
    pre_d= pred_array[dummy];

    if(post_d > dummy || post_d < -dummy || pre_d > dummy || pre_d < -dummy)
    {
        fprintf(stderr,"post_d= %d; pre_d=%d\n",post_d,pre_d);
        report_error("%s: invalid indices\n",__FUNCTION__);
    }

    d_route= route_num[dummy];
    d_start= route[d_route].start;
    d_end= route[d_route].end;

    if(d_start==dummy)
    {
        if(post_d<0)
        {
            report_error("%s: post_d error in removal\n",__FUNCTION__);
        }

        route[d_route].start=post_d;
    }

    if(d_end==dummy)
    {
        if(pre_d<0)
        {
            report_error("%s: pre_d error in removal\n",__FUNCTION__);
        }
        route[d_route].end=pre_d;
    }

    next_array[VRPH_ABS(pre_d)]=post_d;

    if(d_start==dummy)
        next_array[VRPH_ABS(pre_d)]=-post_d;


    pred_array[VRPH_ABS(post_d)]=pre_d;

    if(d_end==dummy)
        pred_array[VRPH_ABS(post_d)]=-pre_d;


    return true;
}
bool VRP::create_default_routes()
{
    ///
    /// This function creates routes VRPH_DEPOT-i-VRPH_DEPOT for all nodes i
    /// and properly initializes all the associated arrays.  Returns
    /// true if successful, false if the default routes violate some
    /// capacity or route length constraint.
    ///


    int i,n;
    bool is_feasible=true;
    // No violations yet...
    violation.capacity_violation = 0;
    violation.length_violation = 0;


    total_route_length=0;

    n = this->num_original_nodes;
    // First, create the initial routes:  VRPH_DEPOT - node - VRPH_DEPOT for all nodes

    routed[VRPH_DEPOT]=true;

    next_array[VRPH_DEPOT] = -1;
    for(i=1;i<=n;i++)
    {
        next_array[i] = -(i+1);
        total_route_length+= (d[VRPH_DEPOT][i] + d[i][VRPH_DEPOT]);

        route_num[i]=i;
        route[i].start=i;
        route[i].end=i;
        route[i].load= nodes[i].demand;
        route[i].length= d[VRPH_DEPOT][i] + d[i][VRPH_DEPOT];

        
        // Check capacities
        if(route[i].load > max_veh_capacity)
            is_feasible=false;
        if(route[i].length > max_route_length)
            is_feasible=false;

        route[i].num_customers=1;

        routed[i]=true;    

    }


    next_array[n]=VRPH_DEPOT;

    route_num[VRPH_DEPOT]=0;
    
    // Now create the associated pred_array implied by the newly created next_array
    create_pred_array();

    total_number_of_routes=n;





    if(is_feasible == false)
    { 
        // We had infeasible routes - record the violation in the Solution and 
        // return false;


        for(i=1;i<=n;i++)
        {
            routed[i]=false;    
            // Set routed status to false since default routes are infeasible

            // Check capacities
            if(route[i].load > max_veh_capacity)
            {
                // Update the worst violations

                printf("Default routes load violation: %d > %d\n",route[i].load,
                    max_veh_capacity);

                if( (route[i].load - max_veh_capacity) > 
                    violation.capacity_violation)
                {
                    violation.capacity_violation = 
                        (route[i].load - max_veh_capacity);
                }

            }
            if(route[i].length > max_route_length)
            {
                // Update the worst violations

                printf("Default routes length violation: %f > %f\n",route[i].length,
                    max_route_length);

                if( (route[i].length -max_route_length ) >
                    violation.length_violation)
                {
                    violation.length_violation = 
                        (route[i].length -max_route_length );
                }


            }


        }

        return false;
    }
    else
    {
        // All routes were feasible
        for(i=1;i<=n;i++)
        {
            routed[i]=true;
        }
        return true;
    }


}


bool VRP::create_default_routes(int day)
{
    ///
    /// This function creates routes VRPH_DEPOT-i-VRPH_DEPOT for all nodes that
    /// require service on the provided day.  Returns
    /// true if successful, false if the default routes violate some
    /// capacity or route length constraint.
    ///

    int i,n;
    bool is_feasible=true;
    // No violations yet...
    violation.capacity_violation = 0;
    violation.length_violation = 0;
    total_route_length=0;

    n = this->num_original_nodes;
    this->num_nodes=n;

    // First, create the initial routes:  VRPH_DEPOT - node - VRPH_DEPOT for all nodes

    routed[VRPH_DEPOT]=true;
    next_array[VRPH_DEPOT] = -1;
    for(i=1;i<=n;i++)
    {

        next_array[i] = -(i+1);
        total_route_length+= (d[VRPH_DEPOT][i] + d[i][VRPH_DEPOT]);

        route_num[i]=i;
        route[i].start=i;
        route[i].end=i;
        route[i].load= nodes[i].demand;
        route[i].length= d[VRPH_DEPOT][i] + d[i][VRPH_DEPOT];
        route[i].num_customers=1;
        routed[i]=true;    

    }


    next_array[n]=VRPH_DEPOT;

    route_num[VRPH_DEPOT]=0;
    
    // Now create the associated pred_array implied by the newly created next_array
    create_pred_array();

    total_number_of_routes=n;

    // Now eject the nodes that don't require service on this day
    for(i=1;i<=n;i++)
    {
        if(this->nodes[i].daily_demands[day]==-1)// indicates no service required
            this->eject_node(i);
    }

    // Now check for feasibility;
    this->normalize_route_numbers();

    // Check capacities
    for(i=1;i<=this->total_number_of_routes;i++)
    {
        if(route[i].load > max_veh_capacity)
            is_feasible=false;
        if(route[i].length > max_route_length)
            is_feasible=false;
    }

    if(is_feasible == false)
    { 
        // We had infeasible routes - record the violation in the Solution and 
        // return false;


        for(i=1;i<=n;i++)
        {
            routed[i]=false;    
            // Set routed status to false since default routes are infeasible

            // Check capacities
            if(route[i].load > max_veh_capacity)
            {
                // Update the worst violations

                printf("Default routes load violation: %d > %d\n",route[i].load,
                    max_veh_capacity);

                if( (route[i].load - max_veh_capacity) > 
                    violation.capacity_violation)
                {
                    violation.capacity_violation = 
                        (route[i].load - max_veh_capacity);
                }

            }
            if(route[i].length > max_route_length)
            {
                // Update the worst violations

                printf("Default routes length violation: %f > %f\n",route[i].length,
                    max_route_length);

                if( (route[i].length -max_route_length ) >
                    violation.length_violation)
                {
                    violation.length_violation = 
                        (route[i].length -max_route_length );
                }
            }
        }
        return false;
    }
    else
        return true;
    
}


int VRP::count_num_routes()
{
    ///
    /// Manually counts the # of routes in the current solution.
    ///

    int current,next;
    int num=0;

    current=VRPH_DEPOT;
    next=-1;

    while(next!=VRPH_DEPOT)
    {
        next= next_array[current];
        if(next<0)
        {
            // We're in a new route
            num++;
            current=-next;
        }
        else
            current=next;
    }

    return num;


}

bool VRP::perturb()
{
    ///
    /// Perturbs the current solution as suggested in Li, et al.
    ///

    int i, n, pre, post;
    int current,next;
    n= num_nodes;
    VRPNeighborElement *v;
    VRPMove M;

    Postsert Postsert;
    Presert  Presert;

    v=new VRPNeighborElement[n];

    current=VRPH_ABS(next_array[VRPH_DEPOT]);
    i=0;
    while(current!=VRPH_DEPOT)
    {
        pre= VRPH_MAX(VRPH_DEPOT,pred_array[current]);
        post= VRPH_MAX(VRPH_DEPOT, next_array[current]);

        // Define s[i]= d(pre(i),i) + d(i,next(i))-d(pre(i),next(i))
        v[i].val=((double) nodes[current].demand) / 
            (VRPH_EPSILON+d[pre][current]+d[current][post]-d[pre][post]);
        v[i].position=current;
        i++;
        next=VRPH_ABS(next_array[current]);
        current=next;
    }


    // Now sort the v[i]'s in ascending order of the val's

    qsort(v,n,sizeof(VRPNeighborElement), VRPNeighborCompare);

    int m=(int) (VRPH_MIN(30,n/5));
    

    // Now insert the first m nodes from the sorted list into new locations.

    int a,b,c,j,k,node1=0, node2=0,b_route,b_load,k_demand, jj, ll;
    double  best_savings,b_len,jk,kl,jl;


    for(j=0;j<m;j++)
    {
        best_savings=VRP_INFINITY;

        k=v[j].position;
        k_demand= nodes[k].demand;

        // Node k will be moved

        // Former neighboring nodes of k
        jj=VRPH_MAX(pred_array[k],0);
        ll=VRPH_MAX(next_array[k],0);

        jk= d[jj][k];
        kl= d[k][ll];
        jl= d[jj][ll];


        // Node k will be moved - look for a new location
        // OLD SITUATION:  jj-k-ll
        b=VRPH_ABS(next_array[VRPH_DEPOT]);
        while(b!=VRPH_DEPOT)
        {
            b_route= route_num[b];
            b_load= route[b].load;
            b_len= route[b].length;

            a=VRPH_MAX(pred_array[b],0);
            c=VRPH_MAX(next_array[b],0);

            if(a!=k && b!=k && c!=k)// This guarantees a new location
            {

                if(a!=VRPH_DEPOT)
                {
                    if(Postsert.evaluate(this,k,a,&M)==true && M.savings<best_savings)
                    {
                        best_savings=M.savings;
                        node1=a;node2=b;
                    }
                }
                else
                {

                    if(Presert.evaluate(this,k,b,&M)==true && M.savings<best_savings)
                    {
                        best_savings=M.savings;
                        node1=a;node2=b;
                    }



                }

                // Now try the edge b-c
                if(b!=VRPH_DEPOT)
                {
                    if(Postsert.evaluate(this,k,b,&M)==true && M.savings<best_savings)
                    {
                        best_savings=M.savings;
                        node1=b;node2=c;
                    }
                }
                else
                {
                    // b is the VRPH_DEPOT
                    if(Presert.evaluate(this,k,c,&M)==true && M.savings<best_savings)
                    {
                        best_savings=M.savings;
                        node1=b;node2=c;
                    }


                }

            }
            // Advance b
            b=VRPH_ABS(next_array[b]);

        }
        if(best_savings!=VRP_INFINITY)
        {
            // Now insert k in between node1 and node2
            if(node1!=VRPH_DEPOT)
                Postsert.move(this,k,node1);
            else
                Presert.move(this,k,node2);
        }

    }



    delete [] v;
    return true;        
}


bool VRP::eject_node(int j)
{
    ///
    /// This function removes node j from the current solution and
    /// adjusts the solution information appropriately. 
    ///

    if(j<=VRPH_DEPOT || routed[j]==false )
    {
        fprintf(stderr,"Tried to eject index %d\n",j);
        report_error("%s: VRPH_DEPOT or unrouted index used in eject node\n",__FUNCTION__);
    }


    int k=j;

    int c, e, k_route, c_route, e_route, k_start, k_end,flag;
    double change,ck,ke,ce;

    flag=0;

    // k is no longer routed
    routed[k]=false;
    
    // Get k's current location - between c and e
    
    c= pred_array[k];
    e= next_array[k];

    k_route= route_num[k];
    c_route= route_num[VRPH_ABS(c)];
    e_route= route_num[VRPH_ABS(e)];

    k_start= route[k_route].start;
    k_end= route[k_route].end;

    if(k_start==k && k_end!=k)
    {
        // c is in a different route
        route[k_route].start=VRPH_ABS(e);
        next_array[VRPH_ABS(c)]=-VRPH_ABS(e);
        pred_array[VRPH_ABS(e)]=-VRPH_ABS(c);
        
    }

    if(k_end==k && k_start!=k)
    {
        // e is in a different route
        route[k_route].end=c;
        next_array[VRPH_ABS(c)]=-VRPH_ABS(e);
        pred_array[VRPH_ABS(e)]=-VRPH_ABS(c);
    }

    if(k_end==k && k_start==k)
    {
        // k is its own route
        next_array[VRPH_ABS(c)]=-VRPH_ABS(e);
        pred_array[VRPH_ABS(e)]=-VRPH_ABS(c);
        flag=1;
    }

    if(k_start!=k && k_end!=k)
    {
        // k was interior--easy case
        next_array[c]=e;
        pred_array[e]=c;

    }



    // Now adjust the objective function value
    // and the route length and load, etc.

    // old route c-k-e
    // new route c-e

    if(e<0)
        e=VRPH_DEPOT;
    if(c<0)
        c=VRPH_DEPOT;

    ce= this->d[c][e];
    ck= this->d[c][k];
    ke= this->d[k][e];

    change=ce-(ck+ke);
    total_route_length+=change;
    route[k_route].length+=change;
    route[k_route].load-= nodes[k].demand;

    // Removing a node from route k_route
    route[k_route].num_customers--;
    num_nodes--;

    if(flag)
        // We removed a singleton route
        this->total_number_of_routes--;
    
    route_num[k]=-1;
    normalize_route_numbers();
    
    return true;
}

bool VRP::eject_route(int r, int *route_buff)
{
    ///
    /// Ejects all nodes from the current solution that are in route r.
    /// Places an ordered list of the ejected nodes in route_buff[].
    ///

    int current,cnt;

    // First copy the route into the buffer
    current=this->route[r].start;
    cnt=0;
    while(current>0)
    {
        route_buff[cnt]=current;
        cnt++;
        current=this->next_array[current];
    }
    route_buff[cnt]=-1;
    // Terminate with a -1

    

    // Now eject the nodes
    for(int i=0;i<cnt;i++)
        this->eject_node(route_buff[i]);
    
    
    return true;

}
bool VRP::check_move(VRPMove *M, int rules)
{

    ///
    /// Evaluates the move in terms of the rules.  Can consider savings,
    /// as well as other aspects of the VRPMove M.
    ///


    double savings;

    savings=M->savings;


    if(this->forbid_tiny_moves)
    {
        // See if it is a "meaningless" move
        if(savings>-VRPH_EPSILON && savings < VRPH_EPSILON)
            return false;
    }



    if( (rules & VRPH_FREE) == VRPH_FREE )
    {
        // use with care!
        return true;
    }


    

    if( (rules & VRPH_DOWNHILL) == VRPH_DOWNHILL)
    {
        if(savings<-VRPH_EPSILON )
            return true;
        else
            return false;

    }

    


    if( rules & VRPH_RECORD_TO_RECORD )
    {
        if(savings<=-VRPH_EPSILON)
            return true;

        // o/w savings is positive but must be less than deviation*record



        if(!has_service_times)
        {
            if(total_route_length+savings<= (1+deviation)*record)
                return true;
            else
                return false;

        }
        else
        {
            // We have service times so remove the service time from the
            // deviation calculation

            if( (total_route_length - total_service_time) + savings <=
                ((1+deviation)*(record-total_service_time)))
                return true;
            else
                return false;
        }
    }

    if( rules & VRPH_SIMULATED_ANNEALING )
    {
        if(M->evaluated_savings==true)
            return true;    // We already checked the random acceptance

        // Otherwise, determine whether to accept or not.
        if( exp(- (M->savings / this->temperature)) > lcgrand(10) )
            return true;
        else
            return false;

    }



    report_error("%s: didn't return yet!\n",__FUNCTION__);

    return false;
    

}


bool VRP::is_feasible(VRPMove *M, int rules)
{
    ///
    /// Determines whether a proposed move is feasible or not.
    /// Could add time window feasibility checks here, etc.  The rules
    /// is currently not used.
    ///

    for(int i=0;i<M->num_affected_routes;i++)
    {
        if( (M->route_lens[i]>this->max_route_length) || (M->route_loads[i]>this->max_veh_capacity) )
            return false;
    }
    
    return true;

}
bool VRP::inject_node(int j)
{
    ///
    /// Takes the node with index j that is currently NOT in the current solution and adds it
    /// to the VRP in the best possible feasible position.
    ///

    if(j==VRPH_DEPOT)
        report_error("%s: Can't inject VRPH_DEPOT!!\n",__FUNCTION__);

    int edge[4];
    double costs[4];
    this->find_cheapest_insertion(j,edge,costs,VRPH_USE_NEIGHBOR_LIST); 

    this->insert_node(j,edge[0],edge[1]);

    return true;

}

bool VRP::insert_node(int j, int i, int k)
{    
    ///
    /// Inserts j in between nodes i and k.  Both i and k
    /// are assumed to be routed while j is not routed.
    ///

    if(routed[j] || !routed[i] || !routed[k])
    {
        fprintf(stderr,"insert_node(%d,%d,%d)\n",j,i,k);
        report_error("%s: Improper nodes used in insert_node\n",__FUNCTION__);
    }

    double increase;
    int r;
    routed[j]=true;

    if(i==k && k==VRPH_DEPOT)
    {
        
        int last_node=VRPH_ABS(pred_array[VRPH_DEPOT]);
        this->next_array[last_node]=-j;
        this->pred_array[j]=-last_node;
        this->next_array[j]=VRPH_DEPOT;
        this->pred_array[VRPH_DEPOT]=-j;

        increase=this->d[0][j]+this->d[j][0];
        this->total_number_of_routes++;
        this->route_num[j]=this->total_number_of_routes;
        this->route[total_number_of_routes].length=increase;
        this->route[total_number_of_routes].load=nodes[j].demand;
        this->route[total_number_of_routes].num_customers=1;
        this->route[total_number_of_routes].start=j;
        this->route[total_number_of_routes].end=j;
        this->num_nodes++;
        this->total_route_length+=increase;

        return true;


        
    }

    if(i!=VRPH_DEPOT && k!=VRPH_DEPOT)
    {
        
        // i-k is interior edge
        if(VRPH_MAX(VRPH_DEPOT,next_array[i]) != k || VRPH_MAX(VRPH_DEPOT,pred_array[k])!=i)
        {
            fprintf(stderr,"edge doesn't exist: next(i(%d)) != k(%d)\n",i,k);
            report_error("%s\n",__FUNCTION__);
        }

        increase=d[i][j]+d[j][k]-d[i][k];
        r=route_num[i];


        num_nodes++;
        next_array[i]=j;
        pred_array[j]=i;
        next_array[j]=k;
        pred_array[k]=j;
        route_num[j]=r;
        route[r].length+=increase;
        route[r].load+= nodes[j].demand;
        route[r].num_customers++;
        total_route_length+=increase;

        return true;
    }

    if(i==VRPH_DEPOT)
    {

        increase=d[i][j]+d[j][k]-d[i][k];

        r=route_num[k];


        int pre=VRPH_ABS(pred_array[k]);

        // pre-VRPH_DEPOT-j-k situation...
        num_nodes++;

        next_array[pre]=-j;
        pred_array[j]=-pre;
        next_array[j]=k;
        pred_array[k]=j;
        route_num[j]=r;
        route[r].start=j;
        route[r].length+=increase;
        route[r].load+= nodes[j].demand;
        route[r].num_customers++;

        total_route_length+=increase;
        return true;

    }

    if(k==VRPH_DEPOT)
    {

        increase=d[i][j]+d[j][k]-d[i][k];
        r=route_num[i];

        int post=VRPH_ABS(next_array[i]);
        // i-j-VRPH_DEPOT-post situation...

        num_nodes++;

        next_array[i]=j;
        pred_array[j]=i;
        next_array[j]=-post;
        pred_array[post]=-j;
        route_num[j]=r;
        route[r].length+=increase;
        route[r].load+= nodes[j].demand;
        route[r].end=j;
        route[r].num_customers++;
        total_route_length+=increase;
        return true;

    }

    return false;


}

void VRP::find_cheapest_insertion(int j, int *edge, double *costs, int rules)
{
    ///
    /// Finds the cheapest insertion of node j into the current solution.
    /// We store both the best feasible insertion as well as the best overall 
    /// insertion.
    /// The value of edge[0] is the start node of the best feasible edge to be broken and edge[1]
    /// is the end node.  Similarly, edge[2] and edge[3] represent the start and end of the
    /// best overall edge, disregarding feasibility (may be the same as edge[0] and edge[1]).  
    /// The costs of these insertions are placed
    /// in costs[0] and costs[1].  If rules==VRPH_USE_NEIGHBOR_LIST, then only the neighbor 
    /// list (plus the VRPH_DEPOT to guarantee a singleton route) is searched.  If 
    /// rules==VRPH_NO_NEW_ROUTE, then we do not allow the customer to be added in a new 
    /// singleton route.
    ///

    int h,i,k,m, best_route, new_route, next_node;
    double ij,ik,jk,min_feasible_increase, increase, min_increase;

    this->normalize_route_numbers();
    best_route = -1;
    k=-1;

    // Set the increase to be a singleton route since this must be feasible

    min_feasible_increase = this->d[VRPH_DEPOT][j] + this->d[j][VRPH_DEPOT];
    min_increase = this->d[VRPH_DEPOT][j] + this->d[j][VRPH_DEPOT];
    edge[0]=edge[1]=edge[2]=edge[3]=VRPH_DEPOT;


    if(!(rules & VRPH_USE_NEIGHBOR_LIST))
    {
        i=VRPH_DEPOT;
        next_node=VRPH_ABS(next_array[i]);
        int cnt=0;
        while(next_node != VRPH_DEPOT)
        {
            cnt++;
            if(next_node > 0)
            {
                k=next_node;

                // Try to put j between i and k

                ij= d[i][j];
                ik= d[i][k];
                jk= d[j][k];
                increase=ij+jk-ik;

                if(increase<min_increase)
                {
                    min_increase=increase;
                    edge[2]=i;
                    edge[3]=k;
                }
                if(increase<min_feasible_increase)
                {
                    // Check feasibility
                    new_route= route_num[k];

                    if( (route[new_route].length+increase <= max_route_length) &&
                        (route[new_route].load + nodes[j].demand <= max_veh_capacity) )
                    {
                        edge[0]=i;
                        edge[1]=k;
                        best_route=new_route;
                        min_feasible_increase=increase;

                    }

                }
                // Move on to the next node.
                i=k;

            }
            else
            {
                // next_node <= 0, implying the edges i-VRPH_DEPOT, VRPH_DEPOT-VRPH_ABS(next_node)
                // First, consider i-VRPH_DEPOT
                k=VRPH_DEPOT;

                ij= d[i][j];
                ik= d[i][k];
                jk= d[j][k];
                increase=ij+jk-ik;

                if(increase<min_increase)
                {
                    min_increase=increase;
                    edge[2]=i;
                    edge[3]=k;
                }

                if(increase<min_feasible_increase)
                { 
                    // Check feasibility
                    new_route= route_num[i];

                    if( (route[new_route].length+increase <= max_route_length) &&
                        (route[new_route].load + nodes[j].demand <= max_veh_capacity) )
                    {
                        edge[0]=i;
                        edge[1]=k;
                        best_route=new_route;
                        min_feasible_increase=increase;
                    }
                }

                // Now consider the edge VRPH_DEPOT-VRPH_ABS(next_node)
                i=VRPH_DEPOT;
                k=VRPH_ABS(next_node);

                ij= d[i][j];
                ik= d[i][k];
                jk= d[j][k];
                increase=ij+jk-ik;

                if(increase<min_increase)
                {
                    min_increase=increase;
                    edge[2]=i;
                    edge[3]=k;
                }

                if(increase<min_feasible_increase)
                {
                    new_route= route_num[k];

                    if( (route[new_route].length+increase <= max_route_length )&&
                        (route[new_route].load + nodes[j].demand <= max_veh_capacity ) )
                    {
                        edge[0]=i;
                        edge[1]=k;
                        best_route=new_route;
                        min_feasible_increase=increase;
                    }

                }
                i=k;
            }


            next_node= next_array[i];

        }
        // Consider the final edge
        k=VRPH_DEPOT;
        ij= d[i][j];
        ik= d[i][k];
        jk= d[j][k];
        increase=ij+jk-ik;

        if(increase<min_increase)
        {
            min_increase=increase;
            edge[2]=i;
            edge[3]=k;
        }

        if(increase<min_feasible_increase)
        { 
            // Check feasibility
            new_route= route_num[i];

            if( (route[new_route].length+increase <= max_route_length) &&
                (route[new_route].load + nodes[j].demand <= max_veh_capacity) )
            {
                edge[0]=i;
                edge[1]=k;
                best_route=new_route;
                min_feasible_increase=increase;
            }
        }

        costs[0]=min_feasible_increase;
        costs[1]=min_increase;
        return;

    }
    else
    {
        // Only search the neighbor list for positions
        // We already computed the VRPH_DEPOT-j-VRPH_DEPOT position.
        for(m=0;m<neighbor_list_size;m++)
        {
            i=nodes[j].neighbor_list[m].position;
            if(routed[i])
            {
                h=VRPH_MAX(VRPH_DEPOT,pred_array[i]);
                increase=d[h][j]+d[j][i]-d[h][i];            

                if(increase<min_increase)
                {
                    min_increase=increase;
                    edge[2]=h;
                    edge[3]=i;
                }

                if(increase<min_feasible_increase)
                { 
                    // Check feasibility
                    if(i!=VRPH_DEPOT)
                        new_route= route_num[i];
                    else
                        new_route= route_num[k];


                    if( (route[new_route].length+increase <= max_route_length) &&
                        (route[new_route].load + nodes[j].demand <= max_veh_capacity) )
                    {
                        edge[0]=h;
                        edge[1]=i;
                        best_route=new_route;
                        min_feasible_increase=increase;
                    }
                }

                k=VRPH_MAX(VRPH_DEPOT,next_array[i]);
                increase=d[i][j]+d[j][k]-d[i][k];    

                if(increase<min_increase)
                {
                    min_increase=increase;
                    edge[2]=i;
                    edge[3]=k;
                }

                if(increase<min_feasible_increase)
                { 
                    // Check feasibility
                    if(i!=VRPH_DEPOT)
                        new_route= route_num[i];
                    else
                        new_route= route_num[k];


                    if( (route[new_route].length+increase <= max_route_length) &&
                        (route[new_route].load + nodes[j].demand <= max_veh_capacity) )
                    {
                        edge[0]=i;
                        edge[1]=k;
                        best_route=new_route;
                        min_feasible_increase=increase;
                    }
                }
            }
        }

        costs[0]=min_feasible_increase;
        costs[1]=min_increase;
        return;
    }
}
int VRP::inject_set(int num, int *nodelist, int rules, int attempts)
{
    ///
    /// Injects num different nodes in the nodelist[] array into the current solution.
    /// If rules=VRPH_RANDOM_SEARCH, then we try attempts different random permutations.
    /// If rules=VRPH_REGRET_SEARCH, then we try attempts different permutations
    /// and can backtrack by undoing certain moves that we end up regretting.
    ///

    if(rules != VRPH_RANDOM_SEARCH && rules != VRPH_REGRET_SEARCH)
        report_error("%s: invalid rules\n",__FUNCTION__);

    int i,j,k;
    double best_obj=VRP_INFINITY;
    int *best_sol, *orderings,*best_ordering, *start_sol;
    int best_index=0;

    best_sol=orderings=best_ordering=start_sol=NULL;

    for(i=0;i<num;i++)
    {
        if(nodelist[i]==VRPH_DEPOT)
        {
            fprintf(stderr,"nodelist[%d] of %d=VRPH_DEPOT\n",i,num);
            report_error("%s: Cannot inject VRPH_DEPOT!\n",__FUNCTION__);
        }
    }

    best_sol=new int[3+(this->num_nodes)+num];//!!! The eventual sol_buff is larger !!!!
    start_sol=new int[3+(this->num_nodes)+num];
    this->export_solution_buff(start_sol);
    this->import_solution_buff(start_sol);
    
    if(rules==VRPH_RANDOM_SEARCH)
    {
        // Try "attempts" different random orderings to inject the nodes
        orderings=new int[num];
        best_ordering=new int[num];
        int edge[4];
        double costs[2];
        for(i=0;i<num;i++)
            orderings[i]=i;
        
        for(i=0;i<attempts;i++)
        {
            // Create a random permutation
            random_permutation(orderings,num);
            for(j=0;j<num;j++)
            {
                if(nodelist[orderings[j]]==VRPH_DEPOT)
                {
                    fprintf(stderr,"inject_set: Bizarre nodelist[%d]:\n",j);
                    for(k=0;k<num;k++)
                        fprintf(stderr,"(%d,%d) ",orderings[k],nodelist[orderings[k]]);

                    report_error("%s: Found VRPH_DEPOT in nodelist!\n",__FUNCTION__);
                }
                this->inject_node(nodelist[orderings[j]]);
            }

            // Look for a new best
            // Look for a new best
            if(this->total_route_length < best_obj && VRPH_ABS(this->total_route_length-best_obj)>VRPH_EPSILON)
            {    
                best_index=i;
                best_obj=this->total_route_length;
                this->export_solution_buff(best_sol);
                memcpy(best_ordering,orderings,num*sizeof(int));
            }

            // Now return to the original solution...
            for(j=0;j<num;j++)
                this->eject_node(nodelist[orderings[j]]);
        }
        // Now revisit the best ordering found and try to improve it!
        for(j=0;j<num;j++)
        {
            this->find_cheapest_insertion(nodelist[best_ordering[j]],edge,costs,VRPH_USE_NEIGHBOR_LIST);
            this->insert_node(nodelist[best_ordering[j]],edge[0],edge[1]);
        }        
    }

    if(rules==VRPH_REGRET_SEARCH)
    {
        // Try "attempts" different random orderings to inject the nodes
        orderings=new int[num];
        best_ordering=new int[num];

        int *current_list;
        current_list=new int[num];
        int edge[4];
        double costs[2];
        for(i=0;i<num;i++)
            orderings[i]=i;
        
        for(i=0;i<attempts;i++)
        {
            // Create a random permutation
            random_permutation(orderings,num);
            for(j=0;j<num;j++)
                current_list[j]=nodelist[orderings[j]];

            // It is unfortunately possible to enter some cycles here - this is a cheap
            // way of getting out
            int cycle_ctr=0;
            for(j=0;j<num;j++)
            {
                cycle_ctr++;
                if(cycle_ctr==VRPH_MAX_CYCLES)
                {
                    fprintf(stderr,"Cycle encountered in REGRET SEARCH!\nReverting to original solution\n");
                    if(best_obj<VRP_INFINITY)
                    {
                        this->import_solution_buff(best_sol);
                        delete [] best_sol;
                        delete [] orderings;
                        delete [] best_ordering;
                        delete [] start_sol;
                        delete [] current_list;
                        return 0;
                    }
                    else
                        report_error("%s: Couldn't escape cycle in REGRET search!\n",__FUNCTION__);
                }
                
                // Find the cheapest way to insert the present node
                this->find_cheapest_insertion(current_list[j],edge,costs,VRPH_USE_NEIGHBOR_LIST);
                // Now loop over the nodes we already injected and compute their ejection costs
                double max_ejection_cost=-VRP_INFINITY;
                double current_cost;
                int node_to_eject=-1;
                for(k=0;k<j;k++)
                {
                    // Make sure the node we are testing isn't involved in the cheapest insertion
                    if(current_list[k]!=edge[0] && current_list[k]!=edge[1])
                    {
                        if(( current_cost = this->ejection_cost(current_list[k])) > max_ejection_cost)
                        {
                            max_ejection_cost=current_cost;
                            node_to_eject=current_list[k];
                            if(!routed[node_to_eject])
                            {
                                fprintf(stderr,"%d NOT ROUTED!!\n",node_to_eject);
                                report_error("%s: Error in inject_set\n",__FUNCTION__);
                            }
                        }
                    }
                }

                if(node_to_eject==-1 || costs[0]>= max_ejection_cost)
                {
                    // Insert
                    this->insert_node(current_list[j],edge[0],edge[1]);
                }
                else
                {
                    // Eject and Insert
                    this->eject_node(node_to_eject);
                    this->insert_node(current_list[j],edge[0],edge[1]);
                    current_list[j]=node_to_eject;
                    j--;
                }        
            }

            // Look for a new best
            if(this->total_route_length < best_obj && VRPH_ABS(this->total_route_length-best_obj)>VRPH_EPSILON)
            {    
                best_index=i;
                best_obj=this->total_route_length;
                this->export_solution_buff(best_sol);
                memcpy(best_ordering,orderings,num*sizeof(int));
            }

            // Now return to the original solution...
            this->import_solution_buff(start_sol);
        }
        delete [] current_list;
    }

    this->import_solution_buff(best_sol);

    delete [] best_sol;
    delete [] orderings;
    delete [] best_ordering;
    delete [] start_sol;


    return best_index;
}

void VRP::eject_neighborhood(int j, int num, int *nodelist)
{
    ///
    /// Ejects num different randomly selected nodes that are in the vicinity
    /// of node j, placing the list of ejected nodes in the nodelist[] array.
    /// Node j is also ejected!  The candidates for ejection are randomly chosen
    /// from the 2*num nearest neighbors of j.
    ///

    int i,k,cnt;
    int *ejected;

    ejected=new int[this->num_nodes+1];
    memset(ejected,0,(this->num_nodes+1)*sizeof(int));

    cnt=0;
    
    // Always eject j
    nodelist[cnt]=j;
    ejected[j]=1;
    cnt++;

    i=0;
    while(cnt<num)
    {
        // Find a node near j to eject
        i=(int)(lcgrand(17)*2*num);
        
        // Grab it from the neighborlist
        i=VRPH_MIN(MAX_NEIGHBORLIST_SIZE-1,i);

        // Get the index
        k=this->nodes[j].neighbor_list[i].position;
        if(ejected[k]==0)
        {
            if(k!=VRPH_DEPOT)
            {
                nodelist[cnt]=k;
                ejected[k]=1;
                cnt++;
            }
        }
            
    }

    // We now have cnt nodes to eject in nodelist
    for(i=0;i<cnt;i++)
    {
        if(nodelist[i]==VRPH_DEPOT)
        {
            fprintf(stderr,"Trying to eject VRPH_DEPOT??? j=%d\n",j);
            report_error("%s: Error in eject_neighborhood\n",__FUNCTION__);
        }

        this->eject_node(nodelist[i]);
    }

#if 0
    this->verify_routes("After ejecting neighborhood\n");
#endif
    delete [] ejected;

    return;
}




void VRP::normalize_route_numbers()
{
    ///
    /// Renumbers the routes so that with R total routes, they
    /// are numbered 1,2, ..., R instead of all over the place
    /// as they typically are after Clarke Wright.
    ///

    int current, end, n, R, i;
    int current_route;
    int *indices;
    R= count_num_routes();
    n= this->num_original_nodes;


    // First check to see if we are already normalized

    // Make the arrays 1-based to avoid insanity...
    indices=new int[n+1];

    for(i=0;i<=n;i++)
        indices[i]=1;

    int ctr=0;
    i=VRPH_ABS(next_array[VRPH_DEPOT]);
    while(i!=VRPH_DEPOT)
    {
        if(route_num[i] <= R && routed[i]==true)
        {
            // route_num[i] is less than R, so we have to
            // avoid this index.
            indices[route_num[i]]=0;
        }
        else
            ctr++;

        i=VRPH_ABS(next_array[i]);
    }
    
    if(ctr==0)
    {
        delete [] indices;
        return;// route numbers are already normalized since no route_nums were >R.
    }

    // Now all of the available indices that are less than or equal to R
    // are marked with a 1. The unavailable indices are set to zero.

    int next_index=1;
    while(indices[next_index]==0)
        next_index++;

    // So next_index is the first index available for use.

    // Get the first route and change the information
    current=VRPH_ABS(next_array[VRPH_DEPOT]);

    while(current!=VRPH_DEPOT)
    {
        current_route= route_num[current];
        end= route[current_route].end;

        // Get the next available index
        while(indices[next_index]==0)
            next_index++;

        // Modify the current index if necessary
        if(current_route>R)
        {
            // Update the route_num
            i=VRPH_ABS(next_array[VRPH_DEPOT]);
            while(i!=VRPH_DEPOT)
            {
                if(route_num[i]==current_route)
                    route_num[i]=next_index;
                i=VRPH_ABS(next_array[i]);

            }
            // Update the route information
            route[next_index].start = route[current_route].start;
            route[next_index].end = route[current_route].end;
            route[next_index].length = route[current_route].length;
            route[next_index].load = route[current_route].load;
            route[next_index].num_customers = route[current_route].num_customers;
            
            next_index++;


        }

        // Set current to be the first node in the next route
        current=VRPH_ABS(next_array[end]);


    }

    delete [] indices;

    return;



}


bool VRP::create_search_neighborhood(int j, int rules)
{
    ///
    /// Creates the search_size and search_space fields for the
    /// current VRP in terms of the given node j and the rules.
    ///

    int i,k, cnt;
    // Define the search space
    

    if( rules & VRPH_USE_NEIGHBOR_LIST )
    {

        // Search only those nodes that are in the neighbor list
        search_size=0;
        cnt=0;
        for(i=0;i<neighbor_list_size;i++)
        {
            // Consider node k
            k=nodes[j].neighbor_list[i].position;
            if( routed[k] == true)
            {
                // The node is routed 
                if(rules & VRPH_INTER_ROUTE_ONLY)
                {
                    // Make sure k is in a different route
                    if(route_num[k]!=route_num[j])
                    {
                        search_space[cnt] = k;
                        cnt++;
                        
                    }
                }
                else
                {
                    if(rules & VRPH_INTRA_ROUTE_ONLY)
                    {
                        // Make sure k is in the same route

                        if(route_num[k]==route_num[j])
                        {
                            // The node is in a different route - add to the search space
                            search_space[cnt] = k;
                            cnt++;
                            
                        }
                    }
                    else
                    {
                        // No intra/inter restrictions
                        search_space[cnt] = k;
                        cnt++;
                        
            
                    }
                }
            }
        }
        search_size=cnt;
        goto randomize;
    }


    // Otherwise no neighborlist
    if( (rules & VRPH_INTRA_ROUTE_ONLY) )
    {        
        // Search_space is just the route itself
        search_space[0]=VRPH_DEPOT;
        search_space[1]=this->route[route_num[j]].start;
        for(i=2;i<=this->route[route_num[j]].num_customers;i++)
            search_space[i]=this->next_array[search_space[i-1]];

        search_size=this->route[route_num[j]].num_customers+1;//add 1 for depot

        goto randomize;

        /*

        search_size=0;
        i=0;
        search_space[i]=VRPH_DEPOT;
        i++;
        search_space[i]=route[route_num[j]].start;
        for(;;)
        {
            search_space[i]=next_array[search_space[i]];
            if(search_space[i]<=VRPH_DEPOT)
            {
                search_size=i;
                goto randomize;
            }
            i++;
        }*/

    }

    if( (rules & VRPH_INTER_ROUTE_ONLY) )
    {        
        // Only nodes in a different route        
        i=0;
        search_space[i]=VRPH_DEPOT;
        search_size=1;
        k=VRPH_ABS(next_array[search_space[i]]);    
        i++;

        for(;;)
        {
            if(k==VRPH_DEPOT)
                // end of solution
                goto randomize;

            if(route_num[k]!=route_num[j])
            {
                search_space[i]=k;
                i++;
                search_size++;
            }
            k=VRPH_ABS(next_array[k]);
        }

    }

    
    // Search space will consist of all positions in routes moving in the
    // given direction

    if((rules & VRPH_FORWARD))
    {
        search_size=0;

        i=0;
        search_space[i]=VRPH_ABS(next_array[route[route_num[j]].end]);
        search_size++;

        // First node in next route in the solution
        for(;;)
        {
            search_space[i+1]=VRPH_ABS(next_array[search_space[i]]);    
            search_size++;
            if(search_space[i+1]==VRPH_DEPOT)
                goto randomize;
            i++;

        }

    }

    if((rules & VRPH_BACKWARD))
    {
        search_size=0;

        i=0;
        search_space[i]=VRPH_ABS(pred_array[route[route_num[j]].start]);
        search_size++;

        // Last node in previous route in the solution
        for(;;)
        {
            search_space[i+1]=VRPH_ABS(pred_array[search_space[i]]);    
            search_size++;
            if(search_space[i+1]==VRPH_DEPOT)
                goto randomize;
            i++;

        }
    
    }

    // No rules given - search space is set of all nodes
    search_size=0;
    i=0;

    search_space[i]=VRPH_ABS(next_array[VRPH_DEPOT]);
    search_size++;

    for(;;)
    {
        search_space[i+1]=VRPH_ABS(next_array[search_space[i]]);    
        search_size++;
        if(search_space[i+1]==VRPH_DEPOT)
            goto randomize;
        i++;
    }

randomize:

    // Now permute it if the rules is VRPH_RANDOMIZED
    if(rules & VRPH_RANDOMIZED)
        random_permutation(search_space, search_size);

    return true;
    


}


double VRP::insertion_cost(int u, int a, int b)
{
    ///
    /// Returns the increased cost to the route containing a-b of
    /// inserting node u in between a and b (assumed to be adjacent!)
    ///

    // a-b is an existing edge
    // u is the node we are trying to insert between a and b
    // Returns the increased cost to the route containing a-b of
    // inserting u in between a and b

    if(a==b && b==VRPH_DEPOT)
        return d[VRPH_DEPOT][u]+d[u][VRPH_DEPOT];// assume feasible!!
    
    if(a==u || u==b || u==VRPH_DEPOT) 
        report_error("%s: overlap or VRPH_DEPOT found\n",__FUNCTION__);

    if(routed[a]==false || routed[b]==false)
    {
        fprintf(stderr,"%d,%d:  not routed!\n",a,b);
        report_error("%s: Unrouted nodes in insertion_cost\n",__FUNCTION__);
    }

    if( (a!=VRPH_DEPOT) && VRPH_MAX(next_array[a],VRPH_DEPOT)!=b)
    {
        fprintf(stderr,"%d,%d:  not an edge!\n",a,b);
        report_error("%s: Invalid nodes in insertion_cost\n",__FUNCTION__);
    }

    if( (a==VRPH_DEPOT) && VRPH_MAX(pred_array[b],VRPH_DEPOT)!=a)
    {
        fprintf(stderr,"%d,%d:  not an edge!\n",a,b);
        report_error("%s: Invalid nodes in insertion_cost\n",__FUNCTION__);
    }


    int new_route;
    if(a==VRPH_DEPOT)
        new_route=route_num[b];
    else
        new_route=route_num[a];

    if(nodes[u].demand+route[new_route].load>max_veh_capacity)
        return VRP_INFEASIBLE;
    
    double increase=d[a][u] + d[u][b] - d[a][b];
    if(route[new_route].length+increase>max_route_length)
        return VRP_INFEASIBLE;

    return increase;

}



double VRP::ejection_cost(int u)
{
    ///
    /// Returns the reduction in the objective function value obtained by
    /// ejecting u from the current solution.
    ///

    if( u==VRPH_DEPOT)
        report_error("%s: Cannot eject the VRPH_DEPOT!\n",__FUNCTION__);

    if(!routed[u])
        return -VRP_INFINITY;

    // t-u-v --> t-v

    return this->d[VRPH_MAX(pred_array[u],VRPH_DEPOT)][u]+d[u][VRPH_MAX(next_array[u],VRPH_DEPOT)] -
        d[VRPH_MAX(pred_array[u],VRPH_DEPOT)][VRPH_MAX(next_array[u],VRPH_DEPOT)];


}
void VRP::clean_route(int r, int heuristics)
{
    ///
    /// Runs the provided set of heuristics on route r
    /// until a local minimum is reached.
    ///

    int i,j, r_start, r_end;
    double start_val, end_val, start_rlen, end_rlen;

    OnePointMove    OPM;
    TwoPointMove    TPM;
    TwoOpt            TO;
    ThreeOpt        ThreeO;
    OrOpt            OrO;

    int rules= VRPH_INTRA_ROUTE_ONLY+VRPH_DOWNHILL+VRPH_FIRST_ACCEPT+VRPH_SAVINGS_ONLY;


start_improving:

    start_val = route[r].length;

#if CLEAN_DEBUG
    printf("CLEAN::start_val=%f\n",start_val);
#endif

    end_val = - VRP_INFINITY;
    r_start=route[r].start;
    r_end=route[r].end;

    if((heuristics & ONE_POINT_MOVE)==ONE_POINT_MOVE)
    {
        // Try the ONE_POINT_MOVE for all nodes in route r;
        r_start=route[r].start;

opm_search:
        start_rlen=route[r].length;
        i=r_start;
        j=VRPH_MAX(next_array[i],0);
        while(i!=VRPH_DEPOT)
        {
            while(OPM.search(this,i,rules));
            i=j;
            j=VRPH_MAX(next_array[j],0);
        }

        end_rlen = route[r].length;
        if( (end_rlen<start_rlen) && (VRPH_ABS(end_rlen-start_rlen)>VRPH_EPSILON))
            goto opm_search;

#if CLEAN_DEBUG
        printf("CLEAN::OPM end_rlen=%f\n",end_rlen);
#endif

    }

    if((heuristics & TWO_POINT_MOVE)==TWO_POINT_MOVE)
    {
        // Try the TWO_POINT_MOVE for all nodes in route r;
tpm_search:
        r_start=route[r].start;
        start_rlen=route[r].length;
        i=r_start;
        j=VRPH_MAX(next_array[i],0);
        while(i!=VRPH_DEPOT)
        {
            while(TPM.search(this,i,rules));
            i=VRPH_MAX(next_array[i],VRPH_DEPOT);
        }

        end_rlen = route[r].length;
        if( (end_rlen<start_rlen) && (VRPH_ABS(end_rlen-start_rlen)>VRPH_EPSILON))
            goto tpm_search;

#if CLEAN_DEBUG
        printf("CLEAN::TPM end_rlen=%f\n",end_rlen);
#endif


    }

    if((heuristics & TWO_OPT)==TWO_OPT)
    {

to_search:
        r_start=route[r].start;
        start_rlen=route[r].length;
        i=r_start;
        j=VRPH_MAX(next_array[i],0);
        while(i!=VRPH_DEPOT)
        {
            while(TO.search(this,i,rules));
            i=VRPH_MAX(next_array[i],0);
        }

        end_rlen = route[r].length;
        if( (end_rlen<start_rlen) && (VRPH_ABS(end_rlen-start_rlen)>VRPH_EPSILON))
            goto to_search;

#if CLEAN_DEBUG
        printf("CLEAN::TO end_rlen=%f\n",end_rlen);
#endif


    }

    if((heuristics & OR_OPT)==OR_OPT)
    {
        // Use strings of length 2 and 3
or_search:
        r_start=route[r].start;
        start_rlen=route[r].length;
        i=r_start;
        j=VRPH_MAX(next_array[i],0);
        while(i!=VRPH_DEPOT)
        {
            OrO.search(this,i,3,rules);
            i=VRPH_MAX(next_array[i],0);
        }

        i=r_start;
        j=VRPH_MAX(next_array[i],0);
        while(i!=VRPH_DEPOT)
        {
            OrO.search(this,i,2,rules);
            i=VRPH_MAX(next_array[i],0);
        }
        end_rlen = route[r].length;
        if( (end_rlen<start_rlen) && (VRPH_ABS(end_rlen-start_rlen)>VRPH_EPSILON))
            goto or_search;

#if CLEAN_DEBUG
        printf("CLEAN::TO end_rlen=%f\n",end_rlen);
#endif

    }

    if((heuristics & THREE_OPT)==THREE_OPT)
    {
        //show_route(r);

        while(ThreeO.route_search(this,r,rules))

            end_val = route[r].length;
#if CLEAN_DEBUG
        printf("CLEAN::3O end_val=%f\n",end_val);
#endif


    }

    end_val= route[r].length;

#if CLEAN_DEBUG
    printf("CLEAN::final end_val=%f\n",end_val);
#endif


    if(VRPH_ABS(start_val-end_val)>VRPH_EPSILON)
        goto start_improving;
    else
        return;

}

bool VRP::before(int a, int b)
{
    ///
    /// This function returns TRUE if a comes before b in their route
    /// and FALSE if b is before a. An error is reported if a and b are in different routes.
    /// Should be used sparingly as it loops and can be slow for large routes.
    ///

    int i;

    if(a==VRPH_DEPOT || b==VRPH_DEPOT)
        report_error("%s: before called with VRPH_DEPOT\n",__FUNCTION__);
    
    if(route_num[a]!=route_num[b])
    {
        fprintf(stderr,"Ordering error: before called with %d and %d not in the same route!\n",a,b);
        report_error("%s: differnet routes\n",__FUNCTION__);
    }

    if(next_array[a]==b)
        return true;
    if(next_array[b]==a)
        return false;

    i=a;
    while(i>0 && i!=b)
        i=next_array[i];

    // At the end of this loop, if i<=0, then we're at the end of a route
    // and haven't encountered b ==> must have a after b in the route
    // If i==b, then we know that b follows a

    if(i==b)
        return true;
    else
        return false;

}


void VRP::update(VRPMove *M)
{
    ///
    /// Updates the solution information in terms of the move M.
    ///

    if(M->num_affected_routes==0)    // Nothing to do!
        return;    

    int i;

    for(i=0; i < M->num_affected_routes; i++)
    {
        // Update length
        route[M->route_nums[i]].length = M->route_lens[i];

        // Update load
        route[M->route_nums[i]].load = M->route_loads[i];

        // Update # of customers
        route[M->route_nums[i]].num_customers = M->route_custs[i];
    }

    // Now update total_route_length
    total_route_length = M->new_total_route_length;

    // # of routes
    total_number_of_routes = M->total_number_of_routes; 

    return;

}




void VRP::compute_route_center(int r)
{
    ///
    /// Computes the mean x and y coord's of the nodes in a route,
    /// storing the information in the VRPRoute[] array
    ///

    int current_node = route[r].start;
    double total_x=0;
    double total_y=0;

    while(current_node != VRPH_DEPOT)
    {
        total_x += nodes[current_node].x;
        total_y += nodes[current_node].y;
        current_node=VRPH_MAX(VRPH_DEPOT, next_array[current_node]);
    }

    route[r].x_center = total_x / ((double) route[r].num_customers);
    route[r].y_center = total_y / ((double) route[r].num_customers);    

}
void VRP::find_neighboring_routes()
{
    ///
    /// Finds the nearest set of neighboring routes, placing
    /// the corresponding set of route numbers in each route's
    /// neighboring_routes[] array.  
    ///

    int i,j;
    VRPNeighborElement **rd;

    // Compute the route centers
    normalize_route_numbers();
    for(i=1;i<=total_number_of_routes;i++)
    {
        compute_route_center(i);
    }

    // Create the matrix
    rd = new VRPNeighborElement *[total_number_of_routes + 1];
    rd[0]= new VRPNeighborElement[(total_number_of_routes +1) * (total_number_of_routes + 1)];
    for(i=1;i<total_number_of_routes+1;i++)
        rd[i]=rd[i-1] + (total_number_of_routes+1);

    // Fill the matrix with inter-route distances
    for(i=1;i<=this->total_number_of_routes;i++)
    {
        rd[i][0].position=VRP_INFINITY;
        rd[i][0].val=VRP_INFINITY;
        for(j=1;j<=this->total_number_of_routes;j++)
        {
            rd[i][j].position = j;
            rd[i][j].val = VRPDistance(VRPH_EUC_2D,route[i].x_center, route[i].y_center,
                route[j].x_center,route[j].y_center);
        }
    }

    // Now sort each row in put the top MAX_NEIGHBORING_ROUTES in the route structs.

    
    for(i=1;i<=total_number_of_routes;i++)
        qsort(rd[i],total_number_of_routes+1,sizeof(VRPNeighborElement),VRPNeighborCompare);
    
    

    for(i=1;i<=total_number_of_routes;i++)
    {
        for(j=0;j<MAX_NEIGHBORING_ROUTES;j++)
        {
            route[i].neighboring_routes[j]=rd[i][j+1].position;
            // don't include the route itself!
        }
    }

    delete [] rd[0];
    delete [] rd;

    

    return;
}

void VRP::capture_best_solution()
{
    ///
    /// Determines if the current solution is the best found so far.
    ///

    if( (this->total_route_length < this->best_total_route_length) && 
        (VRPH_ABS(this->total_route_length - this->best_total_route_length) > VRPH_EPSILON) )
    {
        this->best_total_route_length=this->total_route_length;
        this->export_solution_buff(this->best_sol_buff);
        
    }

    
    if(this->total_route_length < this->solution_wh->worst_obj || 
        this->solution_wh->num_sols < this->solution_wh->max_size)
    {
        VRPSolution this_sol(this->num_nodes);

        this_sol.obj=this->total_route_length;
        this_sol.in_IP=false;

        // Export buffer
        this->export_canonical_solution_buff(this_sol.sol);
        
        this->solution_wh->add_sol(&this_sol, 0); 
        // We don't know any information to help us know where to insert
    }

    return;

    
}

void VRP::update_solution_wh()
{
    ///
    /// Attempts to add the given solution to the warehouse.
    ///

    VRPSolution this_sol(this->num_nodes);

    this_sol.obj=this->total_route_length;
    this_sol.in_IP=false;

    // Export buffer
    this->export_canonical_solution_buff(this_sol.sol);

    this->solution_wh->add_sol(&this_sol, 0); 
    // We don't know any information to help us know where to insert
    

    return ;


}


void VRP::update_route(int j, VRPRoute *R)
{
    ///
    /// Copies the fields of route j in the current solution to the
    /// VRPRoute R, also updating the ordering, x, and y arrays.
    /// The ordering is normalized by having start<end.  
    ///

    int i, current;
    double st=0;// for the service time

    R->x[0]=this->nodes[VRPH_DEPOT].x;
    R->y[0]=this->nodes[VRPH_DEPOT].y;
    R->start=route[j].start;
    R->end=route[j].end;
    R->num_customers=route[j].num_customers;
    R->load=route[j].load;
    R->length=route[j].length;
    R->obj_val=this->total_route_length-this->total_service_time;


    if(R->start < R->end)
    {
        current=R->start;
        // Just output the normal ordering
        R->ordering[0]=current;
        R->x[1]=this->nodes[current].x;
        R->y[1]=this->nodes[current].y;
        st+=this->nodes[current].service_time;

        for(i=1; i<R->num_customers; i++)
        {
            current=this->next_array[current];
            st+=this->nodes[current].service_time;
            R->ordering[i]=current;
            R->x[i+1]=this->nodes[current].x;
            R->y[i+1]=this->nodes[current].y;
        }

        R->total_service_time=st;
        
        return;
    }

    // Otherwise start > end - store the reverse
    current=R->end;
    R->ordering[0]=current;
    R->x[1]=this->nodes[current].x;
    R->y[1]=this->nodes[current].y;
    st+=this->nodes[current].service_time;


    for(i=1; i<R->num_customers; i++)
    {
        current=this->pred_array[current];
        st+=this->nodes[current].service_time;
        R->ordering[i]=current;
        R->x[i+1]=this->nodes[current].x;
        R->y[i+1]=this->nodes[current].y;

    }

    R->total_service_time=st;

    return;

}



double VRP::split(double p)
{
    ///
    /// Splits an existing VRP into two parts by drawing a random
    /// ray from the VRPH_DEPOT.  We repeatedly try to split the problem in this way
    /// until we have two sets of nodes that each has k nodes where k is between p*num_nodes and
    /// (1-p)*num_nodes.  The value of p must be less than .5.  Returns the angle theta used
    /// to split the problem
    ///

    if(p>.5)
        report_error("%s: p must be less than .5\n");

    int i,j,k;
    double beta;
    struct double_int *thetas;

    thetas=new double_int[this->num_nodes];

    for(i=1;i<=this->num_nodes;i++)
    {
        thetas[i-1].k=i;
        thetas[i-1].d=this->nodes[i].theta;
    }

    // Sort the list of thetas
    qsort(thetas,this->num_nodes,sizeof(struct double_int),double_int_compare);

    // Select a random beta in (min_theta, max_theta ) and see how the split is
    // Stop when we get a satisfactory split

    
    for(;;)
    {
        beta = this->min_theta + (double)(lcgrand(10))*.5*(max_theta - min_theta);

        // Now find out how many nodes we get in each half
        k=0;
        for(j=0;j<this->num_nodes;j++)
        {
            // Count the nodes above the line
            if(this->nodes[j+1].y >= tan(beta)*(this->nodes[j+1].x))
                k++;
        }
    
        if( (k>=(int)(p*(double)(this->num_nodes))) && (k<=(int)((1-p)*(double)(this->num_nodes))))
            break;    // The split is good
    }

    // Eject them - note that we assume to have begun with a full solution...
    for(i=1;i<=this->num_original_nodes;i++)
    {
        if(routed[i])
        {
            if(this->nodes[i].y >= tan(beta)*(this->nodes[i].x))
                this->eject_node(i);
        }
    }

    
    delete [] thetas;
    return beta;
}


int VRP::split_routes(double p, int **ejected_routes, double *t)
{
    ///
    /// Splits an existing VRP into two parts by drawing a random
    /// ray from the VRPH_DEPOT.  We repeatedly try to split the problem in this way
    /// until we have two sets of nodes that each has k nodes where k is between p*num_nodes and
    /// (1-p)*num_nodes.  Next, we take the larger part of the split solution and then 
    /// keep all the routes that have at least one node in the larger part.  The route-based
    /// decisions are based on the currently loaded solution.  The ejected routes
    /// are placed in the ejected_routes[][] array and the function returns the number
    /// of routes ejected.  The final value of theta that splits the problem is placed in t.
    ///

    if(p>.5)
        report_error("%s: p must be less than .5\n");

    int i,j,k;
    double beta;
    struct double_int *thetas;

    thetas=new double_int[this->num_nodes];

    for(i=1;i<=this->num_nodes;i++)
    {
        thetas[i-1].k=i;
        thetas[i-1].d=this->nodes[i].theta;
    }

    // Sort the list of thetas
    qsort(thetas,this->num_nodes,sizeof(struct double_int),double_int_compare);

    // Select a random beta in (min_theta, max_theta ) and see how the split is
    // Stop when we get a satisfactory split

    
    for(;;)
    {
        beta = this->min_theta + (double)(lcgrand(10))*.5*(max_theta - min_theta);

        // Now find out how many nodes we get in each half
        k=0;
        for(j=0;j<this->num_nodes;j++)
        {
            // Count the nodes above the line
            if(this->nodes[j+1].y >= tan(beta)*(this->nodes[j+1].x))
                k++;
        }
    
        if( (k>=(int)(p*(double)(this->num_nodes))) && (k<=(int)((1-p)*(double)(this->num_nodes))))
        {
            break;    // The split is good
        }
    }

    // Eject the routes that do not have any nodes in the larger portion

    int num_ejected=0;
    for(i=1;i<=this->total_number_of_routes;i++)
    {
        int start=this->route[i].start;
        int current=start;
        bool will_eject=true;

        while(current!=VRPH_DEPOT)
        {
            
            if(this->nodes[current].y <= tan(beta)*(this->nodes[current].x))
            {
                // We have at least one node inside the large part
                will_eject=false;
                break;
            }

            current=VRPH_MAX(VRPH_DEPOT,this->next_array[current]);
        }
        if(will_eject)
        {
            // Eject route i from the solution - place the ejected nodes in 
            // the ejected_routes[][] array
            this->eject_route(i, ejected_routes[num_ejected]);
            num_ejected++;

        }
        
    }
    
    delete [] thetas;

    *t=beta;
    return num_ejected;
}


void VRP::fix_edge(int start, int end)
{
    /// 
    /// Forces that the provided edge always remains in the solution.
    /// The edge may not currently be in the solution, but once it is encountered
    /// it will remain in the solution until the edge is unfixed.  Note that the
    /// fixing of edges is only relevant if you use the heuristics with a 
    /// VRPH_FIXED_EDGES rules.
    ///

    this->fixed[start][end]=true;
    this->fixed[end][start]=true;

    // Handle the dummy_node
    if(start==VRPH_DEPOT)
    {
        this->fixed[dummy_index][start]=true;
        this->fixed[start][dummy_index]=true;
    }

    if(end==VRPH_DEPOT)
    {
        this->fixed[dummy_index][end]=true;
        this->fixed[end][dummy_index]=true;
    }


}

void VRP::unfix_edge(int start, int end)
{
    /// 
    /// Unfixes an edge that is already fixed.
    ///

    if(this->fixed[start][end])
        report_error("%s: Edge %d-%d is not already fixed!\n",__FUNCTION__,start,end);

    this->fixed[start][end]=false;
    this->fixed[end][start]=false;

    // Handle the dummy_node
    if(start==VRPH_DEPOT)
    {
        this->fixed[dummy_index][start]=false;
        this->fixed[start][dummy_index]=false;
    }

    if(end==VRPH_DEPOT)
    {
        this->fixed[dummy_index][end]=false;
        this->fixed[end][dummy_index]=false;
    }


}

void VRP::unfix_all()
{
    ///
    /// Unfixes any and all edges that may be currently fixed.
    ///

    for(int i=0;i<=matrix_size;i++)
        for(int j=0;j<=matrix_size;j++)
            fixed[i][j]=false;
}

void VRP::fix_string(int *node_string, int k)
{
    ///
    /// Fixes all edges in the node_string[] array.
    /// For example, given the array {1,3,5,4,2}, the edges
    /// 1-3, 3-5, 5-4, 4-2 will be fixed.  The value of k
    /// is the number of nodes in the string.
    ///


    for(int i=0;i<k-1;i++)
        this->fix_edge(node_string[i], node_string[i+1]);

    
}





int VRP::read_fixed_edges(const char *filename)
{
    ///
    /// Reads a file of edges to be fixed.  Letting k be the
    /// number of edges to be fixed, the file has the format
    ///
    /// k
    /// start_1 end_1
    /// start_2 end_2
    /// ...
    /// start_k end_k
    ///
    /// Returns the number of edges that are fixed.
    ///


    int i, a, b, k;
    FILE *in;

    if( (in=fopen(filename,"r"))==NULL)
    {
        fprintf(stderr,"Error opening %s for reading\n",filename);
        report_error("%s: Bad file in read_fixed_edges\n",__FUNCTION__);
    }

    fscanf(in,"%d\n",&k);

    for(i=0;i<k;i++)
    {
        fscanf(in,"%d %d\n",&a, &b);
        if(a<0 || b<0 || a>this->num_original_nodes || b>this->num_original_nodes)
        {
            fprintf(stderr,"Tried to fix edge %d-%d\n",a,b);
            report_error("%s: Error in read_fixed_edges\n",__FUNCTION__);
        }
        this->fix_edge(a,b);
    }

    // Return the number of fixed edges read in
    return k;
}
void VRP::list_fixed_edges(int *fixed_list)
{
    ///
    /// Looks through the current solution and places all edges that
    /// are currently fixed and in the solution in the fixed_list[] array.
    /// So if fixed[2][3]==true and fixed[3][7]==true, the array
    /// fixed_list[] is {2,3,3,7}.
    ///

    int pos=0;
    int current=VRPH_DEPOT;
    int next=VRPH_ABS(next_array[current]);

    while(next!=VRPH_DEPOT)
    {
        if(fixed[current][next])
        {
            fixed_list[pos]=current;
            fixed_list[pos+1]=next;
            pos+=2;
        }

        current=next;
        next=next_array[current];

        if(next<0)
        {
            if(fixed[current][VRPH_DEPOT])
            {
                fixed_list[pos]=current;
                fixed_list[pos+1]=VRPH_DEPOT;
                pos+=2;
            }

            if(fixed[VRPH_DEPOT][-next])
            {
                fixed_list[pos]=VRPH_DEPOT;
                fixed_list[pos+1]=-next;
                pos+=2;
            }

            next=-next;
        }
    }



}
void VRP::perturb_locations(double c)
{
    ///
    ///    Perturbs the current problem instance by moving each node slightly as in
    /// the TSP paper of Codenotti.  The perturbations are scaled in terms of the
    /// value c.  The distance matrix is recomputed after all locations have been
    /// perturbed.
    ///

    int i, pre, post;
    double v,theta;

    // First export the solution buffer
    this->export_solution_buff(this->current_sol_buff);

    for(i=1;i<=this->num_nodes;i++)
    {
        pre= VRPH_MAX(VRPH_DEPOT,this->pred_array[i]);
        post=VRPH_MAX(VRPH_DEPOT,this->next_array[i]);

        v= (this->d[pre][i]+this->d[i][post]) - (this->nodes[i].service_time + this->nodes[post].service_time);
        v= v*c;

        // Now alter the location of node i by shifting v units in a random direction theta
        theta= VRPH_PI*lcgrand(8);

        this->nodes[i].x += v*cos(theta);
        this->nodes[i].y += v*sin(theta);
        
    }

    // Now re-create the distance matrix

    this->create_distance_matrix(this->edge_weight_type);

    // Now re-import the solution with the new distances
    this->import_solution_buff(this->current_sol_buff);

    return;

}
void VRP::add_route(int *route_buff)
{
    ///
    /// Adds the route in the provided buffer to the solution.  The new route should
    /// not have any nodes in common with the existing solution! The route_buff[] should
    /// be terminated with a -1.
    ///

    // We will do this by exporting the current solution to a buffer, appending the 
    // new route to this buffer, and then importing the new resulting solution.
    // Probably not the fastest way, but we have many fields to update when adding a
    // new route!

    int *temp_buff;

    this->verify_routes("Before adding route\n");

    temp_buff=new int[this->num_original_nodes+2];
    this->export_solution_buff(temp_buff);


    int old_num=this->num_nodes;
    int i=0;
    while(route_buff[i]!=-1)
    {
        temp_buff[old_num+1+i]=route_buff[i];
        if(i==0)
            temp_buff[old_num+1+i]=-temp_buff[old_num+1+i];

        temp_buff[0]++;
        i++;
    }
    
    temp_buff[old_num+1+i]=VRPH_DEPOT;
    
    // Now import the new solution
    this->import_solution_buff(temp_buff);
    
    this->verify_routes("After adding route\n");

    delete [] temp_buff;

}
void VRP::append_route(int *sol_buff, int *route_buff)
{
    ///
    /// Appends the single route contained in route_buff[] (which ends in a -1)
    /// to the solution buffer sol_buff, updating the first entry in sol_buff
    /// which is the # of nodes in the solution.  Does NOT import the resulting
    /// solution and assumes that route_buff and sol_buff are disjoint.
    ///

    int i,j,current_num;

    current_num=sol_buff[0]; // The # of nodes in the sol_buff solution

    // Now find out how many nodes are in the route_buff - ends in a -1;

    j=0;
    while(route_buff[j]!=-1)
        j++;

    // The route has j nodes - we assume they are all different from those in the solution!
    // Increment the # of nodes in sol_buff;
    sol_buff[0]+=j;

    sol_buff[current_num+1]=-route_buff[0];
    for(i=1;i<j;i++)
        sol_buff[current_num+1+i]=route_buff[i];
    

    // End at the VRPH_DEPOT;
    sol_buff[current_num+1+j]=VRPH_DEPOT;
        
    return;


}

int VRP::intersect_solutions(int *new_sol, int **route_list, int *sol1, int *sol2, int min_routes)
{
    ///
    /// Takes the two solutions sol1 and sol2 and constructs a smaller instance by
    /// ejecting the routes that are in both sol1 and sol2.  If the resulting
    /// solution contains less than min_routes routes, then we add more random routes
    /// from sol1 until the solution has min_routes.  Returns the number of 
    /// routes ejected from sol1 and places these route buffers in the route_list[][] array which
    /// is a 0-based array of routes.  Note that sol1 and sol2 are assumed to be
    /// full solutions to the VRP instance.  Imports the smaller solution new_sol
    /// before returning.
    ///

    int i,j,k,m,num_routes;
    int *rnums;

    rnums=new int[this->num_original_nodes+1]; // Plenty big...

    // Find the routes in common
    j = this->find_common_routes(sol1, sol2, rnums);
    
    if(j==0)
    {
        // No routes in common - just put sol1 into new_sol
        memcpy(new_sol,sol1, (this->num_original_nodes+2)*sizeof(int));
        delete[] rnums;
        return 0;
    }

    // Otherwise we have some routes in common - copy them to the routes[] array
    // Import sol1 to do this
    this->import_solution_buff(sol1);
    num_routes=this->total_number_of_routes;
    k=0;
    for(i=0;i<j;i++)
    {
        // Copy route i from sol1 into the route_list[] array
        route_list[i][0]=this->route[rnums[i]].start;

        m=0;
        while(route_list[i][m]!=this->route[rnums[i]].end)
        {
            m++;
            route_list[i][m]=this->next_array[route_list[i][m-1]];
        }
        // Add a -1 to indicate the end of the route
        m++;
        route_list[i][m]=-1;

        k++;
        num_routes--;
        if(num_routes==min_routes)
            break;
    }

    // We actually copied k of the j routes in the intersection
    // k may be less than j

    // Now eject the routes from sol1 (already imported)
    int *junk;
    junk=new int[this->num_original_nodes];
    // won't use this
    for(i=0;i<k;i++)
        this->eject_route(rnums[i], junk);

    // Now export the solution after ejections to new_sol
    this->export_canonical_solution_buff(new_sol);

    // Now import the new (possibly partial) solution
    this->import_solution_buff(new_sol);

    delete [] rnums;
    delete [] junk;

    return k;

}
bool VRP::osman_insert(int k, double alpha)
{
    ///
    /// Inserts node k into a new location in the solution 
    /// according to Osman's savings heuristic.  Given existing
    /// edges i-k-j and l-m, with parameter alpha, we move k to
    /// new location l-k-m such that the quantity
    /// ( ik+kj-lm ) - alpha*(lk+km-ij) is minimized.
    ///

    int i,j,l,ll,m,mm, best_l, best_m;
    double savings, best_savings, ik,kj,ij;
    Postsert postsert;
    Presert presert;
    VRPMove M;    

    if(k==VRPH_DEPOT)
        return false;

    i=VRPH_MAX(this->pred_array[k],VRPH_DEPOT);
    j=VRPH_MAX(this->next_array[k],VRPH_DEPOT);


    ik=this->d[i][k];
    kj=this->d[k][j];
    ij=this->d[i][j];

    best_savings=VRP_INFINITY;
    best_l=-1;
    best_m=-1;

    l=VRPH_DEPOT;
    m=abs(this->next_array[l]);
    while(m!=VRPH_DEPOT)
    {
        if(m>0)
        {
            savings = (ik + kj - this->d[l][m]) - alpha*(this->d[l][k] + this->d[k][m] - ij);
            if(savings<best_savings && l!=i && m!=j)
            {
                // Now check feasibility-put k before m
                if(presert.evaluate(this,k,m,&M)==true)
                {
                    best_savings=savings;
                    best_l=l;
                    best_m=m;
                }
            }
        }
        else
        {
            // m<0 indicating the end of a route
            
            // consider the edge l-VRPH_DEPOT
            mm=VRPH_DEPOT;
            savings = (ik + kj - this->d[l][mm]) - alpha*(this->d[l][k] + this->d[k][mm] - ij);
            if(savings<best_savings && l!=i && mm!=j)
            {
                // put k after l
                if(postsert.evaluate(this,k,l,&M)==true)
                {
                    best_savings=savings;
                    best_l=l;
                    best_m=mm;
                }
            }

            // Now consider the edge VRPH_DEPOT-abs(m)
            ll=VRPH_DEPOT;
            m=abs(m);
            savings = (ik + kj - this->d[ll][m]) - alpha*(this->d[ll][k] + this->d[k][m] - ij);
            if(savings<best_savings && ll!=i && m!=j)
            {
                // put k before m
                if(presert.evaluate(this,k,m,&M)==true)
                {
                    best_savings=savings;
                    best_l=ll;
                    best_m=m;
                }
            }
        }
        // Now advance to the next edge
        l=m;
        m=this->next_array[l];
    }

    // We now have the best location to move node k by 
    // placing in between best_l and best_m;

    if(best_savings==VRP_INFINITY)
    {
        // No locations found
        return false;
    }

    if(best_l!=VRPH_DEPOT)
    {
        if(postsert.move(this,k,best_l)==false)
            report_error("%s: postsert.move is false\n",__FUNCTION__);
    }
    else
    {
        if(presert.move(this,k,best_m)==false)
            report_error("%s: presert.move is false\n",__FUNCTION__);
    }

    return true;

}

int VRP::osman_perturb(int num, double alpha)
{
    ///
    /// Perturbs the existing solution by attempting to move num different random
    /// nodes into new positions using Osman parameter alpha.  Gives up after attempting
    /// 2*V.num_nodes moves.
    ///

    int tot=0;
    int k;
    int num_attempts=0;

    while(tot<num)
    {
        // Select a random node to move
        k=VRPH_MAX((int)(this->num_original_nodes * lcgrand(10)),1);//to avoid inserting VRPH_DEPOT

        if(routed[k])
        {

            // Try to insert
            if(this->osman_insert(k,alpha)==true)
                tot++;

            num_attempts++;
            if(num_attempts>2*this->num_original_nodes)
                return tot;// Only moved tot nodes
        }
    }

    // We were able to move num nodes

    return num;

    
}

bool VRP::check_fixed_edges(const char *message)
{
    ///
    /// Makes sure that all fixed edges are still in the solution.
    /// If fixed edges are missing from the solution, then some information
    /// is displayed and the provided message is printed before exiting.
    ///



    int i,j;

    for(i=0;i<=this->num_original_nodes;i++)
    {
        for(j=0;j<=this->num_original_nodes;j++)
        {
            if(this->fixed[i][j])
            {
                // Make sure i-j or j-i exists
                if(i!=VRPH_DEPOT)
                {
                    if(VRPH_MAX(this->next_array[i],VRPH_DEPOT)!=j && VRPH_MAX(this->pred_array[i],VRPH_DEPOT)!=j)
                    {
                        fprintf(stderr,"Fixed edge %d-%d not in solution!!",i,j);
                        fprintf(stderr,"%d-%d-%d\n",VRPH_MAX(this->pred_array[i],VRPH_DEPOT),i,
                            VRPH_MAX(this->next_array[i],VRPH_DEPOT));
                        fprintf(stderr,message);

                        if(this->fixed[j][i])
                            fprintf(stderr,"%d-%d also fixed\n",j,i);
                        else
                            fprintf(stderr,"%d-%d NOT fixed!\n",j,i);

                        return false;
                    }
                }

                // Make sure i-j or j-i exists
                if(j!=VRPH_DEPOT)
                {
                    if(VRPH_MAX(this->next_array[j],VRPH_DEPOT)!=i && VRPH_MAX(this->pred_array[j],VRPH_DEPOT)!=i)
                    {
                        fprintf(stderr,"Fixed edge %d-%d not in solution!!",i,j);
                        fprintf(stderr,"%d-%d-%d\n",VRPH_MAX(this->pred_array[j],VRPH_DEPOT),j,
                            VRPH_MAX(this->next_array[j],VRPH_DEPOT));
                        fprintf(stderr,message);
                        if(this->fixed[j][i])
                            fprintf(stderr,"%d-%d also fixed\n",j,i);
                        else
                            fprintf(stderr,"%d-%d NOT fixed!\n",j,i);
                        return false;

                    }
                }
            }
    
        }

    }

    return true;
}
int VRP::find_common_routes(int *sol1, int *sol2, int *route_nums)
{
    ///
    /// Finds the routes that are shared by the two solutions sol1 and sol2.
    /// Places the numbers of these routes (numbers from sol1 which are 1-based) 
    /// into the route_nums[] buffer and returns the number of shared routes.
    ///

    int *h11, *h12, *h21, *h22; // to store the hash values
    double *L1, *L2;
    int i, j, cnt, r1, r2;    // to store the # of routes.
    VRPRoute rte(num_nodes);

    // First, import sol1
    this->import_solution_buff(sol1);
    
    r1=this->total_number_of_routes;
    h11=new int[r1+1]; h12=new int[r1+1];// 1-based arrays
    L1=new double[r1+1];
    for(i=1;i<=r1;i++)
    {
        // Import route i into rte
        this->update_route(i,&rte);
        // Hash the route
        h11[i]=rte.hash(SALT_1);
        h12[i]=rte.hash(SALT_2);
        L1[i]=rte.length;
    }

    // Now import sol2
    this->import_solution_buff(sol2);
    
    r2=this->total_number_of_routes;
    h21=new int[r2+1]; h22=new int[r2+1];// 1-based arrays
    L2=new double[r2+1];
    for(i=1;i<=r2;i++)
    {
        this->update_route(i,&rte);
        h21[i]=rte.hash(SALT_1);
        h22[i]=rte.hash(SALT_2);
        L2[i]=rte.length;
    }
    


    // Now compare the hash values
    cnt=0;
    for(i=1;i<=r1;i++)
    {
        for(j=1;j<=r2;j++)
        {
            if(h11[i]==h21[j] && h12[i]==h22[j] && VRPH_ABS(L1[i]-L2[j])<VRPH_EPSILON)
            {
                // route i in sol1 is the same as route j in sol2
                route_nums[cnt]=i;
                cnt++;
            }
        }
    }


    delete [] h11; delete [] h12; delete [] h21; delete [] h22;
    delete [] L1; delete [] L2;
    return cnt;

}

void VRP::set_daily_demands(int day)
{
    ///
    /// Sets the demand of each node equal to the daily demand value
    /// for the given day.  Used for period VRPs.  The day should be a positive
    /// integer.  If a day of 0 is given, then we set the demand to the mean value.
    ///

    int i;

    if(day>0)
    {
        for(i=0;i<=this->num_original_nodes;i++)
        {
            if(this->nodes[i].daily_demands[day]>=0)
                this->nodes[i].demand=this->nodes[i].daily_demands[day];
            else
                this->nodes[i].demand=-1;

        }
    }
    else
    {
        // Day is 0 - use the mean demand
        for(i=0;i<=this->num_original_nodes;i++)
        {
            int mean_demand=0;
            int k=0;
            for(int j=1;j<=this->num_days;j++)
            {
                if(this->nodes[i].daily_demands[j]>=0)
                {
                    mean_demand+=this->nodes[i].daily_demands[j];
                    k++;
                }
            }
            if(k>0)
                this->nodes[i].demand=(int)((double)mean_demand/(double)k);
            else
                this->nodes[i].demand=-1;

        }    

    }
}

void VRP::set_daily_service_times(int day)
{
    ///
    /// Sets the service time of each node equal to the daily service time
    /// for the given day.  Used for period VRPs.
    ///

    int i;

    for(i=0;i<=this->num_original_nodes;i++)
    {
        this->nodes[i].service_time=this->nodes[i].daily_service_times[day];
    }

    // We also have to recompute the distance matrix if the service times are not identical
    // across the days
    this->create_distance_matrix(this->edge_weight_type);
}


void VRP::update_arrival_times()
{
    ///
    /// Computes the arrival time at all customers.
    ///

    int routenum, next, current;
    double t;

    // Set the arrival times to -1 so that the only ones with positive 
    // arrival_time values are those that are actually visited
    for(int i=1;i<=this->num_original_nodes;i++)
        this->nodes[i].arrival_time=-1;


    for(int i=1;i<=this->num_original_nodes;i++)
    {

        if(routed[i])
        {
            // Get the route number
            routenum=this->route_num[i];

            // Start at the beginning of this route
            current=this->route[routenum].start;
            t=this->d[VRPH_DEPOT][current];

            while(current!=i)
            {
                next=VRPH_MAX(VRPH_DEPOT, this->next_array[current]);
                t+=this->d[current][next];
                current=next;

            }

            // Now subtract the service time at t as this is included in the distance 
            // matrix
            t-=this->nodes[i].service_time;
            this->nodes[i].arrival_time=t;
        }
    }

}

bool VRP::check_tabu_status(VRPMove *M, int *old_sol)
{
    ///
    /// The tabu search rules is entirely route-based.  We hash
    /// each of the affected routes and see if the values are in the
    /// tabu list.  If the move is tabu, then we revert back to the old solution
    /// and return false.  Otherwise, we allow the move and return true.
    /// When the move is allowed, we update the tabu list using a circular buffer.
    ///


#if VRPH_TABU_DEBUG
    printf("Checking tabu status\n");
#endif


    int i,j;

    // We will always accept a move that reduces the # of routes
    if(M->num_affected_routes>1)
    {
        for(i=0;i<M->num_affected_routes;i++)
        {
            if(M->route_custs[i]==0)
                return true;
        }
    }    
    
    VRPRoute r(this->num_nodes);
    int num_tabu_routes=0;

#if VRPH_TABU_DEBUG
    printf("Checking tabu status of %d routes\n",M->num_affected_routes);
#endif

    for(i=0;i<M->num_affected_routes;i++)
    {
        this->update_route(M->route_nums[i],&r);
        r.hash_val = r.hash(SALT_1);
        r.hash_val2 = r.hash(SALT_2);    
        
        for(j=0;j<this->tabu_list->num_entries;j++)
        {
            if(r.hash_val==this->tabu_list->hash_vals1[j] && r.hash_val2==this->tabu_list->hash_vals2[j])
                // The move is tabu!
                num_tabu_routes++;            
        }
        
    }
    

    if(num_tabu_routes>0)
    {
        // At least one tabu route
        this->import_solution_buff(old_sol);
#if VRPH_TABU_DEBUG
                printf("Move is Tabu! Reverting to old solution\n");
#endif
        return false;
    }

#if VRPH_TABU_DEBUG
    printf("Move is not Tabu.  Updating list of %d routes\n",M->num_affected_routes);
#endif

    // The move is not tabu - update the tabu list 
    for(i=0;i<M->num_affected_routes;i++)
    {    
        this->update_route(M->route_nums[i],&r);

        this->tabu_list->update_list(&r);
    }

    return true;

}


void VRP::print_stats()
{
    ///
    /// Prints the # of evaluations and moves performed for each
    /// heuristic operator.
    ///

    printf("                       (Moves,      Evaluations)\n"); 
    printf("     One Point Move:   (%010d, %010d)\n",
        this->num_moves[ONE_POINT_MOVE_INDEX], this->num_evaluations[ONE_POINT_MOVE_INDEX]);
    printf("     Two Point Move:   (%010d, %010d)\n",
        this->num_moves[TWO_POINT_MOVE_INDEX], this->num_evaluations[TWO_POINT_MOVE_INDEX]);
    printf("   Three Point Move:   (%010d, %010d)\n",
        this->num_moves[THREE_POINT_MOVE_INDEX], this->num_evaluations[THREE_POINT_MOVE_INDEX]);
    printf("       Two-opt Move:   (%010d, %010d)\n",
        this->num_moves[TWO_OPT_INDEX], this->num_evaluations[TWO_OPT_INDEX]);
    printf("     Three-opt Move:   (%010d, %010d)\n",
        this->num_moves[THREE_OPT_INDEX], this->num_evaluations[THREE_OPT_INDEX]);
    printf("        Or-opt Move:   (%010d, %010d)\n",
        this->num_moves[OR_OPT_INDEX], this->num_evaluations[OR_OPT_INDEX]);
    printf("Cross-Exchange Move:   (%010d, %010d)\n\n",
        this->num_moves[CROSS_EXCHANGE_INDEX], this->num_evaluations[CROSS_EXCHANGE_INDEX]);

    return;

}


void VRP::reset()
{
    ///
    /// Liquidates the solution memory and sets all nodes to unrouted.
    ///

    this->solution_wh->liquidate();
    for(int j=1;j<=this->num_original_nodes;j++)
        this->routed[j]=false;
}

