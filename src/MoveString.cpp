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

bool MoveString::evaluate(VRP *V, int a, int b, int u, int v, VRPMove *M)
{
    ///
    /// Evaluates the move of taking the string between u and v (i.e. t-u-j-k-l-m-v-w)
    /// and inserting between a and b (assumed to currently be an existing edge), 
    /// yielding t-w & a-u-j-k-l-m-b
    ///

    // First make sure u or v is not the VRPH_DEPOT

    if(u==VRPH_DEPOT || v==VRPH_DEPOT || u==v)
    {
        fprintf(stderr,"(%d,%d,%d,%d)\n",a,b,u,v);
        report_error("%s: evaluate called with u or v as VRPH_DEPOT or u==v\n",__FUNCTION__);

    }

    int t,w;

    t=VRPH_MAX(V->pred_array[u],0);
    w=VRPH_MAX(V->next_array[v],0);    

    int a_route, u_route;

    u_route = V->route_num[u];
    if(u_route!= V->route_num[v])
        report_error("%s: u and v not in same route\n",__FUNCTION__);

    if(a!=VRPH_DEPOT)
        a_route = V->route_num[a];
    else
        a_route = V->route_num[b];

    double tu, vw, ab, au, vb, tw, savings, new_a_len=0, new_u_len=0;
    int new_a_load=0, new_u_load=0;

    ab= V->d[a][b];
    tu= V->d[t][u];
    vw= V->d[v][w];
    au= V->d[a][u];
    vb= V->d[v][b];
    tw= V->d[t][w];

    //savings=new-old
    savings = (au+vb+tw) - (ab+tu+vw);


    // Now find the total length and demand of the string
    // Should use get segment info here...
    VRPSegment S;
    V->get_segment_info(u,v,&S);


#    if(STRING_DEBUG>1)
    printf("STRING::string_len: %f;  string_load: %d; string_custs: %d\n",string_len, string_load);
#endif

    // Check feasibility
    if(a_route==u_route)
    {
        // Same route - only check length change
        new_a_len= V->route[a_route].length + savings;
        if(new_a_len > V->max_route_length)
        {
#            if(STRING_DEBUG>1)
            printf("STRING::a_len too long: %f\n",new_a_len);
#endif
            // Exceeds length capacity
            return false;
        }

        M->num_affected_routes=1;
        M->route_nums[0]=a_route;
        M->savings = savings;
        M->route_lens[0]=new_a_len;
        M->route_loads[0]=V->route[a_route].load;
        M->route_custs[0]= V->route[a_route].num_customers;
        M->move_type=MOVE_STRING;
        M->num_arguments=4;
        M->move_arguments[0]=a;
        M->move_arguments[1]=b;
        M->move_arguments[2]=u;
        M->move_arguments[3]=v;
        M->new_total_route_length=V->total_route_length+savings;



        return true;


    }
    else
    {
        // Different routes!
        new_a_len = V->route[a_route].length + ( (au+vb+S.len)-ab);
        if(new_a_len > V->max_route_length)
        {
# if STRING_DEBUG>1
            printf("STRING::a_len too long: %f\n",new_a_len);
#endif
            // Exceeds length 
            return false;
        }

        new_u_len = V->route[u_route].length + ( (tw)-(tu+vw+S.len) );
        if(new_u_len > V->max_route_length)
        {
#            if(STRING_DEBUG>1)
            printf("STRING::u_len too long: %f\n",new_u_len);
#endif
            // Exceeds length 
            return false;
        }

        // Now check loads;

        new_a_load = V->route[a_route].load + S.load;
        if(new_a_load > V->max_veh_capacity)
        {
#            if(STRING_DEBUG>1)
            printf("STRING::a_load too big: %f\n",new_a_load);
#endif
            // Exceeds capacity
            return false;
        }

        new_u_load = V->route[u_route].load - S.load;
        // new_u_load decreases, so no point checking capacity

        // If we get here, the move is feasible-record it.

        M->num_affected_routes=2;
        M->route_nums[0]=a_route;
        M->route_nums[1]=u_route;
        M->savings = savings;
        M->route_lens[0]=new_a_len;
        M->route_lens[1]=new_u_len;
        M->route_loads[0]=new_a_load;
        M->route_loads[1]=new_u_load;
        M->route_custs[0]= V->route[a_route].num_customers+S.num_custs;
        M->route_custs[1]= V->route[u_route].num_customers-S.num_custs;
        M->new_total_route_length= V->total_route_length+M->savings;
        M->move_type=MOVE_STRING;
        M->num_arguments=4;
        M->move_arguments[0]=a;
        M->move_arguments[1]=b;
        M->move_arguments[2]=u;
        M->move_arguments[3]=v;


        // See if we reduce the # of routes
        if(u== V->route[u_route].start && v== V->route[u_route].end)
        {
            // We are moving an entire route!
            M->total_number_of_routes = V->total_number_of_routes - 1;
        }
        else
            M->total_number_of_routes= V->total_number_of_routes;


        return true;
    }

}

bool MoveString::move(VRP *V, int a, int b, int u, int v)
{
    ///
    /// Takes the string of nodes between u and v (inclusive) and places it between a and b.
    ///

    VRPMove M;

    if(evaluate(V,a,b,u,v, &M)==false)
    {
        report_error("%s: evaluate is false in MoveString.move\n",__FUNCTION__);

    }


    if(a!=VRPH_DEPOT)
    {
        // We can do this via a sequence of postserts    
        Postsert postsert;

        int string[100];
        int len=1;
        int i;

        // Find the length of the string of nodes and then store it in the
        // string[] array
        int current=u;
        while(current!=v)
        {
            current= VRPH_MAX(VRPH_DEPOT,V->next_array[current]);
            len++;
        }

        string[0]=u;
        for(i=1;i<len;i++)
        {
            string[i]= VRPH_MAX(V->next_array[string[i-1]],0);

        }

        // Artificially inflate the constraints!
        double real_max_len= V->max_route_length;
        int real_veh_max= V->max_veh_capacity;

        V->max_route_length=VRP_INFINITY;
        V->max_veh_capacity=VRP_INFINITY;

        postsert.move(V,string[0],a);
        for(i=1;i<len;i++)
        {    
            postsert.move(V,string[i],string[i-1]);
        }

        V->max_route_length=real_max_len;
        V->max_veh_capacity=real_veh_max;

#if STRING_DEBUG
        printf("Routes after move:\n");
        V->show_route(u_route);
        V->show_route(V->route_num[a]);

#endif


#if STRING_VERIFY
        V->verify_routes("After Postsert move string\n");
#endif
    }
    else
    {
        // Have to use presert
        Presert presert;

        int next_to_presert=v;
        int next_node=b;
        int t=VRPH_MAX(V->pred_array[u],0);

        // Artificially inflate the constraints!
        double real_max_len= V->max_route_length;
        int real_veh_max= V->max_veh_capacity;

        V->max_route_length=VRP_INFINITY;
        V->max_veh_capacity=VRP_INFINITY;


        while(next_to_presert!=t)
        {
            presert.move(V,next_to_presert,next_node);
            next_node=next_to_presert;
            next_to_presert= VRPH_MAX(VRPH_DEPOT, V->pred_array[next_to_presert]);//V->pred(next_to_presert);

        }

        V->max_route_length=real_max_len;
        V->max_veh_capacity=real_veh_max;

        // Don't update with M since the postsert/presert did it for us...


#if STRING_VERIFY
        V->verify_routes("After movestring\n");
#endif
    }

    return true;
}



