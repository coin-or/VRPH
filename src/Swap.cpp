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


bool Swap::evaluate(class VRP *V, int u, int i, VRPMove *M)
{
    ///
    /// Evaluates the move of swapping the positions of nodes u and i in 
    /// the current   For example, 
    /// Current situation:    t-u-v and h-i-j 
    /// New situation:        t-i-v and h-u-k        
    ///

    if(V->routed[u]==false || V->routed[i]==false)
        return false;

    if(u==VRPH_DEPOT || i==VRPH_DEPOT)
        report_error("%s: called with VRPH_DEPOT node\n",__FUNCTION__);

    if(u==i)
    {
        fprintf(stderr,"SWAP::%d=%d??\n",u,i);
        V->show_route(V->route_num[u]);
        report_error("%s: called with u=i\n",__FUNCTION__);
    }

    int t,v,h,j,u_route,i_route,swap_type=0;
    double savings,u_change=0,i_change=0;

    t=VRPH_MAX(V->pred_array[u],0);
    v=VRPH_MAX(V->next_array[u],0);
    h=VRPH_MAX(V->pred_array[i],0);
    j=VRPH_MAX(V->next_array[i],0);

    // Don't bother with this move if we have u or i as a singleton route
    if( (h==VRPH_DEPOT && j==VRPH_DEPOT) || (t==VRPH_DEPOT && v==VRPH_DEPOT) )
        return false;

    savings=VRP_INFINITY;

    //savings=new-old 
    if(h==u)
    {
        // t-h/u-i/v-j --> t-i/v-h/u-j
        savings=(V->d[t][i]+V->d[i][h]+V->d[u][j])-
            (V->d[t][u]+V->d[u][v]+V->d[i][j]);
        swap_type=1;
    }
    
    if(h==v)
    {
        savings=(V->d[t][i]+V->d[i][h]+V->d[v][u]+V->d[u][j])-
            (V->d[t][u]+V->d[u][v]+V->d[h][i]+V->d[i][j]);
        swap_type=2;
    }

    if(j==t && t!=VRPH_DEPOT)//!!!
    {
        savings=(V->d[h][u]+V->d[u][t]+V->d[j][i]+V->d[i][v])-
            (V->d[h][i]+V->d[i][j]+V->d[t][u]+V->d[u][v]);
        swap_type=3;
    }

    if(j==u)
    {
        savings=(V->d[h][j]+V->d[j][i]+V->d[i][v])-
            (V->d[h][i]+V->d[i][j]+V->d[u][v]);
        swap_type=4;
    }

    if(swap_type==0)
    {
        // Normal case w/ no exceptions
        savings=(V->d[t][i]+V->d[i][v] + V->d[h][u]+V->d[u][j])-
            (V->d[t][u]+V->d[u][v] + V->d[h][i]+V->d[i][j]);

        i_change=(V->d[h][u]+V->d[u][j])-(V->d[h][i]+V->d[i][j]);
        
        // Route u was t-u-v and is now t-i-v
        u_change=(V->d[t][i]+V->d[i][v])-(V->d[t][u]+V->d[u][v]);

        swap_type=5;
    }

    // Now need to check feasibility

    u_route= V->route_num[u];
    i_route= V->route_num[i];

    if(u_route==i_route)
    {
        // Same route, so the length changes by savings
        if(V->route[u_route].length+savings>V->max_route_length)
            return false;    // infeasible

        // Otherwise record the move

        M->num_affected_routes=1;
        M->savings=savings;
        M->route_nums[0]=u_route;
        M->route_lens[0]=V->route[u_route].length+savings;
        M->route_loads[0]=V->route[u_route].load;
        M->route_custs[0]= V->route[u_route].num_customers; // no change
        M->new_total_route_length= V->total_route_length+savings;
        M->total_number_of_routes = V->total_number_of_routes;
        M->move_type=SWAP;
        M->num_arguments=2;
        M->move_arguments[0]=u;
        M->move_arguments[1]=i;
    }
    else
    {
        // Different routes--but in this case we can't have any of the 
        // overlap conditions unless the VRPH_DEPOT is involved!!

        
        M->num_affected_routes=2;
        M->route_nums[0] = u_route;
        M->route_nums[1] = i_route;
        M->move_type=SWAP;
        M->num_arguments=2;
        M->move_arguments[0]=u;
        M->move_arguments[1]=i;

        if(swap_type==1)
        {
            report_error("%s: should not be in different routes(1)\n",__FUNCTION__);
            
        }

        if(swap_type==2)
        {
            //h=v:  must have v=h=VRPH_DEPOT
            u_change=(V->d[t][i]+V->d[v][i])-(V->d[t][u]+V->d[u][v]);
            i_change=(V->d[h][u]+V->d[u][j])-(V->d[h][i]+V->d[i][j]);

        }

        if(swap_type==3)
        {
            //j=t=VRPH_DEPOT
            u_change=(V->d[t][i]+V->d[v][i])-(V->d[t][u]+V->d[u][v]);
            i_change=(V->d[h][u]+V->d[u][j])-(V->d[h][i]+V->d[i][j]);

        }

        if(swap_type==4)
        {
            
            //j==u- not possible
            report_error("%s: should not be in different routes(4)\n",__FUNCTION__);
            

        }

        if(swap_type==5)
        {
            // Route i was h-i-j and is now h-u-j
            i_change=(V->d[h][u]+V->d[u][j])-(V->d[h][i]+V->d[i][j]);
            // Route u was t-u-v and is now t-i-v
            u_change=(V->d[t][i]+V->d[i][v])-(V->d[t][u]+V->d[u][v]);

        }

        if( ( M->route_lens[1] = V->route[i_route].length+i_change ) >V->max_route_length  )
            return false;    // route that used to contain i is infeasible


        if( ( M->route_lens[0] = V->route[u_route].length+u_change ) >V->max_route_length  )
            return false;    // route that used to contain u is infeasible

        // Now check capacity constraints

        if( (M->route_loads[1] = V->route[i_route].load+V->nodes[u].demand-
            V->nodes[i].demand ) >  V->max_veh_capacity)
            return false;    // route that used to contain i is infeasible

        if( ( M->route_loads[0]= V->route[u_route].load+V->nodes[i].demand-
            V->nodes[u].demand ) >  V->max_veh_capacity)
            return false;    // route that used to contain u is infeasible

    }
    // The move is feasible - store the rest and check 
    M->savings=savings;
    M->new_total_route_length=V->total_route_length+savings;
    M->total_number_of_routes=V->total_number_of_routes;
    M->route_custs[0]=V->route[u_route].num_customers;
    M->route_custs[1]=V->route[i_route].num_customers;

    return true;
    
}

bool Swap::move(VRP *V, int u, int i)
{
    ///
    /// This modifies the current solution information by swapping
    /// the positions of u and i in the current configuration if the
    /// proposed move meets the rules.  Returns true if the move
    /// succeeds and false otherwise.
    ///

    VRPMove M;
    int u_route, i_route;

    if(evaluate(V,u,i,&M)==false)
    {
        report_error("%s: evaluate is false in move!\n",__FUNCTION__);
        return false;
    }


    // else we make the move

    // Don't update here since some special cases are
    // handled w/ postsert and presert...
    

    int h,j,t,v,hh,jj,tt,vv;
    class Postsert postsert;
    class Presert presert;

    t= V->pred_array[u];
    v= V->next_array[u];
    h= V->pred_array[i];
    j= V->next_array[i];

    hh=VRPH_MAX(V->pred_array[i],0);
    jj=VRPH_MAX(V->next_array[i],0);
    tt=VRPH_MAX(V->pred_array[u],0);
    vv=VRPH_MAX(V->next_array[u],0);

    if(hh==u)
    {
        // old: h-i-j, t-u-v == t-u-i-j
        // new: t-i-u-j
        // t-h/u-i/v-j --> t-i/v-h/u-j

#if SWAP_DEBUG
    printf("swapping u=%d; i=%d;  t-u-v=%d-%d-%d; h-i-j=%d=%d=%d\n",u,i,tt,u,vv,hh,i,jj);
    V->show_route(V->route_num[u]);
    printf("predicted savings is %f\n",M.savings);
    printf("recalculated savings is %f\n",(V->d[tt][i]+V->d[i][u]+V->d[u][jj])-
        (V->d[tt][u]+V->d[u][i]+V->d[i][jj]));

#endif

#if SWAP_VERIFY
            V->verify_routes("Before h==u SWAP\n");
#endif
        // Put u after i 
        if(postsert.move(V,u,i)==false)
        {
            fprintf(stderr,"postsert(%d,%d) is false in SWAP!\n",u,i);
            fprintf(stderr,"predicted savings is %f\n",M.savings);
            V->show_route(V->route_num[u]);
            V->summary();
            report_error("%s: postsert evaluates to false\n",__FUNCTION__);
        }
        else
        {
#if SWAP_VERIFY
            V->verify_routes("After h==u SWAP\n");
#endif

            return true;
        }
    }

    if(jj==u)
    {
#if SWAP_VERIFY
        V->verify_routes("Before j==u SWAP\n");        
#endif
        // We have h-i/t-u/j-v, so just put i after u
        double t1= V->max_route_length;
        int t2= V->max_veh_capacity;
        V->max_route_length=VRP_INFINITY;
        V->max_veh_capacity=VRP_INFINITY;

        if(postsert.move(V,i,u)==false)
            report_error("%s: postsert %d,%d is false!\n",__FUNCTION__,i,u);

        // reset the constraints
        V->max_route_length=t1;
        V->max_veh_capacity=t2;

#if SWAP_VERIFY
        V->verify_routes("After j==u SWAP\n");        
#endif

        return true;

    }
    if(hh==vv)
    {
#if SWAP_VERIFY
        V->verify_routes("Before h==v SWAP\n");        
#endif


        //Current:        t-u-h/v-i-j
        //New:            t-i-h/v-u-j

        // Put u after i first, and then move i

        // To make sure the move isn't prevented due to infeasibility!
        double t1= V->max_route_length;
        int t2= V->max_veh_capacity;
        V->max_route_length=VRP_INFINITY;
        V->max_veh_capacity=VRP_INFINITY;

        if(postsert.move(V,u,i)==false)
            report_error("%s: postsert %d,%d is false!\n",__FUNCTION__,u,i);

        // Now put i where u used to be
        if(t>0)
        {
            if(postsert.move(V,i,t)==false)
            {
                fprintf(stderr,"i=%d, t=%d\n",i,t);
                V->show_route(V->route_num[i]);
                V->show_route(V->route_num[t]);
                report_error("%s: postsert %d,%d is false!\n",__FUNCTION__,i,t);
            }
        }
        else
        {
            // t must be the end of a route, so put i before h/v
            if(presert.move(V,i,v)==false)
                report_error("%s: presert %d,%d is false!\n",__FUNCTION__,i,v);
        }

        // reset the constraints
        V->max_route_length=t1;
        V->max_veh_capacity=t2;
#if SWAP_VERIFY
            V->verify_routes("After h==v SWAP\n");
#endif

        return true;
    }


    if(jj==tt)
    {

#if SWAP_VERIFY
            V->verify_routes("Before j==t SWAP\n");
#endif

        //Current:        h-i-j/t-u-v
        //New:            h-u-j/t-i-v

        // Put i after u first, and then move u


        // To make sure the move isn't prevented due to infeasibility!
        double t1= V->max_route_length;
        int t2= V->max_veh_capacity;
        V->max_route_length=VRP_INFINITY;
        V->max_veh_capacity=VRP_INFINITY;

        if(postsert.move(V,i,u)==false)
            report_error("%s: postsert %d,%d is false!\n",__FUNCTION__,i,u);

        // Now put u where i used to be
        if(h>0)
        {
            if(postsert.move(V,u,h)==false)
                report_error("%s: postsert %d,%d is false!\n",__FUNCTION__,u,h);
        }
        else
        {
            // h must be the end of a route, so put u before j/t
            if(presert.move(V,u,j)==false)
                report_error("%s: presert %d,%d is false!\n",__FUNCTION__,u,j);
        }

        // reset the constraints
        V->max_route_length=t1;
        V->max_veh_capacity=t2;

#if SWAP_VERIFY
            V->verify_routes("After j==t SWAP\n");
#endif

        return true;
    }

#if SWAP_VERIFY
            V->verify_routes("Before normal SWAP\n");
#endif
    // Here is the "normal" case

    V->update(&M);

    // Put u in between h and j;

    if(h>0)
    {
        V->next_array[h]=u;
        V->pred_array[u]=h;
    }
    else
    {
        V->pred_array[u]=h;
        V->next_array[VRPH_ABS(h)]=-u;
    }

    if(j>0)
    {
        V->next_array[u]=j;
        V->pred_array[j]=u;
    }
    else
    {
        V->next_array[u]=j;
        V->pred_array[VRPH_ABS(j)]=-u;
    }


    // Put i in between t and v;

    if(t>0)
    {
        V->next_array[t]=i;
        V->pred_array[i]=t;
    }
    else
    {
        V->pred_array[i]=t;
        V->next_array[VRPH_ABS(t)]=-i;
    }

    if(v>0)
    {
        V->next_array[i]=v;
        V->pred_array[v]=i;
    }
    else
    {
        V->next_array[i]=v;
        V->pred_array[VRPH_ABS(v)]=-i;
    }

    // Now adjust the start and end routes 
    // Get the old route nums
    u_route= V->route_num[u];
    i_route= V->route_num[i];

    if(u_route!=i_route)
    {
        if(V->route[u_route].start==u)
            // i is the new start
            V->route[u_route].start=i;

        if(V->route[u_route].end==u)
            // i is the new end
            V->route[u_route].end=i;

        if(V->route[i_route].start==i)
            // i is the new start
            V->route[i_route].start=u;

        if(V->route[i_route].end==i)
            // i is the new end
            V->route[i_route].end=u;

        V->route_num[u]=i_route;
        V->route_num[i]=u_route;


#if SWAP_VERIFY
        V->verify_routes("After inter SWAP\n");        
#endif

    }
    else
    {
        // Same route

        if(V->route[u_route].start==u)
            // i is the new start
            V->route[u_route].start=i;
        else
        {

            if(V->route[u_route].start==i)
                // u is the new start
                V->route[u_route].start=u;
        }

        if(V->route[u_route].end==u)
            // i is the new end
            V->route[u_route].end=i;
        else
        {

            if(V->route[u_route].end==i)
                // u is the new end
                V->route[u_route].end=u;
        }

#if SWAP_VERIFY
        V->verify_routes("After intra SWAP\n");        
#endif


    }


    return true;
}

