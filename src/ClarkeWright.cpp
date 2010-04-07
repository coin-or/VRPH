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


ClarkeWright::ClarkeWright(int n)
{
    ///
    /// Sets up the data structure for the Clarke Wright savings algorithm
    /// including a matrix of size n(n-1)/2 for the savings values
    ///

    s=new VRPSavingsElement[n*(n-1)/2];
    has_savings_matrix = false;
    // Set this to true once we have the matrix    

}

ClarkeWright::~ClarkeWright()
{
    delete [] this->s;

}

void ClarkeWright::CreateSavingsMatrix(class VRP *V, double lambda, bool use_neighbor_list)
{
    ///
    /// This computes the savings matrix d[0,i]+d[0,j]-lambda*d[i,j]
    /// for all pairs (i,j) with i and j routed.  
    /// Matrix is sorted and each element is of the form [val, i, j]
    ///

    int i,j,k,m,n;

    n = V->num_original_nodes;    // n is the max. # of non-VRPH_DEPOT nodes    


    if(!use_neighbor_list)
    {
        k=0;
        for(i=1;i<=n;i++)
        {
            for(j=i+1;j<=n;j++)
            {
                if(V->routed[i] && V->routed[j])
                {
                    // Make sure both are routed
                    //s[k].savings = V->d[VRPH_DEPOT][i] + V->d[j][VRPH_DEPOT] - lambda * V->d[j][i];// was d[i][j]
                    // d0i + di0 +d0j +dj0 - d0i -dij-dj0 = di0+d0j-sij
                    s[k].savings=V->d[i][VRPH_DEPOT]+V->d[VRPH_DEPOT][j]-lambda*V->d[i][j];
                    s[k].i=i;
                    s[k].j=j;
                    k++;
                }
            }

        }

        // Now sort this list

        qsort (s, k, sizeof(class VRPSavingsElement), VRPSavingsCompare);

        // Set the flag to true
        has_savings_matrix = true;
        savings_matrix_size = k;
    }
    else
    {    
        // Use the neighbor_lists
        k=0;
        for(i=1;i<=n;i++)
        {
            for(m=0;m<V->neighbor_list_size;m++)
            {
                j= V->nodes[i].neighbor_list[m].position;
                if(j==VRPH_DEPOT)
                {
                    m++;
                    if(m== V->neighbor_list_size)
                        break;//exit the m loop
                    else
                        j= V->nodes[i].neighbor_list[m].position;
                }


                if(V->routed[i] && V->routed[j])
                {
                    s[k].savings = V->d[VRPH_DEPOT][i] + V->d[j][VRPH_DEPOT] - lambda * V->d[i][j];
                    s[k].i=i;
                    s[k].j=j;
                    k++;
                }
            }

        }

        // Now sort this list

        qsort (s, k, sizeof(class VRPSavingsElement), VRPSavingsCompare);

        // Set the flag to true
        has_savings_matrix = true;
        savings_matrix_size = k;

    }

    return;

}


bool ClarkeWright::Construct(VRP *V, double lambda, bool use_neighbor_list)
{
    ///
    /// This function constructs the routes via the 
    /// Clarke Wright savings algorithm with the parameter
    /// lambda (see Yellow 19??): s_{ij}=d_{i0}+d_{0j}-lambda*d_{ij}.
    ///


    int i,j,k,m,n,x;
    int num_routes;
    unsigned char *status;
    double savings;    
    int i_route, j_route;

    Postsert postsert;
    Presert presert;
    Concatenate concatenate;

    n= V->num_nodes;    // # of non-VRPH_DEPOT nodes.
    num_routes=n;
    status=new unsigned char[V->num_original_nodes+1];

    memset(status,VRPH_UNUSED,sizeof(unsigned char)*(V->num_original_nodes+1));

    // First, create the initial routes:  VRPH_DEPOT - node - VRPH_DEPOT for all nodes


    // Check to see if there are any routed nodes
    // If so, then we assume that a solution on a subset of nodes is desired
    // Otherwise, we construct the default routes for ALL possible nodes
    int num_routed=0;
    for(i=1;i<=V->num_original_nodes;i++)
    {
        if(V->routed[i])
            num_routed++;
        
    }

#if CW_DEBUG
    printf("CW: %d nodes routed\n",num_routed);
#endif

    if(num_routed==0)
    {
        if(V->create_default_routes() == false)
        {
            report_error("%s: Default CW routes are not feasible!!\n");
            // The default routes are not feasible!  
            return false;

        }
    }


    // Now start merging routes
    // Create the savings matrix 
    CreateSavingsMatrix(V,lambda,use_neighbor_list);

    k= savings_matrix_size; // Size of savings matrix


#if CW_DEBUG
    printf("savings matrix complete\n");
#endif

    for(m=0; m<k; m++)
    {
        // The savings list is already sorted and the savings_element structure contains 
        // the i and j values that we need

        i = s[m].i;
        j = s[m].j;

        savings = s[m].savings;

#if CW_DEBUG
        printf("CW(%d of %d):%d,%d,%f\n",m,k,i,j,savings);
#endif

        // Now check to see if we can merge these routes

        // Default is to skip
        x=-1;

        if(status[i]==VRPH_UNUSED && status[j]==VRPH_UNUSED)
            // Neither has been used yet
            x=0;

        if(status[i]==VRPH_ADDED && status[j]==VRPH_UNUSED)
            // i has been VRPH_ADDED but j is VRPH_UNUSED
            x=1;

        if(status[i]==VRPH_UNUSED && status[j]==VRPH_ADDED)
            // j has been VRPH_ADDED but i is VRPH_UNUSED
            x=2;

        if(status[i]==VRPH_ADDED && status[j]==VRPH_ADDED)
            // i and j both VRPH_ADDED
            x=3;        

        if(status[i]==VRPH_INTERIOR || status[j]==VRPH_INTERIOR  )
            // One is interior or the VRPH_DEPOT - skip and move on
            x=-1;

        if(i==0 || j==0)
        {
            fprintf(stderr,"CW::Savings matrix error!cw[m=%d of %d];  i=%d; j=%d; savings = %f\n",
                m,k,i,j,savings);
            report_error("%s: Error in CW.construct\n");
        }

        // Possible values for x:
        // x=-1-->at least one VRPH_INTERIOR or not routed --do nothing
        // x=0-->both i and j VRPH_UNUSED
        // x=1-->j VRPH_UNUSED and i VRPH_ADDED
        // x=2-->i VRPH_UNUSED and j VRPH_ADDED
        // x=3-->i and j VRPH_ADDED

        switch(x)
        {

        case -1:
            // had an VRPH_INTERIOR node - nothing to do but move on...
            break;

        case 0:
            // both i and j VRPH_UNUSED - merge the two routes making 1-i-j-1
            if(postsert.move(V,j,i)==true)
            {
                status[i]=VRPH_ADDED;
                status[j]=VRPH_ADDED;
                num_routes--;                    
            }
            break;

        case 1:

            // j VRPH_UNUSED and i previously VRPH_ADDED

            if(V->next_array[i]>0)
            {
                // i is the first node in its route
                if(presert.move(V,j,i)==true)
                {
                    status[i]=VRPH_INTERIOR;
                    status[j]=VRPH_ADDED;
                    num_routes--;
                }
            }
            else
            {
                // i is the last node in its route

                if(postsert.move(V,j,i)==true)
                {
                    status[i]=VRPH_INTERIOR;
                    status[j]=VRPH_ADDED;
                    num_routes--;
                }
            }
            break;

        case 2:
            // i VRPH_UNUSED and j previously VRPH_ADDED
            // Since j is not forbidden, it is in either position 2 or -2

            if(V->next_array[j]>0)
            {
                // j is 2nd in its route
                if(presert.move(V,i,j)==true)
                {
                    status[j]=VRPH_INTERIOR;
                    status[i]=VRPH_ADDED;
                    num_routes--;
                }
            }
            else
            {
                // j is -2nd in its route
                if(postsert.move(V,i,j)==true)
                {
                    status[j]=VRPH_INTERIOR;
                    status[i]=VRPH_ADDED;
                    num_routes--;

                }
            }

            break;

        case 3:                
            // i and j both previously added but neither VRPH_INTERIOR
            // Here we have to merge the two routes together        

            // First, make sure they are not in the same route!
            if( V->route_num[i] == V->route_num[j] )
                break;    

            // Now we will rearrange the routes containing both i and j if necessary
            // so that i is the first node and j is the last node in their routes.
            // This will be done by a route reversal if necessary.

            num_routes--;

            if(V->next_array[i]<=0 && V->next_array[j]<=0)
            {
                // Both in position -2 - reverse route containing i and merge
                V->reverse_route(V->route_num[i]);

                // So i is now in 2nd position of its route.
                i_route= V->route_num[i];
                j_route= V->route_num[j];

                if(concatenate.move(V,i_route,j_route)==true)
                {
                    status[i]=VRPH_INTERIOR;
                    status[j]=VRPH_INTERIOR;
                }
                break;
            }


            if(V->pred_array[i]<=0 && V->pred_array[j]<=0)
            {
                // Both in position 2
                V->reverse_route(V->route_num[j]);

                i_route= V->route_num[i];
                j_route= V->route_num[j];

                if(concatenate.move(V,i_route,j_route)==true)
                {

                    status[i]=VRPH_INTERIOR;
                    status[j]=VRPH_INTERIOR;
                }
                break;
            }


            if(V->next_array[i]>0 && V->next_array[j]<=0)
            {
                // i in 2nd and j in -2nd

                i_route= V->route_num[i];
                j_route= V->route_num[j];

                if(concatenate.move(V,i_route,j_route)==true)
                {                        
                    status[i]=VRPH_INTERIOR;
                    status[j]=VRPH_INTERIOR;
                }
                break;
            }

            if(V->next_array[j]>0 && V->next_array[i]<=0)
            {
                // j in 2nd and i in -2nd
                i_route= V->route_num[i];
                j_route= V->route_num[j];

                if(concatenate.move(V,j_route,i_route)==true)
                {
                    status[i]=VRPH_INTERIOR;
                    status[j]=VRPH_INTERIOR;
                }
                break;
            }
            break;
        }
        // End switch statement
    }

#if CW_DEBUG
    printf("Loop complete\n");
#endif

    delete [] status;

#if CW_DEBUG
    printf("Cleanup complete\n");
#endif

    // Set the record
    V->record = V->total_route_length;

    // Normalize the route numbers!
    V->normalize_route_numbers();

#if CW_DEBUG
    printf("Done normalizing\n");
#endif

    return true;
}

