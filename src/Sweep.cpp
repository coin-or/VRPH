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

struct sweep_node
{
    double theta;
    int index;
};

Sweep::Sweep()
{

}

bool Sweep::Construct(class VRP *V)
{
    ///
    /// Constructs an initial VRP solution by the simple sweep method.
    /// Start by picking a random node and then sweep counterclockwise
    /// and add nodes until we reach vehicle capacity or max route length.
    /// Improve after every imp_interval additions by running the
    /// provided heuristics (VRPH_DOWNHILL only).
    ///

    // First make sure that we have normalized everything so that VRPH_DEPOT is at 
    // origin
    if(V->depot_normalized==false)
        report_error("%s: Node locations must be normalized to run sweep\n",__FUNCTION__);

    // Begin by creating default routes - only supports full problems for now
    V->create_default_routes();

    int i,pos;
    int n=V->num_original_nodes;

    // Create a sorted list of nodes and thetas
    double_int *T;
    T = new double_int[n+1];
    for(i=0;i<n;i++)
    {
        T[i].k=i+1;// k is the index
        T[i].d=V->nodes[i+1].theta;// d is theta
    }

    // Sort by theta
    qsort(T,n,sizeof(double_int),double_int_compare);

    // We will start at a random place in this list and then "wrap around"
    // Pick a starting point 
    int start = VRPH_MIN(V->num_original_nodes, (int)(n*(lcgrand(5))));

    Postsert postsert;
    Presert presert;
    VRPMove M1, M2;
    bool post,pre;
    

    for(i=0;i<n;i++)
    {
        pos=start+i;

#if SWEEP_DEBUG
        printf("%5.2f: pos=%d:  Trying to insert %d after %d\n",V->total_route_length,
            pos%n, T[(pos+1)%n].k, T[pos%n].k);
#endif
 
        post=postsert.evaluate(V,T[(pos+1)%n].k, T[pos%n].k, &M1);
        pre=presert.evaluate(V,T[(pos+1)%n].k, T[pos%n].k, &M2);
        
        if(post || pre)
        {
            // At least 1 move is feasible
            if(post==true && pre==false)
                postsert.move(V,T[(pos+1)%n].k, T[pos%n].k);
            else
            {
                if(post==false && pre==true)
                    presert.move(V,T[(pos+1)%n].k, T[pos%n].k);
                else
                {
                    // Both feasible - pick the best move
                    if(M1.savings<=M2.savings)
                        postsert.move(V,T[(pos+1)%n].k, T[pos%n].k);
                    else
                        presert.move(V,T[(pos+1)%n].k, T[pos%n].k);
                }
            }
        }
        else
        {
            // Couldn't add before or after - start a new route
            // We don't have to do anything here since we started w/ default 
            // singleton routes
        
        }
    }
    

    return true;
}

