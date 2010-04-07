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
#include "randvals.h"

VRPSolutionWarehouse::VRPSolutionWarehouse()
{
    ///
    /// Default constructor for the solution warehouse.
    ///

    this->max_size=0;
    this->sols=NULL;
    this->worst_obj=VRP_INFINITY;
    this->num_sols=0;

}

VRPSolutionWarehouse::VRPSolutionWarehouse(int max_sols, int n)
{
    ///
    /// Constructs a warehouse of max_sols solutions, with
    /// sufficient memory for an n-node problem.
    ///

    this->max_size=max_sols;
    this->sols=new VRPSolution[this->max_size];
    // This calls the default constructor so we have to allocate the memory for
    // the sol buffer still... 
    for(int i=0;i<this->max_size;i++)
        this->sols[i].sol=new int[n+2];

    this->hash_table=new struct htable_entry[HASH_TABLE_SIZE];

    // Zeroize the hash table
    for(int i=0;i<HASH_TABLE_SIZE;i++)
        hash_table[i].num_vals=0;

    this->worst_obj=VRP_INFINITY;
    this->num_sols=0;

}
VRPSolutionWarehouse::~VRPSolutionWarehouse()
{
    ///
    /// Destructor for the solution warehouse.
    ///

    if(this->hash_table)
        delete [] this->hash_table;    
    //for(int i=0;i<this->max_size;i++)
    //    this->sols[i].~VRPSolution();

    if(this->sols)
        delete [] this->sols; // Calls VRPSolution destructor

}


int VRPSolutionWarehouse::add_sol(VRPSolution *new_sol, int start_index)
{
    ///
    /// Attempts to add a solution to the warehouse.  Returns the index
    /// that the new solution was placed at.  Returns -1 if the solution
    /// was not placed in the warehouse.  The start_index provides a place
    /// to begin the search -- useful when inserting multiple solutions
    /// whose order is already known.  Use start_index=0 if no information
    /// about the solution's position  is known.  The VRPSolution being passed
    /// in should be in "canonical form" for the hash function to work 
    /// properly!!
    ///

    int i,j;

    if(start_index<0)
        return -1;

    if( (this->num_sols == this->max_size) && (new_sol->obj > this->sols[num_sols-1].obj) )
    {
#if WAREHOUSE_DEBUG
        printf("WH is full and %f>%f(=?=%f)\n",new_sol->obj ,this->sols[num_sols-1].obj,
            this->worst_obj);
#endif
        // It doesn't beat the worst solution and we are full
        return -1;
    }

    // First check the hash table;
    int hash_val=new_sol->hash(SALT_1);
    int hash_val2=new_sol->hash(SALT_2);

    if( (this->hash_table[hash_val].num_vals)>0)
    {
#if WAREHOUSE_DEBUG
    printf("Possible duplicate at position %d\n",hash_val);
#endif
        // We have possibly seen this solution before -- hash again and check the lengths
        for(j=0; j<this->hash_table[hash_val].num_vals;j++)
        {
            if( (VRPH_ABS(this->hash_table[hash_val].length[j] - new_sol->obj)<VRPH_EPSILON) &&
                (this->hash_table[hash_val].hash_val_2[j]==hash_val2) )
            {
                // This must be the same solution
#if WAREHOUSE_DEBUG
                printf("Duplicate solution\n");
#endif
                return -1;
            }

        }
    }

#if WAREHOUSE_DEBUG
    printf("New solution found (%d now at this location)\n",this->hash_table[hash_val].num_vals+1);
#endif

    // We believe this is a new solution that should live in position hash_val
    // Add a new length and hval2 entry
    this->hash_table[hash_val].length[this->hash_table[hash_val].num_vals]=new_sol->obj;
    this->hash_table[hash_val].hash_val_2[this->hash_table[hash_val].num_vals]=hash_val2;
    // Increment the counter
    this->hash_table[hash_val].num_vals++;

    // The hash table is updated - now add the solution in the correct location
    // Otherwise, we have to find out where to put the solution--start searching at
    // start_index-1
    for(i=VRPH_MAX(0,start_index-1); i<this->num_sols;i++)
    {
        // Break if new_sol < sols[i] since the list in the WH is sorted
        if( new_sol->obj < this->sols[i].obj )
            break;
    }

    // This solution should be in position i
#if WAREHOUSE_DEBUG
    printf("Putting %f before %dth[%f]\n",new_sol->obj,i,sols[i].obj);
#endif

    // Make room in position i
    for(j=VRPH_MIN(num_sols,max_size-1); j>i; j--)
    {
        this->sols[j].obj=this->sols[j-1].obj;
        this->sols[j].n=this->sols[j-1].n;
        this->sols[j].in_IP=this->sols[j-1].in_IP;
        this->sols[j].time=this->sols[j-1].time;
        memcpy(this->sols[j].sol,this->sols[j-1].sol,((new_sol->n) + 2)*sizeof(int));
    }
#if WAREHOUSE_DEBUG
    printf("WH after making room\n");
    //this->show();
#endif

    // Insert new solution
    if(i>=this->max_size)
        return -1;

    // Otherwise insert the new solution
    this->sols[i].obj=new_sol->obj;
    this->sols[i].n=new_sol->n;
    this->sols[i].in_IP=false;  // new solution
    this->sols[i].time=new_sol->time;
    memcpy(this->sols[i].sol,new_sol->sol,((new_sol->n) + 2)*sizeof(int));
    if(this->num_sols < this->max_size)
        this->num_sols++;
    // Update worst
    this->worst_obj=this->sols[num_sols-1].obj;

#if WAREHOUSE_DEBUG
    this->show();
    printf("Returning %d-WH has %d sols now\n", i, this->num_sols);
#endif

    return i;
    // Return the value of the solutions index in the WH


}

void VRPSolutionWarehouse::show()
{
    ///
    /// Debugging function to show the current solutions in the warehouse.
    ///

    printf("Solution Warehouse contents\n%d sols, worst is %f\n",num_sols,worst_obj);
    for(int i=0;i<this->num_sols;i++)
    {
        printf("%03d\t%5.3f\t",i,this->sols[i].obj);
        if(this->sols[i].in_IP==true)
            printf("true\n");
        else
            printf("false\n");
    }


    return;



}
bool VRPSolutionWarehouse::liquidate()
{    
    ///
    /// Removes all solutions from the warehouse.
    ///

    int i;

    for(i=0;i<this->num_sols;i++)
        // Erase the existing solutions
        memset(this->sols[i].sol,0,this->sols[i].n*sizeof(int));
    
    this->num_sols=0;
    this->worst_obj=VRP_INFINITY;

    // Zeroize the hash table
    for(i=0;i<HASH_TABLE_SIZE;i++)
    {
        for(int j=0;j<this->hash_table[i].num_vals;j++)
        {
            this->hash_table[i].hash_val_2[j]=0;
            this->hash_table[i].length[j]=0;

        }
        this->hash_table[i].num_vals=0;
        
    }

    return true;

}


void VRPSolutionWarehouse::sort_sols()
{
    ///
    /// Sorts the solutions in the warehouse in increasing order of the
    /// objective function value.
    ///

    qsort(this->sols,this->num_sols,sizeof(VRPSolution),VRPSolutionCompare);
    return;

}
int VRPSolution::hash(int salt)
{
    /// 
    /// Computes a hash of the solution, 
    /// returning an integer in the range [0,n-1].  The solution
    /// buffer should be in canonical form so that the hash value is in terms
    /// of the ordering.
    ///

    int val = 0;
    int i;
    
    for(i=0;i<this->n-1; i++)//was -2??
        val ^= (randvals[(salt + VRPH_ABS(this->sol[i])+VRPH_ABS(this->sol[VRPH_MIN((this->n)-1,i+1)]) )% NUM_RANDVALS]);

    val=val&(HASH_TABLE_SIZE-1);
    // Get the low # of bits
    return val;

}


VRPSolution::VRPSolution(int n)
{
    /// 
    /// Constructor for an n-node VRPSolution.
    ///

    this->n=n;
    this->sol=new int[n+2];
    this->in_IP=false;
    this->obj=0;
    this->time=0;


}


VRPSolution::VRPSolution()
{
    /// 
    /// Default constructor for the VRPSolution.
    ///
    
    this->n=0;
    this->time=0;
    this->in_IP=false;
    this->obj=0;
    this->sol=NULL;

}

VRPSolution::~VRPSolution()
{
    ///
    /// Destructor for the VRPSolution.
    ///

    if(this->sol)
        delete [] this->sol;
}

