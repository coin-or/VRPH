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

VRPTabuList::VRPTabuList()
{
    /// 
    /// Default constructor for the VRPTabuList.
    ///

    this->max_entries=NUM_VRPH_TABU_ROUTES;
    this->num_entries=0;
    this->full=false;
    this->start_index=0;
    this->hash_vals1=NULL;
    this->hash_vals2=NULL;

    
}

VRPTabuList::VRPTabuList(int t)
{
    /// 
    /// Constructor for the VRPTabuList with t tabu routes.
    ///

    this->max_entries=t;
    this->num_entries=0;
    this->full=false;
    this->start_index=0;
    this->hash_vals1=new int[t];
    this->hash_vals2=new int[t];

    int i;
    for(i=0;i<this->max_entries;i++)
    {
        this->hash_vals1[i]=-1;
        this->hash_vals2[i]=-1;
    }
    

}

VRPTabuList::~VRPTabuList()
{
    /// 
    /// Destructor for the VRPTabuList.
    ///

    if(this->hash_vals1)
        delete [] hash_vals1;

    if(this->hash_vals2)
        delete [] hash_vals2;
}

void VRPTabuList::update_list(VRPRoute *r)
{
    ///
    /// Updates the tabu list by adding the route r.
    ///

    r->hash_val=r->hash(SALT_1);
    r->hash_val2=r->hash(SALT_2);

#if VRPH_TABU_DEBUG
    printf("Adding route with hashes (%d, %d) and length, load (%5.2f, %d) to tabu list\n",
        r->hash_val,r->hash_val2,r->length,r->load);
#endif
    
    if(this->num_entries < this->max_entries)
    {
        // Update the lists of hash_vals
        this->hash_vals1[this->num_entries]=r->hash_val;
        this->hash_vals2[this->num_entries]=r->hash_val2;

        this->num_entries++;
#if VRPH_TABU_DEBUG
        printf("Tabu list after updating\n");
        this->show();
#endif

        return;
    
    }

    // The list is full - overwrite the current start_index entry
    // and increment start_index

    this->hash_vals1[this->start_index]=r->hash_val;
    this->hash_vals2[this->start_index]=r->hash_val2;
    this->start_index = ((this->start_index + 1) % (this->num_entries));
    this->full=true;

#if VRPH_TABU_DEBUG
    printf("Tabu list (full) after updating\n");
    this->show();
#endif

    return;
}

void VRPTabuList::empty()
{
    ///
    /// Removes all entries from the tabu list.
    ///

#if VRPH_TABU_DEBUG
    printf("Emptying tabu list -  max_entries is %d\n", this->max_entries);
#endif
    int i;
    for(i=0;i<this->max_entries;i++)
    {
        this->hash_vals1[i]=-1;
        this->hash_vals2[i]=-1;
    }

    this->start_index=0;
    this->num_entries=0;
    this->full=false;

    return;

}

void VRPTabuList::show()
{
    ///
    /// Shows the hash values of the current tabu list, starting with start_index
    /// and listing all current entries.
    ///

    printf("Tabu List currently has %d entries starting at %d (max is %d)\n",
        this->num_entries,this->start_index, this->max_entries);

    for(int i=0;i<this->num_entries;i++)
    {
        printf("Tabu entry %d: (%d,%d)\n",((start_index+i)%this->max_entries),
            this->hash_vals1[((start_index+i)%this->max_entries)],
            this->hash_vals2[((start_index+i)%this->max_entries)]);
    }

    return;


}

