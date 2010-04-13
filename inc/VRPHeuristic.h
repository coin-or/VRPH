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

#ifndef _VRP_HEUR_H
#define _VRP_HEUR_H

// Various rules that can be or'ed together
#define VRPH_DOWNHILL                           1
#define VRPH_RECORD_TO_RECORD                   (1<<1)
#define VRPH_SIMULATED_ANNEALING                (1<<2)
#define VRPH_FIRST_ACCEPT                       (1<<3)
#define VRPH_BEST_ACCEPT                        (1<<4)
#define VRPH_LI_ACCEPT                          (1<<5)
#define VRPH_INTER_ROUTE_ONLY                   (1<<6)
#define VRPH_INTRA_ROUTE_ONLY                   (1<<7)
#define VRPH_USE_NEIGHBOR_LIST                  (1<<8)
#define VRPH_FREE                               (1<<9)
#define VRPH_BALANCED                           (1<<10)
#define VRPH_FORWARD                            (1<<11)
#define VRPH_BACKWARD                           (1<<12)
#define VRPH_RANDOMIZED                         (1<<13)
#define VRPH_SAVINGS_ONLY                       (1<<14)
#define VRPH_MINIMIZE_NUM_ROUTES                (1<<15)
#define VRPH_FIXED_EDGES                        (1<<17)
#define VRPH_ALLOW_INFEASIBLE                   (1<<18)
#define VRPH_NO_NEW_ROUTE                       (1<<19)
#define VRPH_TABU                               (1<<20)

// Heuristic operations
#define ONE_POINT_MOVE                          (1<<21)
#define TWO_POINT_MOVE                          (1<<22)
#define TWO_OPT                                 (1<<23)
#define OR_OPT                                  (1<<24)
#define THREE_OPT                               (1<<25)
#define CROSS_EXCHANGE                          (1<<26)
#define THREE_POINT_MOVE                        (1<<27)
#define KITCHEN_SINK                            (1<<28)

#define ALL_HEURISTICS                          (1<<20)|(1<<21)|(1<<22)|(1<<23)|(1<<24)|(1<<25)|(1<<26)|(1<<27)

// Values of heuristics that can be used in functions such as 
// clean_routes

#define NUM_HEURISTICS                  7

#define ONE_POINT_MOVE_INDEX            0                    
#define TWO_POINT_MOVE_INDEX            1
#define TWO_OPT_INDEX                   2
#define OR_OPT_INDEX                    3
#define THREE_OPT_INDEX                 4
#define CROSS_EXCHANGE_INDEX            5
#define THREE_POINT_MOVE_INDEX          6



// Move types
#define PRESERT                          1
#define POSTSERT                         2
#define CONCATENATE                      3
#define SWAP_ENDS                        4
#define FLIP                             5
#define MOVE_STRING                      6
#define SWAP                             7

#endif





