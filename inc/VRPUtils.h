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

#ifndef _VRP_UTIL_H
#define _VRP_UTIL_H


#define MAX_FILES				20000
#define MAX_FILENAME_LENGTH		40

// Misc
#define NUM_ELITE_SOLUTIONS		200
#define MAX_NUM_COLS			10000
#define NUM_ENTRIES				8
#define MAX_VRPH_TABU_LIST_SIZE 50

#define HASH_TABLE_SIZE			(1<<18)
#define SALT_1					0
#define SALT_2					11

struct htable_entry
{
	///
	/// Each entry in the hash table will contain
	/// an array of num_vals valid entries in hval[].
	/// Each entry is produced by hashing using
	/// SALT_2.  The length array contains the lengths
	/// of the routes in this position in the hash table.
	///

	int num_vals;
	int hash_val_2[NUM_ENTRIES];
	int tot;
	double length[NUM_ENTRIES];

};

struct int_int
{
	int i;
	int j;
};
struct double_int
{
	double d;
	int k;
};

class VRPSavingsElement
{
public:
	// Useful to make some of the bookkeeping simpler when
	// calculating the savings.
	double savings;
	int position;
	int i;
	int j;
};


class VRPNeighborElement
{
public:
	double val;
	int position;
};


class VRPViolation
{
public:
	double length_violation;
	int    capacity_violation;
};



class VRPSeedElement 
{
	double val;
	int position;
	int demand;
};

class VRPNeighborhood
{
public:
	int move_type;
	int node_1, node_2;
	class VRPMove *Moves;
	int size;

	VRPNeighborhood(int n);
};

struct VRPSegment
{
	/// 
	/// Contains information about a particular segment of
	/// a route.
	///

	int segment_start;
	int segment_end;

	int num_custs;
	int load;
	double len;

};



double VRPDistance(int type, double x1, double y1, double x2, double y2);
int VRPDistanceCompare(const void *a, const void *b);
int VRPIntCompare(const void *a, const void *b);
int VRPSavingsCompare (const void *a, const void *b);
int VRPNeighborCompare (const void *a, const void *b);
int VRPAlphaCompare(const void *a, const void *b);
int double_int_compare (const void *a, const void *b);
int int_int_compare (const void *a, const void *b);
int VRPSolutionCompare(const void *a, const void *b);
int VRPCheckTSPLIBString(char *s);
int VRPGetDimension(char *filename);
int VRPGetNumDays(char *filename);


#endif





