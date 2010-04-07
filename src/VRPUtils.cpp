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

double VRPDistance(int type, double x1, double y1, double x2, double y2)
{
	///
	/// Distance function for 2D problems.
	///

	// This variation handles 2D cases
	// Type refers to the type of distance function used in the 
	// problem instance.

	double xd, yd;

	switch(type)
	{
	case VRPH_CEIL_2D:
		return ceil(sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)));

	case VRPH_MAX_2D:
		xd=VRPH_ABS(x1-x2);
		yd=VRPH_ABS(y1-y2);
		return VRPH_MAX(xd,yd);

	case VRPH_MAN_2D:
		xd=VRPH_ABS(x1-x2);
		yd=VRPH_ABS(y1-y2);
		return xd+yd;

	case VRPH_EUC_2D:
        xd=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))-floor(sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)));
        if(xd>=.5)
		    return ceil(sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)));
        else
            return floor(sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)));


	case VRPH_GEO:
		// Taken from Concorde code
		double lat1, lat2, long1, long2;
		double q1, q2, q3, q4, q5;

		lat1 = VRPH_PI * x1 / 180.0;
		lat2 = VRPH_PI * x2 / 180.0;

		long1 = VRPH_PI * y1 / 180.0;
		long2 = VRPH_PI * y2 / 180.0;

		q1 = cos (lat2) * sin(long1 - long2);
		q3 = sin((long1 - long2)/2.0);
		q4 = cos((long1 - long2)/2.0);
		q2 = sin(lat1 + lat2) * q3 * q3 - sin(lat1 - lat2) * q4 * q4;
		q5 = cos(lat1 - lat2) * q4 * q4 - cos(lat1 + lat2) * q3 * q3;
		return  (VRPH_RRR * atan2(sqrt(q1*q1 + q2*q2), q5) + 1.0);
		// Note - divided this by 1000??

    case VRPH_EXACT_2D:
        return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
    }

	fprintf(stderr,"Unknown distance function specified: %d\n",type);
	report_error("%s\n",__FUNCTION__);
	return -1;
}

int VRPDistanceCompare(const void *a, const void *b)
{
	///
	/// Compares two doubles.
	///

	double *d1;
	double *d2;

	d1=(double *)a;
	d2=(double *)b;

	if(d1>d2)
		return -1;
	else
		return 1;

}

int VRPIntCompare(const void *a, const void *b)
{
	///
	/// Compares two int's.
	///

	return ( *(int*)a - *(int*)b );

}

int double_int_compare(const void *a, const void *b)
{
	///
	/// Compares two double_int's using the double field.
	///

	struct double_int *s1;
	struct double_int *s2;

	s1=(struct double_int *)a;
	s2=(struct double_int *)b;

	if(s1->d > s2->d)
		return 1;
	else
		return -1;
}


int int_int_compare(const void *a, const void *b)
{
	///
	/// Compares two int_int's using the j field.
	///

	struct int_int *s1;
	struct int_int *s2;

	s1=(struct int_int *)a;
	s2=(struct int_int *)b;

	if(s1->j > s2->j)
		return 1;
	else
		return -1;
}

int VRPSavingsCompare (const void *a, const void *b)
{
	///
	/// Compares two VRPSavingsElement's using the savings field.
	///

	class VRPSavingsElement *s1;
	class VRPSavingsElement *s2;

	s1= (class VRPSavingsElement *)a;
	s2= (class VRPSavingsElement *)b;

	if( s1->savings > s2->savings)
		return -1;
	else
	{
		if(s1->savings < s2->savings)
			return 1;
		else
			return 0;
	}

}

int VRPNeighborCompare (const void *a, const void *b)
{
	/// 
	/// Compares two VRPNeighborElements using the val field.
	///

	VRPNeighborElement *s1;
	VRPNeighborElement *s2;

	s1= (VRPNeighborElement *)a;
	s2= (VRPNeighborElement *)b;

	if( s1->val > s2->val)
		return 1;
	else
	{
		if( s1->val <= s2->val)
			return -1;
		else
			return 0;

	}
}

int VRPSolutionCompare(const void *a, const void *b)
{
	///
	/// Compares two VRPSolution's using the obj field.
	///
	VRPSolution *s1;
	VRPSolution *s2;

	s1=(VRPSolution*)a;
	s2=(VRPSolution*)b;

	if( s1->obj > s2->obj)
		return 1;

	if( s1->obj <= s2->obj)
		return -1;
	else
		return 0;
}

int VRPAlphaCompare(const void *a, const void *b)
{
	///
	/// Compares two strings and sorts alphabetically.
	///

	return( strcmp(*(char **)a, *(char **)b) );
}

int VRPRouteCompare(const void *a, const void *b)
{

	///
	/// Compares two VRPRoutes using the length field.
	///

	VRPRoute *s1;
	VRPRoute *s2;

	s1=(VRPRoute*)a;
	s2=(VRPRoute*)b;

	if( s1->length> s2->length)
		return 1;

	if( s1->length <= s2->length)
		return -1;
	else
		// impossible
		return 0;


}

