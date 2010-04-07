////////////////////////////////////////////////////////////
//                                                        //
// This file is part of the VRPH C/C++ package for        //
// solving the Vehicle Routing Problem by Chris Groer     //
// Code is free for use by academic researchers.          //
// For other purposes, contact cgroer@gmail.com           //
//                                                        //
////////////////////////////////////////////////////////////

#include "VRPH.h"

void generate_li_vrp(int A, int B, int Q, int L, const char *outfile)
{
	///
	/// Generates a TSPLIB-formatted VRP file given the parameters
	/// using the generator of Li, et al., 2005.
	///

	int i,w,k,n,g,q;
	double x,y;
	FILE *out;

	if( (out=fopen(outfile,"w"))==NULL)
	{
		fprintf(stderr,"Error opening %s for writing\n",outfile);
		report_error("%s: Error in generate_li_vrp\n",__FUNCTION__);
	}

	n= A*B;
	
	fprintf(out,"NAME: Li_Benchmark_%d_%d.vrp\n",A,B);
	fprintf(out,"COMMENT: None\n");
	fprintf(out,"TYPE: CVRP\n");
	fprintf(out,"DIMENSION: %d\n",n+1);
	fprintf(out,"CAPACITY: %d\n",Q);
	fprintf(out,"DISTANCE: %d\n",L);
	fprintf(out,"EDGE_WEIGHT_TYPE: FUNCTION\n");
	fprintf(out,"EDGE_WEIGHT_FORMAT: EUC_2D\n");
	fprintf(out,"NODE_COORD_TYPE: TWOD_COORDS\n");
	fprintf(out,"NODE_COORD_SECTION\n");

	// Now print the coordinates
	w=0;
	for(k=1;k<=B;k++)
	{
		g = 30 * k;
		for(i=1;i<=A;i++)
		{
			w++;
			x = ((double)g) * cos(2.0*(((double)i)-1)*PI/((double)A));
			y = ((double)g) * sin(2.0*(((double)i)-1)*PI/((double)A));
			fprintf(out,"%d %2.4f %2.4f\n",w,x,y);
		}
	}

	fprintf(out,"DEMAND_SECTION\n");

	// Now print the demands
	w=0;
	for(k=1;k<=B;k++)
	{
		g = 30 * k;
		for(i=1;i<=A;i++)
		{
			w++;
			if(i%4==2 || i%4==3)
				q=30;
			else
				q=10;
			fprintf(out,"%d %d\n",w,q);
		}
	}
	fprintf(out,"DEPOT_SECTION\n");
	fprintf(out,"1\n");
	fprintf(out,"-1\n");
	fprintf(out,"EOF\n");

	fclose(out);
	return;
}

