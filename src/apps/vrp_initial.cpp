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

#define RANDOM 0
#define REGRET 1

int main(int argc, char *argv[])
{
    ///
    /// A main() routine to illustrate usage of Clarke Wright and 
    /// Sweep methods to construct an initial solution.
    ///

    VRPH_version();

    int i,n;
    char infile[200];

    if(argc < 5 || (strncmp(argv[1],"-help",5)==0 || strcmp(argv[1],"-h")==0 || strcmp(argv[1],"--h")==0))
    {        
        fprintf(stderr,"Usage: %s -f vrp_file -m method [-c]\n",argv[0]);
        fprintf(stderr,
            "\t method should be 0 for CW, 1 for Sweep\n"
            "\t If -c option is given, then all routes are cleaned up at the end\n"
            "\t\t by running intra-route improvements\n");
            exit(-1);
    }

    bool has_filename=false, clean_up=false;
    for(i=1;i<argc;i++)
    {
        if(strcmp(argv[i],"-f")==0)
        {
            strcpy(infile,argv[i+1]);
            has_filename=true;            
        }
    }

    if(has_filename==false)
        report_error("No input file given\n");

    n=VRPGetDimension(infile);
    // Get # of non-VRPH_DEPOT nodes
    VRP V(n);

    // Now process the options
    int method=-1;
    for(i=1;i<argc;i++)
    {
        if(strcmp(argv[i],"-m")==0)
            method=atoi(argv[i+1]);
         if(strcmp(argv[i],"-c")==0)
            clean_up=true;
    }
    if(method<0)
        report_error("method must be 0 or 1\n");

    // Load the problem data
    V.read_TSPLIB_file(infile);

    Sweep sweep;
    ClarkeWright CW(n);

    if(method==0)
    {
        // Use Clarke Wright with \lambda=1
        printf("Finding initial solution using Clarke-Wright algorithm\n");
        CW.Construct(&V, 1.0, false);
        
    }
    else
    {
        printf("Finding initial solution using Sweep algorithm\n");
        sweep.Construct(&V);
    }

    if(clean_up)
    {
        printf("Total route length before clean up: %f\n",V.get_total_route_length()-V.get_total_service_time());
        V.normalize_route_numbers();
        for(i=1;i<=V.get_total_number_of_routes();i++)
            V.clean_route(i,ONE_POINT_MOVE+TWO_POINT_MOVE+TWO_OPT);
    }

    // Print a summary of the solution
    V.summary();
    return 0;

}

