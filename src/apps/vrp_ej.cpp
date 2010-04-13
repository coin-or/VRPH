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
    /// A main() routine to test the procedures and performance of 
    /// various routines for ejecting and injecting groups of nodes
    /// using two strategies.
    ///

    VRPH_version();

    char infile[VRPH_STRING_SIZE];
    char solfile[VRPH_STRING_SIZE];
    char outfile[VRPH_STRING_SIZE];
    bool has_outfile=false, has_solfile=false;
    int i,j,n,num_ejected, num_trials, num_heur_sols, num_improvements;
    int *ejected_buff, *heur_solbuff, *ej_solbuff, *best_solbuff;
    int method=-1;
    clock_t start, stop;

    // Default values
    bool verbose=false;

    if(argc < 7 || (strncmp(argv[1],"-help",5)==0 || strcmp(argv[1],"-h")==0 || strcmp(argv[1],"--h")==0))
    {        
        fprintf(stderr,"Usage: %s -f vrp_file -j num_ejected -t num_trials -m method [-s sol_file -out out_file -n num_heur_sols -v]\n",argv[0]);
        fprintf(stderr,
            "\t num_ejected should be something less than 20 or so\n"
            "\t num_trials can be fairly large as the procedure is fast (say 1000)\n"
            "\t method must be 0 (RANDOM search), 1 (REGRET search)\n"
            "\t Options:\n"
            "\t Can start with a solution in sol_file or it will generate an initial\n"
            "\t    solution for you\n"
            "\t Can write the final best solution discovered to out_file\n"
            "\t Adding -v will print verbose output\n");
        exit(-1);
    }

    bool has_filename=false;
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

    // Declare some buffers for solutions, nodes to eject, etc.
    ejected_buff=new int[n+2];
    heur_solbuff=new int[n+2];
    ej_solbuff=new int[n+2];
    best_solbuff=new int[n+2];
    num_ejected=num_trials=0;
    num_heur_sols=1;

    // Now process the options
    for(i=1;i<argc;i++)
    {
        if(strcmp(argv[i],"-v")==0)
            verbose=true;

        if(strcmp(argv[i],"-j")==0)
            num_ejected=atoi(argv[i+1]);

        if(strcmp(argv[i],"-n")==0)
            num_heur_sols=atoi(argv[i+1]);

        if(strcmp(argv[i],"-s")==0)
        {
            has_solfile=true;
            num_heur_sols=0;
            strcpy(solfile,argv[i+1]);
        }

        if(strcmp(argv[i],"-t")==0)
            num_trials=atoi(argv[i+1]);

        if(strcmp(argv[i],"-out")==0)
        {
            has_outfile=true;
            strcpy(outfile,argv[i+1]);
        }

        if(strcmp(argv[i],"-m")==0)
        {
            method=atoi(argv[i+1]);
            if(method!=RANDOM && method!=REGRET)
            {
                fprintf(stderr,"Method must be either 0 (RANDOM search) or 1 (REGRET search)\n");
                exit(-1);
            }
        }
    }


    // Load the problem data
    V.read_TSPLIB_file(infile);    
    ClarkeWright CW(n);
    double heur1;
    double best_heur_sol=VRP_INFINITY;
    double best_final_sol=VRP_INFINITY;
    double heur_time=0,ej_time=0;
    double lambda;
    double *heur_sols=new double[num_heur_sols];
    double *final_sols=new double[num_heur_sols];

    for(i=0;i<num_heur_sols;i++)
    {
        // Generate a solution w/ RTR that should be a good starting point for the search
        start=clock();
        lambda=.5+1.5*lcgrand(0);
        // Start with a clean VRP object
        V.reset();
        CW.Construct(&V, lambda, false);
        if(verbose)
            printf("CW solution %d[%5.3f]: %f\n",i,lambda,V.get_total_route_length()-V.get_total_service_time());
        V.RTR_solve(ONE_POINT_MOVE+TWO_POINT_MOVE+TWO_OPT+VRPH_USE_NEIGHBOR_LIST,30,5,2,.01,30,VRPH_LI_PERTURB,
            VRPH_FIRST_ACCEPT,false);
        stop=clock();
        heur_time+=((double)(stop-start))/CLOCKS_PER_SEC;
        heur_sols[i]=V.get_total_route_length()-V.get_total_service_time();
        if(verbose)
            printf("RTR solution %d: %f\n",i,V.get_total_route_length()-V.get_total_service_time());
        
        // Record the value of the first solution
        if(i==0)
            heur1=V.get_total_route_length()-V.get_total_service_time();

        if(i==0 || V.get_total_route_length()-V.get_total_service_time() <= best_heur_sol)
        {
            best_heur_sol = V.get_total_route_length()-V.get_total_service_time();
            if(verbose)
                printf("Found best sol: %f\n",V.get_total_route_length()-V.get_total_service_time());
        }
        // Export this solution to ej_solbuff
        V.export_solution_buff(ej_solbuff);

        double heur_obj=V.get_total_route_length()-V.get_total_service_time();
        if(verbose)
            printf("Starting ejection routines with solution %f\n",heur_obj);

        num_improvements=0;
        double orig_obj=heur_obj;
        start=clock();
        for(j=0;j<num_trials;j++)
        {
            // Start with the best solution derived from this RTR run
            V.import_solution_buff(ej_solbuff);
            // Now pick random nodes to eject - start by finding a random non-VRPH_DEPOT node
            int r=VRPH_DEPOT;
            while(r==VRPH_DEPOT)
                r=(int)(lcgrand(11)*(n-1));

            // Eject a set of random nodes near node r
            V.eject_neighborhood(r,num_ejected,ejected_buff);

            if(method==REGRET)
            {
                // Inject them using "cheapest insertion with regret"
                V.inject_set(num_ejected, ejected_buff,VRPH_REGRET_SEARCH, 50);
                double regret_obj=V.get_total_route_length()-V.get_total_service_time();
                if(regret_obj<orig_obj)
                {
                    if(verbose)
                        printf("Attempt %04d:  REGRET improved original: %f<%f\n",j, regret_obj,orig_obj);
                    V.export_solution_buff(ej_solbuff);
                    orig_obj=regret_obj;
                    num_improvements++;
                }
            }

            if(method==RANDOM)
            {
                // Inject them again using a random order and cheapest insertion
                V.inject_set(num_ejected, ejected_buff,VRPH_RANDOM_SEARCH, 50);        
                double random_obj=V.get_total_route_length();
                if(random_obj<orig_obj)
                {
                   if(verbose)
                        printf("Attempt %04d:  RANDOM improved original: %f<%f\n",j, random_obj,orig_obj);
                    V.export_solution_buff(ej_solbuff);
                    orig_obj=random_obj;
                    num_improvements++;
                }
            }    
        }
        // end j loop
        stop=clock();
        ej_time+=(double)(stop-start)/CLOCKS_PER_SEC;
        
        // Import the best solution we found
        V.import_solution_buff(ej_solbuff);
        if(V.get_total_route_length()-V.get_total_service_time()<best_final_sol)
        {
            best_final_sol=V.get_total_route_length()-V.get_total_service_time();
            V.export_solution_buff(best_solbuff);
        }
        final_sols[i]=V.get_total_route_length()-V.get_total_service_time();
    }

    // Restore the best solution found
    V.import_solution_buff(best_solbuff);


    // Output is
    // heur[i] ej[i]
    // best_heur best_ej heur_time ej_time
    for(i=0;i<num_heur_sols;i++)
        printf("%5.3f %5.3f\n",heur_sols[i], final_sols[i]);
    printf("%5.3f %5.3f %5.3f %5.3f\n",best_heur_sol,
        V.get_total_route_length()-V.get_total_service_time(),heur_time,ej_time);

    if(has_outfile)
        V.write_solution_file(outfile);


    // Clean up
    delete [] ejected_buff;
    delete [] heur_solbuff;
    delete [] ej_solbuff;
    delete [] best_solbuff;
    delete [] heur_sols;
    delete [] final_sols;

    return 0;
}
