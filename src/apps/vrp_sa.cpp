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

int main(int argc, char *argv[])
{
    ///
    /// This is a main() routine that uses Simulated
    /// Annealing to generate VRP solutions.
    ///

    VRPH_version();

    char in[VRPH_STRING_SIZE];
    char out[VRPH_STRING_SIZE];
    char plotfile[VRPH_STRING_SIZE];
    char solfile[VRPH_STRING_SIZE];
    int i;
    int n;

    // Default values
    bool verbose=false;
    double starting_temperature = 2;
    double cooling_ratio = .99;
    int num_loops=200;
    int num_lambdas=3;
    int iters_per_loop=2;
    int nlist_size=10;
    int perturb_type=VRPH_LI_PERTURB ;

    double lambda_vals[VRPH_MAX_NUM_LAMBDAS];
    bool new_lambdas=false;
    bool has_heuristics=false;
    bool has_outfile=false;
    bool has_plotfile=false;
    bool has_solfile=false;
    int heuristics=0;

    if(argc<2 || (strncmp(argv[1],"-help",5)==0)||(strncmp(argv[1],"--help",6)==0)||(strncmp(argv[1],"-h",2)==0))
    {        
        fprintf(stderr,"Usage: %s -f <vrp_input_file> [options]\n",argv[0]);
        fprintf(stderr,"Options:\n");

        fprintf(stderr,"\t-help prints this help message\n"); 

        fprintf(stderr,"\t-sol <solfile> begins with an existing solution contained\n");
        fprintf(stderr,"\t\t in solfile.\n");

        fprintf(stderr,"\t-v prints verbose output to stdout\n");

        fprintf(stderr,"\t-i <num_iters> runs the SA procedure for num_iters iterations\n");
        fprintf(stderr,"\t\t before cooling\n");

        fprintf(stderr,"\t-n <num_loops> runs the SA procedure a total of num_loops times\n");

        fprintf(stderr,"\t-t <starting_temperature> runs the SA procedure starting at\n");
        fprintf(stderr,"\t\t this temperature\n");

        fprintf(stderr,"\t-c <cooling_ratio> runs the SA by decreasing the temperature by a\n");
        fprintf(stderr,"\t\t multiplicative factor of cooling_ratio every num_iters moves\n");

        fprintf(stderr,"\t-h <heuristic> applies the specified heuristics (can be repeated)\n");
        fprintf(stderr,"\t\t default is ONE_POINT_MOVE, TWO_POINT_MOVE, and TWO_OPT\n");
        fprintf(stderr,"\t\t others available are OR_OPT, THREE_OPT, and CROSS_EXCHANGE\n");
        fprintf(stderr,"\t\t Example: -h OR_OPT -h THREE_OPT -h TWO_OPT\n");

        fprintf(stderr,"\t-l <num_lambdas> runs the SA procedure from num_lambdas\n");
        fprintf(stderr,"\t\t different initial CW solutions using a random lambda chosen\n");
        fprintf(stderr,"\t\t from (0.5,2.0)\n");
        fprintf(stderr,"\t\t default is to use lambda in (.6, 1.4, 1.6).\n");

        fprintf(stderr,"\t-s <nlist_size> uses the nlist_size nearest neighbors in the search\n");
        fprintf(stderr,"\t\t (default is 40).\n");

        fprintf(stderr,"\t-o <out_file> writes the solution to the provided file\n");
        fprintf(stderr,"\t-plot <plot_file> plots the best solution to the provided file\n");
        exit(-1);
    }


    bool has_filename=false;
    for(i=1;i<argc;i++)
    {
        if(strcmp(argv[i],"-f")==0)
        {
            // Get the input file name
            strncpy(in,argv[i+1],strlen(argv[i+1]));
            in[strlen(argv[i+1])]='\0';
            has_filename=true;
        }
    }
    if(has_filename==false)
    {
        fprintf(stderr,"No input file given\nUsage: %s -f <filename> [options]\n",argv[0]);
        exit(-1);
    }

    n=VRPGetDimension(in);
    VRP V(n);

    // Now process the options
    for(i=1;i<argc;i++)
    {
        if(strcmp(argv[i],"-v")==0)
            verbose=true;

        if(strcmp(argv[i],"-t")==0)
            starting_temperature=(double)(atof(argv[i+1]));

        if(strcmp(argv[i],"-i")==0)
            iters_per_loop=atoi(argv[i+1]);

        if(strcmp(argv[i],"-n")==0)
            num_loops=atoi(argv[i+1]);

        if(strcmp(argv[i],"-h")==0)
        {
            has_heuristics=true;
            if(strcmp(argv[i+1],"ONE_POINT_MOVE")==0)
                heuristics+=ONE_POINT_MOVE;
            if(strcmp(argv[i+1],"TWO_POINT_MOVE")==0)
                heuristics+=TWO_POINT_MOVE;
            if(strcmp(argv[i+1],"TWO_OPT")==0)
                heuristics+=TWO_OPT;
            if(strcmp(argv[i+1],"OR_OPT")==0)
                heuristics+=OR_OPT;
            if(strcmp(argv[i+1],"THREE_OPT")==0)
                heuristics+=THREE_OPT;
            if(strcmp(argv[i+1],"CROSS_EXCHANGE")==0)
                heuristics+=CROSS_EXCHANGE;
            if(strcmp(argv[i+1],"THREE_POINT_MOVE")==0)
                heuristics+=THREE_POINT_MOVE;
        }

        if(strcmp(argv[i],"-l")==0)
        {
            new_lambdas=true;
            num_lambdas = atoi(argv[i+1]);

            if(num_lambdas>VRPH_MAX_NUM_LAMBDAS)
            {
                fprintf(stderr,"%d>VRPH_MAX_NUM_LAMBDAS\n",num_lambdas);
                exit(-1);
            }

            // Fill the array with random values
            printf("Creating %d random lambdas\n",num_lambdas);
            for(int j=0;j<num_lambdas;j++)
            {
                // Generate a random lambda in (0.5,2)
                lambda_vals[j] = 0.5 + 1.5*((double)lcgrand(0));
            }
        }

        if(strcmp(argv[i],"-sol")==0)
        {
            has_solfile=true;
            strcpy(solfile,argv[i+1]);
        }

        if(strcmp(argv[i],"-c")==0)
            cooling_ratio=(double)(atof(argv[i+1]));

        if(strcmp(argv[i],"-s")==0)
            nlist_size=atoi(argv[i+1]);

        if(strcmp(argv[i],"-p")==0)
        {
            perturb_type=atoi(argv[i+1]);
            if(perturb_type!=0 && perturb_type!=1)
            {
                fprintf(stderr,"Perturb type must be 0 or 1!\n");
                exit(-1);
            }
        }

        if(strcmp(argv[i],"-o")==0)
        {
            has_outfile=true;
            strcpy(out,argv[i+1]);

        }

        if(strcmp(argv[i],"-plot")==0)
        {
            has_plotfile=true;
            strcpy(plotfile,argv[i+1]);

        }

    }

    if(new_lambdas==false)
    {
        num_lambdas=3;
        lambda_vals[0]=.6;
        lambda_vals[1]=1.4;
        lambda_vals[2]=1.6;
    }

    if(has_heuristics==false)
        heuristics=ONE_POINT_MOVE+TWO_POINT_MOVE+TWO_OPT; //default


    // Load the problem data
    V.read_TSPLIB_file(in);

    // Create the neighbor_lists-we may use a smaller size depending on the parameter
    // but we have the largest possible here...
    V.create_neighbor_lists(VRPH_MIN(MAX_NEIGHBORLIST_SIZE,n));
    ClarkeWright CW(n);

    double best_obj=VRP_INFINITY;
    double this_obj, start_obj;
    int *best_sol=new int[n+2];

    time_t start=clock();
    if(has_solfile == false)
    {
        for(i=0;i<num_lambdas;i++)
        {
            CW.Construct(&V,lambda_vals[i], false);

            if(verbose)
                printf("CW[%f] solution: %f\n",lambda_vals[i], V.get_total_route_length());

            this_obj = V.SA_solve(heuristics, starting_temperature, cooling_ratio, iters_per_loop,
                num_loops, nlist_size, verbose);

            if(V.get_best_total_route_length()<best_obj)
            {
                best_obj=V.get_best_total_route_length();
                V.export_canonical_solution_buff(best_sol);
            }

            if(verbose)
                printf("Improved solution: %f\n",this_obj);        

        }
    }
    else
    {
        V.read_solution_file(solfile);
        start_obj=V.get_total_route_length();
        this_obj = V.SA_solve(heuristics, starting_temperature, cooling_ratio, iters_per_loop,
            num_loops, nlist_size, verbose);
        if(V.get_best_total_route_length()<best_obj)
        {
            best_obj=V.get_best_total_route_length();
            V.export_canonical_solution_buff(best_sol);
        }
        if(verbose)
            printf("Improved solution: %f\n",this_obj);    

    }

    // Restore the best solution found
    V.import_solution_buff(best_sol);

    delete [] best_sol;

    if(has_outfile)
        V.write_solution_file(out);
    best_obj=this_obj;

    if(verbose)
        printf("Solution before cleaning individual routes: %5.3f\n",V.get_total_route_length()-
        V.get_total_service_time());
    // Clean up the individual routes since the SA routine doesn't do this for us
    for(i=1;i<=V.get_total_number_of_routes();i++)
        V.clean_route(i,ONE_POINT_MOVE+TWO_POINT_MOVE+TWO_OPT+THREE_OPT+THREE_POINT_MOVE);
    if(verbose)
    {
        printf("Solution after cleaning individual routes: %5.3f\n",V.get_total_route_length()-
            V.get_total_service_time());
        V.summary();
        V.print_stats();
    }

    if(has_plotfile)
    {
#if !HAS_PLPLOT
        fprintf(stderr,"Need PLPLOT to plot solution\n");
        exit(-1);
#endif

        V.plot(plotfile);

    }

    time_t stop=clock();
    double elapsed=(double)(stop-start)/CLOCKS_PER_SEC;
    // Print results to stdout
    printf("%d %5.3f %5.2f",V.get_total_number_of_routes(),
        V.get_total_route_length()-V.get_total_service_time(),
        elapsed);
    if(V.get_best_known()>0 && V.get_best_known()<VRP_INFINITY)
        printf(" %1.3f\n",(V.get_total_route_length()-V.get_total_service_time())/V.get_best_known());
    else
        printf("\n");

    return 0;
}


