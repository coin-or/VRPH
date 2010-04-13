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
#include <time.h>

int main(int argc, char *argv[])
{
    ///
    /// This is the main() routine to construct a command line tool 
    /// for solving VRP's using the record-to-record
    /// travel algorithm.
    ///

    VRPH_version();

    char out[VRPH_STRING_SIZE];
    char plot_file[VRPH_STRING_SIZE];
    char sol_file[VRPH_STRING_SIZE];
    char fixed_edges_file[VRPH_STRING_SIZE];
    int i,n;
    clock_t start, stop;
    double elapsed;

    // Default  parameter settings
    bool verbose=false;
    int intensity =30;
    int max_tries=5;
    int num_lambdas=3;
    int num_perturbs=1;
    int nlist_size=40;
    int perturb_type=VRPH_LI_PERTURB ;
    int accept_type=VRPH_FIRST_ACCEPT;
    int neighbor_lists=VRPH_USE_NEIGHBOR_LIST;
    int tabu=0;
    int tabu_list_size=0;


    double lambda_vals[VRPH_MAX_NUM_LAMBDAS];
    bool new_lambdas=false;
    bool has_heuristics=false;
    bool has_outfile=false;
    bool has_plot_file=false;
    bool has_sol_file=false;
    bool has_fixed_edges_file=false;
    int heuristics=0;
    double dev=.01;
    int *final_sol;
    double final_obj=VRP_INFINITY;
    bool do_pdf=false;
    int *my_sol_buff;


    if(argc<2 || (strncmp(argv[1],"-help",5)==0)||(strncmp(argv[1],"--help",6)==0)||(strncmp(argv[1],"-h",2)==0))
    {        
        fprintf(stderr,"Usage: %s -f <vrp_input_file> [options]\n",argv[0]);
        fprintf(stderr,"Options:\n");
        
        fprintf(stderr,"\t-help prints this help message\n"); 
        
        fprintf(stderr,"\t-a <accept_type> 0 for VRPH_FIRST_ACCEPT or 1 for VRPH_BEST_ACCEPT\n\t\t(default is VRPH_FIRST_ACCEPT)\n");
        
        fprintf(stderr,"\t-d <deviation> runs the RTR search with given deviation\n");
        fprintf(stderr,"\t\t default is dev=.01\n");

        fprintf(stderr,"\t-fix <fixed_edge_file> will fix all of the edges in the provided file\n");
                
        fprintf(stderr,"\t-h <heuristic> applies the specified heuristics (can be repeated)\n");
        fprintf(stderr,"\t\t default is ONE_POINT_MOVE, TWO_POINT_MOVE, and TWO_OPT\n");
        fprintf(stderr,"\t\t others available are OR_OPT, THREE_OPT, and CROSS_EXCHANGE\n");
        fprintf(stderr,"\t\t Example: -h OR_OPT -h THREE_OPT -h TWO_OPT -h ONE_POINT_MOVE\n");
        fprintf(stderr,"\t\t Setting -h KITCHEN_SINK applies all heuristics in the \n");
        fprintf(stderr,"\t\t improvement phase\n");
        
        fprintf(stderr,"\t-sol <sol_file> begins with an existing solution contained\n");
        fprintf(stderr,"\t\t in sol_file\n");
        
        fprintf(stderr,"\t-v prints verbose output to stdout\n");
        
        fprintf(stderr,"\t-k <intensity> runs the RTR search with the provided intensity\n\t\t(default is 30)\n");

        fprintf(stderr,"\t-L <num_lambdas> runs the RTR procedure from num_lambdas\n");
        fprintf(stderr,"\t\t different initial solutions using a random lambda chosen\n");
        fprintf(stderr,"\t\t from (0.5,2.0)\n");
        fprintf(stderr,"\t\t default is to use lambda in {.6, 1.4, 1.6}.\n");
        
        fprintf(stderr,"\t-m <max_tries> gives up after not beating a local minimum after\n");
        fprintf(stderr,"\t\t max_tries consecutive attempts (default is 5)\n");

        fprintf(stderr,"\t-N <nlist_size> uses the nlist_size nearest neighbors in the search\n");
        fprintf(stderr,"\t\t default is 25. Using -N 0 will not use neighbor lists at all\n");
        
        fprintf(stderr,"\t-P <num_perturbs> perturbs the current solution num_perturbs times\n\t\t(default is 1)\n");

        fprintf(stderr,"\t-p <perturb_type> 0 for VRPH_LI_PERTURB or 1 for VRPH_OSMAN_PERTURB\n");

        fprintf(stderr,"\t-plot <plot_file> plots the best solution to the provided file.\n");
        fprintf(stderr,"\t\t This requires a working PLPlot installation\n");

        fprintf(stderr,"\t-pdf will create a .pdf from the .ps file created by -plot\n");

        fprintf(stderr,"\t-r will search the neighborhood in a random fashion\n");

        fprintf(stderr,"\t-t <tabu_list_size> will use a primitive Tabu Search in the uphill phase\n");
        
        fprintf(stderr,"\t-out <out_file> writes the solution to the provided file\n");
        
        
        
        exit(-1);
    }

    bool has_filename=false;
    char *infile=NULL;
    for(i=1;i<argc;i++)
    {
        if(strcmp(argv[i],"-f")==0)
        {
            // Get the input file name
            has_filename=true;
            infile=argv[i+1];
        }
    }
    if(has_filename==false)
    {
        fprintf(stderr,"No input file given\nUsage: %s -f <filename> [options]\n",argv[0]);
        exit(-1);
    }

    // Get # of non-VRPH_DEPOT nodes
    n=VRPGetDimension(infile);
    int num_days=VRPGetNumDays(infile);
    my_sol_buff=new int[n+2];


    VRP V(n,num_days);
       
        

    // Allocate the buffer for the final solution
    final_sol=new int[n+2];    

    // Parse command line
    for(i=2;i<argc;i++)
    {
        if(strcmp(argv[i],"-a")==0)
        {
            accept_type=atoi(argv[i+1]);
            if( (accept_type!=0) && (accept_type !=1))
                report_error("%s: %s: accept_type must be 0 or 1\n");

            if(accept_type==1)
                accept_type=VRPH_BEST_ACCEPT;
            else
                accept_type=VRPH_FIRST_ACCEPT;
        }

        if(strcmp(argv[i],"-d")==0)
            dev=atof(argv[i+1]);

        if(strcmp(argv[i],"-D")==0)
            intensity=atoi(argv[i+1]);

        if(strcmp(argv[i],"-fix")==0)
        {
            has_fixed_edges_file=true;
            strcpy(fixed_edges_file,argv[i+1]);
        }

        if(strcmp(argv[i],"-h")==0)
        {
            has_heuristics=true;
            if(strcmp(argv[i+1],"ONE_POINT_MOVE")==0)
                heuristics|=ONE_POINT_MOVE;
            if(strcmp(argv[i+1],"TWO_POINT_MOVE")==0)
                heuristics|=TWO_POINT_MOVE;
            if(strcmp(argv[i+1],"TWO_OPT")==0)
                heuristics|=TWO_OPT;
            if(strcmp(argv[i+1],"OR_OPT")==0)
                heuristics|=OR_OPT;
            if(strcmp(argv[i+1],"THREE_OPT")==0)
                heuristics|=THREE_OPT;
            if(strcmp(argv[i+1],"CROSS_EXCHANGE")==0)
                heuristics|=CROSS_EXCHANGE;
            if(strcmp(argv[i+1],"THREE_POINT_MOVE")==0)
                heuristics|=THREE_POINT_MOVE;
            if(strcmp(argv[i+1],"KITCHEN_SINK")==0)
                heuristics|=KITCHEN_SINK;
        }

        if(strcmp(argv[i],"-k")==0)
            max_tries=atoi(argv[i+1]);

        if(strcmp(argv[i],"-L")==0)
        {
            new_lambdas=true;
            num_lambdas = atoi(argv[i+1]);

            if(num_lambdas>VRPH_MAX_NUM_LAMBDAS)
            {
                fprintf(stderr,"%d>VRPH_MAX_NUM_LAMBDAS\n",num_lambdas);
                exit(-1);
            }

            for(int j=0;j<num_lambdas;j++)
            {
                // Generate a random lambda
                lambda_vals[j] = 0.5 + 1.5*((double)lcgrand(1));
            }
        }

        if(strcmp(argv[i],"-N")==0)
            nlist_size=atoi(argv[i+1]);

        if(strcmp(argv[i],"-out")==0)
        {
            has_outfile=true;
            strcpy(out,argv[i+1]);
        }

        if(strcmp(argv[i],"-plot")==0)
        {
            has_plot_file=true;
            strcpy(plot_file,argv[i+1]);
        }

        if(strcmp(argv[i],"-p")==0)
        {
            perturb_type=atoi(argv[i+1]);
            if(perturb_type!=0 && perturb_type!=1)
            {
                fprintf(stderr,"Perturb type must be 0 or 1!\n");
                exit(-1);
            }
        }

        if(strcmp(argv[i],"-P")==0)
            num_perturbs=atoi(argv[i+1]);

        if(strcmp(argv[i],"-pdf")==0)
            do_pdf=true;



        if(strcmp(argv[i],"-sol")==0)
        {
            has_sol_file=true;
            strcpy(sol_file,argv[i+1]);
        }

        if(strcmp(argv[i],"-t")==0)
        {
            tabu=VRPH_TABU;    
            tabu_list_size=atoi(argv[i+1]);
        }        

        if(strcmp(argv[i],"-v")==0)
            verbose=true;
        
    }

    // Load the problem data
    V.read_TSPLIB_file(infile);
    // If we have more than one day, just run alg. on day 1
    if(num_days>1)
    {
        printf("Multi-day problem loaded (%d days). Will run only on day 1\n",
            num_days);
        V.set_daily_demands(1);
        V.set_daily_service_times(1);
    }
   

    // Start processing the input options
    if(new_lambdas==false)
    {
        // Use .6, 1.4, 1.6
        num_lambdas=3;
        lambda_vals[0]=.6;
        lambda_vals[1]=1.4;
        lambda_vals[2]=1.6;
    }

    // Start constructing the list of optional heuristics we will we use 
    if(nlist_size==0)
        // No neighbor lists
        neighbor_lists=0;
    else     
        neighbor_lists=VRPH_USE_NEIGHBOR_LIST;
    heuristics|=neighbor_lists;

    if(has_heuristics==false)
        // Use default set of operators
        heuristics|=(ONE_POINT_MOVE|TWO_POINT_MOVE|TWO_OPT);
    

    // Add in Tabu Search if requested
    heuristics+=tabu;

    // Declare a CW object of the right size
    ClarkeWright CW(n);

    if(has_fixed_edges_file)
    {
        V.read_fixed_edges(fixed_edges_file);
        if(has_fixed_edges_file)
            heuristics |= VRPH_FIXED_EDGES;
    }

    start=clock();
    if(!has_sol_file)
    {
        // No solution imported - start from scratch
        for(i=0;i<num_lambdas;i++)
        {
            // Empty the solution warehouse and set nodes to unrouted
            V.reset();

            // Construct a new Clarke Wright solution
            CW.Construct(&V,lambda_vals[i], false);
            CW.has_savings_matrix=false;    

            if(V.get_total_route_length()-V.get_total_service_time() < final_obj)
            {
                final_obj=V.get_total_route_length()-V.get_total_service_time();
                V.export_canonical_solution_buff(final_sol);
            }

            if(verbose)
                printf("CW solution %d[L=%1.4f]: %5.4f\n",i,lambda_vals[i], V.get_total_route_length());

            // Run the record-to-record travel algorithm with the given parameters
            V.RTR_solve(heuristics, intensity, max_tries, num_perturbs, dev, nlist_size, 
                perturb_type, accept_type, verbose);
        
            // Check for a new global best solution - recall the best solution found
            V.get_best_sol_buff(my_sol_buff);
            V.import_solution_buff(my_sol_buff);
            if(V.get_total_route_length()-V.get_total_service_time() < final_obj)
            {
                final_obj = V.get_total_route_length()-V.get_total_service_time();
                V.export_canonical_solution_buff(final_sol);
            }
            
            if(verbose)
                printf("%5.2f\n",V.get_best_total_route_length()-V.get_total_service_time());

            // Reset best
            V.set_best_total_route_length(VRP_INFINITY);
        }
    }
    else
    {
        // Import a solution from the file and call it the best so far
        V.read_solution_file(sol_file);
        printf("Read in solution:\n");
        V.show_routes();
        // CSG - is the line below necessary?
        //V.export_solution_buff(V.best_sol_buff);
        V.set_best_total_route_length(V.get_total_route_length());

        // Now improve it
        V.RTR_solve(heuristics, intensity, max_tries, num_perturbs, dev, nlist_size, 
            perturb_type, accept_type,verbose);

        V.export_canonical_solution_buff(final_sol);
    }

    stop=clock();
   
    // Restore the best solution found
    V.import_solution_buff(final_sol);
    
    elapsed=(double)(stop-start)/CLOCKS_PER_SEC;
  
    if(verbose)
    {
        V.summary();
        V.show_routes();
        V.print_stats();
        printf("\n");
    }

    if(has_outfile)
        V.write_solution_file(out);

    if(has_plot_file)
    {
#if !HAS_PLPLOT
        fprintf(stderr,"Need PLPLOT to plot solution\n");
        exit(-1);
#endif

        bool plot_success=V.plot(plot_file);
        if(plot_success && do_pdf)
        {
            char pdf_string[VRPH_STRING_SIZE];
            sprintf(pdf_string,"%s %s",VRPH_EPS_EXE,plot_file);
            system(pdf_string);
        }
    }

    // Print results to stdout
    printf("%d %5.3f %5.2f",V.get_total_number_of_routes(),
        V.get_total_route_length()-V.get_total_service_time(),
        elapsed);
    if(V.get_best_known()>0 && V.get_best_known()<VRP_INFINITY)
        printf(" %1.3f\n",(V.get_total_route_length()-V.get_total_service_time())/V.get_best_known());
    else
        printf("\n");

    delete [] my_sol_buff;
    delete [] final_sol;

    return 0;

    

}


