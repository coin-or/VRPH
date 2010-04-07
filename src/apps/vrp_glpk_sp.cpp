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

// Example of integrating VRPH with a Mixed Integer Programming solver to allow
// generating solutions to the VRP by solving as a set partitioning problem.
// We use the COIN-OSI interface in order to create "solver agnostic" code


#include "OsiGlpkSolverInterface.hpp"
#include "CoinPackedMatrix.hpp"
#include "CoinPackedVector.hpp"
#include "VRPH.h"

#define MAX_ROUTES  50000
// This is the maximum number of potentially different routes/columns
// that we will bother to store

// We allow only max_columns routes/columns in the IP
// GLPK starts to struggle with more than 500 or so
// Once we get more than this # of columns, we start fixing randomly
// selected variables to 0 (GLPK column deletion routine seems buggy?)
// CPLEX and other solvers can handle a couple thousand columns w/o difficulty
int max_columns;
int num_cols_to_delete; 
// Global variable to control output
bool verbose;
time_t heur_time, mip_time;

void OSI_recover_route(int id, int **orderings, VRPRoute *r)
{
    ///
    /// Populates the route r with the information from orderings[i]. Computes
    /// the number of customers and the hash values of the route.
    ///
    
    // Now get the ordering using this id
    int i=0;
    r->num_customers=0;
    while(orderings[id][i]!=-1)
    {
        r->ordering[i]=orderings[id][i];
        r->num_customers++;
        i++;
    }

    // We now have the # of customers in the route and the ordering
    // Compute the two hash values 
    r->hash_val=r->hash(SALT_1);
    r->hash_val2=r->hash(SALT_2);

    return;
  
}

void OSI_recover_solution(OsiSolverInterface *si, int **orderings, int *sol_buff)
{
    ///
    /// Extracts the solution from the current set partitioning instance
    /// and places it in sol_buff[].  We need to access the column/variable
    /// names since these hold the orderings of the routes.
    ///
 
    int i, j, k, id, nnodes, ncols;
    const double *x;
    std::string colname;

    nnodes= si->getNumRows();
    ncols= si->getNumCols();
    x = si->getColSolution();

    sol_buff[0]=nnodes;

    VRPRoute route(nnodes);

    k=1;
    for(i=0;i<ncols;i++)
    {
        if(VRPH_ABS(x[i]-1)<.01)
        {
            colname=si->getColName(i,VRP_INFINITY);
            id=atoi(colname.c_str());

            OSI_recover_route(id,orderings,&route);
        
            for(j=0;j<route.num_customers;j++)
                sol_buff[k+j]=route.ordering[j];

            sol_buff[k]=-sol_buff[k];

            k += route.num_customers;
        }
    }
    sol_buff[nnodes+1]=0;

    return;
  
}

void OSI_add_route(OsiSolverInterface *si, VRP *V, VRPRoute *r, int id, int **orderings)
{
    ///
    /// Adds a column/route to the current set partitioning problem.  The column is assigned
    /// the name of id, which is converted to a string.
    ///

    if(id>MAX_ROUTES)
    {   
        fprintf(stderr,"Too many routes added! Increase value of MAX_ROUTES=%d\n",MAX_ROUTES);
        exit(-1);
    }   

    int i, j, k, nvars, ncols;
    int *cols_to_delete;
    int *col_indices;
    std::string col_name;
    const double *U;
    time_t start, stop;

    // Create a new route object of the right size
    VRPRoute removed_route(V->get_num_nodes());

    // Create a CoinPackedVector for the new column
    CoinPackedVector col;

    for(i=0;i<r->num_customers;i++)
        col.insert(r->ordering[i]-1, 1);// 0-based

    // Check to see if we have exceeded max_columns
    // Since we have to fix variable values instead of deleting them in GLPK,
    // we have to calculate this
    U=si->getColUpper();
    ncols=si->getNumCols();
    nvars=0;
    for(i=0 ; i < ncols ; i++)
    {
        if(U[i]==1)
            nvars++;
    }

    if(nvars>=max_columns)
    {
        // Solve the problem and then delete num_cols_to_delete variables not in the current
        // optimal solution
        start=clock();
        si->branchAndBound();
        stop=clock();
        mip_time += (stop-start); 
        const double *y=si->getColSolution();
        // Select a set of num_cols_to_delete columns to delete   
        int nn=si->getNumCols();
        cols_to_delete = new int[nn];
        col_indices= new int[num_cols_to_delete];
        memset(cols_to_delete,0,nn*sizeof(int));
        memset(col_indices,0,num_cols_to_delete*sizeof(int));
        U=si->getColUpper();
        
        if(verbose)
            printf("Finding %d columns to delete \n",num_cols_to_delete);
        k=0;
        while(k<num_cols_to_delete)
        {
            while(true)
            {
                j=(int)floor(lcgrand(10)*nn);
                // Make sure the variable/column is not in the optimal solution 
                // Since OSI/GLPK implementation of deleteCols doesn't seem right,
                // Need to make sure that this column isn't already fixed!
                if(y[j]<0.01 && cols_to_delete[j]==0 && U[j]==1)
                {
                    cols_to_delete[j]=1;
                    col_indices[k]=j;
                    k++;
                    break;
                }
            }    
        }
        
        // Delete the columns - this does not seem to work in GLPK - just fix the variable to 0
        // si->deleteCols(num_cols_to_delete,col_indices);
        
        for(k=0;k<num_cols_to_delete;k++)
        {
            col_name=si->getColName(col_indices[k],VRP_INFINITY);
            // Remove the route from VRPH's "Route Warehouse"
            int route_id=atoi(col_name.c_str());
            OSI_recover_route(route_id,orderings, &removed_route);
            
            V->route_wh->remove_route(removed_route.hash_val, removed_route.hash_val2);
            // Fix this variable/column to 0
            si->setColBounds(col_indices[k],0,0);
        }
        if(verbose)
            printf("Deleted/Fixed %d columns/variables\n",num_cols_to_delete);
    
        delete [] cols_to_delete;
        delete [] col_indices;
    }

    // Add the named column to the IP 
    sprintf(r->name,"%d",id);
    std::string new_name(r->name);
    si->addCol(col,0,1,r->length-r->total_service_time,new_name);
    ncols=si->getNumCols();
    // Set as integer (binary)
    si->setInteger(ncols-1); // 0-based

    // Copy the ordering of the route to the orderings[] array
    // Add a -1 to denote the end
    orderings[id]=new int[r->num_customers+1];
    memcpy(orderings[id],r->ordering,r->num_customers*sizeof(int));
    orderings[id][r->num_customers]=-1;
    return;
}


int main(int argc, char **argv)
{
    VRPH_version();

    int i, j, k, n, status, num_attempts, *sol_buff, *IP_sol_buff;
    char in_file[200];
    double lambda, best_heur_sol=VRP_INFINITY;
    bool first_sol=false, bootstrap=false;;
    VRPSolution *fresh_solution;
    OsiSolverInterface *si;
    const double *x;
    int last_num_cols=0, route_id=0;
    time_t start, stop;
    int *orderings[MAX_ROUTES];
    for(i=0;i<MAX_ROUTES;i++)
        orderings[i]=NULL;
    

    // Set timing counters to 0
    heur_time=mip_time=0;

    // Check arguments
    if(argc<5)
    {
        fprintf(stderr,"Usage: %s -f input_file -n num_runs [-v,-b,-c max_columns -d cols_to_delete]\n",
            argv[0]);
        fprintf(stderr,"\t Will solve the problem num_solutions times and add the routes\n");
        fprintf(stderr,"\t to a set partitioning problem.\n");
        fprintf(stderr,"\t Other options:\n");
        fprintf(stderr,"\t -v runs in verbose mode\n");
        fprintf(stderr,"\t -b will use bootstrapping where we send the set partitioning\n"
                       "\t    solution back to the metaheuristic solver\n");
        fprintf(stderr,"\t -c max_columns will allow this many active columns/variables in the IP.\n");
        fprintf(stderr,"\t    Default value is max_columns=500\n");
        fprintf(stderr,"\t -d num_cols_to_delete will delete this many columns once we have too many\n");
        fprintf(stderr,"\t    in the IP. Default value is num_cols_to_delete=100\n");
        exit(-1);
    }

    // Set defaults
    verbose=false;
    max_columns=500;
    num_cols_to_delete=100;

    // Parse command line
    for(i=0;i<argc;i++)
    {
        if(strcmp(argv[i],"-f")==0)
            strcpy(in_file,argv[i+1]);
        if(strcmp(argv[i],"-n")==0)
            num_attempts=atoi(argv[i+1]);
        if(strcmp(argv[i],"-v")==0)
            verbose=true;
         if(strcmp(argv[i],"-b")==0)
            bootstrap=true;
         if(strcmp(argv[i],"-c")==0)
             max_columns=atoi(argv[i+1]);
         if(strcmp(argv[i],"-d")==0)
             num_cols_to_delete=atoi(argv[i+1]);
    }

    // This is the # of non-VRPH_DEPOT nodes
    n=VRPGetDimension(in_file);
    // This will be used to import/export solutions
    fresh_solution = new VRPSolution(n);
    // Create buffers for importing solutions
    sol_buff= new int[n+2];
    IP_sol_buff = new int[n+2];

    // Declare an OSI interface
    si=new OsiGlpkSolverInterface;
    si->setIntParam(OsiNameDiscipline,2);

    for(i=0;i<n;i++)
    {
        si->addRow(0,NULL,NULL,1,1);
    }

    // Declare a VRP of the right size and import the file
    VRP V(n);
    ClarkeWright CW(n);
    VRPRoute route(n);
    V.read_TSPLIB_file(in_file);
    // Set up a "route warehouse" to store the routes to be added to the IP
    V.route_wh=new VRPRouteWarehouse(HASH_TABLE_SIZE);
  
    // Set up a minimization problem
    si->setObjSense(1);
    // Set to error only output
    si->setHintParam(OsiDoReducePrint,true, OsiHintDo);
    // Unfortunately GLPK still prints out something regarding the conflict graph

    for(i=0;i<num_attempts;i++)
    {
        if(i==0 || !bootstrap) 
        {
            lambda=.5+1.5*lcgrand(0);
            // Start with a clean VRP object
            V.reset();
            CW.Construct(&V, lambda, false);
            if(verbose)
                printf("CW solution %d[%5.3f]: %f\n",i,lambda,V.get_total_route_length()-V.get_total_service_time());
        }
        else
            // Use the solution from the IP
            V.import_solution_buff(IP_sol_buff);
        
        // Run VRPH's RTR algorithm to improve the solution
        start=clock();
        V.RTR_solve(ONE_POINT_MOVE | TWO_POINT_MOVE | TWO_OPT | VRPH_USE_NEIGHBOR_LIST,
            30, 5, 2, .01, 30, VRPH_LI_PERTURB, VRPH_FIRST_ACCEPT,false);
        stop=clock();
        heur_time += (stop-start);

        if(verbose)
            printf("RTR Metaheuristic found solution %5.3f\n",V.get_total_route_length()-V.get_total_service_time());
        
        // The RTR algorithm keeps a "warehouse" of the best solutions discovered during
        // the algorithm's search
        // Now go through the solutions in the solution warehouse and add the new routes
        // discovered to the IP
        for(j=0;j<V.solution_wh->num_sols;j++)
        {
            // Import solution j from the warehouse 
            V.import_solution_buff(V.solution_wh->sols[j].sol);
                
            if(V.get_total_route_length()-V.get_total_service_time() < best_heur_sol)
                best_heur_sol = V.get_total_route_length()-V.get_total_service_time() ;

            // Now add the routes from this solution to the IP
            for(k=1;k<=V.get_total_number_of_routes();k++)
            {
                // Clean up the route by running INTRA_ROUTE optimizations only
                // using the route_search method of the different local search
                // heuristics, accepting improving moves only (VRPH_DOWNHILL)
                OnePointMove OPM;
                TwoOpt TO;
                ThreeOpt ThO;
                while(OPM.route_search(&V,k,k,VRPH_DOWNHILL|VRPH_INTRA_ROUTE_ONLY )){}
                while(TO.route_search(&V,k,k,VRPH_DOWNHILL|VRPH_INTRA_ROUTE_ONLY  )){};
                while(ThO.route_search(&V,k,VRPH_DOWNHILL|VRPH_INTRA_ROUTE_ONLY )){};

                // Copy route k from the solution to the VRPRoute R
                V.update_route(k,&route);
                route.create_name();
                // Add it to the "route warehouse" - this uses a hash table to keep track
                // of duplicate columns
                status=V.route_wh->add_route(&route);

                if(status!=DUPLICATE_ROUTE)
                {
                    // This route is not currently in the WH and so it cannot be in the
                    // set partitioning problem
                    //OSI_add_route(si,&V,&route);
                    OSI_add_route(si,&V,&route,route_id,orderings);
                    route_id++;
                }
            }
            
            // Set the row RHS's if we need to
            if(first_sol)
            {
                first_sol=false;
                for(int rownum=0;rownum<n;rownum++)
                    si->setRowBounds(rownum,1,1);
                // Note that changing this to >= would be a set covering problem
                // where each customer can be visited by more than one route
            }
        }

        // Now erase all the solutions from the WH
        V.solution_wh->liquidate();

        if(verbose)
        {
            printf("Attempt %02d:  Solving IP with %d columns\n",i,si->getNumCols());
            printf("%d routes in the WH\n",V.route_wh->num_unique_routes);
        }

        // Solve the current set partitioning problem using the MIP solver
        start=clock();
        si->branchAndBound();
        stop=clock();
        mip_time += (stop-start);

        double opt=si->getObjValue();
        x=si->getColSolution();
        last_num_cols=si->getNumCols();
        if(verbose)
            printf("Optimal solution (%d columns) is %f\n",last_num_cols,opt);

        
        // Now recover the solution from the IP solution
        OSI_recover_solution(si, orderings, IP_sol_buff);
       
        if(verbose)
            printf("IP solution has obj. function value: %5.2f\n"
            "Best heuristic  obj. function value: %5.2f\n",
            si->getObjValue(),best_heur_sol);     
    }

    if(verbose)
        printf(
        "\nResults\n"
        "--------\n"
        "After %d runs\n"
        "IP solution has obj. function value: %5.2f\n"
        "Best heuristic  obj. function value: %5.2f\n",
        num_attempts,si->getObjValue(),best_heur_sol);

    // print to stderr since GLPK prints "conflict graph" to stdout (a known bug...)
    // best_heur_sol best_mip_sol heur_time mip_time
    fprintf(stderr,"%5.3f %5.3f %5.3f %5.3f\n", best_heur_sol, si->getObjValue(),
        (double)(heur_time)/CLOCKS_PER_SEC, (double)(mip_time)/CLOCKS_PER_SEC);

    delete V.route_wh;
    delete fresh_solution;
    delete [] sol_buff;
    delete [] IP_sol_buff;
    delete si;

    for(i=0;i<MAX_ROUTES;i++)
        if(orderings[i])
            delete [] orderings[i];

     
    return 0;
}



