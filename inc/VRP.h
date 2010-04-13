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

#ifndef _VRP_H
#define _VRP_H

// VRP class
class VRP
{

    friend class OnePointMove;
    friend class TwoPointMove;
    friend class ThreePointMove;
    friend class TwoOpt;
    friend class ThreeOpt;
    friend class OrOpt;
    friend class CrossExchange;

    friend class Postsert;
    friend class Presert;
    friend class MoveString;
    friend class Swap;
    friend class SwapEnds;
    friend class Concatenate;
    friend class Flip;

    friend class ClarkeWright;
    friend class Sweep;

public:
    VRP(int n);
    // Constructor for problems with n non-VRPH_DEPOT nodes
    VRP(int n, int ndays);
    // Construct for problems with n non-VRPH_DEPOT nodes and num_days days

    // Destructor
    ~VRP();

    // TSPLIB file processing
    void read_TSPLIB_file(const char *infile);
    // Write problem instance
    void write_TSPLIB_file(const char *outfile);

    // Solution display/debugging
    void show_next_array();
    void show_pred_array();
    bool verify_routes(const char *message);
    bool check_fixed_edges(const char *message);
    void create_pred_array();
    void print_stats();

    // Files (read/write)
    void write_solution_file(const char *filename);
    void write_solutions(int num_sols, const char *filename);
    void write_tex_file(const char *filename);
    void read_solution_file(const char *filename);
    int read_fixed_edges(const char *filename);

    // Solution buffers (import/export)
    void export_solution_buff(int *sol_buff);
    void import_solution_buff(int *sol_buff);
    void export_canonical_solution_buff(int *sol_buff);

    // Solution display
    void show_routes();
    void show_route(int k);
    void summary();

    // Removes all solutions from memory and sets all nodes to unrouted
    void reset();

    // Graphics - only meaningful with PLPlot
    bool plot(const char *filename, int options, int orientation);
    bool plot(const char *filename);
    bool plot_route(int r, const char *filename);

    // Copying
    bool clone(VRP *W);

    // Solvers
    double RTR_solve(int heuristics, int intensity, int max_stuck, int num_perturbs,
        double dev, int nlist_size, int perturb_type, int accept_type, bool verbose);

    double SA_solve(int heuristics, double start_temp, double cool_ratio,
        int iters_per_loop, int num_loops, int nlist_size, bool verbose);

    // The name of the problem (from TSPLIB file)
    char name[VRPH_STRING_SIZE];

    // For multi-day problems
    void set_daily_demands(int day);
    void set_daily_service_times(int day);

    // Creating default routes for CW, etc.
    bool create_default_routes();
    bool create_default_routes(int day);

    // Routines to remove and insert nodes into solution
    bool eject_node(int k);
    int inject_set(int num, int *nodelist, int rules, int attempts);
    void eject_neighborhood(int j, int num, int *nodelist);

    // Route operations
    void refresh_routes();
    void normalize_route_numbers();
    void update_route(int j, VRPRoute *R);
    void clean_route(int r, int heuristics);
    double split(double p);
    int split_routes(double p, int **ejected_routes, double *t);
    void add_route(int *route_buff);
    void append_route(int *sol_buff, int *route_buff);
    int intersect_solutions(int *new_sol, int **routes, int *sol1, int *sol2, int min_routes);
    int find_common_routes(int *sol1, int *sol2, int *route_nums);

    // Fixing edges
    void list_fixed_edges(int *fixed_list);
    void unfix_all();
    void fix_edge(int start, int end);
    void unfix_edge(int start, int end);
    void fix_string(int *node_string, int k);

    // Accessor functions for private data 
    int get_num_nodes();
    double get_total_route_length();
    double get_total_service_time();
    double get_best_sol_buff(int *sol_buff);
    double get_best_total_route_length();
    int get_total_number_of_routes();
    int get_num_original_nodes();
    int get_num_days();        // For multi-day VRPs
    double get_best_known();
    void set_best_total_route_length(double val);
    int get_max_veh_capacity();
    double get_max_route_length();

    // Places to store unique solutions and routes - uses internal hash table
    VRPSolutionWarehouse *solution_wh; // To store additional solutions
    VRPRouteWarehouse *route_wh;     // To store routes/columns

    // Distance matrix creation
    void create_distance_matrix(int type);
    // Neighbor list creation
    void create_neighbor_lists(int nsize);

    // Node injection/ejection
    bool perturb();
    bool eject_route(int r, int *route_buff);
    bool inject_node(int j);

    // Route reversal
    void reverse_route(int i);

    // Statistics
    int num_evaluations[NUM_HEURISTICS];
    int num_moves[NUM_HEURISTICS];

private:  

    // Problem parameters, description, etc.
    int num_nodes;
    double total_route_length;
    double total_service_time;
    int *best_sol_buff;                // Place for the best solution to live
    double best_total_route_length;
    int total_number_of_routes;
    int num_original_nodes;
    double best_known;  // Record of the best known solution for benchmarks
    int num_days;        // For multi-day VRPs
    int problem_type;
    int total_demand;
    int max_veh_capacity;
    int orig_max_veh_capacity;
    double max_route_length;
    double min_route_length;
    double orig_max_route_length;
    int min_vehicles;    // Not currently used    
    bool has_service_times;
    double fixed_service_time;
    int edge_weight_type;
    int coord_type;
    int display_type;
    int edge_weight_format;
    int matrix_size;
    double balance_parameter;    // For VRPH_BALANCED problems
    int dummy_index;
    int neighbor_list_size;
    double temperature;            // For VRPH_SIMULATED_ANNEALING
    double cooling_ratio;
    
    bool symmetric;                 // To keep track of symmetric/asymmetric instances
                                    // Note! Asymmetric instances have received only 
                                    // limited testing!
    bool can_display;

    double **d;                    // The distance matrix d
    bool **fixed;                // Matrix to keep track of fixed edges

    class VRPNode *nodes;        // Array of nodes - contains coordinates, demand
    // amounts, etc.

    bool depot_normalized;        // Set to true if VRPH_DEPOT coords normalized to origin
    // for Euclidean problem.

    bool forbid_tiny_moves;        // Set to true to prevent potentially nonsense moves
    // that have a tiny (perhaps zero) effect on route length

    // Local search neighborhood creation
    bool create_search_neighborhood(int j, int rules);    
    int search_size;            
    int *search_space;            

    // Solution storage
    int *next_array;
    int *pred_array;
    int *route_num;
    bool *routed;            // Indicates whether the customer is in a route yet or not

    class VRPRoute *route;    // Array stores useful information about the routes in a solution

    // Tabu search - very limited testing so far!!
    class VRPTabuList *tabu_list;
    bool check_tabu_status(VRPMove *M, int *old_sol);

    double record;        // For RTR
    double deviation;    // For RTR

    double min_theta;
    double max_theta;    // Polar min/max

    int *current_sol_buff;  // Place for the current solution if desired

    // Accessing edge information
    bool before(int a, int b);

    // To handle infeasibilities
    bool check_feasibility(VRPViolation *VV);
    class VRPViolation violation;
    bool is_feasible(VRPMove *M, int rules);    

    // Solution manipulation
    bool postsert_dummy(int i);
    bool presert_dummy(int i);
    bool remove_dummy();
    bool osman_insert(int k, double alpha);
    int osman_perturb(int num, double alpha);
    bool insert_node(int j, int i, int k);
    void perturb_locations(double c);
    void find_cheapest_insertion(int j, int *edge, double *costs, int rules);    
    double insertion_cost(int u, int a, int b);
    double ejection_cost(int u);
    void update(VRPMove *M);

    // Splitting VRP instances
    void compute_route_center(int r);
    void find_neighboring_routes();

    // Solution memory
    void capture_best_solution();
    void update_solution_wh();    

    // Solution information
    bool get_segment_info(int a, int b, struct VRPSegment *S);
    double get_distance_between(int a, int b);
    int get_string_end(int a, int len);
    int count_num_routes();
    void update_arrival_times();
    bool  check_move(VRPMove *M, int rules);

    // Savings evaluation - inline this to speed things up 
    inline bool check_savings(VRPMove *M, int rules){
        ///
        /// Evaluates the given savings in terms of the rules.
        /// The only part of the rules considered are things such
        /// as VRPH_DOWNHILL, VRPH_RECORD_TO_RECORD, VRPH_SIMULATED_ANNEALING
        ///

#if VRPH_FORBID_TINY_MOVES
        if(M->savings>-VRPH_EPSILON && M->savings < VRPH_EPSILON)
            return false;
#endif

        if( M->savings < -VRPH_EPSILON )
        {
            M->evaluated_savings=true;
            return true;
        }
        // The order needs to be changed if we eventually add additional rules
        // that do not always accept improving moves!!

        if( (rules & VRPH_FREE)  )
            return true;


        if( (rules & VRPH_DOWNHILL) )
        {
            if(M->savings>=-VRPH_EPSILON )
            {
                M->evaluated_savings=true;
                return false;
            }
        }

        if( (rules & VRPH_RECORD_TO_RECORD) )
        {
            if(M->savings<-VRPH_EPSILON)
            {
                M->evaluated_savings=true;
                return true;
            }
            else
            {

                if(has_service_times==false)
                {
                    if( (total_route_length+M->savings<= (1+deviation)*record) )
                    {
                        M->evaluated_savings=true;
                        return true;
                    }
                    else
                        return false;
                }
                else
                {
                    // We have service times so remove the service time from the
                    // deviation calculation
                    if( ((total_route_length - total_service_time) + M->savings <= 
                        ((1+deviation)*(record-total_service_time))) )
                    {
                        M->evaluated_savings=true;
                        return true;
                    }
                    else
                        return false;
                }


            }

        }

        if( rules & VRPH_SIMULATED_ANNEALING )
        {
            if( exp( - ((M->savings) / this->temperature)) > lcgrand(0) )
            {
                M->evaluated_savings=true;
                return true;
            }
            else
                return false;

        }

        return false;


    };




};

#endif


