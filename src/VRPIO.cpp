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

void VRP::read_TSPLIB_file(const char *node_file)
{
    ///
    /// Processes each section of the provided TSPLIB file
    /// and records the relevant data in the VRP structure.  See the example
    /// files for information on my interpretation of the TSPLIB standard
    /// as it applies to VRP's.
    ///


    char *temp;
    char line[VRPH_STRING_SIZE];
    char *temp2;
    int max_id = -VRP_INFINITY;

    int ans;
    int x,y,i,j;
    float a, b;
    double s;

    bool has_depot=false;
    bool has_nodes=false;

    FILE *infile;

    infile  = fopen(node_file, "r");

    if (infile==NULL)
        report_error("%s: file error\n",__FUNCTION__);


    this->edge_weight_format=-1;
    this->edge_weight_type=-1;

    for(;;)
    {
        fgets(line, 100, infile);

#if TSPLIB_DEBUG
        printf("Next line is %s\n",line);
#endif

        temp=strtok(line,":");



#if TSPLIB_DEBUG
        printf("line begins with %s\n",temp);
#endif

        if( (ans=VRPCheckTSPLIBString(temp))<=0 )
        {
            if(ans==0)
                fprintf(stderr,"Unknown string %s found\n",temp);
            else
                fprintf(stderr,"TSPLIB string %s not supported\n",temp);

            report_error("%s\n",__FUNCTION__);
        }

#if TSPLIB_DEBUG
        printf("ans is %d\n",ans);
#endif

        // Otherwise, we know the string - process it accordingly
        switch(ans)
        {
        case 1:
            // NAME

            temp2=strtok(NULL," ");
            strcpy(name,temp2);        
            // Trim the ANNOYING \n if necessary...
            for(i=0;i<(int)strlen(name);i++)
            {
                if(name[i]=='\n' || name[i]=='\r')
                {
                    name[i]=0x00;
                    break;
                }
            }
        


#if TSPLIB_DEBUG
            printf("Name is %s\n",name);
#endif
            break;

        case 2:
            // TYPE
            temp2=strtok(NULL," ");

#if TSPLIB_DEBUG
            printf("Problem type is\n%s\n",temp2);
#endif
            if( (strncmp(temp2,"TSP",3)!= 0) && (strncmp(temp2,"CVRP",4)!=0)
                && (strncmp(temp2,"DCVRP",5)!=0) )
            {
                fprintf(stderr,"Unknown type %s encountered\n",temp2);
                report_error("%s\n",__FUNCTION__);
            }

            if( strncmp(temp2,"TSP",3)== 0) 
                problem_type=VRPH_TSP;
            if(strncmp(temp2,"CVRP",4)==0 || (strncmp(temp2,"DCVRP",5)!=0) )
                problem_type=VRPH_CVRP;

            break;


        case 3:
            // BEST_KNOWN
            temp2=strtok(NULL,"");
            this->best_known=atof(temp2);
            break;
        case 4:
            // DIMENSION
            num_nodes=atoi(strtok(NULL,""));
            num_nodes--;
            matrix_size= num_nodes;
            // num_nodes is the # of non-VRPH_DEPOT nodes
            // The value of DIMENSION includes the VRPH_DEPOT!
            dummy_index = 1+num_nodes;

            break;

        case 5:
            // CAPACITY
            max_veh_capacity=atoi(strtok(NULL,""));
            orig_max_veh_capacity=max_veh_capacity;

#if TSPLIB_DEBUG
            printf("veh capacity is %d\n",max_veh_capacity);
#endif
            break;
        case 6:
            // DISTANCE
            max_route_length=(double)atof(strtok(NULL,""));
            orig_max_route_length=max_route_length;

#if TSPLIB_DEBUG
            printf("max length is %f\n",max_route_length);
#endif
            break;
        case 7:
            // EDGE_WEIGHT_FORMAT - sets edge_weight_format
            temp2=strtok(NULL," ");
            edge_weight_format=-1;

            if(strncmp(temp2,"UPPER_ROW",9)==0)
            {
                edge_weight_format=VRPH_UPPER_ROW;
            }

            if(strncmp(temp2,"FULL_MATRIX",11)==0)
            {
                edge_weight_format=VRPH_FULL_MATRIX;
            }
            
            if(strncmp(temp2,"FUNCTION",8)==0)
            {
                edge_weight_format=VRPH_FUNCTION;

            }
            if(strncmp(temp2,"LOWER_ROW",9)==0)
            {
                edge_weight_format=VRPH_LOWER_ROW;

            }
            if(strncmp(temp2,"UPPER_DIAG_ROW",14)==0)
            {
                edge_weight_format=VRPH_UPPER_DIAG_ROW;

            }
            if(strncmp(temp2,"LOWER_DIAG_ROW",14)==0)
            {
                edge_weight_format=VRPH_LOWER_DIAG_ROW;

            }

            if(edge_weight_format == -1)
            {
                // We didn't find a known type
                fprintf(stderr,"Unknown/Unsupported EDGE_WEIGHT_FORMAT %s encountered\n",temp2);
                report_error("%s\n",__FUNCTION__);
            }
            break;
        case 8:  
            // EDGE_WEIGHT_TYPE
            edge_weight_type    = -1;
            temp2=strtok(NULL," ");


#if TSPLIB_DEBUG
            printf("Weight type is %s\n",temp2);
#endif
            
            // Determine the type of weight format
            if(strncmp(temp2,"EXPLICIT",8)==0)
            {
                edge_weight_type=VRPH_EXPLICIT;
            }

            if(strncmp(temp2,"EUC_2D",6)==0)
            {
                edge_weight_type=VRPH_EUC_2D;
            }

            if(strncmp(temp2,"EUC_3D",6)==0)
            {
                edge_weight_type=VRPH_EUC_3D;

            }

            if(strncmp(temp2,"MAX_2D",6)==0)
            {
                edge_weight_type=VRPH_MAX_2D;

            }

            if(strncmp(temp2,"MAX_3D",6)==0)
            {
                edge_weight_type=VRPH_MAX_3D;

            }

            if(strncmp(temp2,"MAN_2D",6)==0)
            {
                edge_weight_type=VRPH_MAN_2D;

            }

            if(strncmp(temp2,"MAN_3D",6)==0)
            {
                edge_weight_type=VRPH_MAN_3D;

            }

            if(strncmp(temp2,"MAN_3D",6)==0)
            {
                edge_weight_type=VRPH_MAN_3D;

            }

            if(strncmp(temp2,"CEIL_2D",7)==0)
            {
                edge_weight_type=VRPH_CEIL_2D;
            }

            if(strncmp(temp2,"GEO",3)==0)
            {
                edge_weight_type=VRPH_GEO;

            }

            if(strncmp(temp2,"EXACT_2D",8)==0)
            {
                edge_weight_type=VRPH_EXACT_2D;

            }

            if(edge_weight_type == -1)
            {
                // We didn't find a known type
                fprintf(stderr,"Unknown/Unsupported EDGE_WEIGHT_TYPE %s encountered\n",temp2);
                report_error("%s\n",__FUNCTION__);
            }        

            break;
        case 9:
            // NODE_COORD_TYPE - we don't really care about this one
            temp2=strtok(NULL," ");
            if( (strncmp(temp2,"TWOD_COORDS",11)!=0) && (strncmp(temp2,"THREED_COORDS",13)!=0) )
            {
                fprintf(stderr,"Unknown coordinate type %s encountered",temp2);
                report_error("%s\n",__FUNCTION__);
            }

            break;
        case 10:
            // EOF - clean up and exit
            fclose(infile);

#if TSPLIB_DEBUG
            fprintf(stderr,"Found EOF completing calculations...\n");
#endif

            this->max_theta= -VRP_INFINITY;
            this->min_theta=  VRP_INFINITY;

            // Now normalize everything to put the VRPH_DEPOT at the origin
            // if it is a standard EXACT_2D problem or EUC_2D problem
            if( (this->edge_weight_type==VRPH_EXACT_2D || this->edge_weight_type==VRPH_EUC_2D) && has_nodes && has_depot && true)
            {

                this->depot_normalized=true;
                double depot_x=nodes[0].x;
                double depot_y=nodes[0].y;

#if TSPLIB_DEBUG
                fprintf(stderr,"Normalizing...(%f,%f)\n",depot_x,depot_y);
#endif

                for(i=0;i<=this->num_nodes+1;i++)
                {
                    nodes[i].x -= depot_x;
                    nodes[i].y -= depot_y;
                    // Translate everyone

                    // Calculate the polar coordinates as well
                    if(nodes[i].x==0 && nodes[i].y==0)
                    {
                        nodes[i].r=0;
                        nodes[i].theta=0;
                    }
                    else
                    {

                        nodes[i].r=sqrt( ((nodes[i].x)*(nodes[i].x)) + ((nodes[i].y)*(nodes[i].y)) );
                        nodes[i].theta=atan2(nodes[i].y,nodes[i].x);
                        // We want theta in [0 , 2pi]
                        // If y<0, add 2pi
                        if(nodes[i].y<0)
                            nodes[i].theta+=2*VRPH_PI;
                    }

                    // Update min/max theta across all nodes - don't include the VRPH_DEPOT/dummy
                    if(i!=0 && i!=(this->num_nodes+1))
                    {
                        if(nodes[i].theta>this->max_theta)
                            max_theta=nodes[i].theta;
                        if(nodes[i].theta<this->min_theta)
                            min_theta=nodes[i].theta;
                    }

                }
            }

#if TSPLIB_DEBUG
                fprintf(stderr,"Creating neighbor lists...\n");
#endif

            // Create the neighbor_lists-we may use a smaller size depending on the parameter
            // but we will construct the largest possible here...
            this->create_neighbor_lists(VRPH_MIN(MAX_NEIGHBORLIST_SIZE,num_nodes));

#if TSPLIB_DEBUG
            fprintf(stderr,"Done w/ calculations...\n");
#endif
            return;

        case 11:
            // NODE_COORD_SECTION
            // Import the data 
            this->can_display=true;

#if TSPLIB_DEBUG
            printf("num_nodes is %d\n",num_nodes);
#endif

            i=0;
            x=0;
            while(i<=num_nodes)
            {
                fscanf(infile,"%d",&x);
                fscanf(infile,"%f",&a);
                fscanf(infile,"%f\n",&b);

                nodes[i].id= x;

                nodes[i].x=(double) a;
                nodes[i].y=(double) b;

                
                i++;

            }
            has_nodes=true;

            break;

        case 12:
            // DEPOT_SECTION
            // Load in the Depot Coordinates
            fscanf(infile,"%d\n",&x);
            if(x!=1)
            {
                fprintf(stderr,"Expected VRPH_DEPOT to be entry 1 - VRPH does not currently support"
                    " multiple DEPOTs\n");
                report_error("%s\n",__FUNCTION__);
            }
                    
#if TSPLIB_DEBUG
            printf("VRPH_DEPOT has coordinates: %f, %f\n",a,b);
#endif
            //nodes[0].x=a;
            //nodes[0].y=b;
            //nodes[0].id=0;

            has_depot=true;
            fscanf(infile,"%d\n",&x);
            if(x!= -1)
            {
                fprintf(stderr, "Expected -1 at end of DEPOT_SECTION.  Encountered %d instead\n",x);
                report_error("%s\n",__FUNCTION__);
            }

            // Now set the dummy node id to the VRPH_DEPOT!
            nodes[num_nodes+1].x=nodes[0].x;
            nodes[num_nodes+1].y=nodes[0].y;
            nodes[num_nodes+1].id=0;

            // Now calculate distance matrix
            if(edge_weight_format==VRPH_FUNCTION || edge_weight_type!=VRPH_EXPLICIT)
            {

#if TSPLIB_DEBUG
                printf("Creating distance matrix using edge_weight_type %d\n",edge_weight_type);
#endif

                //Allocate memory for the distance matrix
                if(d==NULL)
                {
                    int n= num_nodes;
                    
                    d = new double* [n+2];
                    d[0] = new double [(n+2)*(n+2)];
                    for(i = 1; i < n+2; i++)
                        d[i] = d[i-1] + (n+2) ;
                }



                // Create the distance matrix using the appropriate 
                // distance function...
                create_distance_matrix(edge_weight_type);
            }

            break;
        case 13:

#if TSPLIB_DEBUG
            printf("in case 13: DEMAND_SECTION w/ problem type %d # num_days=%d\n",problem_type,
                this->num_days);
#endif
            // DEMAND_SECTION
            // Read in the demands
                
            if(this->num_days<=1)
            {
                i=0;
                while(i<= num_nodes+1)
                {

                    fscanf(infile,"%d %d\n",&x,&y);
                    nodes[i].id=x;
                    nodes[i].demand=y;                

                    if(nodes[i].id>max_id)
                        max_id=nodes[i].id;

                    if(has_service_times==false)
                        nodes[i].service_time=0;
                    i++;
                }
                nodes[num_nodes+1].demand=0;
                if(has_service_times==false)
                    nodes[num_nodes+1].service_time=0;

            }
            else
            {
                // We have multiple days worth of demands
                i=0;
                while(i<=num_nodes+1)
                {
                    fscanf(infile,"%d",&x);
                    nodes[i].id=x;
                    for(j=1;j<this->num_days;j++)
                    {
                        fscanf(infile,"%d",&y);
                        this->nodes[i].daily_demands[j]=y;
                        
                    }
                    fscanf(infile,"%d\n",&y);
                    this->nodes[i].daily_demands[this->num_days]=y;
                    i++;
                    
                }
                this->nodes[num_nodes+1].demand=0;
                for(j=1;j<=this->num_days;j++)
                    this->nodes[num_nodes+1].daily_demands[j]=0;// Dummmy node
                
#if TSPLIB_DEBUG
                printf("Done with multiple day demands\n");
#endif
            }            


            break;

        case 14:

#if TSPLIB_DEBUG
            printf("Case 14\n");
#endif
           
            // EDGE_WEIGHT_SECTION

            // Make sure distance matrix is allocated
            if(d==NULL)
            {                
                d = new double* [num_nodes+2];
                d[0] = new double [(num_nodes+2)*(num_nodes+2)];
                for(i = 1; i < num_nodes+2; i++)    
                    d[i] = d[i-1] + (num_nodes+2) ;
            }

            if(edge_weight_format==VRPH_UPPER_DIAG_ROW)
            {
                // The zeros on the diagonal should be in the matrix!
                for(i=0;i<= num_nodes;i++)
                {
                    for(j=i;j<= num_nodes;j++)
                    {
                        fscanf(infile,"%lf",&(d[i][j]));
                        d[j][i]=d[i][j];
                    }
                    // Add in column for the dummy-assumed to live at the VRPH_DEPOT
                    d[i][num_nodes+1]=d[i][0];
                }
                // Now add a row for the dummy
                for(j=0;j<=num_nodes+1;j++)
                    d[num_nodes+1][j]=d[0][j];

            }                

            if(edge_weight_format==VRPH_FULL_MATRIX)
            {
                this->symmetric=false;

                for(i=0;i<= num_nodes;i++)
                {
                    for(j=0;j<= num_nodes;j++)
                    {
                        fscanf(infile,"%lf",&(d[i][j]));
                    }
                    // Add in column for the dummy-assumed to live at the VRPH_DEPOT
                    d[i][num_nodes+1]=d[i][0];

                }
                // Now add a row for the dummy
                for(j=0;j<=num_nodes+1;j++)
                    d[num_nodes+1][j]=d[0][j];

            }
            
            // LOWER_DIAG_ROW format
            if(edge_weight_format==VRPH_LOWER_DIAG_ROW)
            {
                // The zeros on the diagonal should be in the matrix!
                for(i=0;i<= num_nodes;i++)
                {
                    for(j=0;j<= i;j++)
                    {
                        fscanf(infile,"%lf",&(d[i][j]));
                        d[j][i]=d[i][j];
                    }
                    // Add in column for the dummy-assumed to live at the VRPH_DEPOT
                    d[i][num_nodes+1]=d[i][0];

                }
                // Now add a row for the dummy
                for(j=0;j<=num_nodes+1;j++)
                    d[num_nodes+1][j]=d[0][j];

            }

            // UPPER_ROW format - no diagonal
            if(edge_weight_format==VRPH_UPPER_ROW)
            {
                for(i=0;i<=num_nodes;i++)
                {
                    for(j=i+1;j<= num_nodes;j++)
                    {
                        fscanf(infile,"%lf",&(d[i][j]));
                        d[j][i]=d[i][j];
                    }
                    // Add in column for the dummy-assumed to live at the VRPH_DEPOT
                    d[i][num_nodes+1]=d[i][0];
                    d[i][i]=0;

                }
                // Now add a row for the dummy
                for(j=0;j<=num_nodes+1;j++)
                    d[num_nodes+1][j]=d[0][j];

            }

             // LOWER_ROW format
            if(edge_weight_format==VRPH_LOWER_ROW)
            {
                for(i=0;i<= num_nodes;i++)
                {
                    for(j=0;j<i;j++)
                    {
                        fscanf(infile,"%lf",&(d[i][j]));
                        d[j][i]=d[i][j];
                    }
                    d[i][i]=0;
                    // Add in column for the dummy-assumed to live at the VRPH_DEPOT
                    d[i][num_nodes+1]=d[i][0];

                }
                // Now add a row for the dummy
                for(j=0;j<=num_nodes+1;j++)
                    d[num_nodes+1][j]=d[0][j];
            }

            // The last fscanf doesn't get the newline...
            fscanf(infile,"\n");
            break;

        case 15:

#if TSPLIB_DEBUG
            printf("Case 15\n");
#endif

            // SERVICE_TIME

            s =(double)(atof(strtok(NULL,"")));
            fixed_service_time=s;
#if TSPLIB_DEBUG
            printf("Setting service time to %f for all nodes\n");
#endif
            total_service_time=0;
            for(i=1;i<=num_nodes;i++)
            {
                // Set the service time to s for each non-depot node
                nodes[i].service_time=s;//+lcgrand(0) - use to test service times!
                total_service_time+=nodes[i].service_time;
            }
            nodes[VRPH_DEPOT].service_time=0;
            nodes[num_nodes+1].service_time=0;

            has_service_times=true;

            break;

        case 16:

#if TSPLIB_DEBUG
            printf("Case 16\n");
#endif

            // VEHICLES

            min_vehicles=atoi(strtok(NULL,""));
            // This is not currently used
#if TSPLIB_DEBUG
            printf("Setting min_vehicles to %d\n",min_vehicles);

#endif
            break;

        case 17:
        
            // NUM_DAYS
            this->num_days=atoi(strtok(NULL,""));
            break;

        case 18:
            // SVC_TIME_SECTION
            
            has_service_times=true;

            if(this->num_days==0)// Standard problem
            {
                i=0;
                while(i<= num_nodes)
                {

                    fscanf(infile,"%d %lf\n",&x,&s);
                    nodes[i].id=x;
                    nodes[i].service_time=s;                
                    total_service_time+=nodes[i].service_time;

                    if(nodes[i].id>max_id)
                        max_id=nodes[i].id;
                    i++;
                }
                nodes[num_nodes+1].service_time=0;

                
            }
            else
            {
                // We have multiple days worth of service times
                i=0;
                while(i<=num_nodes)
                {
                    fscanf(infile,"%d",&x);
                    nodes[i].id=x;
                    for(j=1;j<this->num_days;j++)
                    {
                        fscanf(infile,"%lf",&s);
                        this->nodes[i].daily_service_times[j]=s;
                    }
                    fscanf(infile,"%lf\n",&s);
                    this->nodes[i].daily_service_times[this->num_days]=s;
                    i++;
                    
                }
                this->nodes[num_nodes+1].service_time=0;
                for(j=1;j<=this->num_days;j++)
                    this->nodes[num_nodes+1].daily_service_times[j]=0;// Dummmy node
                
            }        

            
            break;
        case 19:
            // TIME_WINDOW_SECTION
            i=0;
            while(i<= num_nodes+1)
            {
                fscanf(infile,"%d %f %f\n",&x,&a,&b);
                nodes[i].start_tw=a;
                nodes[i].end_tw=b;
                i++;
            }

            break;
        case 20:
            // COMMENT
            // Don't care
            break;
        case 21:
            // DISPLAY_DATA_SECTION
            // Overwrite node's x and y coords with this data

            this->can_display=true;
            i=0;
            x=0;
            while(i<=num_nodes)
            {
                fscanf(infile,"%d",&x);
                fscanf(infile,"%f",&a);
                fscanf(infile,"%f\n",&b);

                nodes[x-1].x=(double) a;
                nodes[x-1].y=(double) b;                
                i++;

            }

            break;
        case 22:
            // TWOD_DISPLAY
            break;
        case 23:
            // DISPLAY_DATA_TYPE

            break;
        case 24:
            // NO_DISPLAY
            break;
        case 25:
            // COORD_DISPLAY
            break;
        }
    }

}    


void VRP::write_TSPLIB_file(const char *outfile)
{
    /// 
    /// Exports the data from an already loaded instance
    /// to a CVRP outfile in TSPLIB format (using EXACT_2D distance).
    ///

    FILE *out;
    int i;

    if( (out=fopen(outfile,"w"))==NULL)
    {
        report_error("%s: Can't open file for writing...\n",__FUNCTION__);
    }

    fprintf(out,"NAME: %s\n",name);
    fprintf(out,"TYPE: CVRP\n");
    if(this->best_known!=-1)
        fprintf(out,"BEST_KNOWN: %5.3f\n", this->best_known);
    fprintf(out,"DIMENSION: %d\n",num_nodes+1);
    fprintf(out,"CAPACITY: %d\n",max_veh_capacity);
    if(max_route_length!=VRP_INFINITY)
        fprintf(out,"DISTANCE: %4.5f\n",max_route_length);
    if(min_vehicles!=-1)
        fprintf(out,"VEHICLES: %d\n",min_vehicles);
    fprintf(out,"EDGE_WEIGHT_TYPE: FUNCTION\n");
    fprintf(out,"EDGE_WEIGHT_FORMAT: EXACT_2D\n");
    fprintf(out,"NODE_COORD_TYPE: TWOD_COORDS\n");
    fprintf(out,"NODE_COORD_SECTION\n");

    // Start numbering at 1!!
    fprintf(out,"%d %4.5f %4.5f\n",1,nodes[0].x,nodes[0].y);
    for(i=1;i<=num_nodes;i++)
    {
        fprintf(out,"%d %4.5f %4.5f\n",i+1,nodes[i].x,nodes[i].y);
    }

    fprintf(out,"DEMAND_SECTION\n");
    // Start numbering at 1!!
    fprintf(out,"1 0\n");
    for(i=1;i<=num_nodes;i++)
    {
        fprintf(out,"%d %d\n",i+1,nodes[i].demand);
    }

    fprintf(out,"DEPOT_SECTION\n");
    fprintf(out,"1\n-1\n");

    fprintf(out,"EOF\n");

    fclose(out);

}



void VRP::write_solution_file(const char *filename)
{
    ///
    /// Exports a solution to the designated filename in canonical form.  
    /// Let N be the # of non-VRPH_DEPOT nodes in the problem. Then the first entry in the file
    /// is N and the following N+1 entries simply traverse the solution in order where we enter
    /// a node's negative index if it is the first node in a route.
    /// The solution is put into canonical form - the routes are traversed in the orientation
    /// where the start index is less than the end index, and the routes are sorted by
    /// the start index.
    /// Example:    Route 1:  0-3-2-0, Route 2:  0-4-1-0
    /// File is then:
    /// 4 -1 4 -2 3 0    
    ///



    int n, current;
    FILE *out;

    int *sol;

    // Open the file
    if( (out = fopen(filename,"w"))==NULL)
    {
        fprintf(stderr,"Error opening %s for writing\n",filename);
        report_error("%s\n",__FUNCTION__);
    }

    // First, determine the # of nodes in the Solution -- this could be different than the
    // VRP.num_nodes value if we are solving a subproblem

    n=0;
    current=VRPH_ABS(next_array[VRPH_DEPOT]);
    while(current!=VRPH_DEPOT)
    {
        current=VRPH_ABS(next_array[current]);
        n++;
    }
    // We have n non-VRPH_DEPOT nodes in the problem
    sol=new int[n+2];
    // Canonicalize
    this->export_canonical_solution_buff(sol);
    this->import_solution_buff(sol);
    fprintf(out,"%d ",n);


    // Now output the ordering - this is actually just the sol buffer
    current=next_array[VRPH_DEPOT];
    fprintf(out,"%d ",current);
    while(current!=VRPH_DEPOT)
    {
        current=next_array[VRPH_ABS(current)];
        fprintf(out,"%d ",current);

    }

    fprintf(out,"\n\n\n");


    fflush(out);
    fprintf(out,"\n\n\nOBJ=\n%5.3f\nBEST_KNOWN=\n%5.3f",this->total_route_length-this->total_service_time,
        this->best_known);

    fflush(out);
    fclose(out);

    delete [] sol;
    return;
}

void VRP::write_solutions(int num_sols, const char *filename)
{
    ///
    /// Writes num_sols solutions from the solution warehouse to the designated filename. 
    /// The format is the same as for write_solution_file.
    ///

    int i,n, current;
    FILE *out;
    int *sol;

    sol=new int[this->num_original_nodes+2]; // should be big enough

    // Open the file
    if( (out = fopen(filename,"w"))==NULL)
    {
        fprintf(stderr,"Error opening %s for writing\n",filename);
        report_error("%s\n",__FUNCTION__);
    }

    for(i=0;i<num_sols;i++)
    {
        if(i>this->solution_wh->num_sols)
            report_error("%s: too many solutions!\n",__FUNCTION__);

        // Import this solution and then write it out
        this->import_solution_buff(this->solution_wh->sols[i].sol);
        this->export_canonical_solution_buff(sol);
        this->import_solution_buff(sol);


        // First, determine the # of nodes in the Solution -- this could be different than the
        // VRP.num_nodes value if we are solving a subproblem

        n=0;
        current=VRPH_ABS(next_array[VRPH_DEPOT]);
        while(current!=VRPH_DEPOT)
        {
            current=VRPH_ABS(next_array[current]);
            n++;
        }
        // We have n non-VRPH_DEPOT nodes in the problem
        fprintf(out,"%d ",n);

        // Now output the ordering
        current=next_array[VRPH_DEPOT];
        fprintf(out,"%d ",current);
        while(current!=VRPH_DEPOT)
        {
            current=next_array[VRPH_ABS(current)];
            fprintf(out,"%d ",current);

        }
        fprintf(out,"\n");
    }

    fflush(out);
    fclose(out);
    return;

}


void VRP::write_tex_file(const char *filename)
{
    ///
    /// Writes the solution in a TeX tabular format using the
    /// longtable package in case the solution spans multiple
    /// pages.
    ///

    int i;

    FILE *out;
    if( (out=fopen(filename,"w"))==NULL)
    {
        fprintf(stderr,"Error opening %s for writing\n",filename);
        report_error("%s\n",__FUNCTION__);
        
    }    

    // The headers for the first page and pages thereafter
    fprintf(out,"%% TeX file automatically generated by VRPH for problem %s\n\n",this->name);
    fprintf(out,"\\renewcommand{\\baselinestretch}{1}\n");
    fprintf(out,"\\footnotesize\n");
    fprintf(out,"\\begin{center}\n");
    fprintf(out,"\\begin{longtable}{|c|r|r|p{4 in}|}\n");

    fprintf(out,"\\hline\n");
    fprintf(out,"Route&\\multicolumn{1}{c|}{Length}&\\multicolumn{1}{c|}{Load}&\\multicolumn{1}{c|}{Ordering}\\\\\n");
    fprintf(out,"\\hline\n");
    fprintf(out,"\\endhead\n");
    fprintf(out,"\\hline\n");
    fprintf(out,"\\multicolumn{3}{|l|}{Problem}&%s\\\\\n",this->name);
    fprintf(out,"\\hline\n");
    fprintf(out,"\\endfirsthead\n");
    fprintf(out,"\\endfoot\n");
    fprintf(out,"\\endlastfoot\n");
    fprintf(out,"\\multicolumn{3}{|l|}{Vehicle capacity}&%d\\\\\n",this->max_veh_capacity);
    if(this->max_route_length!=VRP_INFINITY)
        fprintf(out,"\\multicolumn{3}{|l|}{Maximum route length}&%5.3f\\\\\n",this->max_route_length);
    else
        fprintf(out,"\\multicolumn{3}{|l|}{Maximum route length}&N/A\\\\\n");
    if(this->total_service_time>0)
        fprintf(out,"\\multicolumn{3}{|l|}{Total service time}&%5.3f\\\\\n",this->total_service_time);
    fprintf(out,"\\multicolumn{3}{|l|}{Number of nodes}&%d\\\\\n",this->num_nodes);
    fprintf(out,"\\multicolumn{3}{|l|}{Total route length}&%5.3f\\\\\n",this->total_route_length-
        this->total_service_time);
    fprintf(out,"\\multicolumn{3}{|l|}{Total number of routes}&%d\\\\\n",this->total_number_of_routes);
    fprintf(out,"\\hline\n");
    fprintf(out,"Route&\\multicolumn{1}{c|}{Length}&\\multicolumn{1}{c|}{Load}&\\multicolumn{1}{c|}{Ordering}\\\\\n");
    fprintf(out,"\\hline\n");

    for(i=1;i<=this->total_number_of_routes;i++)
    {
        fprintf(out,"%d&%5.3f&%d&(0",i,this->route[i].length,this->route[i].load);
        int current=this->route[i].start;
        while(current>=0)
        {
            fprintf(out,", %d",current);
            current=this->next_array[current];

        }
        fprintf(out,", 0)\\\\\n");
        fprintf(out,"\\hline\n");
    }
    fprintf(out,"\\caption{The best solution found for problem %s}\n",this->name);
    fprintf(out,"\\end{longtable}\n");
    fprintf(out,"\\end{center}\n");
    fprintf(out,"\\renewcommand{\\baselinestretch}{2}\n");
    fprintf(out,"\\normalsize\n");
    fclose(out);
}


void VRP::read_solution_file(const char *filename)
{
    ///
    /// Imports a solution from filename.  File is assumed to be in the
    /// format produced by VRP.write_solution_file
    ///

    FILE *in;

    if( (in=fopen(filename,"r"))==NULL)
    {
        fprintf(stderr,"Error opening %s for reading\n",filename);
        report_error("%s\n",__FUNCTION__);
    }

    int *new_sol;
    int i,n;
    fscanf(in,"%d",&n);
    new_sol=new int[n+2];
    new_sol[0]=n;
    for(i=1;i<=n+1;i++)
        fscanf(in,"%d",new_sol+i);
    
    // Import the buffer
    this->import_solution_buff(new_sol);
    fclose(in);
    delete [] new_sol;

    this->verify_routes("After read_solution_file\n");

    memcpy(this->best_sol_buff,this->current_sol_buff,(this->num_nodes+2)*sizeof(int));

    return;

}



void VRP::import_solution_buff(int *sol_buff)
{
    ///
    /// Imports a solution from buffer produced by something like
    /// export_solution_buff().  Can be a partial solution if the first
    /// element in sol_buff[] is less than num_original_nodes;
    ///


    int i, n, rnum, current, next, load, num_in_route;
    double len;

    next=-1; //to avoid warning...

    // Set all nodes to unrouted
    for(i=1;i<=this->num_original_nodes;i++)
        routed[i]=false;

    len=0;
    load=0;

    total_route_length=0;

    this->num_nodes = sol_buff[0];

    n=this->num_nodes;
    num_in_route=0;

    rnum=1;

    current=VRPH_ABS(sol_buff[1]);
    routed[current]=true;
    next_array[VRPH_DEPOT]=sol_buff[1];
    route_num[VRPH_ABS(current)]=rnum;
    route[rnum].start=VRPH_ABS(current);
    load+=nodes[VRPH_ABS(current)].demand;
    len+=d[VRPH_DEPOT][VRPH_ABS(current)];
    num_in_route++;

    for(i=2;i<=n;i++)
    {
        next=sol_buff[i];
        routed[VRPH_ABS(next)]=true;

        if(next<0)
        {
            // end of route - current is the last node in this route
            // next is the first node in the next route

            len+=d[VRPH_ABS(current)][VRPH_DEPOT];

            route[rnum].end=VRPH_ABS(current);
            route[rnum].length=len;
            route[rnum].load=load;
            route[rnum].num_customers=num_in_route;
            total_route_length+=len;

            if(rnum>n)
            {
                fprintf(stderr,"%d>%d:  rnum too big in import solution buff!\n",rnum,n);
                for(i=0;i<=n;i++)
                    fprintf(stderr,"%d ",sol_buff[i]);
        
                report_error("%s\n",__FUNCTION__);
            }


            rnum++;
            num_in_route=0;
            len=0;
            load=0;
            len+=d[VRPH_DEPOT][VRPH_ABS(next)];
            route_num[VRPH_ABS(next)]=rnum;
            route[rnum].start=VRPH_ABS(next);
        }
        else
            // Not at the end of a route
            len+=d[VRPH_ABS(current)][VRPH_ABS(next)];



        next_array[VRPH_ABS(current)]=next;
        current=next;

        load+=nodes[VRPH_ABS(current)].demand;
        num_in_route++;

        route_num[VRPH_ABS(current)]=rnum;
    }


    next_array[VRPH_ABS(next)]=VRPH_DEPOT;
    route_num[VRPH_ABS(next)]=rnum;

    len+=d[VRPH_ABS(next)][VRPH_DEPOT];

    route[rnum].end=VRPH_ABS(next);
    route[rnum].length=len;
    route[rnum].load=load;
    route[rnum].num_customers=num_in_route;
    total_route_length+=len;
    total_number_of_routes=rnum;
    create_pred_array();

    // Make sure everything imported successfully!
    verify_routes("After import sol_buff\n");

    // Mark all the nodes as "routed"
    for(i=1;i<=sol_buff[0];i++)
        routed[VRPH_ABS(sol_buff[i])]=true;

    routed[VRPH_DEPOT]=true;

    route_num[VRPH_DEPOT]=0;

    memcpy(this->current_sol_buff,sol_buff,(this->num_nodes+2)*sizeof(int));

    return;

}
void VRP::export_solution_buff(int *sol_buff)
{
    ///
    /// Exports the solution to sol_buff.
    ///

    int i, current;

    sol_buff[0]=num_nodes;    

    // Now output the ordering
    current=next_array[VRPH_DEPOT];
    sol_buff[1]=current;
    i=2;

    while(current!=VRPH_DEPOT)
    {
        current=next_array[VRPH_ABS(current)];
        sol_buff[i]=current;
        i++;
    }

    return;
}

void VRP::export_canonical_solution_buff(int *sol_buff)
{
    ///
    /// Puts the solution into the buffer in a "canonical form".
    /// The orientation of each route is such that start<end.
    /// Also, the ordering of the different routes is determined
    /// so that route i precedes route j in the ordering if
    /// start_i < start_j.
    ///

    int i,j,next;
    int *start_buff;

    start_buff=new int[total_number_of_routes];
    
    this->normalize_route_numbers();

    // First orient each route properly
    for(i=1;i<=total_number_of_routes;i++)
    {
        if(route[i].end<route[i].start)
            reverse_route(i);

        start_buff[i-1]=route[i].start;
    }


    // Sort the start_buff
    qsort(start_buff, total_number_of_routes, sizeof(int), VRPIntCompare);

    sol_buff[0]=this->num_nodes;

    // Now order the routes themselves
    j=1;
    for(i=0;i<total_number_of_routes;i++)
    {
        sol_buff[j]=-start_buff[i];
        for(;;)
        {
            next=this->next_array[VRPH_ABS(sol_buff[j])];
            if(next<=0)
                break; // next route

            j++;
            sol_buff[j]=next;
        }
        j++;
    }
    
    sol_buff[j]=VRPH_DEPOT;

    delete [] start_buff;

    return;

}

void VRP::show_routes()
{
    ///
    /// Displays all routes in the solution.
    ///


    int i, cnt;
    int route_start;
    int next_node_number;
    int current_node, current_route;
    int total_load = 0;


    printf("-----------------------------------------------\n");
    printf("Total route length:  %5.2f\n",total_route_length);
    i = 1;
    cnt = 0;
    route_start = -next_array[VRPH_DEPOT];
    current_node = route_start;
    current_route = route_num[current_node];
    total_load+= route[current_route].load;


    printf("\nRoute %04d(routenum=%d)[0-%d...%d-0, %5.2f, %d, %d]: \n",i,current_route,
        nodes[route[current_route].start].id,
        nodes[route[current_route].end].id,
        route[current_route].length,
        route[current_route].load,
        route[current_route].num_customers);

    printf("%d-%d-",VRPH_DEPOT,nodes[route_start].id);

    cnt++;

    while(route_start != 0 && i<num_nodes+1)
    {
        // When we finally get a route beginning at 0, this is the last route 
        // and there is no next route, so break out
        if(next_array[current_node]==0)
        {
            printf("%d\n",VRPH_DEPOT);
            printf("End of routes.  Totals: (%d routes,%d nodes,%d total load)\n",i,cnt,total_load);
            printf("-----------------------------------------------\n");
            if(cnt!= num_nodes)
            {
                fprintf(stderr,"Not enough nodes! counted=%d; claimed=%d\n",cnt,num_nodes);
                report_error("%s\n",__FUNCTION__);
            }

            return;
        }

        if(next_array[current_node]>0)
        {
            // Next node is somewhere in the middle of a route
            next_node_number = next_array[current_node];
            printf("%d-",nodes[next_node_number].id);

            current_node=next_node_number;
            cnt++;

            if(cnt>num_nodes)
            {
                fprintf(stderr,"Too many nodes--cycle?\n");
                fprintf(stderr,"Count is %d, num_nodes is %d\n",cnt,num_nodes);
                show_next_array();
                report_error("%s\n",__FUNCTION__);
            }
        }
        else
        {
            // We must have a non-positive "next" node indicating the beginning of a new route
            i++;
            printf("%d",VRPH_DEPOT);

            route_start = - (next_array[current_node]);    
            current_route = route_num[route_start];
            current_node = route_start;

            printf("\n\nRoute %04d(routenum=%d)[0-%d...%d-0, %3.2f, %d, %d]: \n",i,current_route,
                nodes[route[current_route].start].id,
                nodes[route[current_route].end].id,
                route[current_route].length,
                route[current_route].load,
                route[current_route].num_customers);


            // Print out the beginning of this route
            total_load += route[current_route].load;
            printf("%d-%d-",VRPH_DEPOT,nodes[current_node].id);
            cnt++;
        }
    }

    
}




void VRP::show_route(int k)
{
    ///
    /// Displays information about route number k.
    ///

    int current_node;
    int i=0;
    if(k<=0)
        report_error("%s: called with non-positive route number\n",__FUNCTION__);

    printf("\nRoute %03d[0-%03d...%03d-0, %5.3f, %d, %d]: \n",k,
        route[k].start,
        route[k].end,
        route[k].length,
        route[k].load,
        route[k].num_customers);

    printf("%d-",VRPH_DEPOT);

    current_node= route[k].start;
    while(current_node != route[k].end)
    {
        printf("%03d-",current_node);
        current_node= next_array[current_node];
        i++;
        if(i>num_nodes)
            report_error("%s: encountered too many nodes!!\n",__FUNCTION__);
    }
    printf("%03d-%d\n\n",current_node,VRPH_DEPOT);

}

void VRP::summary()
{
    ///
    /// This function prints out a summary of the current solution and the individual routes.
    ///

    int i, cnt;
    int route_start;
    int next_node_number;
    int current_node, current_route;
    int total_load = 0;
    int num_in_route=0;
    int total_nodes=0;
    int cust_count=0;
    bool feasible=true;

    printf("\n------------------------------------------------\n");
    printf("Solution for problem %s\n",this->name);
    printf("Total route length:       %5.2f\n",total_route_length-this->total_service_time);
    if(this->best_known!=VRP_INFINITY)
        printf("Best known solution:      %5.2f\n",this->best_known);
    printf("Total service time:       %5.2f\n",this->total_service_time);
    if(this->max_route_length!=VRP_INFINITY)
        printf("Vehicle max route length: %5.2f\n",this->max_route_length);
    else
        printf("Vehicle max route length: N/A\n");
    printf("Vehicle capacity:         %d\n",this->max_veh_capacity);
    printf("Number of nodes visited:  %d\n",this->num_nodes);
    printf("------------------------------------------------\n");
    i = 1;
    cnt = 0;
    route_start = -next_array[VRPH_DEPOT];
    current_node = route_start;
    current_route = route_num[current_node];
    total_load+= route[current_route].load;


    printf("\nRoute %03d[0-%03d...%03d-0\tlen=%03.2f\tload=%04d\t#=%03d]",i,route[current_route].start,
        route[current_route].end,route[current_route].length,
        route[current_route].load,route[current_route].num_customers);
    // Check feasibility
    if(route[current_route].length>this->max_route_length || 
        route[current_route].load > this->max_veh_capacity)
        feasible=false;
    cust_count+= route[current_route].num_customers;

    if(current_node!= dummy_index)
        num_in_route=1;

    total_nodes++;

    cnt++;

    while(route_start != 0 && i<num_nodes+1)
    {
        // When we finally get a route beginning at 0, this is the last route 
        // and there is no next route, so break out
        if(next_array[current_node]==0)
        {
            num_in_route=0;
            if(cnt!= num_nodes)
            {
                fprintf(stderr,"Not enough nodes:  counted=%d; claimed=%d!\n",cnt,num_nodes);
                report_error("%s\n",__FUNCTION__);
            }

            printf("\n\n");
            if(!feasible)
                printf("\nWARNING:  Solution appears to be infeasible!\n");
            return;
        }

        if(next_array[current_node]>0)
        {
            // Next node is somewhere in the middle of a route
            next_node_number = next_array[current_node];
            if(current_node!= dummy_index)
                num_in_route++;

            total_nodes++;
            current_node=next_node_number;
            cnt++;
        }
        else
        {
            // We must have a non-positive "next" node indicating the beginning of a new route
            i++;
            total_nodes++;
            num_in_route=0;

            route_start = - (next_array[current_node]);    
            current_route = route_num[route_start];
            current_node = route_start;

            printf("\nRoute %03d[0-%03d...%03d-0\tlen=%03.2f\tload=%04d\t#=%03d]",i,route[current_route].start,
                route[current_route].end,route[current_route].length,
                route[current_route].load,route[current_route].num_customers);
            cust_count+= route[current_route].num_customers;

            if(route[current_route].length>this->max_route_length || 
                route[current_route].load > this->max_veh_capacity)
                feasible=false;

            total_load += route[current_route].load;
            cnt++;
            if(current_node!= dummy_index)
                num_in_route++;
        }
    }
}

