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
    /// This is the main() routine that demonstrates how to import solutions and 
    /// plot them.
    ///

    VRPH_version();

    char in[VRPH_STRING_SIZE], plotfile[VRPH_STRING_SIZE],pdffile[VRPH_STRING_SIZE],
     solfile[VRPH_STRING_SIZE];
    
    int i,n;
    int orientation=1;
    bool do_pdf=false;

    // Usage: vrp_plotter -f <vrp_file> -s <sol_file> -p <plot_file>
    if(argc<7 )
    {        
        fprintf(stderr,"Usage: %s -f <vrp_file> -s <sol_file> -p <plot_file> [-e -w -r]\n",argv[0]);
        fprintf(stderr,"\t <plot_file> must be .ps format\n");
        fprintf(stderr,"\t .pdf format requires the epstopdf executable\n");
        fprintf(stderr,"\t -pdf <pdf_file> will also create a .pdf image of the solution\n");
        fprintf(stderr,"\t -e will not show the depot edges\n");
        fprintf(stderr,"\t -w will weight the size of the non-depot nodes by their demand\n");
        fprintf(stderr,"\t -o will rotate 90 degrees\n");
        exit(-1);
    }


    int options=VRPH_DEFAULT_PLOT;

    bool has_filename=false;
    for(i=1;i<argc;i++)
    {
        if(strcmp(argv[i],"-f")==0)
        {
            // Get the input file name
            has_filename=true;
            strcpy(in,argv[i+1]);            
        }

        if(strcmp(argv[i],"-s")==0)
        {
            // Get the solution file name
            strcpy(solfile,argv[i+1]);            
        }
        
        if(strcmp(argv[i],"-p")==0)
        {
            // Get the plot file name
            strcpy(plotfile,argv[i+1]);
            
        }

        if(strcmp(argv[i],"-pdf")==0)
        {
            do_pdf=true;
            strncpy(pdffile,argv[i+1],strlen(argv[i+1]));
            pdffile[strlen(argv[i+1])]='\0';
        }

        if(strcmp(argv[i],"-e")==0)
            options+=VRPH_NO_DEPOT_EDGES;

        if(strcmp(argv[i],"-w")==0)
            options+=VRPH_WEIGHTED;     

        if(strcmp(argv[i],"-r")==0)
        {
            // Get the plot file name
            orientation=0;
            
        }
        
    }

    if(!has_filename)
        report_error("No file name given!\n");

    n=VRPGetDimension(in);
    VRP V(n);

    // Load the problem data
    V.read_TSPLIB_file(in);    

    printf("Imported instance\n");

    V.read_solution_file(solfile);
    int *sol;
    sol=new int[V.get_num_nodes()+2];
    V.export_canonical_solution_buff(sol);
    V.import_solution_buff(sol);
    printf("Imported solution\n");

    // Show the solution to stdout
    V.summary();

    printf("Plotfile is %s\n",plotfile);
    V.plot(plotfile,options,orientation);

    if(do_pdf)
    {
        // Create a .pdf file
        char command[VRPH_STRING_SIZE];
        sprintf(command,"%s %s --outfile=%s\n",VRPH_EPS_EXE,plotfile,pdffile);
        system(command);
    }

    delete [] sol;

    return 0;

}


