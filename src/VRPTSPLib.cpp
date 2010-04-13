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

// Lists the various TSPLIB strings that are supported
const char *SupportedTSPLIBStrings[]={
    "NAME","TYPE","BEST_KNOWN","DIMENSION",
    // 4
    "CAPACITY","DISTANCE","EDGE_WEIGHT_FORMAT","EDGE_WEIGHT_TYPE",
    // 8
    "NODE_COORD_TYPE","EOF","NODE_COORD_SECTION","DEPOT_SECTION",
    // 12
    "DEMAND_SECTION","EDGE_WEIGHT_SECTION","SERVICE_TIME","cxd",
    // 16
    "NUM_DAYS","SVC_TIME_SECTION","TIME_WINDOW_SECTION","COMMENT",
    // 20
    "DISPLAY_DATA_SECTION","TWOD_DISPLAY","DISPLAY_DATA_TYPE","NO_DISPLAY",
    // 24
    "COORD_DISPLAY"};


// The lengths of the different supported TSPLIB strings
const int SL[]={4,4,10,9,
                8,8,18,16,
                14,3,18,13,
                13,14,12,8,
                8,16,19,7,
                20,12,17,10,
                13};

// Simply the number of supported strings
const int NumSupportedTSPLIBStrings = 25;

// Lists the various TSPLIB strings that are NOT supported
const char *UnsupportedTSPLIBStrings[]=    {
    "HCP","ATSP","SOP","TOUR","ATT","XRAY1","XRAY2","SPECIAL",
    "LOWER_ROW",
    "LOWER_DIAG_ROW","UPPER_COL","LOWER_COL","UPPER_DIAG_COL",
    "LOWER_DIAG_COL","EDGE_LIST","ADJ_LIST","NO_COORDS",
    "EDGE_DATA_SECTION",
    "TOUR_SECTION"
};
// The number of unsupported TSPLIB strings
const int NumUnsupportedTSPLIBStrings = 20;

int VRPGetDimension(char *filename)
{
    ///
    /// Open up filename (assumed to be in TSPLIB format) and get the dimension
    /// of the problem, scanning for the string "DIMENSION" and makes sure that
    ///    the "EOF" string is also found.
    ///

    FILE *infile;
    char str1[VRPH_STRING_SIZE];
    int i,n;

    n=-1;
    for(i=0;i<VRPH_STRING_SIZE;i++)
        str1[i]=0;

    infile=fopen(filename,"r");
    if(infile==NULL)
    {
        fprintf(stderr,"Unable to open %s for reading\n",filename);
        exit(-1);

    }

    fscanf(infile,"%s",str1);

    while(strncmp(str1,"DIMENSION:",10)!=0 && strncmp(str1,"DIMENSION",9)!=0)
    {
        fscanf(infile,"%s\n",str1);
        if(feof(infile))
        {    
            fprintf(stderr, "The keyword DIMENSION was not found in the TSPLIB file %s\nExiting...\n",filename);
            exit(-1);
        }

    }
    // We found the keyword DIMENSION-get n now - watch out for an annoying possibility involving the colon!
    fscanf(infile,"%s",str1);
    if(strncmp(str1,":",1)==0)
        fscanf(infile,"%d\n",&n);
    else
        n=atoi((const char *)str1);

    //Check for EOF
    rewind(infile);

    // Now make sure we have the string EOF as the very last line

    fscanf(infile,"%s\n",str1);
    while(strncmp(str1,"EOF",3)!=0)
    {
        fscanf(infile,"%s\n",str1);        

        if(feof(infile) && strncmp(str1,"EOF",3)!=0)
        {    
            fprintf(stderr, "The keyword EOF was not found in the TSPLIB file %s\n",
                filename);
            exit(-1);
        }

    }

    fclose(infile);

    // We return the # of non-VRPH_DEPOT nodes
    return n-1;

}


int VRPGetNumDays(char *filename)
{
    ///
    /// Open up filename (assumed to be in TSPLIB format) and get the dimension
    /// of the problem, scanning for the string "NUM_DAYS".  If the string is not 
    /// found, then we assume it is a typical 1-day problem.
    ///

    FILE *infile;
    char str1[VRPH_STRING_SIZE];
    int i,n;

    n=-1;
    for(i=0;i<VRPH_STRING_SIZE;i++)
        str1[i]=0;

    infile=fopen(filename,"r");
    if(infile==NULL)
    {
        fprintf(stderr,"Unable to open %s for reading\n",filename);
        exit(-1);

    }

    fscanf(infile,"%s",str1);

    while(strncmp(str1,"NUM_DAYS",8)!=0 && strncmp(str1,"NUM_DAYS:",9)!=0)
    {
        fscanf(infile,"%s\n",str1);
        if(feof(infile))
        {    
            // Assume a 1-day problem
            fclose(infile);
            return 1;
        }

    }
    // We found the keyword NUM_DAYS - get the value and return
    fscanf(infile,"%d\n",&n);
    fclose(infile);
    return n;



}

int VRPCheckTSPLIBString(char *s)
{
    ///
    /// Determines whether or not a given string in an input file
    /// is a supported TSPLIB string.  Returns the reference number
    /// for the string if supported, and 0 otherwise.
    ///


    int i;
#if TSPLIB_DEBUG
    fprintf(stderr,"Checking string %s\n",s);
#endif

    for(i=0;i<NumSupportedTSPLIBStrings;i++)
    {
        if(strncmp((const char *)s, SupportedTSPLIBStrings[i],SL[i])==0)
            return i+1;
    }

#if TSPLIB_DEBUG
    fprintf(stderr,"Didn't find string %s in supported list\n",s);
#endif

    // Didn't find a supported string - check for other known 
    // unsupported strings


    for(i=0;i<NumUnsupportedTSPLIBStrings;i++)
    {
        if(strcmp((const char *)s, UnsupportedTSPLIBStrings[i])==0)
            return -(i+1);
    }

    // Unknown string encountered
    fprintf(stderr,"Unknown string %s encountered\n", s);
    report_error("%s: Error related to TSPLIB string\n");
    return -1;
}
