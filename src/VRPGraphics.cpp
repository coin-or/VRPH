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

bool VRP::plot(const char *filename, int options, int orientation)
{

    ///
    /// Uses PLPLOT to draw the solution in a .ps file (no other formats
    /// supported).  Valid options are VRPH_BLACK_AND_WHITE, VRPH_COLOR,
    /// VRPH_BARE_BONES, AXES, VRPH_BOXED, TITLED, VRPH_NO_POINTS, VRPH_NO_DEPOT_EDGES,
    /// VRPH_WEIGHTED.  If options==0, then the default is to 
    /// draw a VRPH_BOXED, COLORed plot, with AXES and a TITLE.  Setting VRPH_BOXED draws
    /// a box around the plot with no axes, and setting VRPH_BARE_BONES draws no
    /// BOX or AXES.  If VRPH_NO_POINTS is set, then the nodes are not drawn on the
    /// plot (default is to plot the points).  Setting the option VRPH_NO_DEPOT_EDGES
    /// will draw each route with the first and last edges of each route not shown
    /// and a VRPH_WEIGHTED plot will make the size of the points proportional
    /// to their demand.  The value of
    /// orientation should be 0, 1, 2, or 3, indicating that the image
    /// should be rotated through an angle of Pi/2 times orientation.
    /// A very primitive attempt is made to scale the line width and point size
    /// based on the problem size (number of nodes)
    ///

    if(!this->can_display)
    {
        fprintf(stderr,"Unable to display.  No coordinates known!\n");
        return false;
    }

#ifdef HAS_PLPLOT

    PLFLT *x, *y;
    int current,  i, ctr, rnum;
    PLFLT xmin, xmax, ymin, ymax;
    int colors[]={VRPH_BLUE,VRPH_RED,VRPH_GREEN,VRPH_CYAN,VRPH_BROWN,VRPH_MAGENTA,VRPH_TURQUOISE,VRPH_VIOLET};
    int num_colors = 8;

    rnum = 0;

    // Allocate storage
    x=new PLFLT [3*num_original_nodes+1];
    y=new PLFLT [3*num_original_nodes+1];
    // Max size would be default CW routes--this should be plenty!

    // Rotate
    plsori(orientation);

    // Set filename
    plsfnam(filename);

    if(options & VRPH_BLACK_AND_WHITE)
        plsdev("ps");
    else
        // default
        plsdev("psc");

    // Background color-set to white
    plscolbg (255,255,255);

    // Initialize
    plinit();

    // Set to blue for axes and numbers
    if(!(options & VRPH_BLACK_AND_WHITE))
        plcol0(VRPH_BLUE);

    // Now find the min and max coords
    xmin=VRP_INFINITY;
    ymin=VRP_INFINITY;
    xmax=-VRP_INFINITY;
    ymax=-VRP_INFINITY;

    for(i=0; i <= this->num_original_nodes; i++)
    {
        if((PLFLT)nodes[i].x<xmin)
            xmin=(PLFLT)nodes[i].x;
        if((PLFLT)nodes[i].x>xmax)
            xmax=(PLFLT)nodes[i].x;

        if((PLFLT)nodes[i].y<ymin)
            ymin=(PLFLT)nodes[i].y;
        if((PLFLT)nodes[i].y>ymax)
            ymax=(PLFLT)nodes[i].y;
    }

    // Stretch the window by a factor of .05
    xmin=xmin-.05*VRPH_ABS(xmax-xmin);
    xmax=xmax+.05*VRPH_ABS(xmax-xmin);

    ymin=ymin-.05*VRPH_ABS(ymax-ymin);
    ymax=ymax+.05*VRPH_ABS(ymax-ymin);
    
    // Create environment
    if(options & VRPH_BOXED)
        plenv(xmin,xmax,ymin,ymax,2,-1);
    else
        if(options & VRPH_BARE_BONES)
            plenv(xmin,xmax,ymin,ymax,2,-2);
        else
            // Default
            plenv(xmin,xmax,ymin,ymax,2,0);

    char dest[VRPH_STRING_SIZE];
    if(has_service_times==false)
        sprintf(dest,"%d nodes   %7.2f    %d routes", num_nodes,total_route_length,
        count_num_routes());
    else
        sprintf(dest,"%d nodes   %7.2f    %d routes",num_nodes,total_route_length-total_service_time,
        count_num_routes());


    // Add a label 
    if(!(options & VRPH_NO_TITLE))
        pllab("", "", dest);

    // These may need to be modified for some problems...
    // Defaults
    // Set the symbol size
    plssym(0,.5);
    // Set the line width
    plwid (10);

    double symbol_size=0;
    // Can roughly base this on num_nodes
    if(this->num_nodes<200)
    {
        // Set the symbol size
        plssym(0,.6);
        symbol_size=.6;
        // Set the line width
        plwid (9);
    }

    if(this->num_nodes>=200 && this->num_nodes<500)
    {
        // Set the symbol size
        plssym(0,.4);
        symbol_size=.4;
        // Set the line width
        plwid (8);
    }

    if(this->num_nodes>=500)
    {
        // Set the symbol size
        plssym(0,.2);
        symbol_size=.2;
        // Set the line width
        plwid (8);
    }

    int R;

    normalize_route_numbers();
    R= count_num_routes();

    for(i=1;i<=R;i++)
    {
        // Plot route i
        ctr=0;

        if(!(options&VRPH_NO_DEPOT_EDGES))
        {
            x[ctr]= nodes[VRPH_DEPOT].x;
            y[ctr]= nodes[VRPH_DEPOT].y;
            ctr++;
        }

        current= route[i].start;
        while(current!=VRPH_DEPOT)
        {
            x[ctr]= nodes[current].x;
            y[ctr]= nodes[current].y;
            ctr++;    
            current= VRPH_MAX(VRPH_DEPOT,next_array[current]);
        }

        // Close the route at the VRPH_DEPOT
        if(!(options&VRPH_NO_DEPOT_EDGES))
        {
            x[ctr]= nodes[VRPH_DEPOT].x;
            y[ctr]= nodes[VRPH_DEPOT].y;
            ctr++;
        }

        // Set the color
        if(!(options & VRPH_BLACK_AND_WHITE))
            plcol0(colors[i % num_colors]);

        // Draw the lines
        plline(ctr,x,y);


    }

    // Now draw the points
    if(!(options & VRPH_BLACK_AND_WHITE) || options==VRPH_DEFAULT_PLOT)
        plcol0(VRPH_BLUE);
    int j=0;
    double max_ratio=-VRP_INFINITY;
    for(i=1;i<=this->num_original_nodes;i++)
    {
        if(routed[i])
        {
            x[j]=this->nodes[i].x;
            y[j]=this->nodes[i].y;
            // Plot the point
            if(!(options & VRPH_NO_POINTS))
            {
                if(options & VRPH_WEIGHTED)
                    plssym( 0, 2*sqrt((double)(this->nodes[i].demand)/(double)(this->max_veh_capacity)) );
                else
                    plssym( 0, symbol_size );

                if((double)(this->nodes[i].demand)/(double)(this->max_veh_capacity) >max_ratio)
                    max_ratio=(double)(this->nodes[i].demand)/(double)(this->max_veh_capacity) ;
                plpoin(1,x+j,y+j,4);
            }
            j++;
        }
    }

    j=0;
    for(i=1;i<=this->num_original_nodes;i++)
    {
        if(!(routed[i]))
        {
            x[j]=this->nodes[i].x;
            y[j]=this->nodes[i].y;
            plcol0(VRPH_RED);
            if(!(options & VRPH_NO_POINTS))
                plpoin(1,x+j,y+j,4);
            j++;
        }
    }

    //plcol0(VRPH_RED);
    //if(!(options & VRPH_NO_POINTS))
    //    plpoin(j,x,y,4);

    // Plot the VRPH_DEPOT
    plcol0(VRPH_RED);
    if(options & VRPH_WEIGHTED)
        plssym(0,2);
    else
        plssym(0,1);

    x[0]=nodes[0].x;
    y[0]=nodes[0].y;
    if(!(options & VRPH_NO_POINTS))
        plpoin(1,x,y,4);


    plend();

    delete [] x;
    delete [] y;
    return true;

#else

report_error("%s: PLPlot not installed\n",__FUNCTION__);
exit(-1);

#endif

}
bool VRP::plot(const char *filename)
{
    ///
    /// Assumes default options and plots the .ps file.
    ///

    if(!this->can_display)
    {
        fprintf(stderr,"Unable to display.  No coordinates known!\n");
        return false;
    }
#ifdef HAS_PLPLOT

    this->plot(filename,VRPH_DEFAULT_PLOT,1);
    return true;
#endif
    report_error("Must have PLPLOT installed to plot solutions!\n");
    return false;
}

bool VRP::plot_route(int r, const char *filename)
{

    ///
    /// Uses PLPLOT to plot the individual route number r in a colored .ps file.
    ///

    if(!this->can_display)
    {
        fprintf(stderr,"Unable to display.  No coordinates known!\n");
        return false;
    }

#ifdef HAS_PLPLOT

    PLFLT *x, *y;
    int current,  i, ctr, rnum;
    PLFLT xmin, xmax, ymin, ymax;

    rnum = 0;

    // Allocate storage
    x=new PLFLT [3*num_nodes+1];
    y=new PLFLT [3*num_nodes+1];
    // Max size would be default CW routes

    // Invert-seems necessary??
    plsori(1);

    // Set filename
    plsfnam(filename);

    // Set format-colored postscript only right now
    plsdev("psc");
    //For black and white
    //plsdev("ps");

    // Background color-set to white
    plscolbg (255,255,255);

    // Initialize
    plinit();

    // Set to blue for axes and numbers
    plcol0(VRPH_BLUE);

    
    // Now find the min and max coords so we can get similar plots
    // for different routes
    xmin=VRP_INFINITY;
    ymin=VRP_INFINITY;
    xmax=-VRP_INFINITY;
    ymax=-VRP_INFINITY;

    for(i=0;i<= num_nodes;i++)
    {
        if((PLFLT)nodes[i].x<xmin)
            xmin=(PLFLT)nodes[i].x;
        if((PLFLT)nodes[i].x>xmax)
            xmax=(PLFLT)nodes[i].x;

        if((PLFLT)nodes[i].y<ymin)
            ymin=(PLFLT)nodes[i].y;
        if((PLFLT)nodes[i].y>ymax)
            ymax=(PLFLT)nodes[i].y;
    }

    xmin=xmin-.05*VRPH_ABS(xmax-xmin);
    xmax=xmax+.05*VRPH_ABS(xmax-xmin);

    ymin=ymin-.05*VRPH_ABS(ymax-ymin);
    ymax=ymax+.05*VRPH_ABS(ymax-ymin);


    // Create environment
    // Use this for axes 
    plenv(xmin,xmax,ymin,ymax,2,0);

    // Use this for bare bones
    //plenv(xmin,xmax,ymin,ymax,2,-2);

    // Use this for bare bones with box
    //plenv(xmin,xmax,ymin,ymax,2,-1);

    char dest[VRPH_STRING_SIZE];
    sprintf(dest,"Route %d (%7.2f, %d, %d)", r, route[r].length,
        route[r].load, route[r].num_customers);

    // Add a label 
    pllab("", "", dest);


    // Set the line width
    plwid (0);

    // Draw the route in red
    plcol0(VRPH_RED);    

    ctr=0;
    x[ctr]= nodes[0].x;
    y[ctr]= nodes[0].y;

    ctr++;
    current= route[r].start;
    while(current!=VRPH_DEPOT)
    {
        x[ctr]= nodes[current].x;
        y[ctr]= nodes[current].y;
        ctr++;    
        current= VRPH_MAX(VRPH_DEPOT,next_array[current]);
    }

    // Close the route at the VRPH_DEPOT
    x[ctr]= nodes[VRPH_DEPOT].x;
    y[ctr]= nodes[VRPH_DEPOT].y;
    ctr++;

    // Draw the lines
    plline(ctr,x,y);

    // Plot the VRPH_DEPOT
    plcol0(VRPH_BLUE);
    plssym(1,5);
    x[0]=nodes[0].x;
    y[0]=nodes[0].y;
    plpoin(1,x,y,0);

    // Plot all the other points
    plssym(2,1);
    for(i=1;i<=this->num_nodes;i++)
    {
        x[i-1]=this->nodes[i].x;
        y[i-1]=this->nodes[i].y;

    }
    plpoin(this->num_nodes,x,y,4);

    plend();

    delete [] x;
    delete [] y;

    return true;

#else

report_error("%s: PLPlot not installed\n");
exit(-1);

#endif

}

