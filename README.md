# VRPH

VRPH is an open-source library of heuristics for generating solutions to Vehicle Routing Problems (VRPs).  The VRPH software includes the following:
  * Many well-known heuristics for generating and improving feasible solutions to VRP instances (Clarke-Wright, two-opt, three-opt, etc.).
  * Two metaheuristics that can usually generate good solutions to large instances containing hundreds of customer locations in a matter of seconds.  On well-studied benchmark problems, the solutions are typically within a percent or two of the best-known solution.
  * An algorithm that combines a metaheuristic with a set partitioning problem for even better solutions.  This implementation currently uses the COIN Open Solver Interface (OSI) and the GLPK solver.
  * A modular design that can be extended to handle additional constraints.

VRPH was developed by Chris Groer (cgroerATgmailDOTcom), currently at Oak Ridge National Lab

## News

 * April 2009:  VRPH-1.0.0 is released.

 * Click [here](https://github.com/coin-or/VRPH/commits/master) to see the current change log.

## Supported Platforms

 * GNU/Linux (g++)
 * Microsoft Windows
    * CYGWIN (w/ g++ compiler)
    * Visual C++
 * Mac OSX (g++)

## Download

The current stable version of VRPH is [1.0](https://github.com/coin-or/VRPH/tree/stable/1.0) and the current release is [1.0.0](https://github.com/coin-or/VRPH/releases/tag/releases%2F1.0.0). 

**Source code** can be obtained either by

 * Downloading a snapshot of the source code for VRPH 1.0.0 from the [VRPH source code download page](http://www.coin-or.org/download/source/VRPH), or
 * Checking out the latest stable source using a Git client.

The recommended method is to use Git because it makes it easier to obtain updates. 

**Quick Start Guide for Unix-like Environments**

In a Unix-like environment (such as Linux or CYGWIN), the following commands may be used to obtain and compile the current release of VRPH, and then run a quick test on a small benchmark problem instance:
```
git clone -b releases/1.0.0 https://github.com/coin-or/VRPH.git
cd VRPH
make
make test
```

## Getting More Help

 * To report a bug, please submit a [trouble issue](https://github.com/coin-or/VRPH/issues/new).

 * There is a VRPH user's mailing list for discussion. Please visit the [mailing list home](http://list.coin-or.org/mailman/listinfo/coin-vrph) to join or view the archives.

## Licensing and Authors

VRPH is written in C++ and is distributed as open source code under the [Common Public License (CPL)](http://www.opensource.org/licenses/cpl.php).
It is available from the [COIN-OR initiative](http://www.coin-or.org/).  The author of the code is

 * Chris Groer (cgroerATgmailDOTcom)
 
VRPH was mostly developed while at the University of Maryland with advisor [Bruce Golden](http://www.rhsmith.umd.edu/faculty/bgolden/).
