# Set compiler and flags
CC=g++
CFLAGS= -O3 -Wall

# Set directory for static library and binaries 
# Defaults to ./lib and ./bin
VRPH_LIB_DIR = ./lib
VRPH_BIN_DIR = ./bin

# Set names of executables
RTR_EXE = $(VRPH_BIN_DIR)/vrp_rtr
EJ_EXE = $(VRPH_BIN_DIR)/vrp_ej
SP_EXE = $(VRPH_BIN_DIR)/vrp_sp
SA_EXE = $(VRPH_BIN_DIR)/vrp_sa
INIT_EXE = $(VRPH_BIN_DIR)/vrp_init
PLOT_EXE = $(VRPH_BIN_DIR)/vrp_plot

# Set name of libraries needed by applicaitons
LIBS= -lvrph -lm

# Set to 0 if you don't have/want Doxygen installed for
# documentation
HAS_DOXYGEN=1
ifeq ($(HAS_DOXYGEN),1)
DOX=doxygen
DOXYFILE=./Doxyfile
else
DOX=
DOXYFILE=
endif

# Set to 0 if you don't have PLPLOT, 1 if you do
# and modify the directories below
HAS_PLPLOT= 0
ifeq ($(HAS_PLPLOT),1)
PLPLOT_INC_DIR= -I$(HOME)/PLPLOT/plplot-5.9.4/x86_64build/include/plplot/
PLPLOT_LIB_DIR= -L$(HOME)/PLPLOT/plplot-5.9.4/x86_64build/lib/
PLPLOT_LIB= -lplplotd -lqsastime -lnistcd 
PLDEF=-DHAS_PLPLOT
else
PLPLOT_INC_DIR= 
PLPLOT_LIB_DIR=
PLPLOT_LIB=
PLDEF=
endif

# Set to 0 if you don't have OSI and GLPK, 1 if you do
# and correct the directories below
HAS_OSI_GLPK= 0
ifeq ($(HAS_OSI_GLPK),1)
GLPK_INC_DIR= -I$(HOME)/GLPK/include
GLPK_LIB_DIR= -L$(HOME)/GLPK/lib
GLPK_DEF=-DHAS_GLPK
OSI_DEF=-DHAS_OSI
OSI_INC_DIR= -I$(HOME)/OSI/x86_64_build/include/coin
OSI_LIB_DIR=-L$(HOME)/OSI/x86_64_build/lib
GLPK_LIBS= -lglpk
OSI_LIBS= -lOsiGlpk -lOsi -lCoinUtils
else
OSI_DEF=
GLPK_DEF=
GLPK_INC_DIR=
OSI_INC_DIR=
OSI_GLPK_INC_DIR=
endif 

# Various directories needed by the library and applications
INC_DIR= -I./inc/
LIB_DIR = -L$(VRPH_LIB_DIR)
VRPH_LIB = $(VRPH_LIB_DIR)/libvrph.a
TEST_OUTPUT = ./test_sols.out
SRCS= ./src/ClarkeWright.cpp ./src/Concatenate.cpp ./src/CrossExchange.cpp ./src/MoveString.cpp \
./src/OnePointMove.cpp ./src/OrOpt.cpp ./src/Postsert.cpp ./src/Presert.cpp ./src/Flip.cpp \
./src/RNG.cpp ./src/Swap.cpp ./src/SwapEnds.cpp ./src/Sweep.cpp ./src/ThreeOpt.cpp \
./src/ThreePointMove.cpp ./src/VRPTSPLib.cpp ./src/TwoOpt.cpp ./src/TwoPointMove.cpp ./src/VRP.cpp \
./src/VRPIO.cpp ./src/VRPDebug.cpp ./src/VRPMove.cpp  ./src/VRPNode.cpp ./src/VRPRoute.cpp \
./src/VRPSolution.cpp ./src/VRPSolvers.cpp ./src/VRPTabuList.cpp ./src/VRPUtils.cpp ./src/VRPGraphics.cpp

OBJS=$(SRCS:.cpp=.o)

RTR_SRC= ./src/apps/vrp_rtr.cpp
SP_SRC= ./src/apps/vrp_glpk_sp.cpp
SA_SRC= ./src/apps/vrp_sa.cpp
EJ_SRC= ./src/apps/vrp_ej.cpp
INIT_SRC= ./src/apps/vrp_initial.cpp
PLOT_SRC= ./src/apps/vrp_plotter.cpp

all: $(VRPH_LIB) vrp_rtr vrp_sa vrp_init vrp_ej vrp_sp vrp_plot

$(VRPH_LIB): $(OBJS)
	mkdir -p $(VRPH_LIB_DIR)
	$(AR) $(ARFLAGS) $@ $(OBJS)
	ranlib $@
	rm -rf $(OBJS)
	
.cpp.o:
	$(CC) $(CFLAGS) $(PLDEF) -c $(INC_DIR) $(PLPLOT_INC_DIR) $< -o $@

# An implementation of an RTR-based algorithm for generating solutions
vrp_rtr: $(OBJS) $(RTR_SRC)
	mkdir -p $(VRPH_BIN_DIR)
	$(CC) $(CFLAGS) $(PLDEF) $(PLPLOT_INC_DIR) $(RTR_SRC) $(INC_DIR) $(LIB_DIR) $(PLPLOT_LIB_DIR) $(LIBS) $(PLPLOT_LIB) -o $(RTR_EXE) 

# An implementation of a Simulated Annealing-based algorithm for generating solutions
vrp_sa: $(OBJS) $(SA_SRC)
	mkdir -p $(VRPH_BIN_DIR)
	$(CC) $(CFLAGS) $(PLDEF) $(PLPLOT_INC_DIR) $(SA_SRC) $(INC_DIR) $(LIB_DIR) $(PLPLOT_LIB_DIR) $(LIBS) $(PLPLOT_LIB) -o $(SA_EXE) 

# An implementation of a simple routine that demonstrates the Clarke Wright and Sweep algorithms
vrp_init: $(OBJS) $(INIT_SRC)
	mkdir -p $(VRPH_BIN_DIR)
	$(CC) $(CFLAGS) $(INIT_SRC) $(INC_DIR) $(LIB_DIR) $(LIBS) -o $(INIT_EXE) 

# An implementation of a tool to plot solutions using PLPLOT
vrp_plot: $(OBJS) $(PLOT_SRC)
ifeq ($(HAS_PLPLOT),1)
	mkdir -p $(VRPH_BIN_DIR)
	$(CC) $(CFLAGS) $(INC_DIR) $(PL_DEF) $(PLPLOT_INC_DIR) $(PLPLOT_LIB_DIR) $(LIB_DIR) $(PLOT_SRC) $(LIBS) $(PLPLOT_LIB) -o $(PLOT_EXE)
endif

# A utility to improve solutions by ejecting/injecting random neighborhoods
vrp_ej: $(OBJS) $(EJ_SRC)
	mkdir -p $(VRPH_BIN_DIR)
	$(CC) $(CFLAGS) $(INC_DIR) $(EJ_SRC) $(LIB_DIR) $(LIBS) -o $(EJ_EXE)

# An implementation combining RTR with GLPK and OSI
# Only builds if USE_OSI_GLPK=1 above in this makefile
vrp_sp: $(OBJS) $(SP_SRC)
ifeq ($(HAS_OSI_GLPK),1)
	mkdir -p $(VRPH_BIN_DIR)
	$(CC) $(CFLAGS) $(INC_DIR) $(OSI_INC_DIR) $(GLPK_INC_DIR) $(OSI_LIB_DIR) $(GLPK_LIB_DIR) $(LIB_DIR) $(SP_SRC) $(LIBS) $(GLPK_LIBS) $(OSI_LIBS) -o $(SP_EXE)
endif


# test - just run the binaries on the test_instance 
# output is sent to $(TEST_OUTPUT) file
test:
	-rm -rf $(TEST_OUTPUT).tmp
	-rm -rf $(TEST_OUTPUT)
	@echo "*************************************"
	@echo Testing vrp_rtr on test_instance.vrp
	./bin/vrp_rtr -f ./test_instance.vrp -v >> $(TEST_OUTPUT).tmp
	@echo "*************************************"
	@echo Testing vrp_sa on test_instance.vrp
	./bin/vrp_sa -f ./test_instance.vrp -v >> $(TEST_OUTPUT).tmp
	@echo "*************************************"
	@echo Testing vrp_init on test_instance.vrp
	./bin/vrp_init -f ./test_instance.vrp -m 0 >> $(TEST_OUTPUT).tmp
	@echo "*************************************"
	@echo Testing vrp_ej on test_instance.vrp
	./bin/vrp_ej -f ./test_instance.vrp -j 15 -t 500 -m 0 -v >> $(TEST_OUTPUT).tmp
ifeq ($(HAS_OSI_GLPK),1)
	@echo "*************************************"
	@echo Testing vrp_sp on test_instance.vrp
	./bin/vrp_sp -f ./test_instance.vrp -n 5 -v >> $(TEST_OUTPUT).tmp
endif	

ifeq ($(HAS_PLPLOT),1)
	@echo "*************************************"
	@echo Testing vrp_plot on test_instance.vrp
	./bin/vrp_rtr -f ./test_instance.vrp -out test_instance.sol > /dev/null
	./bin/vrp_plot -f ./test_instance.vrp -s test_instance.sol -p test_instance.ps >> $(TEST_OUTPUT).tmp
	@echo Postscript plot created in test_instance.ps
	-rm test_instance.sol
endif	
	mv $(TEST_OUTPUT).tmp $(TEST_OUTPUT)
	@echo "*************************************"
	@echo "*************************************"
	@echo All tests appeared to pass. File $(TEST_OUTPUT) contains results.

# Doxygen automatic documentation generation
doc: $(DOXYFILE)
	@echo Creating Doxygen documentation in ./doc directory
	mkdir -p ./doc
	$(DOX) $(DOXYFILE)
	@echo Run pdflatex or latex on ./doc/latex/refman.tex to create Doxygen manual

clean:
	-rm -rf $(OBJS)
	-rm -rf $(VRPH_LIB)
	-rm -rf $(RTR_EXE)
	-rm -rf $(EJ_EXE)
	-rm -rf $(SP_EXE)
	-rm -rf $(SA_EXE)
	-rm -rf $(PLOT_EXE)
	-rm -rf $(INIT_EXE)
	-rm -rf $(TEST_OUTPUT).tmp
	-rm -rf $(TEST_OUTPUT)
	-rm -rf test_instance.sol
	-rm -rf test_instance.ps

