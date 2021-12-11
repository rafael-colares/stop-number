SYSTEM = x86-64_linux
LIBFORMAT = static_pic


# ---------------------------------------------------------------------
# Compiler options
# ---------------------------------------------------------------------
CCC = g++
CC  = gcc
CCOPT = -fPIC -fexceptions -DNDEBUG -DIL_STD -ldl
COPT  = -fPIC
# ---------------------------------------------------------------------
# Cplex and Concert paths
# ---------------------------------------------------------------------
CONCERTVERSION = concert
CPLEXVERSION = CPLEX_Studio128

CONCERTDIR = /opt/ibm128/ILOG/$(CPLEXVERSION)/$(CONCERTVERSION)
CONCERTINCDIR = $(CONCERTDIR)/include/
CONCERTLIBDIR = $(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)

CPLEXDIR = /opt/ibm128/ILOG/$(CPLEXVERSION)/cplex
CPLEXINCDIR = $(CPLEXDIR)/include/
CPLEXLIBDIR = $(CPLEXDIR)/lib/$(SYSTEM)/$(LIBFORMAT)

# ---------------------------------------------------------------------
# Flags
# ---------------------------------------------------------------------
CCLNFLAGS = -L$(CPLEXLIBDIR) -lilocplex -lcplex -L$(CONCERTLIBDIR) -lconcert -lm -lpthread
CLNFLAGS  = -L$(CPLEXLIBDIR) -lcplex -lm -lpthread
CFLAGS  = $(COPT)  -I$(CPLEXINCDIR)
CCFLAGS = $(CCOPT) -I$(CPLEXINCDIR) -I$(CONCERTINCDIR)

LEMONINCDIR = /home/prof/colares/lemon/include/
LEMONLIBDIR = /home/prof/colares/lemon/lib/
LEMONCFLAGS = -I$(LEMONINCDIR)
LEMONCLNFLAGS = -L$(LEMONLIBDIR) -lemon

# ---------------------------------------------------------------------
# Comands
# ---------------------------------------------------------------------
PRINTLN = echo

#---------------------------------------------------------
# Files
#---------------------------------------------------------
all: main

main:
	$(CCC) -c -Wall -g -std=c++11 $(CCFLAGS) $(LEMONCFLAGS) main.cpp instance.cpp formulation.cpp graph.cpp
	$(CCC) $(CCFLAGS) *.o -g -o main $(CCLNFLAGS) $(LEMONCLNFLAGS) 
	rm -rf *.o *~ ^

clean:
	rm -rf *.o main

