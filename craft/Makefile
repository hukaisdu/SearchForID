OPT= -std=c++17 -O3 -m64 -lm -lpthread
GUROBI_HEAD= -I $(GUROBI_HOME)/include/ 
GUROBI_LIB= -L $(GUROBI_HOME)/lib -lgurobi_c++ -lgurobi95

all: main_automatic

main_automatic: main_automatic.o basic.o craft.o forword.o backword.o MILP.o automatic.o
	g++ -o $@ $^ $(OPT) $(GUROBI_HEAD) $(GUROBI_LIB) 

main_automatic.o: main_automatic.cpp 
	g++ -c main_automatic.cpp $(OPT) $(GUROBI_HEAD) -o main_automatic.o

automatic.o: automatic.cpp forword.h backword.h basic.h log.h MILP.h thread_pool.h 
	g++ -c automatic.cpp $(OPT) $(GUROBI_HEAD) -o automatic.o

basic.o: basic.cpp basic.h
	g++ -c basic.cpp $(OPT) -o basic.o 

craft.o: craft.cpp craft.h basic.h
	g++ -c craft.cpp $(OPT) -o craft.o

forword.o: forword.cpp basic.h
	g++ -c forword.cpp $(OPT) -o forword.o

backword.o: backword.cpp basic.h forword.h backword.h craft.h
	g++ -c backword.cpp $(OPT) -o backword.o

MILP.o: MILP.cpp MILP.h basic.h 
	g++ -c MILP.cpp $(OPT) $(GUROBI_HEAD) -o MILP.o

.PHONY: clean
clean:
	rm -f *.o main_automatic


