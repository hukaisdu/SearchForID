OPT= -std=c++17 -O3 -m64 -lm -lpthread
GUROBI_HEAD= -I $(GUROBI_HOME)/include/ 
GUROBI_LIB= -L $(GUROBI_HOME)/lib -lgurobi_c++ -lgurobi95

all: main

main: main.o basic.o gift.o forword.o backword.o MILP.o automatic.o
	g++ -o $@ $^ $(OPT) $(GUROBI_HEAD) $(GUROBI_LIB)  

#debugger: debugger.o basic.o gift.o forword.o backword.o MILP.o automatic.o
#	g++ -o $@ $^ $(OPT) $(GUROBI_HEAD) $(GUROBI_LIB)  

automatic.o: automatic.cpp
	g++ -c automatic.cpp $(OPT) $(GUROBI_HEAD) -o automatic.o

MILP.o: MILP.cpp
	g++ -c MILP.cpp $(OPT) $(GUROBI_HEAD) -o MILP.o

#debugger.o: debugger.cpp 
#	g++ -c debugger.cpp $(OPT) $(GUROBI_HEAD) -o debugger.o

main.o: main.cpp 
	g++ -c main.cpp $(OPT) $(GUROBI_HEAD) -o main.o

basic.o: basic.cpp 
	g++ -c basic.cpp $(OPT) -o basic.o 

gift.o: gift.cpp 
	g++ -c gift.cpp $(OPT) -o gift.o

forword.o: forword.cpp 
	g++ -c forword.cpp $(OPT) -o forword.o

backword.o: backword.cpp 
	g++ -c backword.cpp $(OPT) -o backword.o

.PHONY: clean
clean:
	rm *.o main debugger verify


