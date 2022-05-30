# Codes for the paper "Finding All Impossible Differentials When Considering the DDT"

All codes for our experiments are provided in this repository, including SKINNY, craft, GIFT and Rijndeal. 
The impossible differentials are also provided in Results folder.

In each folder, you can find a `Makefile`, so, in principle, you can type `make` to compile the codes and run for the results. 

Before you use `make`, however, you may need to install the Gurobi solver, you can find the installer from their [official website](https://www.gurobi.com). 
Their provide [concrete instructions](https://www.gurobi.com/documentation/quickstart.html) for installing Gurobi in your own platform. 
To configure the `makefile`, you need replace the `GUROBI_HEAD` and `GUROBI_LIB` with your platform's own path.  

