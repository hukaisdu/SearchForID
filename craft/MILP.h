#ifndef __MILP_H__
#define __MILP_H__

#include<set>
#include<tuple>
#include<vector>
#include"basic.h"
#include"gurobi_c++.h"

typedef int LimitStatus;
const int FIRSTLIMIT = 0x1;
const int SECONDLIMIT = 0x2;
const int BOTHLIMIT = 0x3;

using std::set;
using std::pair;

/*
void ModelSbox( GRBModel & model, 
        const GRBVar & x0, const GRBVar & x1, const GRBVar & x2, const GRBVar & x3,  
        const GRBVar & y0, const GRBVar & y1, const GRBVar & y2, const GRBVar & y3 );

void ModelMC( GRBModel & model, 
        const GRBVar & x0, const GRBVar & x1, const GRBVar & x2, const GRBVar & x3,  
        const GRBVar & y0, const GRBVar & y1, const GRBVar & y2, const GRBVar & y3 );

void SR( GRBVar * p );
*/

int SKINNY( int r, const NS & inx, const std::set<NS> &outx, std::set<
        std::pair<NS, NS> > & bad );

int SKINNY_Single( int r, const NS &,  const NS &outx, NS &, NS & );

int SKINNY_Truncated( int r, const NS & inx, const NS & outx );

int SKINNY_MultiSolution( int r, const NS & in, const NS &out, std::set< std::pair<NS,
        NS > >  & );
#endif

