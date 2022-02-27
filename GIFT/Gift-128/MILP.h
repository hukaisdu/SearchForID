#ifndef __MILP_H__
#define __MILP_H__

#include<set>
#include<tuple>
#include<vector>
#include"basic.h"
#include"gurobi_c++.h"

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

void GIFT_Inner_Round( int r, const NS & inx, const set<NS> & outx, set<
        pair<NS, NS> > & bad );

int GIFT_Single( int r, const NS & inx, const NS & outx, NS & start, NS & end );

int GIFT_Truncated( int r, const NS & inx, const NS & outx );

int GIFT_PlaintextTruncated( int r, const NS & inx, const NS & outx );
int GIFT_CiphertextTruncated( int r, const NS & inx, const NS & outx );

int GIFT_MultiSolution( int r, const NS & inx, const NS & outx, set< pair<NS,
        NS> > & PIN );

#endif

