#ifndef __MILP_H__
#define __MILP_H__

#include<set>
#include<tuple>
#include<vector>
#include"basic.h"
#include"gurobi_c++.h"

using std::set;
using std::pair;

int SKINNY( int r, const NS & inx, const std::set<NS> &outx, std::set<
        std::pair<NS, NS> > & bad );

int SKINNY_Single( int r, const NS &,  const NS &outx );

#endif

