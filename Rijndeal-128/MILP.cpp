#include"gurobi_c++.h"
#include"MILP.h"
#include"basic.h"
#include<set>
#include<tuple>
#include<fstream>
#include<cmath>
#include<random>

using namespace std;

void ModelMC( GRBModel & model, 
        const GRBVar & x0, const GRBVar & x1, const GRBVar & x2, const GRBVar & x3,  
        const GRBVar & y0, const GRBVar & y1, const GRBVar & y2, const GRBVar &
        y3 )  
{
    GRBVar a = model.addVar( 0, 1, 0, GRB_BINARY );

    model.addConstr( x0 + x1 + x2 + x3 + y0 + y1 + y2 + y3 >= 5*a );
    model.addConstr( a >= x0 );
    model.addConstr( a >= x1 );
    model.addConstr( a >= x2 );
    model.addConstr( a >= x3 );
    model.addConstr( a >= y0 );
    model.addConstr( a >= y1 );
    model.addConstr( a >= y2 );
    model.addConstr( a >= y3 );
}

void SR( GRBVar * p )
{
    GRBVar tmp[16];
    for ( int i = 0; i < 16; i++ )
        tmp[i] = p[i];

    p[0] = tmp[0];
    p[1] = tmp[1];
    p[2] = tmp[2];
    p[3] = tmp[3];

    p[4]  = tmp[5]; 
    p[5]  = tmp[6]; 
    p[6]  = tmp[7]; 
    p[7]  = tmp[4]; 

    p[8]  = tmp[10]; 
    p[9]  = tmp[11]; 
    p[10]  = tmp[8]; 
    p[11]  = tmp[9]; 

    p[12]  = tmp[15]; 
    p[13]  = tmp[12]; 
    p[14]  = tmp[13]; 
    p[15]  = tmp[14]; 
}

// we do not construct every model for each (a, b), we rtemove some constraints to reuse the model
int SKINNY( int r, const NS & inx, const set<NS> &outx,
        set< pair<NS, NS> > & bad )
{
    // generate model
    try 
    { 
        GRBEnv env = GRBEnv( true );
        env.set(GRB_IntParam_OutputFlag,0);

        env.set(GRB_IntParam_LogToConsole, 0);
        env.set(GRB_IntParam_Threads, 1 );

        env.start();

        GRBModel model = GRBModel(env);

        // generate variables
        vector<GRBVar*> X;

        for ( int i = 0; i < r + 2; i++ )
        {
            GRBVar * p = new GRBVar[16];
            for ( int j = 0; j < 16; j++ )
                 p[j] = model.addVar( 0, 1, 0, GRB_BINARY );
            X.push_back( p );
        }

        for ( int i = 0; i < 16; i++ )
            if ( inx[i] == 0 )
                model.addConstr( X[0][i] == 0 );
            else
                model.addConstr( X[0][i] == 1 );

        SR( X[0] );
        ModelMC( model, X[0][0], X[0][4], X[0][8],  X[0][12], 
                        X[1][0], X[1][4], X[1][8],  X[1][12] );
        ModelMC( model, X[0][1], X[0][5], X[0][9],  X[0][13], 
                        X[1][1], X[1][5], X[1][9],  X[1][13] );
        ModelMC( model, X[0][2], X[0][6], X[0][10], X[0][14], 
                        X[1][2], X[1][6], X[1][10], X[1][14] );
        ModelMC( model, X[0][3], X[0][7], X[0][11], X[0][15], 
                        X[1][3], X[1][7], X[1][11], X[1][15] );

        // round functions
        for ( int i = 0; i < r; i++ )
        {
            SR( X[i + 1] );

            // MC
            // column 0
            ModelMC( model, X[i + 1][0], X[i + 1][4], X[i + 1][8], X[i+1][12],
                            X[i+2][0], X[i+2][4], X[i+2][8], X[i+2][12] );

            ModelMC( model, X[i + 1][1], X[i + 1][5], X[i + 1][9], X[i+1][13], 
                            X[i + 2][1], X[i + 2][5], X[i + 2][9], X[i+2][13] );

            ModelMC( model, X[i + 1][2], X[i + 1][6], X[i + 1][10], X[i+1][14], 
                            X[i + 2][2], X[i + 2][6], X[i + 2][10], X[i+2][14] );

            ModelMC( model, X[i + 1][3], X[i + 1][7], X[i + 1][11], X[i+1][15], 
                            X[i + 2][3], X[i + 2][7], X[i + 2][11], X[i+2][15] );
        }

        SR( X[r+1] );

        vector<GRBConstr> R;
        for ( auto & it : outx )
        {
            R.clear();
            // set variable values
            for ( int i = 0; i < 16; i++ )
                if ( it[i] == 0 )
                    R.push_back( model.addConstr( X[r+1][i] == 0 ) );
                else
                    R.push_back( model.addConstr( X[r+1][i] == 1 ) );

            model.update();

            model.optimize();

            if ( model.get( GRB_IntAttr_Status ) == GRB_OPTIMAL )
                ;
            else if ( model.get( GRB_IntAttr_Status ) == GRB_INFEASIBLE )
                bad.insert( make_pair( inx, it ) );
            else
                return -1;

            for ( auto it : R )
                model.remove( it );
            model.update();
        }

        for ( auto it : X )
            delete [] it;
    }
    catch (GRBException e)
    {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    }
    catch (...) 
    {
        cout << "Exception during optimization" << endl;
    }

    if ( bad.size() > 0 )
        return 0;
    else
        return 1;
}

int SKINNY_Single( int r, const NS & inx, const NS & outx )
{
    // generate model
    try 
    { 
        GRBEnv env = GRBEnv( true );
        env.set(GRB_IntParam_OutputFlag, 0);

        env.set(GRB_IntParam_LogToConsole, 0);
        env.set(GRB_IntParam_Threads, 1 );

        env.start();

        //random_device rd;

        //mt19937_64 generator ( rd() );
    
        //uniform_int_distribution<int> distribution(0, 2000000000 - 1 );

        //env.set(GRB_IntParam_Seed, distribution( generator) );
        GRBModel model = GRBModel(env);

        // generate variables
        vector<GRBVar*> X;

        for ( int i = 0; i < r; i++ )
        {
            GRBVar * p = new GRBVar[16];
            for ( int j = 0; j < 16; j++ )
                 p[j] = model.addVar( 0, 1, 0, GRB_BINARY );
            X.push_back( p );
        }

        for ( int i = 0; i < 16; i++ )
            if ( inx[i] == 0 )
                model.addConstr( X[0][i] == 0 );
            else
                model.addConstr( X[0][i] == 1 );


        for ( int i = 0; i < r - 2; i++ )
        {
            // MC
            // column 0
            ModelMC( model, X[i][0], X[i][4], X[i][8], X[i][12], X[i+1][0], X[i+1][4], X[i+1][8], X[i+1][12] );
            ModelMC( model, X[i][1], X[i][5], X[i][9], X[i][13], X[i+1][1], X[i+1][5], X[i+1][9], X[i+1][13] );
            ModelMC( model, X[i][2], X[i][6], X[i][10], X[i][14], X[i+1][2], X[i+1][6], X[i+1][10], X[i+1][14] );
            ModelMC( model, X[i][3], X[i][7], X[i][11], X[i][15], X[i+1][3], X[i+1][7], X[i+1][11], X[i+1][15] );

            SR( X[i +1 ] );
        }

        int i = r - 2;

        ModelMC( model, X[i][0], X[i][4], X[i][8], X[i][12], X[i+1][0], X[i+1][4], X[i+1][8], X[i+1][12] );
        ModelMC( model, X[i][1], X[i][5], X[i][9], X[i][13], X[i+1][1], X[i+1][5], X[i+1][9], X[i+1][13] );
        ModelMC( model, X[i][2], X[i][6], X[i][10], X[i][14], X[i+1][2], X[i+1][6], X[i+1][10], X[i+1][14] );
        ModelMC( model, X[i][3], X[i][7], X[i][11], X[i][15], X[i+1][3], X[i+1][7], X[i+1][11], X[i+1][15] );

        // set variable values
        for ( int i = 0; i < 16; i++ )
            if ( outx[i] == 0 )
                model.addConstr( X[r -1 ][i] == 0 );
            else
                model.addConstr( X[r -1 ][i] == 1 );

        model.optimize();

        if ( model.get( GRB_IntAttr_Status ) == GRB_OPTIMAL )
            return 1;
        else if ( model.get( GRB_IntAttr_Status ) == GRB_INFEASIBLE )
            return 0;
        else
            return -1;

        for ( auto it : X )
            delete [] it;
    
    }
    catch (GRBException e)
    {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    }
    catch (...) 
    {
        cout << "Exception during optimization" << endl;
    }

    return -1;
}

