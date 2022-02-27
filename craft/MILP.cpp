#include"gurobi_c++.h"
#include"MILP.h"
#include"basic.h"
#include<set>
#include<tuple>
#include<fstream>
#include<cmath>
#include<random>

using namespace std;

int Perm[16] = { 15, 12, 13, 14, 10, 9, 8, 11, 6, 5, 4, 7, 1, 2, 3, 0 };

int SboxIneq[][9] = 
{{0, 0, -1, -1, -1, 2, 2, 1, 2}, {0, 2, 2, 1, 2, 0, -1, -1, -1}, {1, 3, 1, -1, 1, -1, 1, -2, 1}, {2, -1, -1, 0, -1, 0, 0, 1, 0}, {1, -1, 1, -2, 1, 3, 1, -1, 1}, {1, 2, 1, 2, 1, -2, -1, 1, -1}, {6, -2, -2, -1, 3, -1, 3, -2, -1}, {3, 0, -2, -1, -2, 0, 1, 2, 1}, {2, 0, 0, 1, 0, -1, -1, 0, -1}, {6, -1, 3, -2, -1, -2, -2, -1, 3}, {2, 2, 1, -1, 3, 0, -1, -3, 2}, {4, -1, -2, -2, 1, 3, -2, 2, 1}, {3, 2, 1, 1, -1, 0, 1, -2, -2}, {1, -2, -1, 1, -1, 2, 1, 2, 1}, {2, 0, 1, -1, -1, 1, 0, 1, -1}, {3, 0, 1, 1, 1, 1, -2, -1, -2}, {6, -1, -1, -2, 3, -2, 3, -1, -2}, {0, 1, 2, 1, 2, -2, 1, 0, 1}, {5, -2, -1, -1, -1, 1, -1, 1, -1}, {3, 2, 1, -1, 1, 1, -2, 1, -2}, {6, -2, 3, -1, -2, -1, -1, -2, 3}, {4, 3, -2, 2, 1, -1, -2, -2, 1}, {5, 1, -1, 1, -1, -2, -1, -1, -1}, {0, -2, 1, 0, 1, 1, 2, 1, 2}, {2, -2, -1, 1, -1, 1, 0, 2, 1}, {0, 3, 3, 1, 2, -2, 2, -2, 1}, {5, 0, -1, -1, -1, 1, -1, -1, -1}, {5, -1, 2, -1, -2, -1, 0, -2, 2}, {2, 0, -1, -1, -1, 0, 0, 1, 1}, {1, 0, -1, 0, -1, 1, 1, 0, 1}, {0, -2, 2, -2, 1, 3, 3, 1, 2}, {3, -1, -1, 1, 1, 0, 0, -1, -1}, {0, 3, 2, 3, 2, -2, -1, 1, -1}, {3, 0, -1, 1, -1, 0, -1, 1, -1}, {2, 1, 1, -1, 0, 1, -1, 1, -1}};

int MCIneq[][9] = 
{{2, 0, -1, 0, 0, 0, -1, 0, -1}, {0, 0, -1, 0, 0, 0, 1, 0, 1}, {0, 0, 1, 0, 0, 0, 1, 0, -1}, {0, 0, 1, 0, 0, 0, -1, 0, 1}, {2, -1, 0, 0, 0, 1, 0, -1, -1}, {2, 1, 0, 0, 0, -1, 0, -1, -1}, {2, -1, 0, 0, 0, -1, 0, 1, -1}, {0, 1, 0, 0, 0, -1, 0, 1, 1}, {0, 1, 0, 0, 0, 1, 0, -1, 1}, {0, 1, 0, 0, 0, 1, 0, 1, -1}, {0, -1, 0, 0, 0, 1, 0, 1, 1}, {2, -1, 0, 0, 0, -1, 0, -1, 1}};

int MCEq[][9] = {{0, 0, 0, 0, 1, 0, 0, 0, -1}, {0, 0, 0, 1, 0, 0, 0, -1, 0}};

void ModelSbox( GRBModel & model, 
        const GRBVar & x0, const GRBVar & x1, const GRBVar & x2, const GRBVar & x3,  
        const GRBVar & y0, const GRBVar & y1, const GRBVar & y2, const GRBVar &
        y3 )  
{
    for ( auto it : SboxIneq )
    {
        GRBLinExpr s = it[0] + it[1] * x0 + it[2] * x1 + it[3] * x2 + it[4] * x3 + 
                             + it[5] * y0 + it[6] * y1 + it[7] * y2 + it[8] * y3; 
        model.addConstr( s >= 0 );
    }
}


void ModelMC( GRBModel & model, 
        const GRBVar & x0, const GRBVar & x1, const GRBVar & x2, const GRBVar & x3,  
        const GRBVar & y0, const GRBVar & y1, const GRBVar & y2, const GRBVar &
        y3 )  
{
    for ( auto it : MCIneq )
    {
        GRBLinExpr s = it[0] + it[1] * x0 + it[2] * x1 + it[3] * x2 + it[4] * x3 + 
                             + it[5] * y0 + it[6] * y1 + it[7] * y2 + it[8] * y3; 
        model.addConstr( s >= 0);
    }
    
    for ( auto it : MCEq )
    {
        GRBLinExpr s = it[0] + it[1] * x0 + it[2] * x1 + it[3] * x2 + it[4] * x3 + 
                             + it[5] * y0 + it[6] * y1 + it[7] * y2 + it[8] * y3; 
        model.addConstr( s == 0);
    }
}

void SR( GRBVar * p )
{
    GRBVar tmp[64];
    for ( int i = 0; i < 64; i++ )
        tmp[i] = p[i];

    for ( int i = 0; i < 16; i++ )
        for ( int j = 0; j < 4; j++ )
            p[4 * i + j] = tmp[ Perm[i] * 4 + j ];
}

// we do not construct every model for each (a, b), we rtemove some constraints to reuse the model
int CRAFT( int r, const NS & in, const set<NS> &out,
        set< pair<NS, NS> > & bad )
{
    // generate model
    try 
    { 
        //GRBEnv env = GRBEnv( true );
        GRBEnv env = GRBEnv( false );
        //env.set( GRB_IntParam_OutputFlag, 0 );

        env.set(GRB_IntParam_LogToConsole, 0);
        env.set(GRB_IntParam_Threads, 1 );

        env.start();

        GRBModel model = GRBModel(env);


        // generate variables
        vector<GRBVar*> X;
        for ( int i = 0; i < r + 1; i++ )
        {
            GRBVar * p = new GRBVar[64];
            for ( int j = 0; j < 64; j++ )
                p[j] = model.addVar( 0, 1, 0, GRB_BINARY );
            X.push_back( p );
        }

        vector<GRBVar*> Y;
        for ( int i = 0; i < r + 1; i++ )
        {
            GRBVar * p = new GRBVar[64];
            for ( int j = 0; j < 64; j++ )
                p[j] = model.addVar( 0, 1, 0, GRB_BINARY );
            Y.push_back( p );
        }

        vector<GRBVar> XT;
        for ( int i = 0; i < 64; i++ )
            XT.push_back( X[0][i] );

        SR( X[0] );

        // round functions
        for ( int i = 0; i < r; i++ )
        {
            // MC
            //
            for ( int k = 0; k < 4; k++ )
                for ( int j = 0; j < 4; j++ )
                    ModelMC( model, X[i][4 * k + 0 + j],   X[i][4 * k + 16 + j], X[i][4 * k + 32 + j], X[i][4 * k + 48 + j],
                                    Y[i][4 * k + 0 + j],   Y[i][4 * k + 16 +j], Y[i][4 * k + 32 + j], Y[i][4 * k +
                                    48 + j] );   

            // SR
            SR( Y[i] );

            for ( int j = 0; j < 16; j++ )
                ModelSbox( model, Y[i][4 * j], Y[i][4 * j + 1], Y[i][4 * j + 2], Y[i][4 * j + 3], 
                                  X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i + 1][4 * j + 2], X[i+1][4 *j + 3] );
        }

        int i = r;

        for ( int k = 0; k < 4; k++ )
            for ( int j = 0; j < 4; j++ )
                ModelMC( model, X[i][4 * k + 0 + j],   X[i][4 * k + 16 + j], X[i][4 * k + 32 + j], X[i][4 * k + 48 + j],
                                Y[i][4 * k + 0 + j],   Y[i][4 * k + 16 + j], Y[i][4 * k + 32 + j], Y[i][4 * k +
                                48 + j] );   


        SR( Y[i] );

        BS inx;
        Nibble_to_Bit( in, inx );


        for ( int i = 0; i < 64; i++ )
            if ( inx[i] == 0 )
                model.addConstr( XT[i] == 0 );
            else
                model.addConstr( XT[i] == 1 );

        vector<GRBConstr> R;
        for ( auto & it : out )
        {
            //printNibbleState( it );
            //cout << endl;
            BS jt;
            Nibble_to_Bit( it, jt );

            R.clear();
            // set variable values
            for ( int i = 0; i < 64; i++ )
                if ( jt[i] == 0 )
                    R.push_back( model.addConstr( Y[r][i] == 0 ) );
                else
                    R.push_back( model.addConstr( Y[r][i] == 1 ) );

            model.update();

            model.optimize();

            if ( model.get( GRB_IntAttr_Status ) == GRB_OPTIMAL )
                ;
            else if ( model.get( GRB_IntAttr_Status ) == GRB_INFEASIBLE )
                bad.insert( make_pair( in, it ) );
            else
                return -1;

            for ( auto it : R )
                model.remove( it );
            model.update();
        }

        for ( auto it : X )
            delete [] it;
        
        for ( auto it : Y )
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

int CRAFT_MultiSolution( int r, const NS & in, const NS & out, set< pair<NS,
        NS > > & S )
{
    BS inx, outx;
    Nibble_to_Bit ( in, inx );
    Nibble_to_Bit ( out, outx );
    // generate model
    try 
    { 
        GRBEnv env = GRBEnv( true );

        env.set( GRB_IntParam_OutputFlag, 0 );

        env.set(GRB_IntParam_LogToConsole, 0);
        env.set(GRB_IntParam_Threads, 1 );
        // solution pool
        //
        env.set(GRB_IntParam_PoolSearchMode, 2); 
        env.set(GRB_IntParam_PoolSolutions, 100); 

        env.start();
        GRBModel model = GRBModel(env);

        // generate variables
        vector<GRBVar*> X;

        for ( int i = 0; i < r; i++ )
        {
            GRBVar * p = new GRBVar[64];
            for ( int j = 0; j < 64; j++ )
                p[j] = model.addVar( 0, 1, 0, GRB_BINARY );
            X.push_back( p );
        }

        vector<GRBVar*> Y;
        for ( int i = 0; i < r; i++ )
        {
            GRBVar * p = new GRBVar[64];
            for ( int j = 0; j < 64; j++ )
                p[j] = model.addVar( 0, 1, 0, GRB_BINARY );
            Y.push_back( p );
        }

        vector<GRBVar> start, end; 

        int i = 0; // r = 0

        for ( int j = 0; j < 16; j++ )
            ModelSbox( model, 
            X[i][4 * j], X[i][4 * j + 1], X[i][4 * j + 2], X[i][4 * j + 3], 
            Y[i][4 * j], Y[i][4 * j + 1], Y[i][4 * j + 2], Y[i][4 * j + 3] );

        for ( int k = 0; k < 4; k++ )
            for ( int j = 0; j < 4; j++ )
                ModelMC( model, Y[i][4 * k + 0 + j],   Y[i][4 * k + 16 + j], Y[i][4 * k + 32 + j], Y[i][4 * k + 48 + j],
                                X[i+1][4 * k + 0 + j], X[i+1][4 * k + 16 +
                                j], X[i+1][4 * k + 32 + j], X[i+1][4 * k +
                                48 + j] );   

        i = 1;

        for ( int j = 0; j < 16; j++ )
            ModelSbox( model, 
            X[i][4 * j], X[i][4 * j + 1], X[i][4 * j + 2], X[i][4 * j + 3], 
            Y[i][4 * j], Y[i][4 * j + 1], Y[i][4 * j + 2], Y[i][4 * j + 3] );

        for ( int j = 0; j < 64; j++ )
            start.push_back( Y[i][j] );

        SR( Y[i] );

        for (  i = 1; i < r - 3; i++ ) 
        {
            for ( int k = 0; k < 4; k++ )
                for ( int j = 0; j < 4; j++ )
                    ModelMC( model, Y[i][4 * k + 0 + j],   Y[i][4 * k + 16 + j], Y[i][4 * k + 32 + j], Y[i][4 * k + 48 + j],
                                    X[i+1][4 * k + 0 + j], X[i+1][4 * k + 16 +
                                    j], X[i+1][4 * k + 32 + j], X[i+1][4 * k +
                                    48 + j] );   

            SR( X[i+1] );

            for ( int j = 0; j < 16; j++ )
                ModelSbox( model, 
                X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i + 1][4 * j + 2], X[i + 1][4 * j + 3], 
                Y[i + 1][4 * j], Y[i + 1][4 * j + 1], Y[i + 1][4 * j + 2], Y[i + 1][4 * j + 3] );

        }

        i = r - 3;

        for ( int k = 0; k < 4; k++ )
            for ( int j = 0; j < 4; j++ )
                ModelMC( model, Y[i][4 * k + 0 + j],   Y[i][4 * k + 16 + j], Y[i][4 * k + 32 + j], Y[i][4 * k + 48 + j],
                                X[i+1][4 * k + 0 + j], X[i+1][4 * k + 16 +
                                j], X[i+1][4 * k + 32 + j], X[i+1][4 * k +
                                48 + j] );   

        SR( X[i+1] );

        for ( int j = 0; j < 64; j++ )
            end.push_back( X[i+1][j] );

        for ( int j = 0; j < 16; j++ )
            ModelSbox( model, 
            X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i + 1][4 * j + 2], X[i + 1][4 * j + 3], 
            Y[i + 1][4 * j], Y[i + 1][4 * j + 1], Y[i + 1][4 * j + 2], Y[i + 1][4 * j + 3] );

        i = r - 2;

        for ( int k = 0; k < 4; k++ )
            for ( int j = 0; j < 4; j++ )
                ModelMC( model, Y[i][4 * k + 0 + j],   Y[i][4 * k + 16 + j], Y[i][4 * k + 32 + j], Y[i][4 * k + 48 + j],
                                X[i+1][4 * k + 0 + j], X[i+1][4 * k + 16 +
                                j], X[i+1][4 * k + 32 + j], X[i+1][4 * k +
                                48 + j] );   

        for ( int j = 0; j < 16; j++ )
            ModelSbox( model, 
            X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i + 1][4 * j + 2], X[i + 1][4 * j + 3], 
            Y[i + 1][4 * j], Y[i + 1][4 * j + 1], Y[i + 1][4 * j + 2], Y[i + 1][4 * j + 3] );


        for ( int i = 0; i < 64; i++ )
            if ( inx[i] == 0 )
                model.addConstr( X[0][i] == 0 );
            else
                model.addConstr( X[0][i] == 1 );

        // set variable values
        for ( int i = 0; i < 64; i++ )
            if ( outx[i] == 0 )
                model.addConstr( Y[r - 1][i] == 0 );
            else
                model.addConstr( Y[r - 1][i] == 1 );

        model.optimize();

        if ( model.get( GRB_IntAttr_Status ) == GRB_OPTIMAL )
        {
            int solCount = model.get(GRB_IntAttr_SolCount);
            cout << "solCount " << solCount << endl;
            for ( int i = 0; i < solCount; i++ )
            {
                BS x, y;
                model.set(GRB_IntParam_SolutionNumber, i );

                for ( int i = 0; i < 64; i++ )
                {
                    x[i] = static_cast<Nibble> ( round ( start[i].get(
                                    GRB_DoubleAttr_Xn ) ) );
                    y[i] = static_cast<Nibble> ( round (    end[i].get(
                                    GRB_DoubleAttr_Xn ) ) );
                }

                NS xx, yy;
                Bit_to_Nibble( x, xx );
                Bit_to_Nibble( y, yy );

                S.insert ( make_pair( xx, yy ) );
            }

            return 1;
        }
        else if ( model.get( GRB_IntAttr_Status ) == GRB_INFEASIBLE )
            return 0;
        else
            return -1;

        for ( auto it : X )
            delete [] it;
        
        for ( auto it : Y )
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

int CRAFT_Single( int r, const NS & in, const NS & out, NS & START, NS & END )
{
    BS inx, outx;
    Nibble_to_Bit( in, inx );
    Nibble_to_Bit( out, outx );

    // generate model
    try 
    { 
        GRBEnv env = GRBEnv( true );

        env.set(GRB_IntParam_OutputFlag, 0 );

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
            GRBVar * p = new GRBVar[64];
            for ( int j = 0; j < 64; j++ )
                p[j] = model.addVar( 0, 1, 0, GRB_BINARY );
            X.push_back( p );
        }

        vector<GRBVar*> Y;
        for ( int i = 0; i < r; i++ )
        {
            GRBVar * p = new GRBVar[64];
            for ( int j = 0; j < 64; j++ )
                p[j] = model.addVar( 0, 1, 0, GRB_BINARY );
            Y.push_back( p );
        }

        vector<GRBVar> start, end; 

        int i = 0; // r = 0

        for ( int j = 0; j < 16; j++ )
            ModelSbox( model, 
            X[i][4 * j], X[i][4 * j + 1], X[i][4 * j + 2], X[i][4 * j + 3], 
            Y[i][4 * j], Y[i][4 * j + 1], Y[i][4 * j + 2], Y[i][4 * j + 3] );

        for ( int k = 0; k < 4; k++ )
            for ( int j = 0; j < 4; j++ )
                ModelMC( model, Y[i][4 * k + 0 + j],   Y[i][4 * k + 16 + j], Y[i][4 * k + 32 + j], Y[i][4 * k + 48 + j],
                                X[i+1][4 * k + 0 + j], X[i+1][4 * k + 16 +
                                j], X[i+1][4 * k + 32 + j], X[i+1][4 * k +
                                48 + j] );   

        i = 1;

        for ( int j = 0; j < 16; j++ )
            ModelSbox( model, 
            X[i][4 * j], X[i][4 * j + 1], X[i][4 * j + 2], X[i][4 * j + 3], 
            Y[i][4 * j], Y[i][4 * j + 1], Y[i][4 * j + 2], Y[i][4 * j + 3] );

        for ( int j = 0; j < 64; j++ )
            start.push_back( Y[i][j] );

        SR( Y[i] );

        for (  i = 1; i < r - 3; i++ ) 
        {
            for ( int k = 0; k < 4; k++ )
                for ( int j = 0; j < 4; j++ )
                    ModelMC( model, Y[i][4 * k + 0 + j],   Y[i][4 * k + 16 + j], Y[i][4 * k + 32 + j], Y[i][4 * k + 48 + j],
                                    X[i+1][4 * k + 0 + j], X[i+1][4 * k + 16 +
                                    j], X[i+1][4 * k + 32 + j], X[i+1][4 * k +
                                    48 + j] );   

            SR( X[i+1] );

            for ( int j = 0; j < 16; j++ )
                ModelSbox( model, 
                X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i + 1][4 * j + 2], X[i + 1][4 * j + 3], 
                Y[i + 1][4 * j], Y[i + 1][4 * j + 1], Y[i + 1][4 * j + 2], Y[i + 1][4 * j + 3] );

        }

        i = r - 3;

        for ( int k = 0; k < 4; k++ )
            for ( int j = 0; j < 4; j++ )
                ModelMC( model, Y[i][4 * k + 0 + j],   Y[i][4 * k + 16 + j], Y[i][4 * k + 32 + j], Y[i][4 * k + 48 + j],
                                X[i+1][4 * k + 0 + j], X[i+1][4 * k + 16 +
                                j], X[i+1][4 * k + 32 + j], X[i+1][4 * k +
                                48 + j] );   

        SR( X[i+1] );

        for ( int j = 0; j < 64; j++ )
            end.push_back( X[i+1][j] );

        for ( int j = 0; j < 16; j++ )
            ModelSbox( model, 
            X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i + 1][4 * j + 2], X[i + 1][4 * j + 3], 
            Y[i + 1][4 * j], Y[i + 1][4 * j + 1], Y[i + 1][4 * j + 2], Y[i + 1][4 * j + 3] );

        i = r - 2;

        for ( int k = 0; k < 4; k++ )
            for ( int j = 0; j < 4; j++ )
                ModelMC( model, Y[i][4 * k + 0 + j],   Y[i][4 * k + 16 + j], Y[i][4 * k + 32 + j], Y[i][4 * k + 48 + j],
                                X[i+1][4 * k + 0 + j], X[i+1][4 * k + 16 +
                                j], X[i+1][4 * k + 32 + j], X[i+1][4 * k +
                                48 + j] );   

        for ( int j = 0; j < 16; j++ )
            ModelSbox( model, 
            X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i + 1][4 * j + 2], X[i + 1][4 * j + 3], 
            Y[i + 1][4 * j], Y[i + 1][4 * j + 1], Y[i + 1][4 * j + 2], Y[i + 1][4 * j + 3] );


        for ( int i = 0; i < 64; i++ )
            if ( inx[i] == 0 )
                model.addConstr( X[0][i] == 0 );
            else
                model.addConstr( X[0][i] == 1 );

        // set variable values
        for ( int i = 0; i < 64; i++ )
            if ( outx[i] == 0 )
                model.addConstr( Y[r - 1][i] == 0 );
            else
                model.addConstr( Y[r - 1][i] == 1 );


        model.optimize();

        if ( model.get( GRB_IntAttr_Status ) == GRB_OPTIMAL )
        {
            BS x, y;
            for ( int i = 0; i < 64; i++ )
            {
                x[i] = static_cast<Nibble> ( round ( start[i].get(
                                GRB_DoubleAttr_X ) ) );
                y[i] = static_cast<Nibble> ( round (    end[i].get(
                                GRB_DoubleAttr_X ) ) );
            }


            Bit_to_Nibble( x, START );
            Bit_to_Nibble( y, END );


            return 1;
        }
        else if ( model.get( GRB_IntAttr_Status ) == GRB_INFEASIBLE )
            return 0;

        else
            return -1;

        for ( auto it : X )
            delete [] it;
        
        for ( auto it : Y )
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

int CRAFT_Truncated( int r, const NS & inx, const NS & outx )
{
    // generate model
    try 
    { 
        GRBEnv env = GRBEnv( true );

        env.set(GRB_IntParam_OutputFlag, 0 );

        env.set(GRB_IntParam_LogToConsole, 0);
        env.set(GRB_IntParam_Threads, 1 );

        env.start();

        GRBModel model = GRBModel(env);

        // generate variables
        vector<GRBVar*> X;
        for ( int i = 0; i < r; i++ )
        {
            GRBVar * p = new GRBVar[64];
            for ( int j = 0; j < 64; j++ )
                p[j] = model.addVar( 0, 1, 0, GRB_BINARY );
            X.push_back( p );
        }

        vector<GRBVar*> Y;
        for ( int i = 0; i < r; i++ )
        {
            GRBVar * p = new GRBVar[64];
            for ( int j = 0; j < 64; j++ )
                p[j] = model.addVar( 0, 1, 0, GRB_BINARY );
            Y.push_back( p );
        }

        int i = 0; // r = 0

        for ( int j = 0; j < 16; j++ )
            ModelSbox( model, 
            X[i][4 * j], X[i][4 * j + 1], X[i][4 * j + 2], X[i][4 * j + 3], 
            Y[i][4 * j], Y[i][4 * j + 1], Y[i][4 * j + 2], Y[i][4 * j + 3] );

        for ( int k = 0; k < 4; k++ )
            for ( int j = 0; j < 4; j++ )
                ModelMC( model, Y[i][4 * k + 0 + j],   Y[i][4 * k + 16 + j], Y[i][4 * k + 32 + j], Y[i][4 * k + 48 + j],
                                X[i+1][4 * k + 0 + j], X[i+1][4 * k + 16 +
                                j], X[i+1][4 * k + 32 + j], X[i+1][4 * k +
                                48 + j] );   

        i = 1;

        for ( int j = 0; j < 16; j++ )
            ModelSbox( model, 
            X[i][4 * j], X[i][4 * j + 1], X[i][4 * j + 2], X[i][4 * j + 3], 
            Y[i][4 * j], Y[i][4 * j + 1], Y[i][4 * j + 2], Y[i][4 * j + 3] );

        SR( Y[i] );

        for (  i = 1; i < r - 3; i++ ) 
        {
            for ( int k = 0; k < 4; k++ )
                for ( int j = 0; j < 4; j++ )
                    ModelMC( model, Y[i][4 * k + 0 + j],   Y[i][4 * k + 16 + j], Y[i][4 * k + 32 + j], Y[i][4 * k + 48 + j],
                                    X[i+1][4 * k + 0 + j], X[i+1][4 * k + 16 +
                                    j], X[i+1][4 * k + 32 + j], X[i+1][4 * k +
                                    48 + j] );   

            SR( X[i+1] );

            for ( int j = 0; j < 16; j++ )
                ModelSbox( model, 
                X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i + 1][4 * j + 2], X[i + 1][4 * j + 3], 
                Y[i + 1][4 * j], Y[i + 1][4 * j + 1], Y[i + 1][4 * j + 2], Y[i + 1][4 * j + 3] );

        }

        i = r - 3;

        for ( int k = 0; k < 4; k++ )
            for ( int j = 0; j < 4; j++ )
                ModelMC( model, Y[i][4 * k + 0 + j],   Y[i][4 * k + 16 + j], Y[i][4 * k + 32 + j], Y[i][4 * k + 48 + j],
                                X[i+1][4 * k + 0 + j], X[i+1][4 * k + 16 +
                                j], X[i+1][4 * k + 32 + j], X[i+1][4 * k +
                                48 + j] );   

        SR( X[i+1] );

        for ( int j = 0; j < 16; j++ )
            ModelSbox( model, 
            X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i + 1][4 * j + 2], X[i + 1][4 * j + 3], 
            Y[i + 1][4 * j], Y[i + 1][4 * j + 1], Y[i + 1][4 * j + 2], Y[i + 1][4 * j + 3] );

        i = r - 2;

        for ( int k = 0; k < 4; k++ )
            for ( int j = 0; j < 4; j++ )
                ModelMC( model, Y[i][4 * k + 0 + j],   Y[i][4 * k + 16 + j], Y[i][4 * k + 32 + j], Y[i][4 * k + 48 + j],
                                X[i+1][4 * k + 0 + j], X[i+1][4 * k + 16 +
                                j], X[i+1][4 * k + 32 + j], X[i+1][4 * k +
                                48 + j] );   

        for ( int j = 0; j < 16; j++ )
            ModelSbox( model, 
            X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i + 1][4 * j + 2], X[i + 1][4 * j + 3], 
            Y[i + 1][4 * j], Y[i + 1][4 * j + 1], Y[i + 1][4 * j + 2], Y[i + 1][4 * j + 3] );


        for ( int i = 0; i < 16; i++ )
            if ( inx[i] == 0 )
                model.addConstr( X[0][4 * i] + X[0][4 * i + 1] + X[0][4 * i + 2]
                        + X[0][4
                        * i + 3] == 0  );
            else
                model.addConstr( X[0][4 * i] + X[0][4 * i + 1] + X[0]
                        [4 * i + 2] + X[0][4
                        * i + 3] >= 1  );

        for ( int i = 0; i < 16; i++ )
            if ( outx[i] == 0 )
                model.addConstr( Y[r - 1][4 * i] + Y[r - 1][4 * i + 1] + Y[r - 1][4 * i + 2]
                        + Y[r - 1][4
                        * i + 3] == 0  );
            else
                model.addConstr( Y[r - 1][4 * i] + Y[r - 1][4 * i + 1] + Y[r - 1]
                        [4 * i + 2] + Y[r - 1][4
                        * i + 3] >= 1  );

        {
            // set variable values
            model.optimize();

            if ( model.get( GRB_IntAttr_Status ) == GRB_OPTIMAL )
            {
                for ( auto it : X )
                    delete [] it;
                
                for ( auto it : Y )
                    delete [] it;
                return 1;
            }

            else if ( model.get( GRB_IntAttr_Status ) == GRB_INFEASIBLE )
            {
                for ( auto it : X )
                    delete [] it;
                
                for ( auto it : Y )
                    delete [] it;
                return 0;
            }

            else
            {
                for ( auto it : X )
                    delete [] it;
                
                for ( auto it : Y )
                    delete [] it;
                return -1;
            }

        }

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

