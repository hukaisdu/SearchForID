#include"gurobi_c++.h"
#include"MILP.h"
#include"gift.h"
#include"basic.h"
#include<set>
#include<tuple>
#include<fstream>
#include<cmath>
#include<random>

using namespace std;

int SboxIneq[][9] = {
    {0, 3, 1, 4, 4, -2, 0, -2, 1}, 
    {0, 1, 1, 0, -3, 3, 1, 2, 2}, 
    {0, 3, -2, -2, 3, 1, 1, 4, 1}, 
    {5, -2, -2, 0, 0, 2, -1, -1, -1}, 
    {0, -1, 1, 2, 2, 0, 0, 2, -1}, 
    {8, -1, 1, -3, 2, -1, 0, -3, -3}, 
    {0, 3, 2, 0, -1, 3, 2, -1, -1}, 
    {0, -2, 2, 1, 2, 0, 0, 1, 1}, 
    {7, -2, -1, -1, 1, 3, -3, 1, -3}, 
    {7, 1, -1, -1, 3, -3, 0, -2, -3}, 
    {8, -1, -3, 1, -1, 2, -3, -3, 1}, 
    {0, 0, 3, -1, -1, 2, 3, -1, 3}, 
    {0, 3, 1, -1, -1, 2, 3, 1, -1}, 
    {0, 1, 2, 2, 3, 0, -3, 1, 1}, 
    {5, -2, -1, -1, 2, 0, 0, -2, -1}, 
    {6, -2, -2, -1, -2, -1, 1, 1, 2}, 
    {4, 1, -1, -1, -1, 0, -1, -1, 0}, 
    {4, 1, -1, -1, 0, -1, -1, 0, -1}, 
    {4, -1, -1, 2, -1, -2, 2, 2, -1}, 
    {0, 2, 3, 3, 2, -1, -1, 1, -1}, 
    {0, 1, -3, 1, 1, 2, 2, 2, 1}, 
    {0, 3, -1, 3, 2, -1, 1, -1, 2}, 
    {6, -1, 1, 2, -1, -2, -2, -2, 1}, 
    {7, -2, -1, -2, -1, -2, 2, -1, 2}, 
    {8, -1, -1, -2, -2, -1, -2, 2, -1}, 
    {4, -1, 1, 0, -1, -1, 0, -1, -1}, 
    {0, -2, 4, 1, 2, 2, -2, 3, 1}, 
    {0, 2, -1, 2, 2, 1, -1, -1, 3}, 
    {4, 1, -1, 1, -2, 1, 1, -1, -2}, 
    {4, -1, -1, -1, -1, -1, 1, 0, 1}, 
    {7, -2, -1, 2, -2, -1, -2, -1, 2}, 
    {0, 2, 2, 0, 0, 2, 1, -1, -1}, 
    {4, 1, -1, 1, 1, -2, 1, -2, -1}, 
    {0, 1, 3, 3, 2, -1, -1, 2, -1}, 
    {4, -1, 1, 1, -1, -1, -1, -1, 0}, 
    {2, 1, 1, -1, -1, 0, 1, 1, -1}
};

int Perm2[32] = { 0,8,16,24,1,9,17,25,2,10,18,26,3,11,19,27, 4,12,20,28,5,13,21,29,6,14,22,30,7,15,23,31 };

void P2( GRBVar * p ) 
{
    GRBVar tmp[128];
    for ( int i = 0; i < 128; i++ )
        tmp[i] = p[i];

    for ( int i = 0; i < 32; i++ )
        for ( int j = 0; j < 4; j++ )
            p[4 * i + j] = tmp[ Perm2[i] * 4 + j ];
}

int Perm1[16] = { 12,1,6,11,8,13,2,7,4,9,14,3,0,5,10,15 };

void P1( GRBVar * p ) 
{
    GRBVar tmp[128];
    for ( int i = 0; i < 128; i++ )
        tmp[i] = p[i];

    for ( int i = 0; i < 16; i++ )
    {
        p[i] = tmp[ Perm1[i] ];
        p[16 + i] = tmp[ 16 + Perm1[i] ];
        p[32 + i] = tmp[ 32 + Perm1[i] ];
        p[48 + i] = tmp[ 48 + Perm1[i] ];
        p[64 + i] = tmp[ 64 + Perm1[i] ];
        p[80 + i] = tmp[ 80 + Perm1[i] ];
        p[96 + i] = tmp[ 96 + Perm1[i] ];
        p[112 + i] = tmp[ 112 + Perm1[i] ];
    }

}

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

void BitPermF ( GRBVar* p )
{
    GRBVar tmp[128];
    for ( int i = 0; i < 128; i++ )
        tmp[i] = p[i];

    for ( int i = 0; i < 128; i++ )
        p[ 127 - BitPerm[i] ] = tmp[ 127 - i ];
}

void GIFT_Inner_Round( int r, const NS & inx, const set<NS> & outx, set<
        pair<NS, NS> > & bad )
{
    //cout << "Gift_Inner" << endl;
    // generate model
    try 
    {
        GRBEnv env = GRBEnv( true );

        env.set( GRB_IntParam_OutputFlag, 0 );

        env.set(GRB_IntParam_LogToConsole, 0);
        env.set(GRB_IntParam_Threads, 1 );
        env.set( GRB_IntParam_MIPFocus, 1 );

        env.start();

        GRBModel model = GRBModel(env);


        // generate variables
        vector<GRBVar*> X;
        for ( int i = 0; i < r + 1; i++ )
        {
            GRBVar * p = new GRBVar[128];
            for ( int j = 0; j < 128; j++ )
                p[j] = model.addVar( 0, 1, 0, GRB_BINARY );
            X.push_back( p );
        }

        // set the plaintext
        BS inxx;
        Nibble_to_Bit( inx, inxx );

        for ( int i = 0; i < 128; i++ )
            if ( inxx[i] == 0 )
                model.addConstr( X[0][i] == 0 );
            else
                model.addConstr( X[0][i] == 1 );

        P2( X[0] );
        BitPermF( X[0] );

        for ( int i = 0; i < r; i++ )
        {
            // SBox
            for ( int j = 0; j < 32; j++ )
                ModelSbox( model, X[i    ][4 * j], X[i    ][4 * j + 1], X[i    ][4 * j + 2], X[i    ][4 * j + 3], 
                                  X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i + 1][4 * j + 2], X[i + 1][4 * j + 3] ); 

            BitPermF( X[i+1] );
        }

        vector<GRBConstr> R;
        for ( auto & jt : outx )
        {
            BS jtx;

            Nibble_to_Bit( jt, jtx );

            R.clear();

            // set variable values
            for ( int i = 0; i < 128; i++ )
            {

                if ( jtx[i] == 0 )
                    R.push_back( model.addConstr( X[r][i] == 0 ) );
                else
                    R.push_back( model.addConstr( X[r][i] == 1 ) );
            }

            model.update();

            model.optimize();

            if ( model.get( GRB_IntAttr_Status ) == GRB_OPTIMAL )
            {
                ;
            }
            else if ( model.get( GRB_IntAttr_Status ) == GRB_INFEASIBLE )
            {
                bad.insert( make_pair( inx, jt ) );
            }

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
}

int GIFT_MultiSolution( int r, const NS & inx, const NS & outx, set< pair<NS,
        NS> > & PIN )
{
    //cout << "Gift_MultiSolution" << endl;
    // generate model
    try 
    {
        GRBEnv env = GRBEnv( true );
        env.set( GRB_IntParam_OutputFlag, 0 );

        env.set(GRB_IntParam_LogToConsole, 0);
        env.set(GRB_IntParam_Threads, 1 );
        // solution pool
        env.set(GRB_IntParam_PoolSearchMode, 2); 
        env.set(GRB_IntParam_PoolSolutions, 100 ); 

        env.start();

        GRBModel model = GRBModel(env);

        // generate variables
        vector<GRBVar*> X;
        for ( int i = 0; i < r + 1; i++ )
        {
            GRBVar * p = new GRBVar[128];
            for ( int j = 0; j < 128; j++ )
                p[j] = model.addVar( 0, 1, 0, GRB_BINARY );
            X.push_back( p );
        }

        // set the plaintext
        BS inxx;
        Nibble_to_Bit( inx, inxx );

        for ( int i = 0; i < 128; i++ )
            if ( inxx[i] == 0 )
                model.addConstr( X[0][i] == 0 );
            else
                model.addConstr( X[0][i] == 1 );

        vector<GRBVar> START(128);
        vector<GRBVar> END(128);

        int i = 0;
        for ( int j = 0; j < 32; j++ )
            ModelSbox( model, 
                    X[i][4 * j],     X[i][4 * j + 1],     X[i][4 * j + 2],     X[i][4 * j + 3], 
                    X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i + 1][4 * j + 2], X[i + 1][4 * j + 3] ); 

        P1( X[i+1] );

        i = 1;
        for ( int j = 0; j < 32; j++ )
            ModelSbox( model, 
                    X[i][4 * j],     X[i][4 * j + 1],     X[i][4 * j + 2],     X[i][4 * j + 3], 
                    X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i + 1][4 * j + 2], X[i + 1][4 * j + 3] ); 

        for ( int j = 0; j < 128; j++ )
            START[j] = X[i+1][j];

        P2( X[i+1] );

        BitPermF( X[i+1] );

        for ( int i = 2; i < r - 2; i++ )
        {
            // SBox
            for ( int j = 0; j < 32; j++ )
                ModelSbox( model, X[i][4 * j], X[i][4 * j + 1], X[i][4 * j + 2], X[i][4 * j + 3], 
                                  X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i +
                                  1][4 * j + 2], X[i + 1][4 * j + 3] ); 

            BitPermF( X[i+1] );
        }

        // SBox
        i = r-2;

        for ( int j = 0; j < 128; j++ )
            END[j] = X[i][j];

        for ( int j = 0; j < 32; j++ )
            ModelSbox( model, X[i][4 * j], X[i][4 * j + 1], X[i][4 * j + 2], X[i][4 * j + 3], 
                              X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i +
                              1][4 * j + 2], X[i + 1][4 * j + 3] ); 

        P1( X[i+1] );

        i = r - 1;

        for ( int j = 0; j < 32; j++ )
            ModelSbox( model, X[i][4 * j], X[i][4 * j + 1], X[i][4 * j + 2], X[i][4 * j + 3], 
                              X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i +
                              1][4 * j + 2], X[i + 1][4 * j + 3] ); 

        // set variable values
        BS jtx;
        Nibble_to_Bit( outx, jtx );

        for ( int i = 0; i < 128; i++ )
        {
            if ( jtx[i] == 0 )
                model.addConstr( X[r][i] == 0 );
            else
                model.addConstr( X[r][i] == 1 );
        }

        model.update();

        model.optimize();

        if ( model.get( GRB_IntAttr_Status ) == GRB_OPTIMAL )
        {
            int solCount = model.get(GRB_IntAttr_SolCount);
            //cout << "solCount " << solCount << endl;
            for ( int i = 0; i < solCount; i++ )
            {
                BS x, y;
                model.set(GRB_IntParam_SolutionNumber, i );
                for ( int i = 0; i < 128; i++ )
                {
                    x[i] = static_cast<Nibble> ( round ( START[i].get(
                                    GRB_DoubleAttr_Xn ) ) );
                    y[i] = static_cast<Nibble> ( round (  END[i].get(
                                    GRB_DoubleAttr_Xn ) ) );
                }

                NS xx, yy;
                Bit_to_Nibble( x, xx );
                Bit_to_Nibble( y, yy );

                PIN.insert ( make_pair( xx, yy ) );
            }

            for ( auto it : X )
                delete [] it;
            return 1;
        }
        else if ( model.get( GRB_IntAttr_Status ) == GRB_INFEASIBLE )
        {
            for ( auto it : X )
                delete [] it;
            return 0;
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

int GIFT_Single( int r, const NS & inx, const NS & outx, NS & start, NS & end
        )
{
    //cout << "Gift_Single" << endl;
    // generate model
    try 
    {
        GRBEnv env = GRBEnv( true );
        env.set( GRB_IntParam_OutputFlag, 0 );

        env.set(GRB_IntParam_LogToConsole, 0);
        env.set(GRB_IntParam_Threads, 1 );

        env.start();

        GRBModel model = GRBModel(env);

        // generate variables
        vector<GRBVar*> X;
        for ( int i = 0; i < r + 1; i++ )
        {
            GRBVar * p = new GRBVar[128];
            for ( int j = 0; j < 128; j++ )
                p[j] = model.addVar( 0, 1, 0, GRB_BINARY );
            X.push_back( p );
        }

        // set the plaintext
        BS inxx;
        Nibble_to_Bit( inx, inxx );

        for ( int i = 0; i < 128; i++ )
            if ( inxx[i] == 0 )
                model.addConstr( X[0][i] == 0 );
            else
                model.addConstr( X[0][i] == 1 );

        vector<GRBVar> START(128);
        vector<GRBVar> END(128);

        int i = 0;
        for ( int j = 0; j < 32; j++ )
            ModelSbox( model, 
                    X[i][4 * j],     X[i][4 * j + 1],     X[i][4 * j + 2],     X[i][4 * j + 3], 
                    X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i + 1][4 * j + 2], X[i + 1][4 * j + 3] ); 

        P1( X[i+1] );

        i = 1;
        for ( int j = 0; j < 32; j++ )
            ModelSbox( model, 
                    X[i][4 * j],     X[i][4 * j + 1],     X[i][4 * j + 2],     X[i][4 * j + 3], 
                    X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i + 1][4 * j + 2], X[i + 1][4 * j + 3] ); 

        for ( int j = 0; j < 128; j++ )
            START[j] = X[i+1][j];

        P2( X[i+1] );

        BitPermF( X[i+1] );

        for ( int i = 2; i < r - 2; i++ )
        {
            // SBox
            for ( int j = 0; j < 32; j++ )
                ModelSbox( model, X[i][4 * j], X[i][4 * j + 1], X[i][4 * j + 2], X[i][4 * j + 3], 
                                  X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i +
                                  1][4 * j + 2], X[i + 1][4 * j + 3] ); 

            BitPermF( X[i+1] );
        }

        // SBox
        i = r-2;

        for ( int j = 0; j < 128; j++ )
            END[j] = X[i][j];

        for ( int j = 0; j < 32; j++ )
            ModelSbox( model, X[i][4 * j], X[i][4 * j + 1], X[i][4 * j + 2], X[i][4 * j + 3], 
                              X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i +
                              1][4 * j + 2], X[i + 1][4 * j + 3] ); 

        P1( X[i+1] );

        i = r - 1;

        for ( int j = 0; j < 32; j++ )
            ModelSbox( model, X[i][4 * j], X[i][4 * j + 1], X[i][4 * j + 2], X[i][4 * j + 3], 
                              X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i +
                              1][4 * j + 2], X[i + 1][4 * j + 3] ); 


        // set variable values
        BS jtx;
        Nibble_to_Bit( outx, jtx );

        for ( int i = 0; i < 128; i++ )
        {
            if ( jtx[i] == 0 )
                model.addConstr( X[r][i] == 0 );
            else
                model.addConstr( X[r][i] == 1 );
        }

        model.update();

        model.optimize();

        if ( model.get( GRB_IntAttr_Status ) == GRB_OPTIMAL )
        {
            BS x, y;

            for ( int i = 0; i < 128; i++ )
            {
                x[i] = static_cast<Nibble> ( round ( START[i].get(
                                GRB_DoubleAttr_X ) ) );
                y[i] = static_cast<Nibble> ( round ( END[i].get(
                                GRB_DoubleAttr_X ) ) );
            }

            Bit_to_Nibble( x, start );
            Bit_to_Nibble( y, end );

            for ( auto it : X )
                delete [] it;

            return 1;
        }
        else if ( model.get( GRB_IntAttr_Status ) == GRB_INFEASIBLE )
        {
            for ( auto it : X )
                delete [] it;

            return 0;
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

int GIFT_Truncated( int r, const NS & inx, const NS & outx )
{
    //cout << "Gift_Truncated" << endl;
    // generate model
    try 
    {
        GRBEnv env = GRBEnv( true );
        env.set( GRB_IntParam_OutputFlag, 0 );

        env.set(GRB_IntParam_LogToConsole, 0);
        env.set(GRB_IntParam_Threads, 1 );

        env.start();

        GRBModel model = GRBModel(env);

        // generate variables
        vector<GRBVar*> X;
        for ( int i = 0; i < r + 1; i++ )
        {
            GRBVar * p = new GRBVar[128];
            for ( int j = 0; j < 128; j++ )
                p[j] = model.addVar( 0, 1, 0, GRB_BINARY );
            X.push_back( p );
        }

        for ( int i = 0; i < 32; i++ )
            if ( inx[i] == 0 )
                model.addConstr( X[0][4 * i] + X[0][4 * i + 1] + X[0][4 * i + 2]
                        + X[0][4
                        * i + 3] == 0  );
            else
                model.addConstr( X[0][4 * i] + X[0][4 * i + 1] + X[0]
                        [4 * i + 2] + X[0][4
                        * i + 3] >= 1  );


        int i = 0;
        for ( int j = 0; j < 32; j++ )
            ModelSbox( model, 
                    X[i][4 * j],     X[i][4 * j + 1],     X[i][4 * j + 2],     X[i][4 * j + 3], 
                    X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i + 1][4 * j + 2], X[i + 1][4 * j + 3] ); 

        P1( X[i+1] );

        i = 1;
        for ( int j = 0; j < 32; j++ )
            ModelSbox( model, 
                    X[i][4 * j],     X[i][4 * j + 1],     X[i][4 * j + 2],     X[i][4 * j + 3], 
                    X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i + 1][4 * j + 2], X[i + 1][4 * j + 3] ); 

        P2( X[i+1] );

        BitPermF( X[i+1] );

        for ( int i = 2; i < r - 2; i++ )
        {
            // SBox
            for ( int j = 0; j < 32; j++ )
                ModelSbox( model, X[i][4 * j], X[i][4 * j + 1], X[i][4 * j + 2], X[i][4 * j + 3], 
                                  X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i +
                                  1][4 * j + 2], X[i + 1][4 * j + 3] ); 

            BitPermF( X[i+1] );
        }

        // SBox
        i = r-2;

        for ( int j = 0; j < 32; j++ )
            ModelSbox( model, X[i][4 * j], X[i][4 * j + 1], X[i][4 * j + 2], X[i][4 * j + 3], 
                              X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i +
                              1][4 * j + 2], X[i + 1][4 * j + 3] ); 

        P1( X[i+1] );

        i = r - 1;

        for ( int j = 0; j < 32; j++ )
            ModelSbox( model, X[i][4 * j], X[i][4 * j + 1], X[i][4 * j + 2], X[i][4 * j + 3], 
                              X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i +
                              1][4 * j + 2], X[i + 1][4 * j + 3] ); 

        for ( int i = 0; i < 32; i++ )
            if ( outx[i] == 0 )
                model.addConstr( X[r][4 * i] + X[r][4 * i + 1] + X[r][4 * i + 2]
                        + X[r][4
                        * i + 3] == 0  );
            else
                model.addConstr( X[r][4 * i] + X[r][4 * i + 1] + X[r]
                        [4 * i + 2] + X[r][4
                        * i + 3] >= 1  );

        model.update();

        model.optimize();

        if ( model.get( GRB_IntAttr_Status ) == GRB_OPTIMAL )
        {
            for ( auto it : X )
                delete [] it;

            return 1;
        }
        else if ( model.get( GRB_IntAttr_Status ) == GRB_INFEASIBLE )
        {
            for ( auto it : X )
                delete [] it;

            return 0;
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

int GIFT_PlaintextTruncated( int r, const NS & inx, const NS & outx )
{
    //cout << "Gift_Truncated" << endl;
    // generate model
    try 
    {
        GRBEnv env = GRBEnv( true );
        env.set( GRB_IntParam_OutputFlag, 0 );

        env.set(GRB_IntParam_LogToConsole, 0);
        env.set(GRB_IntParam_Threads, 1 );

        env.start();
        GRBModel model = GRBModel(env);

        // generate variables
        vector<GRBVar*> X;
        for ( int i = 0; i < r + 1; i++ )
        {
            GRBVar * p = new GRBVar[128];
            for ( int j = 0; j < 128; j++ )
                p[j] = model.addVar( 0, 1, 0, GRB_BINARY );
            X.push_back( p );
        }

        for ( int i = 0; i < 32; i++ )
            if ( inx[i] == 0 )
                model.addConstr( X[0][4 * i] + X[0][4 * i + 1] + X[0][4 * i + 2]
                        + X[0][4
                        * i + 3] == 0  );
            else
                model.addConstr( X[0][4 * i] + X[0][4 * i + 1] + X[0]
                        [4 * i + 2] + X[0][4
                        * i + 3] >= 1  );


        int i = 0;
        for ( int j = 0; j < 32; j++ )
            ModelSbox( model, 
                    X[i][4 * j],     X[i][4 * j + 1],     X[i][4 * j + 2],     X[i][4 * j + 3], 
                    X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i + 1][4 * j + 2], X[i + 1][4 * j + 3] ); 

        P1( X[i+1] );

        i = 1;
        for ( int j = 0; j < 32; j++ )
            ModelSbox( model, 
                    X[i][4 * j],     X[i][4 * j + 1],     X[i][4 * j + 2],     X[i][4 * j + 3], 
                    X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i + 1][4 * j + 2], X[i + 1][4 * j + 3] ); 

        P2( X[i+1] );

        BitPermF( X[i+1] );

        for ( int i = 2; i < r - 2; i++ )
        {
            // SBox
            for ( int j = 0; j < 32; j++ )
                ModelSbox( model, X[i][4 * j], X[i][4 * j + 1], X[i][4 * j + 2], X[i][4 * j + 3], 
                                  X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i +
                                  1][4 * j + 2], X[i + 1][4 * j + 3] ); 

            BitPermF( X[i+1] );
        }

        // SBox
        i = r-2;

        for ( int j = 0; j < 32; j++ )
            ModelSbox( model, X[i][4 * j], X[i][4 * j + 1], X[i][4 * j + 2], X[i][4 * j + 3], 
                              X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i +
                              1][4 * j + 2], X[i + 1][4 * j + 3] ); 

        P1( X[i+1] );

        i = r - 1;

        for ( int j = 0; j < 32; j++ )
            ModelSbox( model, X[i][4 * j], X[i][4 * j + 1], X[i][4 * j + 2], X[i][4 * j + 3], 
                              X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i +
                              1][4 * j + 2], X[i + 1][4 * j + 3] ); 


        BS p;
        Nibble_to_Bit ( outx, p );
        for ( int i = 0; i < 128; i++ )
            if ( p[i] == 0 )
                model.addConstr( X[r][i] == 0 );
            else
                model.addConstr( X[r][i] == 1 );

        model.update();

        model.optimize();

        if ( model.get( GRB_IntAttr_Status ) == GRB_OPTIMAL )
        {
            for ( auto it : X )
                delete [] it;

            return 1;
        }
        else if ( model.get( GRB_IntAttr_Status ) == GRB_INFEASIBLE )
        {
            for ( auto it : X )
                delete [] it;

            return 0;
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

int GIFT_CiphertextTruncated( int r, const NS & inx, const NS & outx )
{
    //cout << "Gift_Truncated" << endl;
    // generate model
    try 
    {
        GRBEnv env = GRBEnv();
        env.set( GRB_IntParam_OutputFlag, 0 );


        env.set(GRB_IntParam_LogToConsole, 0);
        env.set(GRB_IntParam_Threads, 1 );

        env.start();
        GRBModel model = GRBModel(env);

        // generate variables
        vector<GRBVar*> X;
        for ( int i = 0; i < r + 1; i++ )
        {
            GRBVar * p = new GRBVar[128];
            for ( int j = 0; j < 128; j++ )
                p[j] = model.addVar( 0, 1, 0, GRB_BINARY );
            X.push_back( p );
        }

        BS p;
        Nibble_to_Bit ( inx, p );
        for ( int i = 0; i < 128; i++ )
            if ( p[i] == 0 )
                model.addConstr( X[0][i] == 0 );
            else
                model.addConstr( X[0][i] == 1 );


        int i = 0;
        for ( int j = 0; j < 32; j++ )
            ModelSbox( model, 
                    X[i][4 * j],     X[i][4 * j + 1],     X[i][4 * j + 2],     X[i][4 * j + 3], 
                    X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i + 1][4 * j + 2], X[i + 1][4 * j + 3] ); 

        P1( X[i+1] );

        i = 1;
        for ( int j = 0; j < 32; j++ )
            ModelSbox( model, 
                    X[i][4 * j],     X[i][4 * j + 1],     X[i][4 * j + 2],     X[i][4 * j + 3], 
                    X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i + 1][4 * j + 2], X[i + 1][4 * j + 3] ); 

        P2( X[i+1] );

        BitPermF( X[i+1] );

        for ( int i = 2; i < r - 2; i++ )
        {
            // SBox
            for ( int j = 0; j < 32; j++ )
                ModelSbox( model, X[i][4 * j], X[i][4 * j + 1], X[i][4 * j + 2], X[i][4 * j + 3], 
                                  X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i +
                                  1][4 * j + 2], X[i + 1][4 * j + 3] ); 

            BitPermF( X[i+1] );
        }

        // SBox
        i = r-2;

        for ( int j = 0; j < 32; j++ )
            ModelSbox( model, X[i][4 * j], X[i][4 * j + 1], X[i][4 * j + 2], X[i][4 * j + 3], 
                              X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i +
                              1][4 * j + 2], X[i + 1][4 * j + 3] ); 

        P1( X[i+1] );

        i = r - 1;

        for ( int j = 0; j < 32; j++ )
            ModelSbox( model, X[i][4 * j], X[i][4 * j + 1], X[i][4 * j + 2], X[i][4 * j + 3], 
                              X[i + 1][4 * j], X[i + 1][4 * j + 1], X[i +
                              1][4 * j + 2], X[i + 1][4 * j + 3] ); 


        for ( int i = 0; i < 32; i++ )
            if ( outx[i] == 0 )
                model.addConstr( X[0][4 * i] + X[0][4 * i + 1] + X[0][4 * i + 2]
                        + X[0][4
                        * i + 3] == 0  );
            else
                model.addConstr( X[0][4 * i] + X[0][4 * i + 1] + X[0]
                        [4 * i + 2] + X[0][4
                        * i + 3] >= 1  );

        model.update();

        model.optimize();

        if ( model.get( GRB_IntAttr_Status ) == GRB_OPTIMAL )
        {
            for ( auto it : X )
                delete [] it;

            return 1;
        }
        else if ( model.get( GRB_IntAttr_Status ) == GRB_INFEASIBLE )
        {
            for ( auto it : X )
                delete [] it;

            return 0;
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











