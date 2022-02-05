#include"basic.h"
#include"forword.h"
#include"backword.h"
#include"skinny.h"
#include"log.h"

#include<set>
#include<vector>
#include<algorithm>

using namespace std;

void getInvPatterns(const set<Word> & E0, set<Word> & backIS0 )
{
    backIS0 = getIS_back( E0 );
    NarrowSet_inner_to_back( backIS0, E0 ); // forword
    logger( string ("IS0.size " ) + to_string( backIS0.size() ) );
  
}

void InvPassSuperSbox( Word in, set<Word> & S )
{
    /*
     * 1 0 1 1
     * 1 0 0 0 
     * 0 1 1 0 
     * 1 0 1 0 
     */
    Nibble x[4];
    Word_to_Nibble( in, x[0], x[1], x[2], x[3] );


    set<Word> X[4];
    for ( int i = 0; i < 4; i++ )
    {
        Word y[4] = { 0 };
        for ( int j = 0; j < 4; j++ )
            if ( InvMATRIX[i][j] == 1 )  
                y[i] += x[j];

        if ( y[i] == 0 )
            X[i].insert( 0 ); 
        else if ( y[i] == 1 )
            X[i].insert( 1 );
        else
        {
            X[i].insert( 0 );
            X[i].insert( 1 );
        }
    }

    for ( auto it0 : X[0] )
        for ( auto it1 : X[1] )
            for ( auto it2 : X[2] )
                for ( auto it3 : X[3] )
                    S.insert( Nibble_to_Word( it0, it1, it2, it3 )  );
}

Word Select_inner_to_back( const set<Word> & inner, const set<Word> & E )
{
    set<Word> SS, tmp;
    int max = 0;
    Word maxValue = 0;

    int n = 0;
    for ( auto it : inner )
    {
        // select randomly an element from inner
        SS.clear();
        tmp.clear();

        // see what it can reach
        PassSuperSbox( it, SS );

        // hit elements
        set_intersection( E.begin(), E.end(), SS.begin(), SS.end(), insert_iterator< set<Word> > ( tmp, tmp.begin() ) );

        // find the max element hitting the most elements
        if  ( tmp.size() > max )
        {
            maxValue = it;
            max = tmp.size();
        }
        n++;
    }
    return maxValue;
}


void InitializeMapBack( const set<Word> & S, map<Word,
        set<Word>> & Map, const set<Word> & total )
{
    for ( auto it : S )
    {
        set<Word> xS;
        PassSuperSbox( it, xS );

        set<Word> inres;
        set_intersection( xS.begin(), xS.end(), total.begin(),
                total.end(), insert_iterator<set<Word>> ( inres, inres.begin() ) );

        xS.swap( inres );

        for ( auto jt : Map )
        {
            set<Word> res;
            set_difference( xS.begin(), xS.end(), jt.second.begin(),
                    jt.second.end(), insert_iterator<set<Word>> ( res, res.begin() ) );

            xS.swap( res );
        }

        Map[it] = xS;
    }
}


set<Word> getIS_back( const set<Word> & xS )
{
    set<Word> S( xS.begin(), xS.end() );
    set<Word> IS;

    IS.clear();

    set<Word> SS, SSS;
    set<Word> Stemp;

    while ( ! S.empty() )
    {
        // choose first element only
        auto x = getRandom( S );

        SS.clear();
        InvPassSuperSbox( x, SS ); // SS is the output diff 

        auto it = Select_inner_to_back( SS, S );

        IS.insert ( it );

        SSS.clear();

        PassSuperSbox( it, SSS ); // SS is the output diff 

        Stemp.clear();
        set_difference( S.begin(), S.end(), SSS.begin(), SSS.end(),
                  insert_iterator<set<Word>>( Stemp, Stemp.begin() ) );

        S.swap( Stemp );
    }

    return IS;
}

void NarrowSet_inner_to_back( set<Word> & S, const set<Word> &
        total )
{
    map<Word, set<Word>> Map;
    InitializeMapBack( S, Map, total );

    for ( auto it = S.begin(); it != S.end(); )
        if ( Map[*it].size() == 0 ) 
            S.erase( it++ );
        else
            ++it;
}

