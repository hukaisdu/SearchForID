#include"basic.h"
#include"forword.h"
#include"backword.h"
#include"gift.h"
#include<map>
#include<set>
#include<vector>
#include<algorithm>
#include"log.h"

using namespace std;

void getBackwardPattern( const set<Word> & P, set<Word> & seed, int select )
{
    seed = getIS_back( table, Invtable, P, select );
    NarrowSet_inner_to_back( seed, table, P ); // Forword
    logger( "Seed size: " + to_string( seed.size() ) );
}

Word InvSuperBox( Word x )
{
    Nibble x0, x1, x2, x3;
    Word_to_Nibble( x, x0, x1, x2, x3 );

    auto y0 = InvSbox[x0];
    auto y1 = InvSbox[x1];
    auto y2 = InvSbox[x2];
    auto y3 = InvSbox[x3];

    Word z = InvSmallBitPerm( Nibble_to_Word( y0, y1, y2, y3 ) );

    Nibble z0, z1, z2, z3;
    Word_to_Nibble( z, z0, z1, z2, z3 );

    auto w0 = InvSbox[z0];
    auto w1 = InvSbox[z1];
    auto w2 = InvSbox[z2];
    auto w3 = InvSbox[z3];

    return Nibble_to_Word( w0, w1, w2, w3 );
}

void generateInvSuperDDT( int ** Invtable )
{
    for ( int i = 0; i < 65536; i++ )
        for ( int j = 0; j < 65536; j++ )
            Invtable[i][ InvSuperBox( j ) ^ InvSuperBox( i ^ j ) ] ++;
}


/*
void InvPassSuperSbox( Word in, int ** Invtable, set<Word> & S )
{
    for ( int i = 0; i < 65536; i++ )
        if ( Invtable[in][i] != 0 )
            S.insert ( i );
}
*/

void InitializeMapBack( const set<Word> & S, int ** table, map<Word,
        set<Word>> & Map, const set<Word> & total )
{
    for ( auto it : S )
    {
        set<Word> xS;
        PassSuperSbox( it, table, xS );

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

void InvPassSuperSbox( Word in, int ** Invtable, set<Word> & S )
{
    Nibble x0, x1, x2, x3;
    Word_to_Nibble( in, x0, x1, x2, x3 );

    set<Nibble> X0, X1, X2, X3;

    X0 = InvtableMap.at( x0 );
    X1 = InvtableMap.at( x1 );
    X2 = InvtableMap.at( x2 );
    X3 = InvtableMap.at( x3 );

    for ( auto xx0 : X0 )
        for ( auto xx1 : X1 )
            for ( auto xx2 : X2 )
                for ( auto xx3 : X3 )
                {
                    auto y = Nibble_to_Word( xx0, xx1, xx2, xx3 );

                    Word z;
                    z = InvSmallBitPerm( y );

                    Nibble z0, z1, z2, z3;
                    Word_to_Nibble( z, z0, z1, z2, z3 );

                    set<Nibble> Z0, Z1, Z2, Z3;

                    Z0 = InvtableMap.at( z0 );
                    Z1 = InvtableMap.at( z1 );
                    Z2 = InvtableMap.at( z2 );
                    Z3 = InvtableMap.at( z3 );

                    for ( auto zz0 : Z0 )
                        for ( auto zz1 : Z1 )
                            for ( auto zz2 : Z2 )
                                for ( auto zz3 : Z3 )
                                {
                                    auto p = Nibble_to_Word( zz0, zz1, zz2, zz3 );
                                    S.insert ( p );
                                }
                }
}

Word Select_inner_to_back( int ** table, const set<Word> & S, const set<Word> & E, int select_number )
{
    set<Word> SS, tmp;
    int max = 0;
    Word maxValue = 0;
    int n = 0;

    while ( n < select_number ) 
    {
        auto it = getRandom ( S );
        SS.clear();
        tmp.clear();

        PassSuperSbox( it, table, SS );

        set_intersection( E.begin(), E.end(), SS.begin(), SS.end(), insert_iterator< set<Word> > ( tmp, tmp.begin() ) );

        if  ( tmp.size() > max )
        {
            maxValue = it;
            max = tmp.size();
        }
        n++;
    }
    return maxValue;
}

set<Word> getIS_back(int ** table, int ** Invtable, const set<Word> & xS, int
        select_number )
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
        InvPassSuperSbox( x, Invtable, SS ); // SS is the output diff 

        auto it = Select_inner_to_back( table, SS, S, select_number );

        IS.insert ( it );

        SSS.clear();

        PassSuperSbox( it, table, SSS ); // SS is the output diff 

        Stemp.clear();
        set_difference( S.begin(), S.end(), SSS.begin(), SSS.end(),
                  insert_iterator<set<Word>>( Stemp, Stemp.begin() ) );

        S.swap( Stemp );
    }

    return IS;
}

set<Word> findUnique_inner_to_back( Word x, int** table, set<Word> S, const
        set<Word> & total )
{
    set<Word> xS;
    PassSuperSbox( x, table, xS );

    // the rest elements
    S.erase( x );
    set<Word> yS;
    for ( auto& it : S )
        PassSuperSbox( it, table, yS );

    // the set difference
    set<Word> res;
    set_difference( xS.begin(), xS.end(), yS.begin(), yS.end(), insert_iterator<set<Word>> ( res, res.begin() ) );

    set<Word> finalRes;
    set_intersection( res.begin(), res.end(), total.begin(), total.end(),
            insert_iterator<set<Word>> ( finalRes, finalRes.begin() ) );

    return finalRes;
}

void NarrowSet_inner_to_back( set<Word> & S, int ** table, const set<Word> &
        total )
{
    /*
    set<Word> SS( S.begin(), S.end() );

    S.clear();

    for ( auto it : SS )
    {
        auto res = findUnique_inner_to_back( it, table, SS, total );

        if ( res.size() > 0 )
            S.insert( it );
    }
    */

    map<Word, set<Word>> Map;
    InitializeMapBack( S, table, Map, total );

    for ( auto it = S.begin(); it != S.end(); )
        if ( Map[*it].size() == 0 ) 
            S.erase( it++ );
        else
            ++it;
}

void Reduce_inner_to_back( Word x, int ** table, set<Word> & S )
{
    set<Word> SS;
    PassSuperSbox( x, table, SS );

    set<Word> tmp;
    set_difference( S.begin(), S.end(), SS.begin(), SS.end(),
            insert_iterator<set<Word>> ( tmp, tmp.begin() ) );

    S.swap( tmp );
    if ( S.size() == 0 )
        S.insert ( 0 );
}


