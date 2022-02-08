#include"basic.h" 
#include"backword.h"
#include"midori.h"
#include"forword.h"
#include"log.h"
#include<iostream>
#include<set>
#include<algorithm>
#include<vector>
#include<map>

using namespace std;

void getPatterns ( const set<Word> & P, set<Word> & seed, int select ) 
{
    seed = getIS_front( table, Invtable, P, select );
    //frontIS0.insert( 0 ); 
    NarrowSet_inner_to_front( seed, Invtable, P ); // Forword
    logger( "Seed size: " + to_string( seed.size() ) );
}


// in -> S expand
void PassSuperSbox( Word in, int ** table, set<Word> & S )
{
    Nibble x0, x1, x2, x3;
    Word_to_Nibble( in, x0, x1, x2, x3 );

    vector<Nibble> X0, X1, X2, X3;

    for ( int i = 0; i < 16; i++ )
    {
        if ( table[x0][i] > 0 )
            X0.push_back( static_cast<Nibble> ( i ) );
        if ( table[x1][i] > 0 )
            X1.push_back( static_cast<Nibble> ( i ) );
        if ( table[x2][i] > 0 )
            X2.push_back( static_cast<Nibble> ( i ) );
        if ( table[x3][i] > 0 )
            X3.push_back( static_cast<Nibble> ( i ) );
    }

    for ( auto xx0 : X0 )
        for ( auto xx1 : X1 )
            for ( auto xx2 : X2 )
                for ( auto xx3 : X3 )
                {
                    auto y = Nibble_to_Word( xx0, xx1, xx2, xx3 );

                    Word z;
                    z = MixColumn( y );

                    Nibble z0, z1, z2, z3;
                    Word_to_Nibble( z, z0, z1, z2, z3 );

                    vector<Nibble> Z0, Z1, Z2, Z3;
                    for ( int i = 0; i < 16; i++ )
                    {
                        if ( table[z0][i] > 0 )
                            Z0.push_back( static_cast<Nibble> ( i ) );
                        if ( table[z1][i] > 0 )
                            Z1.push_back( static_cast<Nibble> ( i ) );
                        if ( table[z2][i] > 0 )
                            Z2.push_back( static_cast<Nibble> ( i ) );
                        if ( table[z3][i] > 0 )
                            Z3.push_back( static_cast<Nibble> ( i ) );
                    }

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

Word Select_inner_to_front( int ** Invtable, const set<Word> & inner, const set<Word> & E, int select_number )
{
    set<Word> SS, tmp;
    int max = 0;
    Word maxValue = 0;

    int n = 0;
    while ( n < select_number ) 
    {
        // select randomly an element from inner
        auto it = getRandom( inner );

        SS.clear();
        tmp.clear();

        // see what it can reach
        InvPassSuperSbox( it, Invtable, SS );

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

// S is the plaintext set
set<Word> getIS_front(int ** table, int ** Invtable, const set<Word> & xS, int
        select_number )
{
    set<Word> S( xS.begin(), xS.end() );
    // IS is the inner set we want
    set<Word> IS;

    set<Word> SS, SSS;
    set<Word> Stemp;

    while ( ! S.empty() )
    {
        // choose randonly an element 
        auto x = getRandom( S );
        SS.clear();
        // expand
        PassSuperSbox( x, table, SS ); // SS is the inner set

        auto it = Select_inner_to_front( Invtable, SS, S, select_number );

        IS.insert ( it );

        SSS.clear();

        // remove those elements that have been hit 
        InvPassSuperSbox( it, Invtable, SSS ); 

        Stemp.clear();
        set_difference( S.begin(), S.end(), SSS.begin(), SSS.end(),
                  insert_iterator< set<Word> >( Stemp, Stemp.begin() ) );
        //cout << Stemp.size() << endl;

        S.swap( Stemp );
    }
    return IS;
}

// remove redudant elements
void NarrowSet_inner_to_front( set<Word> & S, int ** Invtable, const set<Word> &
        total )
{
    /*
    set<Word> SS( S.begin(), S.end() );

    S.clear();

    for ( auto it : SS )
    {
        auto res = findUnique_inner_to_front( it, Invtable, SS, total );

        if ( res.size() > 0 )
            S.insert( it );
    }
    */
    map<Word, set<Word>> Map;
    InitializeMapFront( S, Invtable, Map, total );

    for ( auto it = S.begin(); it != S.end(); )
        if ( Map[*it].size() == 0 ) 
            S.erase( it++ );
        else
            ++it;
}

void InitializeMapFront( const set<Word> & S, int ** Invtable, map<Word,
        set<Word>> & Map, const set<Word> & total )
{
    for ( auto it : S )
    {
        set<Word> xS;
        InvPassSuperSbox( it, Invtable, xS );

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

set<Word> findUnique_inner_to_front( Word x, int** Invtable, set<Word> S, const
        set<Word> & total )
{
    set<Word> xS;
    InvPassSuperSbox( x, Invtable, xS );

    // the rest elements
    S.erase( x );
    set<Word> yS;

    for ( auto it : S )
        InvPassSuperSbox( it, Invtable, yS );

    // the set difference
    set<Word> res;
    set_difference( xS.begin(), xS.end(), yS.begin(), yS.end(), insert_iterator<set<Word>> ( res, res.begin() ) );

    set<Word> finalRes;
    set_intersection( res.begin(), res.end(), total.begin(), total.end(),
            insert_iterator<set<Word>> ( finalRes, finalRes.begin() ) );

    return finalRes;
}

void Reduce_inner_to_front( Word x, int ** Invtable, set<Word> & S )
{
    set<Word> SS;
    InvPassSuperSbox( x, Invtable, SS );

    set<Word> tmp;
    set_difference( S.begin(), S.end(), SS.begin(), SS.end(),
            insert_iterator<set<Word>> ( tmp, tmp.begin() ) );

    S.swap( tmp );
    if ( S.size() == 0 )
        S.insert ( 0 );
}


/*
int main()
{
        Word xx = 0x0001;

        Nibble x[16] = { 0 };
        Word w0, w1, w2, w3;

        // front
        Bit_to_Nibble( xx, x );
        InvMC( x );
        InvShiftRows( x );
        State_to_Word( x, w0, w1, w2, w3 );
        auto start0 = findUnique_inner_to_front( w0, Invtable, frontIS0 );
        auto start1 = findUnique_inner_to_front( w1, Invtable, frontIS1 );
        auto start2 = findUnique_inner_to_front( w2, Invtable, frontIS2 );
        auto start3 = findUnique_inner_to_front( w3, Invtable, frontIS3 );

        cout << "start 0, 1, 2, 3 size " << endl;
        cout << start0.size() <<  " " 
             << start1.size() <<  " " 
             << start2.size() <<  " " 
             << start3.size() <<  endl;
        cout << "start0" << endl;
        for ( auto & jt : start0 ) 
            cout << hex << int( jt ) << " ";
        cout << endl;
        cout << "start1" << endl;
        for ( auto & jt : start1 ) 
            cout << hex << int( jt ) << " ";
        cout << endl;
        cout << "start2" << endl;
        for ( auto & jt : start2 ) 
            cout << hex << int( jt ) << " ";
        cout << endl;
        cout << "start3" << endl;
        for ( auto & jt : start3 ) 
            cout << hex << int( jt ) << " ";
        cout << endl;
}
*/

