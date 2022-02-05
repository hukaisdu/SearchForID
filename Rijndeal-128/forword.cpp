#include"basic.h" 
#include"backword.h"
#include"skinny.h"
#include"forword.h"
#include"log.h"
#include<iostream>
#include<set>
#include<algorithm>
#include<vector>
#include<map>

using namespace std;

void getPatterns ( const set<Word> & P, set<Word> & seed ) 
{
    seed = getIS_front(  P );
    //frontIS0.insert( 0 ); 
    NarrowSet_inner_to_front( seed, P ); // Forword
    logger( "Seed size: " + to_string( seed.size() ) );
}

// in -> S expand
void PassSuperSbox( Word in, set<Word> & S )
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
            if ( MATRIX[i][j] == 1 )  
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

Word Select_inner_to_front( const set<Word> & inner, const set<Word> & E )
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
        InvPassSuperSbox( it, SS );

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
set<Word> getIS_front( const set<Word> & xS )
{
    set<Word> S( xS.begin(), xS.end() );
    // IS is the inner set we want
    set<Word> IS;

    set<Word> SS, SSS;
    set<Word> Stemp;

    while ( ! S.empty() )
    {
        auto x = getRandom( S );

        SS.clear();
        // expand
        PassSuperSbox( x, SS ); // SS is the inner set

        auto it = Select_inner_to_front( SS, S );

        IS.insert ( it );

        SSS.clear();

        // remove those elements that have been hit 
        InvPassSuperSbox( it, SSS ); 

        Stemp.clear();
        set_difference( S.begin(), S.end(), SSS.begin(), SSS.end(),
                  insert_iterator< set<Word> >( Stemp, Stemp.begin() ) );

        S.swap( Stemp );
    }
    return IS;
}

// remove redudant elements
void NarrowSet_inner_to_front( set<Word> & S, const set<Word> &
        total )
{
    map<Word, set<Word>> Map;
    InitializeMapFront( S, Map, total );

    for ( auto it = S.begin(); it != S.end(); )
        if ( Map[*it].size() == 0 ) 
            S.erase( it++ );
        else
            ++it;
}

void InitializeMapFront( const set<Word> & S, map<Word,
        set<Word>> & Map, const set<Word> & total )
{
    for ( auto it : S )
    {
        set<Word> xS;
        InvPassSuperSbox( it, xS );

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

