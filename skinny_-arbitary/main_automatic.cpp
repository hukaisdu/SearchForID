#include"gurobi_c++.h"
#include"forword.h"
#include"backword.h"
#include"skinny.h"
#include"basic.h"
#include"log.h"
#include"automatic.h"
#include"MILP.h"
#include"thread_pool.h"
#include<iostream>
#include<set>
#include<tuple>
#include<thread>
#include<future>
#include<cstdio>

int ROUND;

int ** table;
int ** Invtable;

int main()
{
    ROUND = 12;
    // generate DDT of the Sbox
    table =  new int* [16]; 
    for ( int i = 0; i < 16; i++ )
        table[i] = new int[16];

    genDDT( Sbox, table );

    // generate DDT of the inverse Sbox
    Invtable =  new int* [16]; 
    for ( int i = 0; i < 16; i++ )
        Invtable[i] = new int[16];

    genDDT( InvSbox, Invtable );

    Word PP[16];
    for ( int i = 0; i < 16; i++ )
    {
        int i0 = i >> 3 & 0x1;
        int i1 = i >> 2 & 0x1;
        int i2 = i >> 1 & 0x1;
        int i3 = i >> 0 & 0x1;

        PP[i] = Nibble_to_Word( i0, i1, i2, i3 ); 
    }

    set<Word> P ( PP, PP + 16 );

    set<pair<NS, NS>> badPattern;

    set<Word> frontIS0, frontIS1, frontIS2, frontIS3, backIS0, backIS1, backIS2,
        backIS3;

    set<Word> tmpSeed[64];

    vector<thread> threads;

    for ( int i = 0; i < 64; i++ )
        threads.push_back(  thread( getPatterns, ref( P ), ref( tmpSeed[i] ),
                    100) ); 

    for ( auto & th : threads )
        th.join();

    set<Word> seed;
    int size = 100;
    for ( int i = 0; i < 64; i++ )
        if ( tmpSeed[i].size() < size )
        {
            seed = tmpSeed[i];
            size = tmpSeed[i].size();
        }

    cout << "Seed size: " << seed.size() << endl;

    frontIS0 = seed;
    frontIS1 = seed;
    frontIS2 = seed;
    frontIS3 = seed;

    set<Word> tmpSeed2[64];

    threads.clear();

    for ( int i = 0; i < 64; i++ )
        threads.push_back(  thread( getInvPatterns, ref( P ), ref( tmpSeed2[i] ), 50 ) ); 

    for ( auto & th : threads )
        th.join();

    size = 100;
    for ( int i = 0; i < 64; i++ )
        if ( tmpSeed2[i].size() < size )
        {
            seed = tmpSeed[i];
            size = tmpSeed[i].size();
        }

    cout << "Seed size: " << seed.size() << endl;

    backIS0 = seed;
    backIS1 = seed;
    backIS2 = seed;
    backIS3 = seed;

    getBadPattern( frontIS0, frontIS1, frontIS2, frontIS3, backIS0, backIS1, backIS2,
            backIS3, 
            badPattern );

    map<Word, set<Word> > MapFront0;
    InitializeMapFront( frontIS0, Invtable, MapFront0, P );

    map<Word, set<Word> > MapFront1;
    InitializeMapFront( frontIS1, Invtable, MapFront1, P );

    map<Word, set<Word> > MapFront2;
    InitializeMapFront( frontIS2, Invtable, MapFront2, P );

    map<Word, set<Word> > MapFront3;
    InitializeMapFront( frontIS3, Invtable, MapFront3, P );

    map<Word, set<Word> > MapBack0;
    InitializeMapBack( backIS0, table, MapBack0, P );

    map<Word, set<Word> > MapBack1;
    InitializeMapBack( backIS1, table, MapBack1, P );

    map<Word, set<Word> > MapBack2;
    InitializeMapBack( backIS2, table, MapBack2, P );

    map<Word, set<Word> > MapBack3;
    InitializeMapBack( backIS3, table, MapBack3, P );

    set< pair<NS,NS> > impossible;
    set< pair<NS,NS> > truncated;

    int n = 0;
    for ( auto it : badPattern )
    {
        cout << "===============================" << endl;
        cout << "The " << n << "-th bad pattern!" << endl;
        auto res = processBadPattern ( it, P, P, P, P, P, P, P , P , 
                MapFront0,
                MapFront1,
                MapFront2,
                MapFront3,
                MapBack0,
                MapBack1,
                MapBack2,
                MapBack3,
                impossible, truncated );
        n++;
        cout << endl;
    }

    cout << "Impossible pattern" << endl;

    for ( auto it : impossible )
    {
        printNibbleState( it.first );
        cout << " --/--> ";

        InvMC( it.second );
        InvShiftRows( it.second );
        printNibbleState( it.second );
        cout << endl;
    }

    cout << "truncated impossible differential pattern" << endl;

    for ( auto it : truncated )
    {
        printNibbleState( it.first );
        cout << " --/--> ";

        printNibbleState( it.second );
        cout << endl;
    }
}

