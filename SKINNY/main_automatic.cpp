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

const int THREAD = 64;

int main()
{
    ROUND = 13;
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

    Word PP[65536];
    for ( int i = 0; i < 65536; i++ )
        PP[i] = i;
    set<Word> P ( PP, PP + 65536 );

    set<pair<NS, NS>> badPattern;

    // X10, X11, X12, X13, X20, X21, X22, X23
    set<Word> frontIS0, frontIS1, frontIS2, frontIS3, backIS0, backIS1, backIS2, backIS3;

    /*
    set<Word> tmpSeed0[THREAD];

    vector<thread> threads;

    for ( int i = 0; i < THREAD; i++ )
        threads.push_back(  thread( getPatterns, ref( P ), ref( tmpSeed0[i] ), 500 ) ); 

    for ( auto & th : threads )
        th.join();

    set<Word> seed;
    int size = 100;
    for ( int i = 0; i < THREAD; i++ )
        if ( tmpSeed0[i].size() < size )
        {
            seed = tmpSeed0[i];
            size = tmpSeed0[i].size();
        }

    cout << "Front seed size: " << seed.size() << endl;
    for ( auto it : seed )
        cout << hex << it << " ";
    cout << endl;
    */

    set<Word> seedf { 0x0, 0xd0, 0x6000, 0xb0a0, 0xd0de, 0xde9d, 0xee04 }; 
    set<Word> seedf0 { 0 };

    // X1
    frontIS0 = seedf;
    frontIS1 = seedf;
    frontIS2 = seedf;
    frontIS3 = seedf;

    /*
    set<Word> tmpSeed1[THREAD];

    threads.clear();

    for ( int i = 0; i < THREAD; i++ )
        threads.push_back(  thread( getInvPatterns, ref( P ), ref( tmpSeed1[i]
                        ), 500 ) ); 

    for ( auto & th : threads )
        th.join();

    size = 100;
    for ( int i = 0; i < THREAD; i++ )
        if ( tmpSeed1[i].size() < size )
        {
            seed = tmpSeed1[i];
            size = tmpSeed1[i].size();
        }

    cout << "Back seed size: " << seed.size() << endl;
    for ( auto it : seed )
        cout << hex << it << " ";
    cout << endl;
    */

    set<Word> seedb { 0x0, 0x4, 0x363, 0x40e, 0x600, 0xee76, 0xf3f0 }; 
    set<Word> seedb0 { 0 };

    backIS0 = seedb;
    backIS1 = seedb;
    backIS2 = seedb;
    backIS3 = seedb;

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

