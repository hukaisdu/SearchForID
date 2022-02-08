#include"gurobi_c++.h"
#include"forword.h"
#include"backword.h"
#include"midori.h"
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
    ROUND = 8;
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

    set<Word> frontIS0, frontIS1, frontIS2, frontIS3, backIS0, backIS1, backIS2,
        backIS3;

    /*
    set<Word> tmpSeed0[64];

    vector<thread> threads;

    for ( int i = 0; i < 64; i++ )
        threads.push_back(  thread( getPatterns, ref( P ), ref( tmpSeed0[i] ),
                    1000 ) ); 

    for ( auto & th : threads )
        th.join();

    set<Word> seed;
    int size = 100;
    for ( int i = 0; i < 64; i++ )
        if ( tmpSeed0[i].size() < size )
        {
            seed = tmpSeed0[i];
            size = tmpSeed0[i].size();
        }

    cout << "Seed size: " << seed.size() << endl;
    for ( auto it : seed )
        cout << it << " ";
    cout << endl;

    //set<Word> seedh { 0, 1971, 2457, 8277, 13700, 14512, 15200, 26980, 34822, 44547, 45188, 57466, 59205, 59209, 62652 };

    set<Word> seed0 { 0 };

    frontIS0 = seed;
    frontIS1 = seed;
    frontIS2 = seed;
    frontIS3 = seed;

    cout << frontIS0.size() << " "
         << frontIS1.size() << " " 
         << frontIS2.size() << " " 
         << frontIS3.size() << endl; 

    set<Word> tmpSeed1[64];

    threads.clear();

    for ( int i = 0; i < 64; i++ )
        threads.push_back(  thread( getInvPatterns, ref( P ), ref( tmpSeed1[i]
                        ), 1000 ) ); 

    for ( auto & th : threads )
        th.join();

    size = 100;
    for ( int i = 0; i < 64; i++ )
        if ( tmpSeed1[i].size() < size )
        {
            seed = tmpSeed1[i];
            size = tmpSeed1[i].size();
        }

    cout << "Seed size: " << seed.size() << endl;
    for ( auto it : seed )
        cout << it << " ";
    cout << endl;
    */

    set<Word> seedf { 0, 827, 4353, 26336, 49507, 57446, 58982, 61395 }; 

    cout << "Seedf size: " << seedf.size() << endl;
    for ( auto it : seedf )
        cout << it << " ";
    cout << endl;

    set<Word> seedb { 0, 3822, 13747, 39177, 45115, 45872, 60465, 61166  };

    cout << "Seedb size: " << seedb.size() << endl;
    for ( auto it : seedb )
        cout << it << " ";
    cout << endl;

    set<Word> seed0 { 0 };

    frontIS0 = seedf;
    frontIS1 = seed0;
    frontIS2 = seed0;
    frontIS3 = seed0;

    backIS0 = seedb;
    backIS1 = seedb;
    backIS2 = seed0;
    backIS3 = seed0;


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

