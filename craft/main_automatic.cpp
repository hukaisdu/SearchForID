#include"gurobi_c++.h"
#include"forword.h"
#include"backword.h"
#include"craft.h"
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
const int THREAD = 64;

int ** table;
int ** Invtable;

int main()
{
    ROUND = 13;

    /*
    for ( int i = 0; i < 16; i++ ) 
        for ( int j = 0; j < 16; j++ )
        {
            NS a = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  
            NS b = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; 

            a[i] = 1; b[j] = 1;

            if ( SKINNY_Truncated( ROUND, a, b ) == 0 )
            {
                printS( a );
                cout << endl;
                cout << endl;
                printS( b );
                cout << endl;
            }

        }

    return 0;
    */
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
    set<Word> tmpSeed0[THREAD];

    vector<thread> threads;

    for ( int i = 0; i < THREAD; i++ )
        threads.push_back(  thread( getPatterns, ref( P ), ref( tmpSeed0[i] ),
                    1000 ) ); 

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

    cout << "Seed size: " << seed.size() << endl;
    for ( auto it : seed )
        cout << hex << int( it ) << " ";
    cout << endl;
    */

    set<Word> seedf { 0x0, 0x600, 0x3303, 0x5fe3, 0x6000, 0x9c00, 0xb0b0, 0xb430, 0xbd0c, 0xce3e }; 

    frontIS0 = seedf;
    frontIS1 = seedf;
    frontIS2 = seedf;
    frontIS3 = seedf;

    /*
    set<Word> tmpSeed1[THREAD];

    threads.clear();

    for ( int i = 0; i < THREAD; i++ )
        threads.push_back(  thread( getInvPatterns, ref( P ), ref( tmpSeed1[i]
                        ), 1000 ) ); 

    for ( auto & th : threads )
        th.join();

    size = 100;
    for ( int i = 0; i < THREAD; i++ )
        if ( tmpSeed1[i].size() < size )
        {
            seed = tmpSeed1[i];
            size = tmpSeed1[i].size();
        }

    cout << "Seed size: " << seed.size() << endl;
    for ( auto it : seed )
        cout << hex << int( it ) << " ";
    cout << endl;
    */

    set<Word> seedb { 0x0, 0xc00, 0x30b0, 0x3b0b, 0x594f, 0x6000, 0x9100, 0x9363, 0xb404, 0xb9b0 };

    backIS0 = seedb;
    backIS1 = seedb;
    backIS2 = seedb;
    backIS3 = seedb;

    getBadPatternX( frontIS0, frontIS1, frontIS2, frontIS3, backIS0, backIS1, backIS2,
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

