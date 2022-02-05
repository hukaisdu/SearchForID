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
using namespace thread_pool;

int ROUND;


int ** table;
int ** Invtable;
int ** MATRIX;
int ** InvMATRIX;

int main()
{
    ROUND = 4;

    // generate DDT of the Sbox
    // 
    //
    int matrix[4][4] = { 
        { 1, 0, 1, 1 },
        { 1, 0, 0, 0 },
        { 0, 1, 1, 0 },
        { 1, 0, 1, 0 } };

    int invmatrix[4][4] = { 
        { 0, 1, 0, 0 },
        { 0, 1, 1, 1 },
        { 0, 1, 0, 1 },
        { 1, 0, 0, 1 } };
    
    MATRIX = new int*[4];
    for ( int i = 0; i < 4; i++ )
        MATRIX[i] = new int[4];

    for ( int i = 0; i < 4; i++ )
        for ( int j = 0; j < 4; j++ )
            MATRIX[i][j] = matrix[i][j];

    InvMATRIX = new int*[4];
    for ( int i = 0; i < 4; i++ )
        InvMATRIX[i] = new int[4];

    for ( int i = 0; i < 4; i++ )
        for ( int j = 0; j < 4; j++ )
            InvMATRIX[i][j] = invmatrix[i][j];

    Word PP[16];
    for ( int i = 0; i < 16; i++ )
    {
        Nibble i0 = i >> 3 & 0x1; 
        Nibble i1 = i >> 2 & 0x1; 
        Nibble i2 = i >> 1 & 0x1; 
        Nibble i3 = i >> 0 & 0x1; 

        PP[i] = Nibble_to_Word( i0, i1, i2, i3 );
    }
    set<Word> P ( PP, PP + 16 );

    set<Word> seed;
    getPatterns( P, seed );

    cout << "Seed size" << seed.size() << endl;

    for ( auto it : seed )
        cout << hex << int ( it ) << endl; 

    set<Word> frontIS0, frontIS1, frontIS2, frontIS3, backIS0, backIS1, backIS2,
        backIS3;

    set<Word> seedx;
    seedx.insert( 0 ) ;

    frontIS0 = seed;
    frontIS1 = seed;
    frontIS2 = seed;
    frontIS3 = seed;

    seed.clear();
    getInvPatterns( P, seed ); 

    backIS0 = seed;
    backIS1 = seed;
    backIS2 = seed;
    backIS3 = seed;

    for (auto it : frontIS0 )
        cout << it << endl;

    set<pair<NS, NS>> badPattern;

    getBadPattern( frontIS0, frontIS1, frontIS2, frontIS3, backIS0, backIS1, backIS2,
            backIS3, 
            badPattern );

    map<Word, set<Word> > MapFront0;
    InitializeMapFront( frontIS0,  MapFront0, P );

    map<Word, set<Word> > MapFront1;
    InitializeMapFront( frontIS1,  MapFront1, P );

    map<Word, set<Word> > MapFront2;
    InitializeMapFront( frontIS2, MapFront2, P );

    map<Word, set<Word> > MapFront3;
    InitializeMapFront( frontIS3, MapFront3, P );

    map<Word, set<Word> > MapBack0;
    InitializeMapBack( backIS0, MapBack0, P );

    map<Word, set<Word> > MapBack1;
    InitializeMapBack( backIS1, MapBack1, P );

    map<Word, set<Word> > MapBack2;
    InitializeMapBack( backIS2, MapBack2, P );

    map<Word, set<Word> > MapBack3;
    InitializeMapBack( backIS3, MapBack3, P );

    set< pair<NS,NS> > impossible;

    ThreadPool thread_pool{};


    vector<std::future<void>> futures;
    int n = 0;
    for ( auto it : badPattern )
    {
        futures.emplace_back( thread_pool.Submit( 
        processBadPattern, ref( it ),  
                ref( MapFront0 ),
                ref( MapFront1 ),
                ref( MapFront2 ),
                ref( MapFront3 ),
                ref( MapBack0 ),
                ref( MapBack1 ),
                ref( MapBack2 ),
                ref( MapBack3 ),
                ref( impossible ), n++ ) );

    }

    for ( auto & it : futures )
        it.get();


    ofstream os ;
    os.open( "file.txt", ios::out );
    os << "Impossible pattern" << endl;

    for ( auto it : impossible )
    {
        printNibbleState( it.first, os );
        os << " --/--> ";

        printNibbleState( it.second, os );
        os << endl;
    }

}

