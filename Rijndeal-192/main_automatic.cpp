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
    ROUND = 6;

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

    set<Word> frontIS0, frontIS1, frontIS2, frontIS3, frontIS4, frontIS5,  
        backIS0, backIS1, backIS2, backIS3, backIS4, backIS5;

    frontIS0 = seed;
    frontIS1 = seed;
    frontIS2 = seed;
    frontIS3 = seed;
    frontIS4 = seed;
    frontIS5 = seed;

    seed.clear();
    getInvPatterns( P, seed ); 

    backIS0 = seed;
    backIS1 = seed;
    backIS2 = seed;
    backIS3 = seed;
    backIS4 = seed;
    backIS5 = seed;

    set<pair<NS, NS>> badPattern;

    getBadPattern( 
            frontIS0, frontIS1, frontIS2, frontIS3, frontIS4, frontIS5,
            backIS0, backIS1, backIS2, backIS3, backIS4, backIS5,
            badPattern );

    map<Word, set<Word> > MapFront0;
    InitializeMapFront( frontIS0,  MapFront0, P );

    map<Word, set<Word> > MapFront1;
    InitializeMapFront( frontIS1,  MapFront1, P );

    map<Word, set<Word> > MapFront2;
    InitializeMapFront( frontIS2, MapFront2, P );

    map<Word, set<Word> > MapFront3;
    InitializeMapFront( frontIS3, MapFront3, P );

    map<Word, set<Word> > MapFront4;
    InitializeMapFront( frontIS4, MapFront4, P );

    map<Word, set<Word> > MapFront5;
    InitializeMapFront( frontIS5, MapFront5, P );

    map<Word, set<Word> > MapBack0;
    InitializeMapBack( backIS0, MapBack0, P );

    map<Word, set<Word> > MapBack1;
    InitializeMapBack( backIS1, MapBack1, P );

    map<Word, set<Word> > MapBack2;
    InitializeMapBack( backIS2, MapBack2, P );

    map<Word, set<Word> > MapBack3;
    InitializeMapBack( backIS3, MapBack3, P );

    map<Word, set<Word> > MapBack4;
    InitializeMapBack( backIS4, MapBack4, P );

    map<Word, set<Word> > MapBack5;
    InitializeMapBack( backIS5, MapBack5, P );

    set< pair<NS,NS> > impossible;

    ThreadPool thread_pool{};

    vector<std::future<void>> futures;
    for ( auto it : badPattern )
    {
        futures.emplace_back( thread_pool.Submit( 
        processBadPattern, ref( it ),  
                ref( MapFront0 ),
                ref( MapFront1 ),
                ref( MapFront2 ),
                ref( MapFront3 ),
                ref( MapFront4 ),
                ref( MapFront5 ),
                ref( MapBack0 ),
                ref( MapBack1 ),
                ref( MapBack2 ),
                ref( MapBack3 ),
                ref( MapBack4 ),
                ref( MapBack5 ),
                ref( impossible ) ) );
    }

    for ( auto & it : futures )
        it.get();


    ofstream os;
    os.open( "file.txt", ios::out );

    os << "Impossible pattern" << endl;

    for ( auto it : impossible )
    {
        printS( it.first, os );
        os << endl;
        os << " --/--> ";

        os << endl;
        printS( it.second, os );
        os << endl;

        os << endl;
    }

}

