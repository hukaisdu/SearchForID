#include"basic.h"
#include"MILP.h"
#include"forword.h"
#include"backword.h"
#include"gift.h"
#include"log.h"
#include"automatic.h"
#include<iostream>
#include<vector>
#include<set>
#include<thread>
#include<sstream>
#include"thread_pool.h"


using namespace std;
using namespace thread_pool;

const int SN = 1000;

int THREAD_NUMBER;
int ** table;
int ** Invtable;

map<Nibble, set<Nibble>> tableMap;

map<Nibble, set<Nibble>> InvtableMap;

int ROUND;

set<pair<NS, NS>> getBadPatternSamples( string filename )
{
    set< pair<NS, NS>> S;
    ifstream ifs;
    ifs.open( filename, ios::in );

    if ( ! ifs )
    {
        cerr << filename << " cannot be openned!" << endl;
        exit(-1);
    }
    stringstream ss;

    string line;

    while ( getline( ifs, line ) )
    {
        //cout << line << endl;
        pair< NS, NS > pattern = { {0}, {0} };
        int p[16], q[16];
        auto first = line.substr(0, 32);

        ss << first;
        for ( int i = 0; i < 16; i++ )
            ss >> hex >> p[i];

        auto second = line.substr(40, 32 );

        ss << second;
        for ( int i = 0; i < 16; i++ )
            ss >> hex >> q[i];

        for ( int i = 0; i < 16; i++ )
            pattern.first[i] = static_cast<Nibble> ( p[i] );
        for ( int i = 0; i < 16; i++ )
            pattern.second[i] = static_cast<Nibble> ( q[i] );

        S.insert( pattern );
    }

    ifs.close();

    return S;
}

int main()
{
    ROUND = 7;
    THREAD_NUMBER = 64;
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

    for ( int i = 0; i < 16; i++ )
        for ( int j = 0; j < 16; j++ )
            if ( table[i][j] > 0 )
                tableMap[i].insert( j );

    for ( int i = 0; i < 16; i++ )
        for ( int j = 0; j < 16; j++ )
            if ( Invtable[i][j] > 0 )
                InvtableMap[i].insert( j );

    InitializePerm();
    logger( "Initialized!" );

    Word PP[65536];
    for ( int i = 0; i < 65536; i++ )
        PP[i] = i;
    set<Word> P ( PP, PP + 65536 );

    set<pair<NS, NS>> badPattern = getBadPatternSamples( "badPattern.txt" );

    set<Word> frontIS0; 
    set<Word> frontIS1; 
    set<Word> frontIS2;
    set<Word> frontIS3;

    set<Word> backIS0; 
    set<Word> backIS1;
    set<Word> backIS2; 
    set<Word> backIS3; 

    /*
    set<Word> seed;
    vector<thread> threads;
    int size = 100;

    set<Word> tmpSeed0[64];

    seed.clear();
    threads.clear();

    for ( int i = 0; i < 64; i++ )
        threads.push_back(  thread( getForwardPattern, ref( P ), ref( tmpSeed0[i] ),
                    1000 ) ); 

    for ( auto & th : threads )
        th.join();

    for ( int i = 0; i < 64; i++ )
        if ( tmpSeed0[i].size() < size )
        {
            seed = tmpSeed0[i];
            size = tmpSeed0[i].size();
        }

    cout << "Seed size: " << seed.size() << endl;

    for ( auto it : seed )
        cout << hex << it << " ";
    cout << endl;

    set<Word> seedz;
    seedz.insert( 0 );

    frontIS0 = seed;
    frontIS1 = seedz;
    frontIS2 = seedz;
    frontIS3 = seedz;

    
    threads.clear();

    set<Word> tmpSeed1[64];
    for ( int i = 0; i < 64; i++ )
        threads.push_back(  thread( getBackwardPattern, ref( P ), ref( tmpSeed1[i]
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

    backIS0 = seed;
    backIS1 = seedz;
    backIS2 = seedz;
    backIS3 = seedz;

    for ( auto it : seed )
        cout << hex << it << " ";
    cout << endl;
    */

    set<Word> seedF { 0, 0x50b, 0xf39, 0x5a97, 0xa9d9, 0xb35f, 0xb3d0, 0xb706,
        0xd0b3, 0xd5f0  };

    set<Word> seedB { 0, 0xec, 0xd90, 0xe0f, 0x9b7b, 0xcd7e, 0xe00f, 0xe7cf,
        0xfdd7  };

    //set<Word> seed0 { 0 };

    frontIS0 = seedF;
    frontIS1 = seedF;
    frontIS2 = seedF;
    frontIS3 = seedF;

    backIS0 = seedB;
    backIS1 = seedB;
    backIS2 = seedB;
    backIS3 = seedB;

    /*
    getBadPatternX( frontIS0, frontIS1, frontIS2, frontIS3, backIS0, backIS1, backIS2,
            backIS3, 
            badPattern );
        */


    map<Word, set<Word> > MapFront0;
    InitializeMapFront( frontIS0, Invtable, MapFront0, P);

    map<Word, set<Word> > MapFront1;
    InitializeMapFront( frontIS1, Invtable, MapFront1, P );

    map<Word, set<Word> > MapFront2;
    InitializeMapFront( frontIS2, Invtable, MapFront2, P );

    map<Word, set<Word> > MapFront3;
    InitializeMapFront( frontIS3, Invtable, MapFront3, P );
//{'9b7b', '0000', '0d90', '00ec'//}
    map<Word, set<Word> > MapBack0;
    InitializeMapBack( backIS0, table, MapBack0, P );

    map<Word, set<Word> > MapBack1;
    InitializeMapBack( backIS1, table, MapBack1, P );

    map<Word, set<Word> > MapBack2;
    InitializeMapBack( backIS2, table, MapBack2, P );

    map<Word, set<Word> > MapBack3;
    InitializeMapBack( backIS3, table, MapBack3 , P);

    set< pair<NS,NS> > impossible;
    set< pair<NS,NS> > truncated;
    set< pair<NS,NS> > pTruncated;
    set< pair<NS,NS> > cTruncated;

    ThreadPool thread_pool{};

    vector<std::future<void>> futures;

    int n = 0;
    for ( auto it : badPattern )
    {
        /*
        cout << dec << n++ << "-th" << endl;

        processBadPattern ( it, MapFront0, MapFront1, MapFront2, MapFront3,
                MapBack0, MapBack1, MapBack2, MapBack3, impossible, truncated,
                truncated, cTruncated );  
        */

        futures.emplace_back( thread_pool.Submit( processBadPattern,  ref( it ), 
                ref( MapFront0 ), ref( MapFront1 ), ref ( MapFront2 ), ref(
                    MapFront3 ), 
                ref( MapBack0 ),  ref( MapBack1 ),  ref( MapBack2 ),  ref(
                    MapBack3 ), 
                ref( impossible ), 
                ref( truncated  ), 
                ref( pTruncated ), 
                ref( cTruncated ) ) );
    }
    for ( auto & th : futures )
        th.get();

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

    cout << "plaintext truncated impossible differential pattern" << endl;

    for ( auto it : pTruncated )
    {
        printNibbleState( it.first );
        cout << " --/--> ";
        printNibbleState( it.second );
        cout << endl;
    }

    cout << "ciphertext truncated impossible differential pattern" << endl;

    for ( auto it : cTruncated )
    {
        printNibbleState( it.first );
        cout << " --/--> ";
        printNibbleState( it.second );
        cout << endl;
    }

}
