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
#include<sstream>

using namespace std;
using namespace thread_pool;

int ROUND;

int ** table;
int ** Invtable;

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
    ROUND = 12;

    /*
    for ( int i = 0; i < 16; i++ ) 
        for ( int j = 0; j < 16; j++ )
            for ( int k = 0; k < 16; k++ )
        {
            NS a = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  
            NS b = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; 

            a[i] = 1; a[j] = 1;
            b[k] = 1;

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
        //= getBadPatternSamples( "part0.txt" );

    set<Word> frontIS0, frontIS1, frontIS2, frontIS3, backIS0, backIS1, backIS2,
        backIS3;

    set<Word> tmpSeed0[64];
    map<Word, set<Word>> Map0[64];
    map<Word, set<Word> > Mapf;

    vector<thread> threads;

    for ( int i = 0; i < 64; i++ )
        threads.push_back(  thread( getPatterns, ref( P ), ref( tmpSeed0[i] ),
                    100  ) ); 

    for ( auto & th : threads )
        th.join();

    set<Word> seedf;
    int size = 100;
    for ( int i = 0; i < 64; i++ )
        if ( tmpSeed0[i].size() < size )
        {
            seedf = tmpSeed0[i];
            Mapf = Map0[i];
            size = tmpSeed0[i].size();
        }

    cout << "Front Seed size: " << seedf.size() << endl;
    for ( auto it : seedf )
        cout << hex << int( it ) << " ";
    cout << endl;


    set<Word> seedb;
    set<Word> tmpSeed1[64];
    map<Word, set<Word>> Map1[64];
    map<Word, set<Word> > Mapb;

    threads.clear();

    for ( int i = 0; i < 64; i++ )
        threads.push_back(  thread( getInvPatterns, ref( P ), ref( tmpSeed1[i]
                        ), 100 ) ); 

    for ( auto & th : threads )
        th.join();

    size = 100;
    for ( int i = 0; i < 64; i++ )
        if ( tmpSeed1[i].size() < size )
        {
            seedb = tmpSeed1[i];
            Mapb = Map1[i];
            size = tmpSeed1[i].size();
        }

    cout << "Back Seed size: " << seedb.size() << endl;
    for ( auto it : seedb)
        cout << hex << int( it ) << " ";
    cout << endl;

    /*
    set<Word> seedf {0,  0x66e, 0x3330, 0x4404 ,0x6066, 0x6ee6 ,0xbace, 0xc1e3
    };
    set<Word> seedb {0,  0xee6, 0x3303, 0x33b0, 0x7cf4, 0x9099, 0xc31e, 0xe66e  };

    set<Word> seed0 { 0 };
    map<Word, set<Word>> Map0;
    Map0[0] = seed0;

    frontIS0 = seedf;
    frontIS1 = seed0;
    frontIS2 = seed0;
    frontIS3 = seed0;


    backIS0 = seedb;
    backIS1 = seed0;
    backIS2 = seed0;
    backIS3 = seed0;
    */

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

    /*
    set<Word> H0, H1, H2, H3, E0, E1, E2, E3;
    getBadWord( badPatterm, H0, H1, H2, H3, E0, E1, E2, E3 ); 

    map<Word, set<Word>> Mapf;

    InitializeMapFront( seedf, Invtable, Mapf, P, H0 );

    map<Word, set<Word>> Mapb;
    InitializeMapBack( seedb, table, Mapb, P, E0 );


    map<Word, set<Word> > MapFront0 = Mapf;
    map<Word, set<Word> > MapFront1 = Map0;
    map<Word, set<Word> > MapFront2 = Map0;
    map<Word, set<Word> > MapFront3 = Map0;

    map<Word, set<Word> > MapBack0 = Mapb;
    map<Word, set<Word> > MapBack1 = Map0;
    map<Word, set<Word> > MapBack2 = Map0;
    map<Word, set<Word> > MapBack3 = Map0;
    */

//    set< pair<NS,NS> > *impossible = new set<pair<NS, NS>> [badPattern.size() ]; 

    set< pair<NS,NS> > impossible;

    set< pair<NS,NS> > truncated;

    ThreadPool thread_pool{};
    vector<std::future<void>> futures;

    int n = 0;
    for ( auto it : badPattern )
    {
        cout << "===============================" << endl;
        cout << "The " << n << "-th bad pattern!" << endl;
        processBadPattern ( it, 
                MapFront0,
                MapFront1,
                MapFront2,
                MapFront3,
                MapBack0,
                MapBack1,
                MapBack2,
                MapBack3,
                impossible, truncated );

        /*
        futures.emplace_back( thread_pool.Submit( 
                processBadPattern,  
                ref( it ), 
                ref( MapFront0 ), 
                ref( MapFront1 ), 
                ref( MapFront2 ), 
                ref( MapFront3 ), 
                ref( MapBack0 ),  
                ref( MapBack1 ),  
                ref( MapBack2 ),  
                ref( MapBack3 ), 
                ref( impossible[n] ), 
                ref( truncated[n]  ) 
                ) );
        */
        n++;
    }

    for ( auto & it :  futures )
        it.get();


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

