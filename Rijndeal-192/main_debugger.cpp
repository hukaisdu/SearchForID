#include"gurobi_c++.h"
#include"automatic.h"
#include"forword.h"
#include"backword.h"
#include"skinny.h"
#include"basic.h"
#include"log.h"
#include<iostream>
#include<set>
#include"MILP.h"
#include<tuple>
#include<vector>
#include<thread>
#include<future>
#include"thread_pool.h"
#include<cstdio>
#include<cstdlib>
#include<sstream>

using namespace std;
using namespace thread_pool;

int ** table;
int ** Invtable;
int ROUND;

set<pair<NS, NS>> getBadPatternSamplesSet( string filename )
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

vector<pair<NS, NS>> getBadPatternSamples( string filename )
{
    vector< pair<NS, NS>> S;
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

        S.push_back( pattern );
    }

    ifs.close();

    return S;
}

bool WordSat( Word x, const NS & a, int n )
{
    Nibble W[4];

    W[0] = a[n];
    W[1] = a[n + 4];
    W[2] = a[n + 8];
    W[3] = a[n + 12];

    Nibble M[4];
    Word_to_Nibble( x, M[0], M[1], M[2], M[3] );

    for ( int i = 0; i < 4; i++ )
        if ( ( ( W[i] == 0 ) && ( M[i] != 0 ) )  || 
             ( ( W[i] != 0 ) && ( M[i] == 0 ) )  ) 
            return false;

    return true;
}


int main( int argc, char ** argv )
{
    ROUND = 11;

    NS a = { 0, 0, 0 ,0 ,0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 1 };
    NS b = { 0, 0, 0 ,0 ,0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 } ;

    ShiftRows( b );
    printS ( b );
    cout << endl;
    MC( b );
    printS ( b );
        cout << endl;

    cout << SKINNY_Truncated( 11, a, b ) << endl;
    return 0;


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

    /*
    Word tmp0[7] = { 0x0, 0xc0, 0x10d0, 0xb000, 0xc0cc, 0xcf0f, 0xd4ed };
    Word tmp1[7] = { 0x0, 0xf0, 0x4000, 0x4080, 0xdedd, 0xe0de, 0xed0d }; 
    Word tmp2[8] = { 0x0, 0xb0, 0x9010, 0x9883, 0xc0cf, 0xcf8c, 0xe000, 0xe40d }; 
    Word tmp3[8] = { 0x0, 0x60, 0x3ecd, 0xb000, 0xd010, 0xde0d, 0xe0fe, 0xfbff }; 

    Word ttmp0[8] = { 0x0, 0xa, 0x3ff, 0xa00, 0xf0e, 0xcc35, 0xf330, 0xf337 }; 
    Word ttmp1[8] = { 0x0, 0x4, 0x300, 0x40f, 0xee7, 0x3360, 0xd336, 0xfe66 }; 
    Word ttmp2[8] = { 0x0, 0xf, 0x404, 0xf00, 0xf3f, 0x3360, 0xaee6, 0xfe67 };
    Word ttmp3[8] = { 0x0, 0x7, 0x603, 0xa00, 0xee7, 0x6f60, 0xcc31, 0xffff }; 
    */


    Word tmp0[8] = { 0x0, 0x60, 0x9000, 0xb080, 0xc0cf, 0xccd2, 0xee04, 0xeecd
    };
    Word tmp1[8] = { 0x0, 0x60, 0x30b0, 0xab17, 0xd0ee, 0xee0f, 0xeefe, 0xf000
    }; 
    Word tmp2[8] = { 0x0, 0x60, 0xa8ad, 0xc000, 0xc0e0, 0xcf0f, 0xe0fe, 0xeffe
    };
    Word tmp3[8] = { 0x0, 0x90, 0x1000, 0xd0a0, 0xd0dd, 0xd404, 0xeeed, 0xf93c
    }; 

    Word ttmp0[8] = { 0x0, 0xe, 0x400, 0x6ff, 0xf06, 0x3ff0, 0x4a7d, 0xf336
        };
    Word ttmp1[7] = { 0x0, 0xe, 0x400, 0xa03, 0xeee, 0x6ff3, 0xff30 }; 
    Word ttmp2[8] = { 0x0, 0xf, 0x600, 0x603, 0xee7, 0x3e6f, 0x56f4, 0x77e0
        };
    Word ttmp3[8] = { 0x0, 0x6, 0x505, 0xa00, 0xf3f, 0x7ee0, 0xbd7a, 0xff63
        };

    set<Word> frontIS0 ( tmp0, tmp0 + 8 );
    set<Word> frontIS1 ( tmp1, tmp1 + 8 );
    set<Word> frontIS2 ( tmp2, tmp2 + 8 );
    set<Word> frontIS3 ( tmp3, tmp3 + 8 );

    set<Word> backIS0 ( ttmp0, ttmp0 + 8 );
    set<Word> backIS1 ( ttmp1, ttmp1 + 7 );
    set<Word> backIS2 ( ttmp2, ttmp2 + 8 );
    set<Word> backIS3 ( ttmp3, ttmp3 + 8 );

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

    auto badPattern1 = getBadPatternSamples( "badpattern.txt" );

    set<pair<NS, NS>> badPattern;

    badPattern.insert( badPattern1[199] );

    set<pair<NS, NS>> impossible, truncated;

    int n = 0;
    for ( auto it : badPattern )
    {
        cout << "===============================" << endl;
        cout << "The " << dec << n << "-th bad pattern!" << endl;

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

        cout << endl;
        n++;
    }

    ofstream os;
    os.open( "RESULT/res_11_round_de.txt", ios::app );

    os << "Impossible pattern" << endl;

    for ( auto it : impossible )
    {
        printNibbleState( it.first, os );
        os << " --/--> ";

        InvMC( it.second );
        InvShiftRows( it.second );

        printNibbleState( it.second, os );
        os << endl;
    }

    os << "Truncated impossible differential pattern" << endl;

    for ( auto it : truncated )
    {
        printNibbleState( it.first, os );
        os << " --/--> ";

        printNibbleState( it.second, os );
        os << endl;
    }

    os.close();
}


