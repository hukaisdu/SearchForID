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

using namespace std;

const int SN = 1000;

int THREAD_NUMBER;
int ** table;
int ** Invtable;
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

    logger( "Initialized!" );


    Word PP[65536];
    for ( int i = 0; i < 65536; i++ )
        PP[i] = i;
    set<Word> P ( PP, PP + 65536 );

    Word ftmp0[12] = {
        0x0,0xb7,0x399,0x390f,0x50f7,0x7990,0x95f7,0x9750,0x9f09,0xd9aa,0xf0d5,0xfa35
            };
    Word ftmp1[12] = {
        0x0,0x5b7,0xf70,0x39ba,0x5075,0x5aa3,0x5f79,0x705f,0x790b,0xadd9,0xf750,0xfaa9
            };

    Word ftmp2[12] = {
        0x0,0xb7,0x37d,0x5b75,0x709f,0x799f,0x9079,0x9a03,0x9ddd,0xca29,0xf390,0xf705
            };

    Word ftmp3[11] = {
        0x0,0x583,0xf79,0x359f,0x390b,0x5000,0x79a0,0x903d,0x9d59,0xdb35,0xf090
    };

    Word btmp0[10] = {
        0x0,0x10,0xf0,0xd9c,0x7df7,0x9b99,0xcf7f,0xd7dd,0xe0f9,0xfb00 };
    Word btmp1[12] = {
        0x0,0xcd,0x552,0x6ac,0xecd,0x1e5b,0x700d,0xd000,0xd7d0,0xdfe5,0xed9f,0xfbd9
            };
    Word btmp2[10] = {
        0x0,0xf0,0xfec,0x690d,0x9dbf,0xb00d,0xd0d7,0xde50,0xeccd,0xfbf9 };

    Word btmp3[12] = {
        0x0,0xde,0xe0d,0xb9f9,0xbcdc,0xcfbd,0xcfc0,0xde0d,0xe0cd,0xe0d0,0xe77f,0xfdd5
            };

    set<Word> frontIS0 ( ftmp0, ftmp0 + 12 ); 
    set<Word> frontIS1 ( ftmp1, ftmp1 + 12 ); 
    set<Word> frontIS2 ( ftmp2, ftmp2 + 12 );
    set<Word> frontIS3 ( ftmp3, ftmp3 + 11 );

    set<Word> backIS0 ( btmp0, btmp0 + 10 ); 
    set<Word> backIS1 ( btmp1, btmp1 + 12 ); 
    set<Word> backIS2 ( btmp2, btmp2 + 10 ); 
    set<Word> backIS3 ( btmp3, btmp3 + 12 ); 

    map<Word, set<Word> > MapFront0;
    InitializeMapFront( frontIS0, Invtable, MapFront0, P );

    map<Word, set<Word> > MapFront1;
    InitializeMapFront( frontIS1, Invtable, MapFront1, P );

    map<Word, set<Word> > MapFront2;
    InitializeMapFront( frontIS2, Invtable, MapFront2, P );

    map<Word, set<Word> > MapFront3;
    InitializeMapFront( frontIS3, Invtable, MapFront3, P );

    map<Word, set<Word> > MapBack0;
    InitializeMapFront( backIS0, table, MapBack0, P );

    map<Word, set<Word> > MapBack1;
    InitializeMapFront( backIS1, table, MapBack1, P );

    map<Word, set<Word> > MapBack2;
    InitializeMapFront( backIS2, table, MapBack2, P );

    map<Word, set<Word> > MapBack3;
    InitializeMapFront( backIS3, table, MapBack3, P );

    set<pair<NS, NS>> badPattern = getBadPatternSamples( "badp.txt" );


    /*
    for ( auto it : Map )
    {
        cout << it.second.size() << endl;
    }
    */

    //set<pair<NS, NS>> badPattern ;

    /*
    set<Word> frontIS0 ; 
    set<Word> frontIS1 ; 
    set<Word> frontIS2 ;
    set<Word> frontIS3 ;

    set<Word> backIS0 ; 
    set<Word> backIS1 ;
    set<Word> backIS2 ; 
    set<Word> backIS3 ; 
    */

    //getBadPattern(  P, P, P, P, P, P, P , P , 
    //        frontIS0, frontIS1, frontIS2, frontIS3, backIS0, backIS1, backIS2,
    //        backIS3, 
    //        badPattern, 1 );

    set< pair<NS,NS> > impossible;

    set< pair<NS,NS> > truncated;
    set< pair<NS,NS> > pTruncated;
    set< pair<NS,NS> > cTruncated;

    int n = 0;

    set<NS> remaining;

    for ( auto it : badPattern  )
    {
        cout << "===============================" << endl;
        cout << "The " << n << "-th bad pattern!" << endl;
        auto res = processBadPattern ( it, P, P, P, P, P, P, P , P , 
                MapFront0, MapFront1, MapFront2, MapFront3, 
                MapBack0,  MapBack1,  MapBack2,  MapBack3, 
                impossible, truncated, pTruncated, cTruncated );
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
