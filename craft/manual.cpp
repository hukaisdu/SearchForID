#include"gurobi_c++.h"
#include"forword.h"
#include"backword.h"
#include"skinny.h"
#include"basic.h"
#include"log.h"
#include"MILP.h"
#include"thread_pool.h"
#include"automatic.h"
#include<iostream>
#include<set>
#include<tuple>
#include<thread>
#include<future>
#include<cstdio>
#include<fstream>
#include<sstream>
#include<string>
#include<unistd.h>
#include<dirent.h>
#include<cstring>
#include<cmath>

using namespace std;
using namespace thread_pool;

const int TH = (1 << 28);

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

//0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0  --/--> 1 4 0 7 1 5 0 0 0 1 0 7 1 4 0 7 
    // process samples

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

        //ShiftRows( pattern.first );

        //InvMC( pattern.second );
        //InvShiftRows( pattern.second );

        //printS( pattern.first );
        //printS( pattern.second );
        //cout << endl;

        S.insert( pattern );
    }

    ifs.close();

    return S;
}

set<pair<NS, NS>> getTruncatedPatternFromSamples( string filename, pair<NS, NS>
        & manualPattern)
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
    getline( ifs, line ); // read in ****
    getline( ifs, line ); // read in ****
    cout << "Manual Pattern: " << endl
         << line << endl;

    int p[16], q[16];
    auto fp = line.substr(0, 32);

    ss << fp;
    for ( int i = 0; i < 16; i++ )
        ss >> hex >> p[i];

    auto sp = line.substr(40, 32 );

    ss << sp;

    for ( int i = 0; i < 16; i++ )
        ss >> hex >> q[i];

    for ( int i = 0; i < 16; i++ )
        manualPattern.first[i] = static_cast<Nibble> ( p[i] );
    for ( int i = 0; i < 16; i++ )
        manualPattern.second[i] = static_cast<Nibble> ( q[i] );


    getline( ifs, line ); // read in ****



//0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0  --/--> 1 4 0 7 1 5 0 0 0 1 0 7 1 4 0 7 
    // process samples

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

        ShiftRows( pattern.first );

        InvMC( pattern.second );
        InvShiftRows( pattern.second );

        //printS( pattern.first );
        //printS( pattern.second );
        //cout << endl;

        pair <NS, NS> truncated;
        for ( int i = 0; i < 16; i++ )
            if ( pattern.first[i] == 0 )
                truncated.first[i] = 0;
            else
                truncated.first[i] = 1;

        for ( int i = 0; i < 16; i++ )
            if ( pattern.second[i] == 0 )
                truncated.second[i] = 0;
            else
                truncated.second[i] = 1;
        

        S.insert( truncated );
    }

    ifs.close();

    return S;
}

void reduceStart( set<Word> & start0, set<Word> & start1, set<Word> & start2,
        set<Word>
        & start3, 
                  NS end,  
                  set< pair<BS, BS> > & impossible,
                  set< pair<NS, NS> > & truncated
                  )
{
    int n = 0;
    while ( start0.size() * start1.size() * start2.size() * start3.size() > 1 &&
            n< 1000 )
    {
        cout << "Start Size: "  << start0.size() * start1.size() * start2.size() * start3.size()
            << endl;
        n++;

        Word s0 = getRandom( start0 );
        Word s1 = getRandom( start1 );
        Word s2 = getRandom( start2 );
        Word s3 = getRandom( start3 );

        NS start; Word_to_State( s0, s1, s2, s3, start );

        InvShiftRows( start );
        BS plaintext;
        Nibble_to_Bit( start, plaintext );

        BS ciphertext;
        ShiftRows( end );
        MC ( end );
        Nibble_to_Bit( end, ciphertext );
        pair<NS, NS> inner;
        auto res = SKINNY_Single( ROUND, plaintext, ciphertext, inner.first,
                inner.second );
        if ( res == 1 ) // soluable
        {
            logger( "Solution Reduce" );

            auto t = inner.first;

            InvMC( t ); InvShiftRows( t );

            Word x0, x1, x2, x3;
            State_to_Word( t, x0, x1, x2, x3 );

            set<Word> S0, S1, S2, S3;
            InvPassSuperSbox( x0, Invtable, S0 );
            InvPassSuperSbox( x1, Invtable, S1 );
            InvPassSuperSbox( x2, Invtable, S2 );
            InvPassSuperSbox( x3, Invtable, S3 );

            ReduceSet( start0, S0 );
            ReduceSet( start1, S1 );
            ReduceSet( start2, S2 );
            ReduceSet( start3, S3 );
        }
    }
}

void reduceEnd(   NS start, 
        set<Word> & end0, set<Word> & end1, set<Word> & end2, set<Word> &
        end3,
                  set< pair<BS, BS> > & impossible,
                  set< pair<NS, NS> > & truncated
                  )
{
    logger ( "*********************************************" );
    logger (" reduce end" );
    int n = 0;
    while ( end0.size() * end1.size() * end2.size() * end3.size() > 1 && n <
            1000 )
    {
       cout << "End size: " << end0.size() * end1.size() * end2.size() *
           end3.size() << endl;
        n++;
        Word e0 = getRandom( end0 );
        Word e1 = getRandom( end1 );
        Word e2 = getRandom( end2 );
        Word e3 = getRandom( end3 );

        NS end; 
        Word_to_State( e0, e1, e2, e3, end );

        BS ciphertext; 

        ShiftRows( end );
        MC ( end );

        Nibble_to_Bit( end, ciphertext );

        BS plaintext;

        InvShiftRows( start );

        Nibble_to_Bit( start, plaintext );

        pair<NS, NS> inner;
        auto res = SKINNY_Single( ROUND, plaintext, ciphertext, inner.first,
                inner.second );

        if ( res == 1 )// soluable
        {
            logger( "Solution Reduce" );
            NS t = inner.second;
            ShiftRows( t );

            Word x0, x1, x2, x3;

            State_to_Word( t, x0, x1, x2, x3 );

            set<Word> S0, S1, S2, S3;
            PassSuperSbox( x0, table, S0 );
            PassSuperSbox( x1, table, S1 );
            PassSuperSbox( x2, table, S2 );
            PassSuperSbox( x3, table, S3 );

            ReduceSet( end0, S0 );
            ReduceSet( end1, S1 );
            ReduceSet( end2, S2 );
            ReduceSet( end3, S3 );
        }

        /*
        else  
        {
            logger( "impossible differential found !" );
            // check if it is a TID

            InvMC( end );
            InvShiftRows( end );

            pair < NS, NS > t;

            for ( int i = 0; i < 16; i++ )
                if ( start[i] == 0 )
                    t.first[i] = 0;
                else
                    t.first[i] = 1;

            for ( int i = 0; i < 16; i++ )
                if ( end[i] == 0 )
                    t.second[i] = 0;
                else
                    t.second[i] = 1;

            auto res = SKINNY_Truncated( ROUND, t.first, t.second );
            if ( res == 1 ) // soluable : a normal ID
            {
                impossible.insert( make_pair( plaintext, ciphertext ) );
                Word w0, w1, w2, w3;
                State_to_Word( end, w0, w1, w2, w3 );
                if ( w0 != 0 )
                    end0.erase( w0 );
                if ( w1 != 0 )
                    end1.erase( w1 );
                if ( w2 != 0 )
                    end2.erase( w2 );
                if ( w3 != 0 )
                    end3.erase( w3 );
            }
            else // ID
            {
                truncated.insert( t ); 
            }


        }
        */
    }
}

void findOneSolution( Word s0, Word s1, Word s2, Word s3, Word e0, Word e1, Word
        e2, Word e3, 
                      set<Word> & xs0, set<Word> & xs1, set<Word> & xs2,
                      set<Word> &
                      xs3,  
                      set<Word> & xe0, set<Word> & xe1, set<Word> & xe2,
                      set<Word> &
                      xe3, 
                      set<Word> & frontIS0, set<Word> & frontIS1, set<Word> &
                      frontIS2, set<Word> & frontIS3,
                      set<Word> & backIS0, set<Word> & backIS1, set<Word> &
                      backIS2, set<Word> & backIS3,
                      set< pair< BS, BS > > & impossible, 
                      set< pair< NS, NS > > & truncated
                      ) 
{
    pair < NS, NS > inner;
    pair < NS, NS > d;
    Word_to_State( s0, s1, s2, s3, d.first );
    Word_to_State( e0, e1, e2, e3, d.second );

    InvShiftRows( d.first ); 
    ShiftRows( d.second) ; 
    MC( d.second );

    BS p, c;

    printNibbleState( d.first );
    cout << " --?--> ";
    printNibbleState( d.second );
    cout << endl;
    Nibble_to_Bit( d.first, p ); Nibble_to_Bit( d.second, c );

    // MILP model

    auto res = SKINNY_Single( ROUND, p, c, inner.first, inner.second );

    cout << "Res = " << res << endl;

    if ( res == 1 )
    {
        ShiftRows( inner.second );

        // front
        Word w0, w1, w2, w3;
        State_to_Word( inner.first, w0, w1, w2, w3 );

        frontIS0.insert( w0 );
        frontIS1.insert( w1 );
        frontIS2.insert( w2 );
        frontIS3.insert( w3 );

        InvPassSuperSbox( w0, Invtable, xs0 );
        InvPassSuperSbox( w1, Invtable, xs1 );
        InvPassSuperSbox( w2, Invtable, xs2 );
        InvPassSuperSbox( w3, Invtable, xs3 );

        State_to_Word( inner.second, w0, w1, w2, w3 );

        PassSuperSbox( w0, table, xe0 );
        PassSuperSbox( w1, table, xe1 );
        PassSuperSbox( w2, table, xe2 );
        PassSuperSbox( w3, table, xe3 );

        backIS0.insert( w0 );
        backIS1.insert( w1 );
        backIS2.insert( w2 );
        backIS3.insert( w3 );

        cout << "Back sets are inserted" << endl;
    }
    else
    {
        logger( "truncated ID found" );
        // reduce the totalSet based on the ID
        pair< NS, NS > t;
        InvMC( d.second );
        InvShiftRows( d.second );
        for ( int i= 0;i < 16; i++ )
        {
            if ( d.second[i] == 0 )
                t.second[i] = 0;
            else
                t.second[i] = 1;

            if ( d.first[i] == 0 )
                t.first[i] = 0;
            else
                t.first[i] = 1;
        }
        truncated.insert(  t );
    }
}

void processBadPatternManual (  
                         const set<Word> & start0, 
                         const set<Word> & start1, 
                         const set<Word> & start2, 
                         const set<Word> & start3, 
                         const set<Word> & end0, 
                         const set<Word> & end1, 
                         const set<Word> & end2, 
                         const set<Word> & end3, 

                         set<pair<BS, BS> > & impossible, 
                         set<pair<NS, NS> > & truncated )
{
     set<Word>  frontIS0;
     set<Word>  frontIS1; 
     set<Word>  frontIS2; 
     set<Word>  frontIS3; 

     set<Word>  backIS0;
     set<Word>  backIS1;
     set<Word>  backIS2;
     set<Word>  backIS3;

    set<Word> xs0, xs1, xs2, xs3;
    set<Word> xe0, xe1, xe2, xe3;

    set<Word> S0 ( start0 );
    set<Word> S1 ( start1 );
    set<Word> S2 ( start2 );
    set<Word> S3 ( start3 );

    set<Word> E0 ( end0 );
    set<Word> E1 ( end1 );
    set<Word> E2 ( end2 );
    set<Word> E3 ( end3 );

    int cnt = 0;

    while ( true && cnt < 10000 )
    {
        auto s0 = getRandom ( S0 );
        auto s1 = getRandom ( S1 );
        auto s2 = getRandom ( S2 );
        auto s3 = getRandom ( S3 );

        auto e0 = getRandom ( E0 );
        auto e1 = getRandom ( E1 );
        auto e2 = getRandom ( E2 );
        auto e3 = getRandom ( E3 );

        // check if impossible
        pair< NS, NS > he; 
        Word_to_State( s0, s1, s2, s3, he.first );
        Word_to_State( e0, e1, e2, e3, he.second);

        InvShiftRows( he.first );

        if ( satSet( he, truncated ) )
        {
            cnt++;
            continue;
        }

        cout << "Random" << endl;
        cout << hex << s0 << " " 
             << s1 << " " 
             << s2 << " " 
             << s3 << " " 
             << e0 << " " 
             << e1 << " " 
             << e2 << " " 
             << e3 << dec << endl; 
        

        findOneSolution( s0, s1, s2, s3, e0, e1, e2, e3, 
                      xs0, xs1, xs2, xs3,  
                      xe0, xe1, xe2, xe3,
                      frontIS0, frontIS1, frontIS2, frontIS3,  
                      backIS0, backIS1, backIS2, backIS3, impossible, truncated
                      ); 

        cout << dec 
             << xs0.size() << " " 
             << xs1.size() << " " 
             << xs2.size() << " " 
             << xs3.size() << " " 
             << xe0.size() << " " 
             << xe1.size() << " " 
             << xe2.size() << " " 
             << xe3.size() << endl; 

        set<Word> tS0, tS1, tS2, tS3;
        set<Word> tE0, tE1, tE2, tE3;

        set_difference( S0.begin(), S0.end(), xs0.begin(), xs0.end(),
                  insert_iterator< set<Word> >( tS0, tS0.begin() ) );

        set_difference( S1.begin(), S1.end(), xs1.begin(), xs1.end(),
                  insert_iterator< set<Word> >( tS1, tS1.begin() ) );

        set_difference( S2.begin(), S2.end(), xs2.begin(), xs2.end(),
                  insert_iterator< set<Word> >( tS2, tS2.begin() ) );

        set_difference( S3.begin(), S3.end(), xs3.begin(), xs3.end(),
                  insert_iterator< set<Word> >( tS3, tS3.begin() ) );

        /*
        set_difference( E0.begin(), E0.end(), xe0.begin(), xe0.end(),
                  insert_iterator< set<Word> >( tE0, tE0.begin() ) );

        set_difference( E1.begin(), E1.end(), xe1.begin(), xe1.end(),
                  insert_iterator< set<Word> >( tE1, tE1.begin() ) );

        set_difference( E2.begin(), E2.end(), xe2.begin(), xe2.end(),
                  insert_iterator< set<Word> >( tE2, tE2.begin() ) );

        set_difference( E3.begin(), E3.end(), xe3.begin(), xe2.end(),
                  insert_iterator< set<Word> >( tE3, tE3.begin() ) );
                  */

        S0.swap( tS0 );
        S1.swap( tS1 );
        S2.swap( tS2 );
        S3.swap( tS3 );

        /*
        E0.swap( tE0 );
        E1.swap( tE1 );
        E2.swap( tE2 );
        E3.swap( tE3 );
        */

        if ( ( S0.size() == 0 )  && 
             ( S1.size() == 0 )  && 
             ( S2.size() == 0 )  && 
             ( S3.size() == 0 )  ) 
             //( E0.size() == 0 )  && 
             //( E1.size() == 0 )  && 
             //( E2.size() == 0 )  && 
             //( E3.size() == 0 ) )
            break;
    }

    xs0.clear();
    xs1.clear();
    xs2.clear();
    xs3.clear();

    xe0.clear();
    xe1.clear();
    xe2.clear();
    xe3.clear();

    set<Word> NS0 ( start0 );
    set<Word> NS1 ( start1 );
    set<Word> NS2 ( start2 );
    set<Word> NS3 ( start3 );

    set<Word> NE0 ( end0 );
    set<Word> NE1 ( end1 );
    set<Word> NE2 ( end2 );
    set<Word> NE3 ( end3 );

    cout << "Start cnt " << cnt << endl;

    cnt = 0;
    while ( true && cnt < 10000 )
    {
        auto s0 = getRandom ( NS0 );
        auto s1 = getRandom ( NS1 );
        auto s2 = getRandom ( NS2 );
        auto s3 = getRandom ( NS3 );

        auto e0 = getRandom ( NE0 );
        auto e1 = getRandom ( NE1 );
        auto e2 = getRandom ( NE2 );
        auto e3 = getRandom ( NE3 );

        pair< NS, NS > he; 
        Word_to_State( s0, s1, s2, s3, he.first );
        Word_to_State( e0, e1, e2, e3, he.second);

        InvShiftRows( he.first );

        if ( satSet( he, truncated ) )
        {
            cnt ++;
            continue;
        }

        cout << "Random" << endl;
        cout << hex << s0 << " " 
             << s1 << " " 
             << s2 << " " 
             << s3 << " " 
             << e0 << " " 
             << e1 << " " 
             << e2 << " " 
             << e3 << dec << endl; 

        findOneSolution( s0, s1, s2, s3, e0, e1, e2, e3, 
                      xs0, xs1, xs2, xs3,  
                      xe0, xe1, xe2, xe3,
                      frontIS0, frontIS1, frontIS2, frontIS3,  
                      backIS0, backIS1, backIS2, backIS3, impossible, truncated  
                      ); 

        cout << dec 
             << xs0.size() << " " 
             << xs1.size() << " " 
             << xs2.size() << " " 
             << xs3.size() << " " 
             << xe0.size() << " " 
             << xe1.size() << " " 
             << xe2.size() << " " 
             << xe3.size() << endl; 

        set<Word> tS0, tS1, tS2, tS3;
        set<Word> tE0, tE1, tE2, tE3;

        /*
        set_difference( S0.begin(), S0.end(), xs0.begin(), xs0.end(),
                  insert_iterator< set<Word> >( tS0, tS0.begin() ) );

        set_difference( S1.begin(), S1.end(), xs1.begin(), xs1.end(),
                  insert_iterator< set<Word> >( tS1, tS1.begin() ) );

        set_difference( S2.begin(), S2.end(), xs2.begin(), xs2.end(),
                  insert_iterator< set<Word> >( tS2, tS2.begin() ) );

        set_difference( S3.begin(), S3.end(), xs3.begin(), xs3.end(),
                  insert_iterator< set<Word> >( tS3, tS3.begin() ) );
                  */

        set_difference( NE0.begin(), NE0.end(), xe0.begin(), xe0.end(),
                  insert_iterator< set<Word> >( tE0, tE0.begin() ) );

        set_difference( NE1.begin(), NE1.end(), xe1.begin(), xe1.end(),
                  insert_iterator< set<Word> >( tE1, tE1.begin() ) );

        set_difference( NE2.begin(), NE2.end(), xe2.begin(), xe2.end(),
                  insert_iterator< set<Word> >( tE2, tE2.begin() ) );

        set_difference( NE3.begin(), NE3.end(), xe3.begin(), xe2.end(),
                  insert_iterator< set<Word> >( tE3, tE3.begin() ) );

        /*
        S0.swap( tS0 );
        S1.swap( tS1 );
        S2.swap( tS2 );
        S3.swap( tS3 );
        */

        NE0.swap( tE0 );
        NE1.swap( tE1 );
        NE2.swap( tE2 );
        NE3.swap( tE3 );

        //if ( ( S0.size() == 0 )  && 
        //     ( S1.size() == 0 )  && 
        //     ( S2.size() == 0 )  && 
        //     ( S3.size() == 0 )  ) 
        if ( ( NE0.size() == 0 )  && 
           ( NE1.size() == 0 )  && 
           ( NE2.size() == 0 )  && 
           ( NE3.size() == 0 ) )
            break;
    }
    cout << "End cnt " << cnt << endl;

    cout << "Proceed ... " << endl;
}

void removeTruncated (   const pair<NS, NS> & badPattern, 
                         const set<Word> & S0, 
                         const set<Word> & S1, 
                         const set<Word> & S2, 
                         const set<Word> & S3, 
                         const set<Word> & E0, 
                         const set<Word> & E1, 
                         const set<Word> & E2, 
                         const set<Word> & E3, 
                         const set<Word> & frontIS0, 
                         const set<Word> & frontIS1,
                         const set<Word> & frontIS2,
                         const set<Word> & frontIS3,
                         const set<Word> & backIS0,
                         const set<Word> & backIS1, 
                         const set<Word> & backIS2,
                         const set<Word> & backIS3,
                         set<pair<BS, BS> > & impossible, 
                         const set<pair<NS, NS> > & truncated )
{
    logger( "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" );


    // Stage 1: generate head and end
    logger( "Process the bad patterns..." ); 

    printNibbleState ( badPattern.first );

    cout << " --/--> " ;

    printNibbleState ( badPattern.second );

    cout << endl;

    NS x { badPattern.first };  // x : Sbox -> MC

    // x -> InvMC -> InvSR
    InvMC( x );
    InvShiftRows( x );

    Word w0, w1, w2, w3;

    State_to_Word( x, w0, w1, w2, w3 );

    auto start0 = findUnique_inner_to_front( w0, Invtable, frontIS0, S0 );
    auto start1 = findUnique_inner_to_front( w1, Invtable, frontIS1, S1 );
    auto start2 = findUnique_inner_to_front( w2, Invtable, frontIS2, S2 );
    auto start3 = findUnique_inner_to_front( w3, Invtable, frontIS3, S3 );

    cout << "start 0, 1, 2, 3 size " << endl;
    cout << dec << start0.size() <<  " " 
         << dec << start1.size() <<  " " 
         << dec << start2.size() <<  " " 
         << dec << start3.size() <<  endl;
    /*
    cout << "start0" << endl;
    for ( auto & jt : start0 ) 
        cout << hex << "0x" << int( jt ) << ", ";
    cout << endl;
    cout << "start1" << endl;
    for ( auto & jt : start1 ) 
        cout << hex << "0x" << int( jt ) << ", ";
    cout << endl;
    cout << "start2" << endl;
    for ( auto & jt : start2 ) 
        cout << hex << "0x" << int( jt ) << ", ";
    cout << endl;
    cout << "start3" << endl;
    for ( auto & jt : start3 ) 
        cout << hex << "0x" << int( jt ) << ", ";
    cout << endl;
    */

    // back

    NS y { badPattern.second };

    ShiftRows( y );
    Word m0, m1, m2, m3;
    State_to_Word( y, m0, m1, m2, m3 );

    auto end0 = findUnique_inner_to_back( m0, table, backIS0, E0 );
    auto end1 = findUnique_inner_to_back( m1, table, backIS1, E1 );
    auto end2 = findUnique_inner_to_back( m2, table, backIS2, E2 );
    auto end3 = findUnique_inner_to_back( m3, table, backIS3, E3 );

    cout << "end 0, 1, 2, 3 size " << endl;
    cout << dec << end0.size() <<  " " 
         << dec << end1.size() <<  " " 
         << dec << end2.size() <<  " " 
         << dec << end3.size() <<  endl;
    cout << "end0" << endl;

    /*
    for ( auto & jt : end0 ) 
        cout << hex << "0x" << int( jt ) << ", ";
    cout << endl;
    cout << "end1" << endl;

    for ( auto & jt : end1 ) 
        cout << hex << "0x"<< int( jt ) << ", ";
    cout << endl;
    cout << "end2" << endl;

    for ( auto & jt : end2 ) 
        cout << hex << "0x" << int( jt ) << ", ";
    cout << endl;
    cout << "end3" << endl;

    for ( auto & jt : end3 ) 
        cout << hex << "0x" << int( jt ) << ", ";
    cout << endl;
    */
    logger();

    set<NS> START;
    for ( auto it0 : start0 )
    for ( auto it1 : start1 )
    for ( auto it2 : start2 )
    for ( auto it3 : start3 )
    {
        NS xx;
        Word_to_State( it0, it1, it2, it3, xx );

        START.insert( xx );
    }

    set<NS> END;
    for ( auto it0 : end0 )
    for ( auto it1 : end1 )
    for ( auto it2 : end2 )
    for ( auto it3 : end3 )
    {
        NS xx;
        Word_to_State( it0, it1, it2, it3, xx );

        END.insert( xx );
    }

    set< pair< NS, NS >> totalSet;

    for ( auto it : START )
        for ( auto jt : END )
        {
            auto p = make_pair( it, jt );
            if ( satSet(p,  truncated ) ) 
                continue;

            totalSet.insert (  p );
        }

    cout << "Total Size " << dec << totalSet.size() << endl;
}

vector<string> getFiles(string cate_dir)
{
	vector<string> files;

	DIR *dir;
	struct dirent *ptr;
	char base[1000];

	if ((dir=opendir(cate_dir.c_str())) == NULL)
        {
		perror("Open dir error...");
                exit(1);
        }

	while ((ptr=readdir(dir)) != NULL)
	{
		if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
		        continue;
		else if(ptr->d_type == 8)    ///file
			files.push_back(ptr->d_name);
		else if(ptr->d_type == 10)    ///link file
			continue;
		else if(ptr->d_type == 4)    ///dir
			files.push_back(ptr->d_name);
	}
	closedir(dir);

	sort(files.begin(), files.end());
	return files;
}

/*
int main()
{
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

    Word tmp0[8] = { 0x0, 0x10, 0x6d9b, 0xb060, 0xcc0f, 0xd000, 0xe0ed, 0xee8d };
    Word tmp1[7] = { 0x0, 0xa0, 0x6000, 0xd4ed, 0xe0ee, 0xed0e, 0xf040 }; 
    Word tmp2[8] = { 0x0, 0x10, 0x23cc, 0xb0d0, 0xc000, 0xde0d, 0xe0dd, 0xee9d }; 
    Word tmp3[8] = { 0x0, 0x10, 0x1000, 0x6066, 0xcf0c, 0xd0d0, 0xddee, 0xfed7 }; 

    Word ttmp0[8] = { 0x0, 0x6, 0x7ee, 0xf00, 0xf03, 0x3ff0, 0x3ff3, 0x7cb9 }; 
    Word ttmp1[8] = { 0x0, 0xa, 0x500, 0xeee, 0xf0a, 0x3eea, 0x6330, 0xe7e7 }; 
    Word ttmp2[9] = { 0x0, 0x6, 0x663, 0xac5, 0xe00, 0xe05,  0x6f63, 0xa3a9,
        0xf660 }; 
    Word ttmp3[8] = { 0x0, 0x3, 0x300, 0x33f, 0xf0a, 0x3360, 0x3b67, 0x55fa }; 

    set<Word> frontIS0 ( tmp0, tmp0 + 8 );
    set<Word> frontIS1 ( tmp1, tmp1 + 7 );
    set<Word> frontIS2 ( tmp2, tmp2 + 8 );
    set<Word> frontIS3 ( tmp3, tmp3 + 8 );

    set<Word> backIS0 ( ttmp0, ttmp0 + 8 );
    set<Word> backIS1 ( ttmp1, ttmp1 + 8 );
    set<Word> backIS2 ( ttmp2, ttmp2 + 9 );
    set<Word> backIS3 ( ttmp3, ttmp3 + 8 );

        //string filename = "Data/Truncated/manual_impossible_0.txt";

    auto files = getFiles( "Data/Truncated/" );
    for ( auto it : files )
    {
        string filename = string( "Data/Truncated/" ) + it;
        cout << filename << endl;


        pair<NS, NS> manualPattern;
        auto S = getTruncatedPatternFromSamples( filename, manualPattern );

        //printS( manualPattern.first );

        //cout << endl;
        //printS( manualPattern.second );

        cout << "truncated" << endl;
        for ( auto & it : S )
        {
            printS( it.first );
            cout << endl;
            printS( it.second );
            cout << endl;
        }

        set< pair<BS, BS>> impossible;
        removeTruncated ( manualPattern, P, P, P, P, P, P, P, P, frontIS0, frontIS1,
                frontIS2, frontIS3, backIS0, backIS1, backIS2, backIS3,impossible, 
                             S );
    }
}
*/

