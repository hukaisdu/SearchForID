#include"gurobi_c++.h"
#include"forword.h"
#include"backword.h"
#include"gift.h"
#include"basic.h"
#include"log.h"
#include<iostream>
#include<set>
#include"MILP.h"
#include<tuple>
#include<thread>
#include<future>
#include"thread_pool.h"
#include<cstdio>
#include<cmath>
#include"automatic.h"
#include<mutex>


using namespace std;
using namespace thread_pool;

mutex mtx; // 保护counter

#define COM(A, B) (A##B)

const int DIRECT = ( 1L << 28 );

set<pair<NS, NS>> BAD;

void Worker( const set<NS> & Head,  const set<NS> & Tail, set< pair<NS, NS> > & Bad )
{
    for ( auto it : Head )
       GIFT_Inner_Round( ROUND - 4, it, Tail, Bad );
}

void Worker_getIS_front( set<Word> & frontIS0, int ** table, int ** Invtable, const set<Word> & S0,
        int SN ) 
{
    frontIS0 = getIS_front( table, Invtable, S0, SN );
    NarrowSet_inner_to_front( frontIS0,Invtable, S0 ); // Forword
}

void Worker_getIS_back( set<Word> & backIS0, int ** table, int ** Invtable,
        const set<Word> & E0,
        int SN ) 
{
    backIS0 = getIS_back( table, Invtable, E0, SN );
    NarrowSet_inner_to_back( backIS0, Invtable, E0 ); // Forword
}

void getBadPatternX( const set<Word> & frontIS0,
                     const set<Word> & frontIS1,
                     const set<Word> & frontIS2,
                     const set<Word> & frontIS3,

                     const set<Word> & backIS0,
                     const set<Word> & backIS1,
                     const set<Word> & backIS2,
                     const set<Word> & backIS3,

                     set< pair<NS,NS> > & badPattern
                     )
{
    set<NS> Head, Tail; // sort by the addresses
    // word to state
    for ( auto & it0 : frontIS0 )
    for ( auto & it1 : frontIS1 )
    for ( auto & it2 : frontIS2 )
    for ( auto & it3 : frontIS3 )
    {
        if ( ( it0 == 0 ) && ( it1 == 0 ) && ( it2 == 0 ) && ( it3 == 0 ) )
            continue;

        NS x;

        Word_to_State( it0, it1, it2, it3, x );   

        Head.insert( x );
    }

    for ( auto & it0 : backIS0 )
    for ( auto & it1 : backIS1 )
    for ( auto & it2 : backIS2 )
    for ( auto & it3 : backIS3 )
    {
        if ( ( it0 == 0 ) && ( it1 == 0 ) && ( it2 == 0 ) && ( it3 == 0 ) )
            continue;

        NS x;
        Word_to_State( it0, it1, it2, it3, x );   
        Tail.insert( x );
    }

    logger( string ( " Head and Tail constructed!\n" ) + 
            string ( " Head size " ) + to_string( Head.size() ) + string( "\n" ) +
            string ( " Tail size " ) + to_string( Tail.size() ) + string( "\n" )  );  


    cout << "Head" << endl;
    for ( auto & itt : Head )
    {
        printNibbleState( itt );
        cout << endl;
    }
    cout << "Tail" << endl;
    for ( auto & itt : Tail )
    {
        printNibbleState( itt );
        cout << endl;
    }

    logger();
    // stage 4: construct the main bridge: MILP models
    logger( "Stage 4: Main bridge Constructing: MILP models" ); 

    int content = Head.size() / THREAD_NUMBER;

    if ( Head.size() % THREAD_NUMBER > 0 )
        content++;

    vector< set<NS> > smallHead ( THREAD_NUMBER );

    int n = 0;
    for ( auto itt : Head )
    {
        smallHead[n / content].insert( itt );
        n++;
    }

    vector< set< pair<NS, NS> > > Bad( THREAD_NUMBER );

    vector<thread> threads;

    for ( int i = 0; i < THREAD_NUMBER; i++ )
        threads.push_back( thread ( Worker, ref( smallHead[i] ), ref ( Tail ), ref( Bad[i] ) ) );

    for ( auto & th : threads ) 
        th.join();

    set< pair<NS, NS> > totalBad;
    for ( int i = 0; i < THREAD_NUMBER; i++ )
        for ( auto itt : Bad[i] )
            totalBad.insert( itt );

    logger( string( "MILP models are all solved!\n" ) + 
            string( "Bad size " ) + to_string( totalBad.size() ) );

    cout << "TotalBad" << endl;

    for ( auto & it : totalBad ) 
    {
        printNibbleState ( it.first );
        cout << " --/--> " ;

        printNibbleState ( it.second );
        cout << endl;

        badPattern.insert( it );
    }

    logger();
}

void getBadPattern(  const set<Word> & S0, 
                     const set<Word> & S1, 
                     const set<Word> & S2, 
                     const set<Word> & S3, 
                     const set<Word> & E0, 
                     const set<Word> & E1, 
                     const set<Word> & E2, 
                     const set<Word> & E3, 

                     set<Word> & frontIS0,
                     set<Word> & frontIS1,
                     set<Word> & frontIS2,
                     set<Word> & frontIS3,

                     set<Word> & backIS0,
                     set<Word> & backIS1,
                     set<Word> & backIS2,
                     set<Word> & backIS3,

                     set< pair<NS,NS> > & badPattern, int SN
                     )
{
    logger( "Stage 1: Front Approach Bridge Constructing..." );

    vector<thread> thread1s;

    thread1s.push_back( thread( Worker_getIS_front, ref( frontIS0 ), table, Invtable, ref( S0 ), SN ) );
    thread1s.push_back( thread( Worker_getIS_front, ref( frontIS1 ), table, Invtable, ref( S1 ), SN ) );
    thread1s.push_back( thread( Worker_getIS_front, ref( frontIS2 ), table, Invtable, ref( S2 ), SN ) );
    thread1s.push_back( thread( Worker_getIS_front, ref( frontIS3 ), table, Invtable, ref( S3 ), SN ) );

    thread1s.push_back( thread( Worker_getIS_back, ref( backIS0 ), table, Invtable, ref( E0 ), SN ) );
    thread1s.push_back( thread( Worker_getIS_back, ref( backIS1 ), table, Invtable, ref( E1 ), SN ) );
    thread1s.push_back( thread( Worker_getIS_back, ref( backIS2 ), table, Invtable, ref( E2 ), SN ) );
    thread1s.push_back( thread( Worker_getIS_back, ref( backIS3 ), table, Invtable, ref( E3 ), SN ) );

    for ( auto & th : thread1s ) 
        th.join();

    logger( string( "The front approach bridge constructed!\n" )
            + string( " frontIS0 size : " ) + to_string( frontIS0.size() ) + string( "\n" )
            + string( " frontIS1 size : " ) + to_string( frontIS1.size() ) + string( "\n" ) 
            + string( " frontIS2 size : " ) + to_string( frontIS2.size() ) + string( "\n" ) 
            + string( " frontIS3 size : " ) + to_string( frontIS3.size() ) + string( "\n" ) );

    logger( "fornt IS0" );
    for ( auto & it : frontIS0 )
        cout << hex << int( it ) << " ";
    cout << dec << endl;
    logger( "fornt IS1" );
    for ( auto & it : frontIS1 )
        cout << hex << int( it ) << " ";
    cout << dec << endl;
    logger( "fornt IS2" );
    for ( auto & it : frontIS2 )
        cout << hex << int( it ) << " ";
    cout << dec << endl;
    logger( "fornt IS3" );
    for ( auto & it : frontIS3 )
        cout << hex << int( it ) << " ";
    cout << dec << endl;

    logger();

    logger( string( "The back approach bridge constructed!\n" )
            + string( " backIS0 size : " ) + to_string( backIS0.size() ) + string( "\n" )
            + string( " backIS1 size : " ) + to_string( backIS1.size() ) + string( "\n" ) 
            + string( " backIS2 size : " ) + to_string( backIS2.size() ) + string( "\n" ) 
            + string( " backIS3 size : " ) + to_string( backIS3.size() ) + string( "\n" ) );

    logger( "back IS0" );
    for ( auto & it : backIS0 )
        cout << hex << int( it ) << " ";
    cout << dec << endl;
    logger( "back IS1" );
    for ( auto & it : backIS1 )
        cout << hex << int( it ) << " ";
    cout << dec << endl;
    logger( "back IS2" );
    for ( auto & it : backIS2 )
        cout << hex << int( it ) << " ";
    cout << dec << endl;
    logger( "back IS3" );
    for ( auto & it : backIS3 )
        cout << hex << int( it ) << " ";
    cout << dec << endl;

    logger();


    // stage 3: obtain the head and tail of the MILP model
    logger( "Stage 3: Get the Head and Tail of the MILP models" );

    set<NS> Head, Tail; // sort by the addresses
    // word to state
    for ( auto & it0 : frontIS0 )
    for ( auto & it1 : frontIS1 )
    for ( auto & it2 : frontIS2 )
    for ( auto & it3 : frontIS3 )
    {
        if ( ( it0 == 0 ) && ( it1 == 0 ) && ( it2 == 0 ) && ( it3 == 0 ) )
            continue;

        NS x;

        Word_to_State( it0, it1, it2, it3, x );   

        Head.insert( x );
    }

    for ( auto & it0 : backIS0 )
    for ( auto & it1 : backIS1 )
    for ( auto & it2 : backIS2 )
    for ( auto & it3 : backIS3 )
    {
        if ( ( it0 == 0 ) && ( it1 == 0 ) && ( it2 == 0 ) && ( it3 == 0 ) )
            continue;

        NS x;
        Word_to_State( it0, it1, it2, it3, x );   
        Tail.insert( x );
    }

    logger( string ( " Head and Tail constructed!\n" ) + 
            string ( " Head size " ) + to_string( Head.size() ) + string( "\n" ) +
            string ( " Tail size " ) + to_string( Tail.size() ) + string( "\n" )  );  

    cout << "Head" << endl;
    for ( auto & itt : Head )
    {
        printNibbleState( itt );
        cout << endl;
    }
    cout << "Tail" << endl;
    for ( auto & itt : Tail )
    {
        printNibbleState( itt );
        cout << endl;
    }

    logger();
    // stage 4: construct the main bridge: MILP models
    logger( "Stage 4: Main bridge Constructing: MILP models" ); 

    int content = Head.size() / THREAD_NUMBER;

    if ( Head.size() % THREAD_NUMBER > 0 )
        content++;

    vector< set<NS> > smallHead ( THREAD_NUMBER );

    int n = 0;
    for ( auto itt : Head )
    {
        smallHead[n / content].insert( itt );
        n++;
    }

    vector< set< pair<NS, NS> > > Bad( THREAD_NUMBER );

    vector<thread> threads;

    for ( int i = 0; i < THREAD_NUMBER; i++ )
        threads.push_back( thread ( Worker, ref( smallHead[i] ), ref ( Tail ), ref( Bad[i] ) ) );

    for ( auto & th : threads ) 
        th.join();

    set< pair<NS, NS> > totalBad;
    for ( int i = 0; i < THREAD_NUMBER; i++ )
        for ( auto itt : Bad[i] )
            totalBad.insert( itt );

    logger( string( "MILP models are all solved!\n" ) + 
            string( "Bad size " ) + to_string( totalBad.size() ) );

    cout << "TotalBad" << endl;

    for ( auto & it : totalBad ) 
    {
        printNibbleState ( it.first );
        cout << " --/--> " ;

        printNibbleState ( it.second );
        cout << endl;

        badPattern.insert( it );
    }

    logger();
}

void Worker_findUnique_front( set<Word> & start0, Word w0, int ** Invtable,
        const set<Word> & frontIS0, const set<Word> & S0 )
{
    start0 = findUnique_inner_to_front( w0, Invtable, frontIS0, S0 );
}

void Worker_findUnique_back( set<Word> & start0, Word w0, int ** Invtable,
        const set<Word> & frontIS0, const set<Word> & S0 )
{
    start0 = findUnique_inner_to_back( w0, Invtable, frontIS0, S0 );
}

void processBadPattern ( pair<NS, NS> & badPattern, 
                         const map<Word, set<Word>> & frontMapIS0, 
                         const map<Word, set<Word>> & frontMapIS1, 
                         const map<Word, set<Word>> & frontMapIS2, 
                         const map<Word, set<Word>> & frontMapIS3, 
                         const map<Word, set<Word>> & backMapIS0, 
                         const map<Word, set<Word>> & backMapIS1, 
                         const map<Word, set<Word>> & backMapIS2, 
                         const map<Word, set<Word>> & backMapIS3, 
                         set<pair<NS, NS> > & impossible1, 
                         set<pair<NS, NS> > & truncated1,
                         set<pair<NS, NS> > & pTruncated1,
                         set<pair<NS, NS> > & cTruncated1 )
{
     set<pair<NS, NS> > impossible;
     set<pair<NS, NS> > truncated;
     set<pair<NS, NS> > pTruncated;
     set<pair<NS, NS> > cTruncated;

    logger( "*******************************************" );

    // Stage 1: generate head and end
    logger( "Process the bad patterns..." ); 

    printNibbleState ( badPattern.first );

    cout << " --/--> " ;

    printNibbleState ( badPattern.second );

    cout << endl;

    NS x { badPattern.first };  // x : Sbox -> MC

    Word w0, w1, w2, w3;

    State_to_Word( x, w0, w1, w2, w3 );


    NS y { badPattern.second };

    Word m0, m1, m2, m3;
    State_to_Word( y, m0, m1, m2, m3 );

    set<Word> start0, start1, start2, start3;
    set<Word> end0, end1, end2, end3;

    start0 = frontMapIS0.at( w0 ) ;
    start1 = frontMapIS1.at( w1 );
    start2 = frontMapIS2.at( w2 );
    start3 = frontMapIS3.at( w3 );

    end0 = backMapIS0.at( m0 );
    end1 = backMapIS1.at( m1 );
    end2 = backMapIS2.at( m2 );
    end3 = backMapIS3.at( m3 );

    logger();

    cout << "TotalSize: " << dec << start0.size() << " * "  
                          << start1.size() << " * "  
                          << start2.size() << " * "  
                          << start3.size() << " * "  
                          << end0.size() << " * "  
                          << end1.size() << " * "  
                          << end2.size() << " * "  
                          << end3.size() << " = "  
                          << start0.size() * start1.size() * start2.size() *
                          start3.size() * end0.size() * end1.size() *
                          end2.size() * end3.size() << " = 2^"  
                          << log( start0.size() * start1.size() * start2.size() *
                          start3.size() * end0.size() * end1.size() *
                          end2.size() * end3.size() ) / log(2) << endl; 

    if( start0.size() * start1.size() * start2.size() *
        start3.size() * end0.size() * end1.size() *
        end2.size() * end3.size() == 0 ) // the empty set
    {
        cerr <<  "The size is 0 " << endl;// TODO
        exit(-1);
    }

    if( start0.size() * start1.size() * start2.size() *
      start3.size() * end0.size() * end1.size() *
      end2.size() * end3.size() >= (1L << 30 )  )
    {
        logger( "*********************************" );
        logger( "Simplify...." );

        int n = 0;
        while ( true && n < 20 )
        {
            set< pair<NS, NS> >  bad; 

            set<Word> f0,f1,f2,f3, b0,b1,b2,b3;

            getBadPattern( start0, start1, start2, start3, end0, end1, end2, end3,  
                        f0, f1, f2, f3, b0, b1, b2, b3, bad, 1 );


            getIntersectionOfBadPattern( bad, start0, start1, start2, start3,
                    end0, end1, end2, end3);

            cout << "start 0, 1, 2, 3 size " << endl;
            cout << dec << start0.size() <<  " " 
                 << dec << start1.size() <<  " " 
                 << dec << start2.size() <<  " " 
                 << dec << start3.size() <<  endl;

            cout << "end 0, 1, 2, 3 size " << endl;
            cout << dec << end0.size() <<  " " 
                 << dec << end1.size() <<  " " 
                 << dec << end2.size() <<  " " 
                 << dec << end3.size() <<  endl;

            if ( start0.empty() ||  
                 start1.empty() || 
                 start2.empty() || 
                 start3.empty() || 
                 end0.empty() || 
                 end1.empty() || 
                 end2.empty() || 
                 end3.empty() )
                return;

            if( start0.size() * start1.size() * start2.size() *
              start3.size() * end0.size() * end1.size() *
              end2.size() * end3.size() < DIRECT  )
                break;

            n++;
        }
    }

    else if( start0.size() * start1.size() * start2.size() *
      start3.size() * end0.size() * end1.size() *
      end2.size() * end3.size() >= DIRECT  )
    {
        logger( "*********************************" );
        logger( "Simplify...." );

        set< pair<NS, NS> >  bad; 

        set<Word> f0,f1,f2,f3, b0,b1,b2,b3;

        getForwardPattern( start0, f0, 1 );
        getForwardPattern( start1, f1, 1 );
        getForwardPattern( start2, f2, 1 );
        getForwardPattern( start3, f3, 1 );

        getBackwardPattern( end0, b0, 1 );
        getBackwardPattern( end1, b1, 1 );
        getBackwardPattern( end2, b2, 1 );
        getBackwardPattern( end3, b3, 1 );


        map<Word, set<Word> > xMapFront0;
        InitializeMapFront( f0, Invtable, xMapFront0, start0 );

        map<Word, set<Word> > xMapFront1;
        InitializeMapFront( f1, Invtable, xMapFront1, start1 );

        map<Word, set<Word> > xMapFront2;
        InitializeMapFront( f2, Invtable, xMapFront2, start2 );

        map<Word, set<Word> > xMapFront3;
        InitializeMapFront( f3, Invtable, xMapFront3, start3 );

        map<Word, set<Word> > xMapBack0;
        InitializeMapBack( b0, table, xMapBack0, end0 );

        map<Word, set<Word> > xMapBack1;
        InitializeMapBack( b1, table, xMapBack1, end1 );

        map<Word, set<Word> > xMapBack2;
        InitializeMapBack( b2, table, xMapBack2, end2 );

        map<Word, set<Word> > xMapBack3;
        InitializeMapBack( b3, table, xMapBack3, end3 );

        getBadPatternX( f0, f1, f2, f3, b0, b1, b2, b3, bad);

        cout << "Bad " << dec << bad.size() << endl;
        int n = 0;
        for ( auto it : bad )
        {
            cout << "===============================" << endl;

            cout << "The " << dec << n << "-th bad pattern!" << endl;


            processBadPattern ( it, 
                    xMapFront0,
                    xMapFront1,
                    xMapFront2,
                    xMapFront3,
                    xMapBack0,
                    xMapBack1,
                    xMapBack2,
                    xMapBack3,
                    impossible, truncated, pTruncated, cTruncated );

            cout << endl;
            n++;
        }

        return;
    }

    logger( "**************************************" );
    logger( "Direct..." );

    cout << "TotalSizeSimpified: " << dec << start0.size() << " * "  
                          << start1.size() << " * "  
                          << start2.size() << " * "  
                          << start3.size() << " * "  
                          << end0.size() << " * "  
                          << end1.size() << " * "  
                          << end2.size() << " * "  
                          << end3.size() << " = "  
                          << start0.size() * start1.size() * start2.size() *
                          start3.size() * end0.size() * end1.size() *
                          end2.size() * end3.size() << " = 2^"  
                          << log( start0.size() * start1.size() * start2.size() *
                          start3.size() * end0.size() * end1.size() *
                          end2.size() * end3.size() ) / log(2) << endl; 

    set< pair< NS, NS >> totalSet;

    /*
    // pre-process
    set<Word> pS0, pS1, pS2, pS3, pE0, pE1, pE2, pE3;

    auto px0 = getRandom( start0 );
    auto px1 = getRandom( start1 );
    auto px2 = getRandom( start2 );
    auto px3 = getRandom( start3 );

    auto py0 = getRandom( end0 );
    auto py1 = getRandom( end1 );
    auto py2 = getRandom( end2 );
    auto py3 = getRandom( end3 );

    NS pxx, pyy;

    Word_to_State( px0, px1, px2, px3, pxx );

    Word_to_State( py0, py1, py2, py3, pyy );

    set < pair<NS, NS> > PIN;
    PIN.clear();

    auto res = GIFT_MultiSolution( ROUND, pxx, pyy, PIN );
        
    cout << dec << "Res = " << res << " Solution IN size : " << PIN.size() << endl;

    for ( auto pinner : PIN )
    {
        Word ppx0, ppx1, ppx2, ppx3, ppy0, ppy1, ppy2, ppy3;

        State_to_Word( pinner.first, ppx0, ppx1, ppx2, ppx3 );


        State_to_Word( pinner.second, ppy0, ppy1, ppy2, ppy3 );

        InvPassSuperSbox( ppx0, Invtable, pS0 );
        InvPassSuperSbox( ppx1, Invtable, pS1 );
        InvPassSuperSbox( ppx2, Invtable, pS2 );
        InvPassSuperSbox( ppx3, Invtable, pS3 );

        PassSuperSbox( ppy0, table, pE0 );
        PassSuperSbox( ppy1, table, pE1 );
        PassSuperSbox( ppy2, table, pE2 );
        PassSuperSbox( ppy3, table, pE3 );
    }
    */

    for ( auto it0 : start0 )
    for ( auto it1 : start1 )
    for ( auto it2 : start2 )
    for ( auto it3 : start3 )
    for ( auto jt0 : end0 )
    for ( auto jt1 : end1 )
    for ( auto jt2 : end2 )
    for ( auto jt3 : end3 )
    {
        NS xx, yy;

        Word_to_State( it0, it1, it2, it3, xx );

        Word_to_State( jt0, jt1, jt2, jt3, yy );

        auto p = make_pair( xx, yy );

        if ( satSet( p,  truncated ) ) 
            continue;

        if ( satSetPlaintext( p,  pTruncated ) ) 
            continue;

        if ( satSetCiphertext( p,  cTruncated ) ) 
            continue;

        totalSet.insert (  p );
    }

    cout << "TotalSize : " <<  dec << totalSet.size() <<  " = 2^" << log(
            totalSet.size() ) / log(2) << endl;


    // remove impossible differentials belong to known truncated
    // differential
    while ( totalSet.size() > 0 )
    {
        auto d = getRandom ( totalSet );

        pair<NS, NS> inner;

        printNibbleState( d.first );
        cout << endl;
        printNibbleState( d.second );
        cout << endl;

        auto res = GIFT_Single( ROUND, d.first, d.second, inner.first,
                inner.second );

        //cout << "Result: " << res << endl;

        if ( res == 0 ) // infeasible
        {
            // get the truncated pattern 
            pair<NS, NS> t;

            for ( int i = 0; i < 16; i++ )
                if ( d.first[i] == 0 )
                    t.first[i] = 0;
                else
                    t.first[i] = 1;

            for ( int i = 0; i < 16; i++ )
                if ( d.second[i] == 0 )
                    t.second[i] = 0;
                else
                    t.second[i] = 1;

            // check if truncated ID
            auto isTruncated = GIFT_Truncated( ROUND, t.first, t.second );
            if ( isTruncated == 0 ) // ID
            {
                logger( "truncated ID found" );
                // reduce the totalSet based on the ID
                cout << dec << totalSet.size() << " -reduce-> ";
                reduceTotalSetFromTruncated( t, totalSet );
                truncated.insert( t );
                cout << dec << totalSet.size() << endl;
            }
            else  
            {
                auto isPlaintextTruncated = GIFT_PlaintextTruncated( ROUND,
                        t.first, d.second );
                if ( isPlaintextTruncated == 0 )
                {
                    cout << "Plaintext ID found" << endl;
                    cout << dec << totalSet.size() << " -reduce-> ";
                    pTruncated.insert( make_pair( t.first, d.second ) );
                    reduceTotalSetFromPlaintextTruncated( make_pair( t.first,
                                d.second ), totalSet );
                    cout << dec << totalSet.size() << endl;
                }
                else 
                {
                    auto isCiphertextTruncated = GIFT_CiphertextTruncated( ROUND,
                            d.first, t.second );
                    if ( isCiphertextTruncated == 0 )
                    {
                        cout << "Ciphertext ID found" << endl;
                        cout << dec << totalSet.size() << " -reduce-> ";
                        cTruncated.insert( make_pair( d.first, t.second ) );
                        reduceTotalSetFromCiphertextTruncated( make_pair( d.first,
                                    t.second ), totalSet );
                        cout << dec << totalSet.size() << endl;
                    }
                    else
                    {
                        cout << "Detailed ID found" << endl;
                        cout << dec << totalSet.size() << " -reduce-> ";
                        impossible.insert( d );
                        totalSet.erase( d );
                        cout << dec << totalSet.size() << endl;
                    }
                }
            }
        }
        else // soluable
        {
            //printNibbleState( inner.first );
            //cout << endl;
            //printNibbleState( inner.second );
            //cout << endl;

            logger( "Solution Reduce" );

            cout << dec << totalSet.size() << " -reduce-> ";

            reduceTotalSetFromInner( inner, totalSet );

            cout << dec << totalSet.size() << endl ;
        }
    }


    std::lock_guard<std::mutex> lock(mtx);
    for ( auto it : impossible )
        impossible1.insert( it );

    for ( auto it : truncated )
        truncated1.insert( it );

    for ( auto it : pTruncated )
        pTruncated1.insert( it );

    for ( auto it : cTruncated )
        cTruncated1.insert( it );

}

void Worker_Intersection( const set<Word> & s0, set<Word> & S0 ) 
{
    set<Word> tmpS0;
    set_intersection( s0.begin(), s0.end(), S0.begin(), S0.end(),
            insert_iterator< set<Word> > ( tmpS0, tmpS0.begin() ) );
    S0.swap( tmpS0 );
}

void getIntersectionOfBadPattern ( const set< pair<NS, NS> > & bp,
                                    set<Word> &S0, 
                                    set<Word> &S1, 
                                    set<Word> &S2, 
                                    set<Word> &S3, 
                                    
                                    set<Word> &E0, 
                                    set<Word> &E1, 
                                    set<Word> &E2,
                                    set<Word> &E3 ) 
{
    cout << "Before: = 2^ " << dec << log( S0.size() * S1.size() * S2.size() * S3.size() *
        E0.size() * E1.size() * E2.size() * E3.size( ) ) / log(2)  
        << endl;

    set<Word> s0,s1,s2,s3,e0,e1,e2,e3;

    set<NS> F, S;
    for ( auto it : bp )
    {
        F.insert( it.first );
        S.insert( it.second );
    }

    for (auto it : F )
    {
        Word m0, m1, m2, m3;
        State_to_Word( it, m0, m1, m2, m3 );

        InvPassSuperSbox( m0, Invtable, s0 );
        InvPassSuperSbox( m1, Invtable, s1 );
        InvPassSuperSbox( m2, Invtable, s2 );
        InvPassSuperSbox( m3, Invtable, s3 );
    }

    for ( auto it : S )
    {
        Word w0, w1, w2, w3;
        State_to_Word( it, w0, w1, w2, w3 );

        PassSuperSbox( w0, table, e0 );
        PassSuperSbox( w1, table, e1 );
        PassSuperSbox( w2, table, e2 );
        PassSuperSbox( w3, table, e3 );
    }

    vector<thread> threads;

    threads.push_back( thread ( Worker_Intersection, ref( s0 ), ref( S0 ) ) );
    threads.push_back( thread ( Worker_Intersection, ref( s1 ), ref( S1 ) ) );
    threads.push_back( thread ( Worker_Intersection, ref( s2 ), ref( S2 ) ) );
    threads.push_back( thread ( Worker_Intersection, ref( s3 ), ref( S3 ) ) );
    threads.push_back( thread ( Worker_Intersection, ref( e0 ), ref( E0 ) ) );
    threads.push_back( thread ( Worker_Intersection, ref( e1 ), ref( E1 ) ) );
    threads.push_back( thread ( Worker_Intersection, ref( e2 ), ref( E2 ) ) );
    threads.push_back( thread ( Worker_Intersection, ref( e3 ), ref( E3 ) ) );

    for ( auto & th : threads ) 
        th.join();

    cout << "After: = 2^ " << dec << log( S0.size() * S1.size() * S2.size() * S3.size() *
        E0.size() * E1.size() * E2.size() * E3.size( ) ) / log(2)  
        << endl;
}


