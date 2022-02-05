#include"gurobi_c++.h"
#include"forword.h"
#include"backword.h"
#include"skinny.h"
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
#include"manual.h"
#include<map>
#include<mutex>

using namespace std;
using namespace thread_pool;

mutex mtx;

int depth = 0;

const int DIRECT = ( 1L << 28 );

void Worker( const set<NS> & Head,  const set<NS> & Tail, set< pair<NS, NS> > & Bad )
{
    for ( auto& it : Head )
    {
        auto res = SKINNY( ROUND - 4, it, Tail, Bad );
        if ( res < 0 )
        {
            cerr << "Error " << endl;
            exit(-1);
        }
    }
}

void getBadPatternX( set<Word> & frontIS0,
                     set<Word> & frontIS1,
                     set<Word> & frontIS2,
                     set<Word> & frontIS3,

                     set<Word> & backIS0,
                     set<Word> & backIS1,
                     set<Word> & backIS2,
                     set<Word> & backIS3,

                     set< pair<NS,NS> > & badPattern 
                     )
{
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
        // ShiftRow
        Head.insert( x );
    }

    RemoveSymmetry( Head );


    for ( auto & it0 : backIS0 )
    for ( auto & it1 : backIS1 )
    for ( auto & it2 : backIS2 )
    for ( auto & it3 : backIS3 )
    {
        if ( ( it0 == 0 ) && ( it1 == 0 ) && ( it2 == 0 ) && ( it3 == 0 ) )
            continue;

        NS x;
        Word_to_State( it0, it1, it2, it3, x );   
        // ShiftRow
        Tail.insert( x );
    }

    logger( string ( "Head and Tail constructed!\n" ) + 
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



void processBadPattern ( pair<NS, NS> & badPattern, 
                         const map<Word, set<Word>> & frontMapIS0, 
                         const map<Word, set<Word>> & frontMapIS1, 
                         const map<Word, set<Word>> & frontMapIS2, 
                         const map<Word, set<Word>> & frontMapIS3, 
                         const map<Word, set<Word>> & backMapIS0, 
                         const map<Word, set<Word>> & backMapIS1, 
                         const map<Word, set<Word>> & backMapIS2, 
                         const map<Word, set<Word>> & backMapIS3, 
                         set<pair<NS, NS> > & impossible, 
                         set<pair<NS, NS> > & truncated )
{
    printNibbleState ( badPattern.first );

    cout << " --/--> " ;

    printNibbleState ( badPattern.second );

    cout << endl;

    set<Word> start0, start1, start2, start3;
    set<Word> end0, end1, end2, end3;

    NS x { badPattern.first };  // x : Sbox -> MC

    Word w0, w1, w2, w3;

    State_to_Word( x, w0, w1, w2, w3 );


    NS y { badPattern.second };

    Word m0, m1, m2, m3;
    State_to_Word( y, m0, m1, m2, m3 );

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

    /*
    if( start0.size() * start1.size() * start2.size() *
      start3.size() * end0.size() * end1.size() *
      end2.size() * end3.size() >= ( 1L << 30 )  )
    {
        logger( "*********************************" );
        logger( "Simplify...." );

        while ( true )
        {

            set< pair<NS, NS> >  bad; 

            set<Word> f0,f1,f2,f3, b0,b1,b2,b3;

            map<Word, set<Word> > xMapFront0;
            map<Word, set<Word> > xMapFront1;
            map<Word, set<Word> > xMapFront2;
            map<Word, set<Word> > xMapFront3;

            getPatterns( start0, f0, 1, xMapFront0 );
            getPatterns( start1, f1, 1, xMapFront1 );
            getPatterns( start2, f2, 1, xMapFront2 );
            getPatterns( start3, f3, 1, xMapFront3 );

            map<Word, set<Word> > xMapBack0;
            map<Word, set<Word> > xMapBack1;
            map<Word, set<Word> > xMapBack2;
            map<Word, set<Word> > xMapBack3;

            getInvPatterns( end0, b0, 1, xMapBack0 );
            getInvPatterns( end1, b1, 1, xMapBack1 );
            getInvPatterns( end2, b2, 1, xMapBack2 );
            getInvPatterns( end3, b3, 1, xMapBack3 );

            getBadPatternX( f0, f1, f2, f3, b0, b1, b2, b3, bad );

            getIntersectionOfBadPattern( bad, 
                    xMapFront0, 
                    xMapFront1, 
                    xMapFront2, 
                    xMapFront3, 
                    xMapBack0,
                    xMapBack1,
                    xMapBack2,
                    xMapBack3,
                    start0, start1, start2, start3,
                    end0, end1, end2, end3);

        cout << "TotalSizeSimpified: " << dec 
                              << start0.size() << " * "  
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
              end2.size() * end3.size() < ( 1L << 30 )  )
                break;
        }
    }
*/

    if( start0.size() * start1.size() * start2.size() *
      start3.size() * end0.size() * end1.size() *
      end2.size() * end3.size() >= DIRECT  )
    {
        logger( "*********************************" );
        logger( "Simplify...." );

        set< pair<NS, NS> >  bad; 

        set<Word> f0,f1,f2,f3, b0,b1,b2,b3;

        //getDifferentBadPattern( start0, start1, start2, start3, end0, end1, end2, end3,  
        //        f0, f1, f2, f3, b0, b1, b2, b3, bad, 1 );
        getPatterns(start0, f0, 1 );
        getPatterns(start1, f1, 1 );
        getPatterns(start2, f2, 1 );
        getPatterns(start3, f3, 1 );

        getInvPatterns( end0, b0, 1 );
        getInvPatterns( end1, b1, 1 );
        getInvPatterns( end2, b2, 1 );
        getInvPatterns( end3, b3, 1 );

        getBadPatternX( f0, f1, f2, f3, b0, b1, b2, b3, bad );

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

        cout << "Bad " << dec << bad.size() << endl;


        cout << "Bad " << dec << bad.size() << endl;
        int n = 0;
        depth ++;
        for ( auto it : bad )
        {
            cout << "===============================" << endl;

            cout << "The " << dec << depth << " floor" << endl;
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
                    impossible, truncated );

            cout << endl;
            n++;
        }
        depth--;

        return;
    }

    logger( "**************************************" );
    logger( "Direct..." );

    cout << "TotalSizeSimpified: " << dec 
                          << start0.size() << " * "  
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
      end2.size() * end3.size() == 0 )
    {
        cout << "Size is 0" << endl;
        exit(-1);
    }


    set< pair< NS, NS >> totalSet;

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

        if ( satSet(p,  truncated ) ) 
            continue;

        totalSet.insert ( p );
    }

    cout << "TotalSize : " <<  dec << totalSet.size() <<  " = 2^" << log(
            totalSet.size() ) / log(2) << endl;


    // remove impossible differentials belong to known truncated
    // differential

    while ( totalSet.size() > 0 )
    {
        auto d = getRandom ( totalSet );


        pair<NS, NS> inner;
        auto res = SKINNY_Single( ROUND, d.first, d.second, inner.first,
                inner.second );

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
            auto isTruncated = SKINNY_Truncated( ROUND, t.first, t.second );

            if ( isTruncated == 0 ) // ID
            {
                logger( "truncated ID found" );
                // reduce the totalSet based on the ID
                cout << dec << totalSet.size() << " -reduce-> ";
                reduceTotalSetFromTruncated( t, totalSet );

                truncated.insert( t );
                cout << dec << totalSet.size() << endl;

                printNibbleState( t.first );
                cout << " --/--> ";
                printNibbleState( t.second );
                cout << endl; 
            }
            else
            {

                impossible.insert( d );
                totalSet.erase( d );
            }
        }
        else // soluable
        {
            //InvMC( inner.first );
            //InvShiftRows( inner.first );
            printNibbleState( inner.first );
            cout << endl;
            printNibbleState( inner.second );
            cout << endl;

            logger( "Solution Reduce" );
            cout << dec << totalSet.size() << " -reduce-> ";
            reduceTotalSetFromInner( inner, totalSet );
            cout << dec << totalSet.size() << endl ;
        }
    }

}

void getIntersectionOfBadPattern ( const set< pair<NS, NS> > & bp,
                                    const map<Word, set<Word>> & fmap0,
                                    const map<Word, set<Word>> & fmap1,
                                    const map<Word, set<Word>> & fmap2,
                                    const map<Word, set<Word>> & fmap3,
                                    const map<Word, set<Word>> & bmap0,
                                    const map<Word, set<Word>> & bmap1,
                                    const map<Word, set<Word>> & bmap2,
                                    const map<Word, set<Word>> & bmap3,
                                    set<Word> &S0, 
                                    set<Word> &S1, 
                                    set<Word> &S2, 
                                    set<Word> &S3, 
                                    
                                    set<Word> &E0, 
                                    set<Word> &E1, 
                                    set<Word> &E2,
                                    set<Word> &E3 ) 

                                                                                
{
    set<Word> s0, s1, s2, s3, e0 , e1, e2, e3;
    for ( auto it : bp )
    {
        Word m0, m1, m2, m3;
        State_to_Word( it.first, m0, m1, m2, m3 );

        Word w0, w1, w2, w3;
        State_to_Word( it.second, w0, w1, w2, w3 );

        auto ts0 = fmap0.at( m0 );
        auto ts1 = fmap1.at( m1 );
        auto ts2 = fmap2.at( m2 );
        auto ts3 = fmap3.at( m3 );

        for ( auto it : ts0 )
            s0.insert( it );
        for ( auto it : ts1 )
            s1.insert( it );
        for ( auto it : ts2 )
            s2.insert( it );
        for ( auto it : ts3 )
            s3.insert( it );

        auto tb0 = bmap0.at( w0 );
        auto tb1 = bmap1.at( w1 );
        auto tb2 = bmap2.at( w2 );
        auto tb3 = bmap3.at( w3 );

        for ( auto it : tb0 )
            e0.insert( it );
        for ( auto it : tb1 )
            e1.insert( it );
        for ( auto it : tb2 )
            e2.insert( it );
        for ( auto it : tb3 )
            e3.insert( it );
    }

    set<Word> tmpS0, tmpS1, tmpS2, tmpS3, tmpE0, tmpE1, tmpE2, tmpE3;
    // start
    set_intersection( s0.begin(), s0.end(), S0.begin(), S0.end(),
            insert_iterator< set<Word> > ( tmpS0, tmpS0.begin() ) );
    S0.swap( tmpS0 );

    set_intersection( s1.begin(), s1.end(), S1.begin(), S1.end(),
            insert_iterator< set<Word> > ( tmpS1, tmpS1.begin() ) );
    S1.swap( tmpS1 );

    set_intersection( s2.begin(), s2.end(), S2.begin(), S2.end(),
            insert_iterator< set<Word> > ( tmpS2, tmpS2.begin() ) );
    S2.swap( tmpS2 );

    set_intersection( s3.begin(), s3.end(), S3.begin(), S3.end(),
            insert_iterator< set<Word> > ( tmpS3, tmpS3.begin() ) );
    S3.swap( tmpS3 );
    // end
    set_intersection( e0.begin(), e0.end(), E0.begin(), E0.end(),
            insert_iterator< set<Word> > ( tmpE0, tmpE0.begin() ) );
    E0.swap( tmpE0 );

    set_intersection( e1.begin(), e1.end(), E1.begin(), E1.end(),
            insert_iterator< set<Word> > ( tmpE1, tmpE1.begin() ) );
    E1.swap( tmpE1 );

    set_intersection( e2.begin(), e2.end(), E2.begin(), E2.end(),
            insert_iterator< set<Word> > ( tmpE2, tmpE2.begin() ) );
    E2.swap( tmpE2 );

    set_intersection( e3.begin(), e3.end(), E3.begin(), E3.end(),
            insert_iterator< set<Word> > ( tmpE3, tmpE3.begin() ) );
    E3.swap( tmpE3 );
}



                                    

