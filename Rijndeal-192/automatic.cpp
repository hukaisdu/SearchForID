#include"gurobi_c++.h"
#include"forword.h"
#include"backword.h"
#include"skinny.h"
#include <mutex>
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

using namespace std;
using namespace thread_pool;

mutex mtx; // 保护counter
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
void getBadPattern(  set<Word> & frontIS0,
                     set<Word> & frontIS1,
                     set<Word> & frontIS2,
                     set<Word> & frontIS3,
                     set<Word> & frontIS4,
                     set<Word> & frontIS5,

                     set<Word> & backIS0,
                     set<Word> & backIS1,
                     set<Word> & backIS2,
                     set<Word> & backIS3,
                     set<Word> & backIS4,
                     set<Word> & backIS5,

                     set< pair<NS,NS> > & badPattern
                     )
{
    // stage 3: obtain the head and tail of the MILP model
    set<NS> Head, Tail; // sort by the addresses
    // word to state
    for ( auto & it0 : frontIS0 )
    for ( auto & it1 : frontIS1 )
    for ( auto & it2 : frontIS2 )
    for ( auto & it3 : frontIS3 )
    for ( auto & it4 : frontIS4 )
    for ( auto & it5 : frontIS5 )
    {
        if ( ( it0 == 0 ) && ( it1 == 0 ) && ( it2 == 0 ) && ( it3 == 0 ) &&
                (it4 == 0 ) && (it5 == 0 ) )
            continue;

        NS x;
        Word_to_State( it0, it1, it2, it3, it4, it5, x );   
        // ShiftRow
        Head.insert( x );
    }

    cout << "Remove " << dec<< Head.size() << endl;
    RemoveSymmetry( Head );
    cout << "Remove " << dec << Head.size() << endl;

    for ( auto & it0 : backIS0 )
    for ( auto & it1 : backIS1 )
    for ( auto & it2 : backIS2 )
    for ( auto & it3 : backIS3 )
    for ( auto & it4 : backIS4 )
    for ( auto & it5 : backIS5 )
    {
        if ( ( it0 == 0 ) && ( it1 == 0 ) && ( it2 == 0 ) && ( it3 == 0 ) &&
                (it4 == 0 ) && ( it5 == 0 ) )
            continue;

        NS x;
        Word_to_State( it0, it1, it2, it3, it4, it5, x );   
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
        //printNibbleState ( it.first );
        //cout << " --/--> " ;

        //printNibbleState ( it.second );
        //cout << endl;

        badPattern.insert( it );
    }

    logger();
}

void processBadPattern ( const pair<NS, NS> & badPattern, 
                         const map<Word, set<Word>> & frontMapIS0, 
                         const map<Word, set<Word>> & frontMapIS1, 
                         const map<Word, set<Word>> & frontMapIS2, 
                         const map<Word, set<Word>> & frontMapIS3, 
                         const map<Word, set<Word>> & frontMapIS4, 
                         const map<Word, set<Word>> & frontMapIS5, 
                         const map<Word, set<Word>> & backMapIS0, 
                         const map<Word, set<Word>> & backMapIS1, 
                         const map<Word, set<Word>> & backMapIS2, 
                         const map<Word, set<Word>> & backMapIS3, 
                         const map<Word, set<Word>> & backMapIS4, 
                         const map<Word, set<Word>> & backMapIS5, 
                         set<pair<NS, NS> > & impossible )
{
    printNibbleState ( badPattern.first );

    cout << " --/--> " ;

    printNibbleState ( badPattern.second );

    cout << endl;

    set<Word> start0, start1, start2, start3, start4, start5;
    set<Word> end0, end1, end2, end3, end4, end5;

    NS x { badPattern.first };  // x : Sbox -> MC

    Word w0, w1, w2, w3, w4, w5;

    State_to_Word( x, w0, w1, w2, w3, w4, w5 );

    NS y { badPattern.second };

    Word m0, m1, m2, m3, m4, m5;

    State_to_Word( y, m0, m1, m2, m3, m4, m5 );

    start0 = frontMapIS0.at( w0 ) ;
    start1 = frontMapIS1.at( w1 );
    start2 = frontMapIS2.at( w2 );
    start3 = frontMapIS3.at( w3 );
    start4 = frontMapIS4.at( w4 );
    start5 = frontMapIS5.at( w5 );

    end0 = backMapIS0.at( m0 );
    end1 = backMapIS1.at( m1 );
    end2 = backMapIS2.at( m2 );
    end3 = backMapIS3.at( m3 );
    end4 = backMapIS4.at( m4 );
    end5 = backMapIS5.at( m5 );

    logger();

    auto size = start0.size() * 
                start1.size() *
                start2.size() *
                start3.size() *
                start4.size() *
                start5.size() *
                end0.size() *
                end1.size() *
                end2.size() *
                end3.size() *
                end4.size() *
                end5.size();

    cout << "TotalSize: " << dec << size << " = " << log(size) / log(2) << endl; 

    logger( "**************************************" );
    logger( "Direct..." );

    /*
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
    */

    
    if ( size == 0 )
    {
        cout << "Size is 0" << endl;
        exit(-1);
    }


    set< pair< NS, NS >> totalSet;

    for ( auto it0 : start0 )
    for ( auto it1 : start1 )
    for ( auto it2 : start2 )
    for ( auto it3 : start3 )
    for ( auto it4 : start4 )
    for ( auto it5 : start5 )
    for ( auto jt0 : end0 )
    for ( auto jt1 : end1 )
    for ( auto jt2 : end2 )
    for ( auto jt3 : end3 )
    for ( auto jt4 : end4 )
    for ( auto jt5 : end5 )
    {
        NS xx, yy;
        Word_to_State( it0, it1, it2, it3, it4, it5, xx );

        Word_to_State( jt0, jt1, jt2, jt3, jt4, jt5, yy );

        auto p = make_pair( xx, yy );

        totalSet.insert ( p );
    }

    cout << "TotalSize : " <<  dec << totalSet.size() <<  " = 2^" << log(
            totalSet.size() ) / log(2) << endl;

    set<pair<NS, NS>> imp;
    SKINNY_M( ROUND, totalSet, imp) ;

    std::lock_guard<std::mutex> lck(mtx);
    for ( auto it : imp )
        impossible.insert( it );
}




                                    

