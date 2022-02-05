#include"gurobi_c++.h"
#include"forword.h"
#include"backword.h"
#include"skinny.h"
#include"basic.h"
#include"log.h"
#include"automatic.h"
#include"manual.h"
#include"thread_pool.h"
#include<iostream>
#include<set>
#include"MILP.h"
#include<tuple>
#include<thread>
#include<future>
#include<cstdio>
#include<algorithm>
#include<cmath>

using namespace std;
using namespace thread_pool;

int ROUND;
int ** table;
int ** Invtable;

set<Word> select( int * ind, 
             const set<Word> & S0,  
             const set<Word> & S1,  
             const set<Word> & S2,  
             const set<Word> & S3,  
             const set<Word> & S4,  
             const set<Word> & S5,  
             const set<Word> & S6,  
             const set<Word> & S7 )  
{
    auto p = max( 
            { S0, S1, S2, S3, S4, S5, S6, S7 }, 
            [] ( const set<Word> & a, const set<Word> & b ) { return a.size() <
            b.size(); } );
    if ( p == S0 )
        ind[0] = 1;
    if ( p == S1 )
        ind[1] = 1;
    if ( p == S2 )
        ind[1] = 1;
    if ( p == S3 )
        ind[3] = 1;
    if ( p == S4 )
        ind[4] = 1;
    if ( p == S5 )
        ind[5] = 1;
    if ( p == S6 )
        ind[6] = 1;
    if ( p == S7 )
        ind[7] = 1;

    return p;
}

int main( int argc, char ** argv )
{
    auto Pattern = getBadPatternSamples( "myPattern.txt" );
    cout << Pattern.size() << endl;
    logger( "bad pattern get" );

    for ( auto it : Pattern )
    {
        if ( it.second[5] != 1 )
            continue;
        if ( ! ( it.second[0] + it.second[1] + it.second[2] + it.second[3] + 
                 it.second[4] + it.second[5] + it.second[6] + it.second[7] + 
                 it.second[8] + it.second[9] + it.second[10] + it.second[11] + 
                 it.second[12] + it.second[13] + it.second[14] + it.second[15]
                 == 1 ) ) 
            continue;


        cout << "*******************" << endl;
        printS( it.first );
        cout << endl;

        printS( it.second );
        cout << endl;

        ShiftRows( it.second );
        MC( it.second );

        printS( it.second );
        cout << endl;
    }


}

