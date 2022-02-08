#include"basic.h"
#include<set>
#include<vector>
#include<iterator>
#include<random>
#include<iostream>
#include<algorithm>
#include"craft.h"
#include"automatic.h"
#include"forword.h"
#include"backword.h"

using namespace std;

/*
bool SymmetricEq ( const NS & a, const NS & b )
{
    Word a0, a1, a2, a3;
    Word b0, b1, b2, b3;
    State_to_Word( a, a0, a1, a2, a3 );
    State_to_Word( b, b0, b1, b2, b3 );

    if ( ( a0 == b0 ) && ( a1 == b1 ) && ( a2 == b2 ) && ( a3 == b3 ) )
        return true;
    if ( ( a0 == b1 ) && ( a1 == b2 ) && ( a2 == b3 ) && ( a3 == b0 ) )
        return true;
    if ( ( a0 == b2 ) && ( a1 == b3 ) && ( a2 == b0 ) && ( a3 == b1 ) )
        return true;
    if ( ( a0 == b3 ) && ( a1 == b0 ) && ( a2 == b1 ) && ( a3 == b2 ) )
        return true;

    return false;
} 

void RemoveSymmetry( set<NS> & S )
{
    set<NS> X = S;
    set<NS> Y = S;
    S.clear();
    for ( auto it : X )
    {
        if ( Y.count( it ) == 1 )
            S.insert( it );
        for ( auto jt = Y.begin(); jt != Y.end(); )
        {
            if ( SymmetricEq( it, *jt ) ) 
                Y.erase( jt++ ); 
            else
                ++jt;
        }
    }
}
*/

bool sat( const pair< NS, NS > & a, const pair<NS, NS> & b ) 
{
    for ( int i = 0; i < 16; i++ )
        if ( ( ( a.first[i] == 0 ) && ( b.first[i] != 0 ) ) || ( ( a.first[i] != 0
                    ) && ( b.first[i] == 0 ) )  )
            return false;

    for ( int i = 0; i < 16; i++ )
        if ( ( ( a.second[i] == 0 ) && ( b.second[i] != 0 ) ) || ( ( a.second[i] != 0
                    ) && ( b.second[i] == 0 ) )  )
            return false;
    return true;
}

bool satSingle( const NS & a, const NS & b ) 
{
    for ( int i = 0; i < 16; i++ )
        if ( ( ( a[i] == 0 ) && ( b[i] != 0 ) ) || ( ( a[i] != 0
                    ) && ( b[i] == 0 ) )  )
            return false;

    return true;
}

bool satSingleSet( const NS & a, const set<NS> & S ) 
{
    for ( auto it : S )
        if ( satSingle( a, it ) == true )
            return true;
    return false;
}

bool satSingleBit( const BS & a, const BS & b ) 
{
    for ( int i = 0; i < 64; i++ )
        if ( ( ( a[i] == 0 ) && ( b[i] != 0 ) ) || ( ( a[i] != 0
                    ) && ( b[i] == 0 ) )  )
            return false;

    return true;
}

bool satSingleBitSet( const BS & a, const set<BS> & S ) 
{
    for ( auto it : S )
        if ( satSingleBit( a, it ) == true )
            return true;
    return false;
}


bool satSet( const pair< NS, NS > & a, const set< pair<NS, NS> > & S ) 
{
    for ( auto it : S )
        if ( sat( a, it ) == true )
            return true;
    return false;
}


Word Nibble_to_Word( Nibble x0, Nibble x1, Nibble x2, Nibble x3 )
{
    return ( static_cast<Word> ( x0 ) << 12 ) 
         ^ ( static_cast<Word> ( x1 ) << 8  ) 
         ^ ( static_cast<Word> ( x2 ) << 4  ) 
         ^ ( static_cast<Word> ( x3 ) << 0  );
}

void Word_to_Nibble( Word x, Nibble & x0, Nibble & x1, Nibble & x2, Nibble & x3 )
{
    x0 = x >> 12 & 0xf;
    x1 = x >> 8  & 0xf;
    x2 = x >> 4  & 0xf;
    x3 = x >> 0  & 0xf;
}

void Nibble_to_Bit( const NS &x, BS &y )
{
    for ( int i = 0; i < 16; i++ )
    {
        y[4 * i    ] = x[i] >> 3 & 0x1; 
        y[4 * i + 1] = x[i] >> 2 & 0x1; 
        y[4 * i + 2] = x[i] >> 1 & 0x1; 
        y[4 * i + 3] = x[i] >> 0 & 0x1; 
    }
}

void Bit_to_Nibble( const BS & x, NS & y )
{
    for ( int i = 0; i < 16; i++ )
        y[i] = ( x[4 * i] << 3 ) ^ ( x[4 * i + 1] << 2 ) ^ ( x[4 * i + 2] << 1 )
            ^ ( x[4 * i + 3] << 0 );
}

void State_to_Word( const NS & s, Word& x0, Word& x1, Word& x2, Word& x3 )
{
    x0 = Nibble_to_Word( s[0], s[4], s[8], s[12] );
    x1 = Nibble_to_Word( s[1], s[5], s[9], s[13] );
    x2 = Nibble_to_Word( s[2], s[6], s[10], s[14] );
    x3 = Nibble_to_Word( s[3], s[7], s[11], s[15] );
}

void Word_to_State( Word x0, Word x1, Word x2, Word x3, NS &s )
{
    Nibble z0, z1, z2, z3;
    Word_to_Nibble( x0, z0, z1, z2, z3 );
    s[0] = z0;
    s[4] = z1;
    s[8] = z2;
    s[12] = z3;

    Word_to_Nibble( x1, z0, z1, z2, z3 );
    s[1] = z0;
    s[5] = z1;
    s[9] = z2;
    s[13] = z3;

    Word_to_Nibble( x2, z0, z1, z2, z3 );
    s[2] = z0;
    s[6] = z1;
    s[10] = z2;
    s[14] = z3;

    Word_to_Nibble( x3, z0, z1, z2, z3 );
    s[3] = z0;
    s[7] = z1;
    s[11] = z2;
    s[15] = z3;
}

// return an element from S randomly
auto select_random( const set<NS> & S, size_t n )
{
    auto it = begin( S );
    advance( it, n );
    return it;
}

NS getRandom( const set<NS>& S ) 
{
    auto size = S.size();
    random_device rd;
    mt19937_64 generator ( rd() );
    
    uniform_int_distribution<int> distribution(0, size - 1);
    int dice_roll = distribution(generator);

    return *select_random( S, dice_roll );
}

auto select_random( const set< pair<NS, NS> > & S, size_t n )
{
    auto it = begin( S );
    advance( it, n );
    return it;
}

pair<NS, NS> getRandom( const set< pair<NS, NS> >& S ) 
{
    auto size = S.size();
    random_device rd;
    mt19937_64 generator ( rd() );
    
    uniform_int_distribution<int> distribution(0, size - 1);
    int dice_roll = distribution(generator);

    return *select_random( S, dice_roll );
}

auto select_random( const set< Word > & S, size_t n )
{
    auto it = begin( S );
    advance( it, n );
    return it;
}

Word getRandom( const set< Word >& S ) 
{
    if ( S.empty() )
        return 0;

    auto size = S.size();
    random_device rd;
    mt19937_64 generator ( rd() );
    
    uniform_int_distribution<int> distribution(0, size - 1);
    int dice_roll = distribution(generator);

    return *select_random( S, dice_roll );
}


void genDDT( const int* Sbox, int ** table )
{
    for ( int i = 0; i < 16; i++ )
        for ( int j = 0; j < 16; j++ )
            table[i][j] = 0;

    for ( int x_xor = 0; x_xor < 16; x_xor++ ) 
        for ( int x = 0; x < 16; x++ )
            table[x_xor][ static_cast<int> ( Sbox[x] ^ Sbox[x^x_xor] ) ]++;
}

void printNibbleState( const NS & s, ostream & os )
{
    for ( int i = 0; i < 16; i++ )
        os << hex << int ( s[i] ) << " ";
}

void printS( const NS & a )
{
    for ( int i = 0; i < 4; i++ )
    {
        for ( int j = 0; j < 4; j++ )
            cout << hex << int ( a[4 * i + j] ) << " "; 
        cout << endl;
    }
}

set<Nibble>* InvSboxDDT( const NS & a )
{
    set<Nibble> * S = new set<Nibble> [16];
    for ( int i = 0; i < 16; i++ )
    {
        for ( int j = 0; j < 16; j++ )
            if ( Invtable[ a[i] ][j] > 0 )
                S[i].insert( j );
    }
    delete [] Invtable;
    return S;
}

void reduceTotalSetFromTruncated( const pair<NS, NS> & x, set< pair<NS, NS> > & totalSet )
{
    for( auto it = totalSet.begin(); it != totalSet.end(); )
        if ( sat( x, *it ) )
            totalSet.erase( it++ );
        else
            ++it;
}

void reduceNSFromTruncated( const NS & p, set<NS> & S )
{
    for( auto it = S.begin(); it != S.end(); )
        if ( satSingle( p, *it ) )
            S.erase( it++ );
        else
            ++it;
}

// x is the inner values 
void reduceTotalSetFromInner( const pair<NS, NS> & x, set< pair<NS, NS> > & totalSet )
{
    // inner to front
    Word f0, f1, f2, f3;
    State_to_Word( x.first, f0, f1, f2, f3 );

    set<Word> S0, S1, S2, S3;
    InvPassSuperSbox( f0, Invtable, S0 ); 
    InvPassSuperSbox( f1, Invtable, S1 ); 
    InvPassSuperSbox( f2, Invtable, S2 ); 
    InvPassSuperSbox( f3, Invtable, S3 ); 

    Word b0, b1, b2, b3;
    State_to_Word( x.second, b0, b1, b2, b3 );

    set<Word> B0, B1, B2, B3;
    PassSuperSbox( b0, table, B0 );
    PassSuperSbox( b1, table, B1 );
    PassSuperSbox( b2, table, B2 );
    PassSuperSbox( b3, table, B3 );

    for ( auto it = totalSet.begin(); it != totalSet.end(); )
    {
        auto t = *it;

        Word tf0, tf1, tf2, tf3, te0, te1, te2, te3;
        State_to_Word( t.first, tf0, tf1, tf2, tf3 );
        State_to_Word( t.second, te0, te1, te2, te3 );

        if ( ( S0.count( tf0 ) == 1 )  &&  
             ( S1.count( tf1 ) == 1 )  &&  
             ( S2.count( tf2 ) == 1 )  &&  
             ( S3.count( tf3 ) == 1 )  &&  
             ( B0.count( te0 ) == 1 )  &&  
             ( B1.count( te1 ) == 1 )  &&  
             ( B2.count( te2 ) == 1 )  &&  
             ( B3.count( te3 ) == 1 )  )  
            totalSet.erase( it++ );
        else
            ++it;
    }
}

void reduceNSFromInner_to_back( const NS & x, set< NS > & totalSet )
{
    // inner to front
    Word f0, f1, f2, f3;
    State_to_Word( x, f0, f1, f2, f3 );

    set<Word> S0, S1, S2, S3;
    PassSuperSbox( f0, table, S0 ); 
    PassSuperSbox( f1, table, S1 ); 
    PassSuperSbox( f2, table, S2 ); 
    PassSuperSbox( f3, table, S3 ); 

    for ( auto it = totalSet.begin(); it != totalSet.end(); )
    {
        auto t = *it;

        Word tf0, tf1, tf2, tf3;

        State_to_Word( t, tf0, tf1, tf2, tf3 );

        if ( ( S0.count( tf0 ) == 1 )  &&  
             ( S1.count( tf1 ) == 1 )  &&  
             ( S2.count( tf2 ) == 1 )  &&  
             ( S3.count( tf3 ) == 1 )  )  
            totalSet.erase( it++ );
        else
            ++it;
    }
}

void reduceNSFromInner_to_front( const NS & x, set< NS > & totalSet )
{
    // inner to front
    Word f0, f1, f2, f3;
    State_to_Word( x, f0, f1, f2, f3 );

    set<Word> S0, S1, S2, S3;
    InvPassSuperSbox( f0, Invtable, S0 ); 
    InvPassSuperSbox( f1, Invtable, S1 ); 
    InvPassSuperSbox( f2, Invtable, S2 ); 
    InvPassSuperSbox( f3, Invtable, S3 ); 

    for ( auto it = totalSet.begin(); it != totalSet.end(); )
    {
        auto t = *it;

        Word tf0, tf1, tf2, tf3;

        State_to_Word( t, tf0, tf1, tf2, tf3 );

        if ( ( S0.count( tf0 ) == 1 )  &&  
             ( S1.count( tf1 ) == 1 )  &&  
             ( S2.count( tf2 ) == 1 )  &&  
             ( S3.count( tf3 ) == 1 )  )  
            totalSet.erase( it++ );
        else
            it++;
    }
}


void ReduceSet( set<Word> & S, const set<Word> & T )
{
    set<Word> tmp;
    set_difference( S.begin(), S.end(), T.begin(), T.end(), insert_iterator< set<Word> >( tmp, tmp.begin() ) );
    S.swap( tmp );

    if ( S.empty() )
        S.insert( 0 );
}

