#include"basic.h"
#include<set>
#include<vector>
#include<iterator>
#include<random>
#include<iostream>
#include<algorithm>
#include"skinny.h"
#include"automatic.h"
#include"forword.h"
#include"backword.h"

using namespace std;

unsigned int rotate( unsigned int a, int n )
{
    return ( ( a << n ) | ( a >> ( 24 - n ) ) ) & 0xffffff;
}


unsigned int Nibble_to_Word( Nibble x0, Nibble x1, Nibble x2, Nibble x3, Nibble x4,
        Nibble x5 )
{
    return 
        ( static_cast<unsigned int> ( x0 ) << 20 ) ^
        ( static_cast<unsigned int> ( x1 ) << 16 ) ^
        ( static_cast<unsigned int> ( x2 ) << 12 ) ^
        ( static_cast<unsigned int> ( x3 ) << 8 ) ^
        ( static_cast<unsigned int> ( x4 ) << 4 ) ^
        ( static_cast<unsigned int> ( x5 ) << 0 );
}


bool SymmetricEq ( const NS & a, const NS & b )
{
    unsigned int a0, a1, a2, a3;
    unsigned b0, b1, b2, b3;

    a0 = Nibble_to_Word( a[0], a[1], a[2], a[3], a[4], a[5]);
    a1 = Nibble_to_Word( a[6], a[7], a[8], a[9], a[10], a[11] );
    a2 = Nibble_to_Word( a[12], a[13], a[14], a[15], a[16] , a[17]);
    a3 = Nibble_to_Word( a[18], a[19], a[20], a[21], a[22], a[23] );

    b0 = Nibble_to_Word( b[0], b[1], b[2], b[3], b[4], b[5]);
    b1 = Nibble_to_Word( b[6], b[7], b[8], b[9], b[10], b[11] );
    b2 = Nibble_to_Word( b[12], b[13], b[14], b[15], b[16] , b[17]);
    b3 = Nibble_to_Word( b[18], b[19], b[20], b[21], b[22], b[23] );

    if ( ( rotate( a0, 0 ) == b0 ) && 
         ( rotate( a1, 0 ) == b1 ) &&
         ( rotate( a2, 0 ) == b2 ) &&
         ( rotate( a3, 0 ) == b3 ) )
        return true;

    if ( ( rotate( a0, 4 ) == b0 ) && 
         ( rotate( a1, 4 ) == b1 ) &&
         ( rotate( a2, 4 ) == b2 ) &&
         ( rotate( a3, 4 ) == b3 ) )
        return true;

    if ( ( rotate( a0, 8 ) == b0 ) && 
         ( rotate( a1, 8 ) == b1 ) &&
         ( rotate( a2, 8 ) == b2 ) &&
         ( rotate( a3, 8 ) == b3 ) )
        return true;

    if ( ( rotate( a0, 12 ) == b0 ) && 
         ( rotate( a1, 12 ) == b1 ) &&
         ( rotate( a2, 12 ) == b2 ) &&
         ( rotate( a3, 12 ) == b3 ) )
        return true;

    if ( ( rotate( a0, 16 ) == b0 ) && 
         ( rotate( a1, 16 ) == b1 ) &&
         ( rotate( a2, 16 ) == b2 ) &&
         ( rotate( a3, 16 ) == b3 ) )
        return true;

    if ( ( rotate( a0, 20 ) == b0 ) && 
         ( rotate( a1, 20 ) == b1 ) &&
         ( rotate( a2, 20 ) == b2 ) &&
         ( rotate( a3, 20 ) == b3 ) )
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

void State_to_Word( const NS & s, Word& x0, Word& x1, Word& x2, Word& x3, Word &
        x4, Word & x5 )
{
    x0 = Nibble_to_Word( s[0], s[6], s[12], s[18] );
    x1 = Nibble_to_Word( s[1], s[7], s[13], s[19] );
    x2 = Nibble_to_Word( s[2], s[8], s[14], s[20] );
    x3 = Nibble_to_Word( s[3], s[9], s[15], s[21] );
    x4 = Nibble_to_Word( s[4], s[10], s[16], s[22] );
    x5 = Nibble_to_Word( s[5], s[11], s[17], s[23] );
}

void Word_to_State( Word x0, Word x1, Word x2, Word x3, Word x4, Word x5, NS &s )
{
    Nibble z0, z1, z2, z3;
    Word_to_Nibble( x0, z0, z1, z2, z3 );
    s[0] = z0;
    s[6] = z1;
    s[12] = z2;
    s[18] = z3;

    Word_to_Nibble( x1, z0, z1, z2, z3 );
    s[1] = z0;
    s[7] = z1;
    s[13] = z2;
    s[19] = z3;

    Word_to_Nibble( x2, z0, z1, z2, z3 );
    s[2] = z0;
    s[8] = z1;
    s[14] = z2;
    s[20] = z3;

    Word_to_Nibble( x3, z0, z1, z2, z3 );
    s[3] = z0;
    s[9] = z1;
    s[15] = z2;
    s[21] = z3;

    Word_to_Nibble( x4, z0, z1, z2, z3 );
    s[4] = z0;
    s[10] = z1;
    s[16] = z2;
    s[22] = z3;

    Word_to_Nibble( x5, z0, z1, z2, z3 );
    s[5] = z0;
    s[11] = z1;
    s[17] = z2;
    s[23] = z3;
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
    for ( int i = 0; i < 24; i++ )
        os << hex << int ( s[i] ) << " ";
}

void printS( const NS & a, ostream & os )
{
    for ( int i = 0; i < 4; i++ )
    {
        for ( int j = 0; j < 6; j++ )
            os << hex << int ( a[6 * i + j] ) << " "; 
        os << endl;
    }
}


