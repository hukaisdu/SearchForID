#include"basic.h"
#include"forword.h"
#include"backword.h"
#include<set>
#include<vector>
#include<iterator>
#include<random>
#include<iostream>
#include<algorithm>

using namespace std;


bool sat( const pair< NS, NS > & a, const pair<NS, NS> & b ) 
{
    for ( int i = 0; i < 32; i++ )
        if ( ( ( a.first[i] == 0 ) && ( b.first[i] != 0 ) ) || ( ( a.first[i] != 0
                    ) && ( b.first[i] == 0 ) )  )
            return false;

    for ( int i = 0; i < 32; i++ )
        if ( ( ( a.second[i] == 0 ) && ( b.second[i] != 0 ) ) || ( ( a.second[i] != 0
                    ) && ( b.second[i] == 0 ) )  )
            return false;
    return true;
}

bool satSingle( const NS & a, const NS & b ) 
{
    for ( int i = 0; i < 32; i++ )
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
    for ( int i = 0; i < 128; i++ )
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

bool satSetPlaintext( const pair< NS, NS > & a, const set< pair<NS, NS> > & S ) 
{
    for ( auto it : S )
        if ( ( satSingle( a.first, it.first ) == true ) && ( a.second ==
                    it.second ) )
            return true;
    return false;
}

bool satSetCiphertext( const pair< NS, NS > & a, const set< pair<NS, NS> > & S ) 
{
    for ( auto it : S )
        if ( ( satSingle( a.second, it.second ) == true ) && ( a.first ==
                    it.first ) )
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
    for ( int i = 0; i < 32; i++ )
    {
        y[4 * i    ] = x[i] >> 3 & 0x1; 
        y[4 * i + 1] = x[i] >> 2 & 0x1; 
        y[4 * i + 2] = x[i] >> 1 & 0x1; 
        y[4 * i + 3] = x[i] >> 0 & 0x1; 
    }
}

void Bit_to_Nibble( const BS & x, NS & y )
{
    for ( int i = 0; i < 32; i++ )
        y[i] = ( x[4 * i] << 3 ) ^ ( x[4 * i + 1] << 2 ) ^ ( x[4 * i + 2] << 1 )
            ^ ( x[4 * i + 3] << 0 );
}

void State_to_Word( const NS & s, Word& x0, Word& x1, Word& x2, Word& x3, Word& x4, Word& x5, Word& x6, Word& x7 )
{
    x0 = Nibble_to_Word( s[0], s[1], s[2], s[3] );
    x1 = Nibble_to_Word( s[4], s[5], s[6], s[7] );
    x2 = Nibble_to_Word( s[8], s[9], s[10], s[11] );
    x3 = Nibble_to_Word( s[12], s[13], s[14], s[15] );
    x4 = Nibble_to_Word( s[16], s[17], s[18], s[19] );
    x5 = Nibble_to_Word( s[20], s[21], s[22], s[23] );
    x6 = Nibble_to_Word( s[24], s[25], s[26], s[27] );
    x7 = Nibble_to_Word( s[28], s[29], s[30], s[31] );
}

/*
void State_to_Word_Group_R( const NS & s, Word & x0, Word & x1, Word & x2, Word
        & x3 )
{
    x0 = Nibble_to_Word( s[0], s[4], s[8], s[12] );
    x1 = Nibble_to_Word( s[1], s[5], s[9], s[13] );
    x2 = Nibble_to_Word( s[2], s[6], s[10], s[14] );
    x3 = Nibble_to_Word( s[3], s[7], s[11], s[15] );
}

void Word_to_State_Group_R( Word x0, Word x1, Word x2, Word x3, NS & s )
{
    Nibble n0, n1, n2, n3;
    Word_to_Nibble( x0, n0, n1, n2, n3 );
    s[0]  = n0;
    s[4]  = n1;
    s[8]  = n2;
    s[12] = n3;

    Word_to_Nibble( x1, n0, n1, n2, n3 );
    s[1]  = n0;
    s[5]  = n1;
    s[9]  = n2;
    s[13] = n3;

    Word_to_Nibble( x2, n0, n1, n2, n3 );
    s[2]  = n0;
    s[6]  = n1;
    s[10] = n2;
    s[14] = n3;

    Word_to_Nibble( x3, n0, n1, n2, n3 );
    s[3]  = n0;
    s[7]  = n1;
    s[11] = n2;
    s[15] = n3;
}

void State_to_Word_Group_Q( const NS & s, Word & x0, Word & x1, Word & x2, Word
        & x3 )
{
    x0 = Nibble_to_Word( s[0], s[1], s[2], s[3] );
    x1 = Nibble_to_Word( s[4], s[5], s[6], s[7] );
    x2 = Nibble_to_Word( s[8], s[9], s[10], s[11] );
    x3 = Nibble_to_Word( s[12], s[13], s[14], s[15] );
}

void Word_to_State_Group_Q( Word x0, Word x1, Word x2, Word x3, NS & s )
{
    Nibble n0, n1, n2, n3;
    // 
    Word_to_Nibble( x0, n0, n1, n2, n3 );
    s[0] = n0;
    s[1] = n1;
    s[2] = n2;
    s[3] = n3;

    Word_to_Nibble( x1, n0, n1, n2, n3 );
    s[4] = n0;
    s[5] = n1;
    s[6] = n2;
    s[7] = n3;

    Word_to_Nibble( x2, n0, n1, n2, n3 );
    s[8]  = n0;
    s[9]  = n1;
    s[10] = n2;
    s[11] = n3;

    Word_to_Nibble( x3, n0, n1, n2, n3 );
    s[12] = n0;
    s[13] = n1;
    s[14] = n2;
    s[15] = n3;
}
*/

void Word_to_State( Word x0, Word x1, Word x2, Word x3, Word x4, Word x5, Word x6, Word x7, NS &s )
{
    Nibble z0, z1, z2, z3;
    Word_to_Nibble( x0, z0, z1, z2, z3 );
    s[0] = z0;
    s[1] = z1;
    s[2] = z2;
    s[3] = z3;

    Word_to_Nibble( x1, z0, z1, z2, z3 );
    s[4] = z0;
    s[5] = z1;
    s[6] = z2;
    s[7] = z3;

    Word_to_Nibble( x2, z0, z1, z2, z3 );
    s[8] = z0;
    s[9] = z1;
    s[10] = z2;
    s[11] = z3;

    Word_to_Nibble( x3, z0, z1, z2, z3 );
    s[12] = z0;
    s[13] = z1;
    s[14] = z2;
    s[15] = z3;

    Word_to_Nibble( x4, z0, z1, z2, z3 );
    s[16] = z0;
    s[17] = z1;
    s[18] = z2;
    s[19] = z3;

    Word_to_Nibble( x5, z0, z1, z2, z3 );
    s[20] = z0;
    s[21] = z1;
    s[22] = z2;
    s[23] = z3;

    Word_to_Nibble( x6, z0, z1, z2, z3 );
    s[24] = z0;
    s[25] = z1;
    s[26] = z2;
    s[27] = z3;

    Word_to_Nibble( x7, z0, z1, z2, z3 );
    s[28] = z0;
    s[29] = z1;
    s[30] = z2;
    s[31] = z3;
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
    for ( int i = 0; i < 32; i++ )
        os << hex << int ( s[i] ) << " ";
}

/*
void printS( const NS & a )
{
    for ( int i = 0; i < 4; i++ )
    {
        for ( int j = 0; j < 4; j++ )
            cout << hex << int ( a[4 * i + j] ) << " "; 
        cout << endl;
    }
}
*/

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
            it++;
}

void reduceTotalSetFromPlaintextTruncated( const pair<NS, NS> & x, set< pair<NS, NS> > & totalSet )
{
    for( auto it = totalSet.begin(); it != totalSet.end(); )
        if ( satSingle( x.first, (*it).first ) && ( x.second == (*it).second ) )
            totalSet.erase( it++ );
        else
            it++;
}

void reduceTotalSetFromCiphertextTruncated( const pair<NS, NS> & x, set< pair<NS, NS> > & totalSet )
{
    for( auto it = totalSet.begin(); it != totalSet.end(); )
        if ( satSingle( x.second, (*it).second ) && ( x.first == (*it).first ) )
            totalSet.erase( it++ );
        else
            it++;
}

void reduceNSFromTruncated( const NS & p, set<NS> & S )
{
    for( auto it = S.begin(); it != S.end(); )
        if ( satSingle( p, *it ) )
            S.erase( it++ );
        else
            it++;
}

// x is the inner values 
void reduceTotalSetFromInner( const pair<NS, NS> & x, set< pair<NS, NS> > & totalSet )
{
    // inner to front
    Word f0, f1, f2, f3, f4, f5, f6, f7;

    State_to_Word( x.first, f0, f1, f2, f3, f4, f5, f6, f7 );

    set<Word> S0, S1, S2, S3, S4, S5, S6, S7;

    InvPassSuperSbox( f0, Invtable, S0 ); 
    InvPassSuperSbox( f1, Invtable, S1 ); 
    InvPassSuperSbox( f2, Invtable, S2 ); 
    InvPassSuperSbox( f3, Invtable, S3 ); 
    InvPassSuperSbox( f4, Invtable, S4 ); 
    InvPassSuperSbox( f5, Invtable, S5 ); 
    InvPassSuperSbox( f6, Invtable, S6 ); 
    InvPassSuperSbox( f7, Invtable, S7 ); 

    Word b0, b1, b2, b3, b4, b5, b6, b7;

    State_to_Word( x.second, b0, b1, b2, b3, b4, b5, b6, b7 );

    set<Word> B0, B1, B2, B3, B4, B5, B6, B7;

    PassSuperSbox( b0, table, B0 );
    PassSuperSbox( b1, table, B1 );
    PassSuperSbox( b2, table, B2 );
    PassSuperSbox( b3, table, B3 );
    PassSuperSbox( b4, table, B4 );
    PassSuperSbox( b5, table, B5 );
    PassSuperSbox( b6, table, B6 );
    PassSuperSbox( b7, table, B7 );

    for ( auto it = totalSet.begin(); it != totalSet.end(); )
    {
        auto t = *it;

        Word tf0, tf1, tf2, tf3, tf4, tf5, tf6, tf7, te0, te1, te2, te3, te4, te5, te6, te7;

        State_to_Word( t.first, tf0, tf1, tf2, tf3, tf4, tf5, tf6, tf7 );

        State_to_Word( t.second, te0, te1, te2, te3, te4, te5, te6, te7 );

        if ( ( S0.count( tf0 ) == 1 )  &&  
             ( S1.count( tf1 ) == 1 )  &&  
             ( S2.count( tf2 ) == 1 )  &&  
             ( S3.count( tf3 ) == 1 )  &&  
             ( S4.count( tf4 ) == 1 )  &&  
             ( S5.count( tf5 ) == 1 )  &&  
             ( S6.count( tf6 ) == 1 )  &&  
             ( S7.count( tf7 ) == 1 )  &&  
             ( B0.count( te0 ) == 1 )  &&  
             ( B1.count( te1 ) == 1 )  &&  
             ( B2.count( te2 ) == 1 )  &&  
             ( B3.count( te3 ) == 1 )  &&  
             ( B4.count( te4 ) == 1 )  &&  
             ( B5.count( te5 ) == 1 )  &&  
             ( B6.count( te6 ) == 1 )  &&  
             ( B7.count( te7 ) == 1 )  )  
            totalSet.erase( it++ );
        else
            it++;
    }
}

set<Word> remaining( const set<Word> & S, Word  x)
{
    set<Word> SS;
    for ( auto it : S )
    {
        if ( it != x )
            SS.insert( it );
    }
    return SS;
}
