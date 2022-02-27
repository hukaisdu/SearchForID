#include<set>
#include<iostream>
#include<array>
#include<tuple>

using namespace std;

typedef unsigned short int Word;
typedef unsigned char Nibble;
typedef unsigned char Bit;

int SmallP[16] = { 0, 5, 10, 15, 12, 1, 6, 11, 8, 13, 2, 7, 4, 9, 14, 3 };

Word SmallBitPerm( Word x )
{
    Bit X[16] = { 0 }, Y[16] = { 0 };
    
    for ( int i = 0; i < 16; i++ )
        X[i] = x >> i & 0x1;

    for ( int i = 0; i < 16; i++ )
        Y[ SmallP[i] ] = X[i];

    x = 0;
    for ( int i = 0; i < 16; i++ )
        x ^= ( Y[i] << i );

    return x;
}

Word InvSmallBitPerm( Word x )
{
    Bit X[16] = { 0 }, Y[16] = { 0 };
    for ( int i = 0; i < 16; i++ )
        X[i] = x >> i & 0x1;

    for ( int i = 0; i < 16; i++ )
        Y[i] = X[ SmallP[i] ];

    x = 0;
    for ( int i = 0; i < 16; i++ )
        x ^= ( Y[i] << i );

    return x;
}

Word SuperSbox( Word x )
{
    
}

int main()
{
    array<Nibble, 3> a { 1, 2, 3 };
    array<Nibble, 3> b { 0, 2, 3 };

    cout << ( a == b ) << endl;
}
