#include"basic.h"
#include"gift.h"

static int Perm[65536];
static int InvPerm[65536];


Word SmallBitPerm( Word x )
{
    return Perm[x];
}

Word InvSmallBitPerm( Word x )
{
    return InvPerm[x];
}

Word SmallBitPermI( Word x )
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

Word InvSmallBitPermI( Word x )
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

void InitializePerm()
{
    for ( int i = 0; i < 65536; i++ )
    {
        Perm[i] = SmallBitPermI( i );
        InvPerm[i] = InvSmallBitPermI( i );
    }
}
