#include"skinny.h"
#include"basic.h"

/*
1 0 1 1
1 0 0 0
0 1 1 0
1 0 1 0 
*/

Word MixColumn( Word x )
{
    Nibble x0, x1, x2, x3;
    Word_to_Nibble ( x, x0, x1, x2, x3 );

    Nibble y0, y1, y2, y3;
    y0 = x0 ^ x2 ^ x3;
    y1 = x1 ^ x3; 
    y2 = x2;
    y3 = x3;

    return Nibble_to_Word( y0, y1, y2, y3 );
}

/*
   0 1 0 0 
   0 1 1 1
   0 1 0 1
   1 0 0 1
*/
Word InvMixColumn( Word x )
{
    Nibble x0, x1, x2, x3;
    Word_to_Nibble ( x, x0, x1, x2, x3 );

    Nibble y0, y1, y2, y3;
    y0 = x0 ^ x2 ^ x3;
    y1 = x1 ^ x3;
    y2 = x2;
    y3 = x3;

    return Nibble_to_Word( y0, y1, y2, y3 );
}

void MC( NS & s )
{
    Word x0, x1, x2, x3;
    State_to_Word( s, x0, x1, x2, x3 );

    x0 = MixColumn( x0 );
    x1 = MixColumn( x1 );
    x2 = MixColumn( x2 );
    x3 = MixColumn( x3 );
     
    Word_to_State( x0, x1, x2, x3, s );
}

void InvMC( NS & s )
{
    Word x0, x1, x2, x3;
    State_to_Word( s, x0, x1, x2, x3 );

    x0 = InvMixColumn( x0 );
    x1 = InvMixColumn( x1 );
    x2 = InvMixColumn( x2 );
    x3 = InvMixColumn( x3 );
     
    Word_to_State( x0, x1, x2, x3, s );
}

void ShiftRows( NS & s )
{
    Nibble tmp[16];
    for ( int i = 0; i < 16; i++ )
        tmp[i] = s[i];

    s[0] = tmp[15];
    s[1] = tmp[12];
    s[2] = tmp[13];
    s[3] = tmp[14];

    s[4] = tmp[10];
    s[5] = tmp[9];
    s[6] = tmp[8];
    s[7] = tmp[11];

    s[8] = tmp[6];
    s[9] = tmp[5];
    s[10] = tmp[4];
    s[11] = tmp[7];

    s[12] = tmp[1];
    s[13] = tmp[2];
    s[14] = tmp[3];
    s[15] = tmp[0];
}

void InvShiftRows( NS & s )
{
    Nibble tmp[16];
    for ( int i = 0; i < 16; i++ )
        tmp[i] = s[i];

    s[0] = tmp[15];
    s[1] = tmp[12];
    s[2] = tmp[13];
    s[3] = tmp[14];

    s[4] = tmp[10];
    s[5] = tmp[9];
    s[6] = tmp[8];
    s[7] = tmp[11];

    s[8] = tmp[6];
    s[9] = tmp[5];
    s[10] = tmp[4];
    s[11] = tmp[7];

    s[12] = tmp[1];
    s[13] = tmp[2];
    s[14] = tmp[3];
    s[15] = tmp[0];
}


