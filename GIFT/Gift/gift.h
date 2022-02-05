#ifndef __GIFT_H__
#define __GIFT_H__

#include"basic.h"

const int Sbox[16] = 
{
0x1, 0xa, 0x4, 0xc, 0x6, 0xf, 0x3, 0x9, 0x2, 0xd, 0xb, 0x7, 0x5, 0x0, 0x8,
0xe
};

const int InvSbox[16] = 
{
    0xd, 0x0, 0x8, 0x6, 0x2, 0xc, 0x4, 0xb, 0xe, 0x7, 0x1, 0xa, 0x3, 0x9, 0xf,
    0x5
};

const int SmallP[16] = { 0, 5, 10, 15, 12, 1, 6, 11, 8, 13, 2, 7, 4, 9, 14, 3 };

const int BitPerm[] = { 0, 17, 34, 51, 48, 1, 18, 35, 32, 49, 2, 19, 16, 33, 50, 3, 
           4, 21, 38, 55, 52, 5, 22, 39, 36, 53, 6, 23, 20, 37, 54, 7, 
           8, 25, 42, 59, 56, 9, 26, 43, 40, 57, 10, 27, 24, 41, 58, 11, 
           12, 29, 46, 63, 60, 13, 30, 47, 44, 61, 14, 31, 28, 45, 62, 15 };

Word SmallBitPerm( Word );
Word InvSmallBitPerm( Word );

//Word InvMixColumn( Word );
//void MC( NS & );
//void InvMC( NS & );
//void ShiftRows( NS & );
//void InvShiftRows( NS & );
void InitializePerm();

#endif
