#include<iostream>
#include<set>
using namespace std;

int SboxIneq[][9] = 
{{2, 0, -1, 0, 0, 0, -1, 0, -1}, {0, 0, -1, 0, 0, 0, 1, 0, 1}, {0, 0, 1, 0, 0, 0, 1, 0, -1}, {0, 0, 1, 0, 0, 0, -1, 0, 1}, {2, -1, 0, 0, 0, 1, 0, -1, -1}, {2, 1, 0, 0, 0, -1, 0, -1, -1}, {2, -1, 0, 0, 0, -1, 0, 1, -1}, {0, 1, 0, 0, 0, -1, 0, 1, 1}, {0, 1, 0, 0, 0, 1, 0, -1, 1}, {0, 1, 0, 0, 0, 1, 0, 1, -1}, {0, -1, 0, 0, 0, 1, 0, 1, 1}, {2, -1, 0, 0, 0, -1, 0, -1, 1}};

int Eq[][9] = 
{{0, 0, 0, 0, 1, 0, 0, 0, -1}, {0, 0, 0, 1, 0, 0, 0, -1, 0}};

int main()
{
    for (  int i = 0; i < 256; i++ )
    {
        int x0 = i >> 7 & 0x1;
        int x1 = i >> 6 & 0x1;
        int x2 = i >> 5 & 0x1;
        int x3 = i >> 4 & 0x1;
        int y0 = i >> 3 & 0x1;
        int y1 = i >> 2 & 0x1;
        int y2 = i >> 1 & 0x1;
        int y3 = i >> 0 & 0x1;

        bool flag = true;

        for ( auto it : SboxIneq ) 
            if ( it[0]  + 
            it[1] * x0  +  
            it[2] * x1  +  
            it[3] * x2  +  
            it[4] * x3  +  
            it[5] * y0  +  
            it[6] * y1  +  
            it[7] * y2  +  
            it[8] * y3  < 0 ) 
            {
                flag = false;
                break;
            }

        if ( flag == true )
        {
        for ( auto it : Eq ) 
            if ( it[0]  + 
            it[1] * x0  +  
            it[2] * x1  +  
            it[3] * x2  +  
            it[4] * x3  +  
            it[5] * y0  +  
            it[6] * y1  +  
            it[7] * y2  +  
            it[8] * y3  != 0 ) 
            {
                flag = false;
                break;
            }
        }

        if ( flag == true )
            cout << hex << "("<< ( i >> 4 & 0xf )
                 << "," << ( i & 0xf ) << ")" << endl;

   }

}
