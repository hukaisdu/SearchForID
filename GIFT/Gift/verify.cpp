#include"basic.h"
#include"MILP.h"
#include<iostream>
#include<vector>
#include<set>
#include<thread>
#include<sstream>

using namespace std;

int ** table;
int ** Invtable;

int main()
{
    for ( int i = 1; i < 16; i++ )
    {
    NS a { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 }; 
    NS b { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, i }; 
    NS start, end;
    cout << i << " " << GIFT_Single( 6, a, b, start, end ) << endl;
    }
}    


