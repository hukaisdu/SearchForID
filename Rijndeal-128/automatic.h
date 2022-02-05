#ifndef __AUTOMATIC_H__
#define __AUTOMATIC_H__

#include<set>
#include<map>
#include"basic.h"


const int THREAD_NUMBER = 64;
const int THRESHOLD = 1000;

using std::set;
using std::pair;


void Worker( const std::set<NS> &,  const std::set<NS> &, std::set<
        std::pair<NS, NS> > & );

void getBadPattern(  std::set<Word> &,
                     std::set<Word> &,
                     std::set<Word> &,
                     std::set<Word> &,

                     std::set<Word> &,
                     std::set<Word> &,
                     std::set<Word> &,
                     std::set<Word> &,

                     std::set< std::pair<NS,NS> > &
                     );


using std::map;
using std::set;
void processBadPattern ( const std::pair<NS, NS> & badPattern, 
                         const map<Word, set<Word>> & frontMapIS0, 
                         const map<Word, set<Word>> & frontMapIS1, 
                         const map<Word, set<Word>> & frontMapIS2, 
                         const map<Word, set<Word>> & frontMapIS3, 
                         const map<Word, set<Word>> & backMapIS0, 
                         const map<Word, set<Word>> & backMapIS1, 
                         const map<Word, set<Word>> & backMapIS2, 
                         const map<Word, set<Word>> & backMapIS3, 
                         std::set<std::pair<NS, NS> > & impossible, int );

#endif
