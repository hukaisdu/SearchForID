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

void getDifferentBadPattern( const set<Word> & S0,  
                    const set<Word> & S1,  
                    const set<Word> & S2,  
                    const set<Word> & S3,  
                    const set<Word> & E0,  
                    const set<Word> & E1,  
                    const set<Word> & E2,  
                    const set<Word> & E3,  
                        
                     set<Word> & frontIS0,
                     set<Word> & frontIS1,
                     set<Word> & frontIS2,
                     set<Word> & frontIS3,

                     set<Word> & backIS0,
                     set<Word> & backIS1,
                     set<Word> & backIS2,
                     set<Word> & backIS3,

                     set< pair<NS,NS> > & badPattern, int
                     );

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
int processBadPattern ( std::pair<NS, NS> & badPattern, 
                        const std::set<Word> & S0, 
                        const std::set<Word> & S1, 
                        const std::set<Word> & S2, 
                        const std::set<Word> & S3, 
                        const std::set<Word> & E0, 
                        const std::set<Word> & E1, 
                        const std::set<Word> & E2, 
                        const std::set<Word> & E3, 
                         const map<Word, set<Word>> & frontMapIS0, 
                         const map<Word, set<Word>> & frontMapIS1, 
                         const map<Word, set<Word>> & frontMapIS2, 
                         const map<Word, set<Word>> & frontMapIS3, 
                         const map<Word, set<Word>> & backMapIS0, 
                         const map<Word, set<Word>> & backMapIS1, 
                         const map<Word, set<Word>> & backMapIS2, 
                         const map<Word, set<Word>> & backMapIS3, 
                         std::set<std::pair<NS, NS> > & impossible, 
                         std::set<std::pair<NS, NS> > & truncated ); 

void getIntersectionOfBadPattern ( const set< pair<NS, NS> > & bp,
                                    set<Word> & S0, 
                                    set<Word> & S1, 
                                    set<Word> & S2, 
                                    set<Word> & S3,  
                                    
                                    set<Word> & E0, 
                                    set<Word> & E1,  
                                    set<Word> & E2,
                                    set<Word> & E3 ); 
#endif
