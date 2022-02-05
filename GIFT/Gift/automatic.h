#ifndef __AUTOMATIC_H__
#define __AUTOMATIC_H__

#include<set>
#include"basic.h"


using std::set;
using std::pair;


void Worker( const std::set<NS> &,  const std::set<NS> &, std::set<
        std::pair<NS, NS> > & );

void getBadPatternX( const set<Word> & frontIS0,
                     const set<Word> & frontIS1,
                     const set<Word> & frontIS2,
                     const set<Word> & frontIS3,

                     const set<Word> & backIS0,
                     const set<Word> & backIS1,
                     const set<Word> & backIS2,
                     const set<Word> & backIS3,

                     set< pair<NS,NS> > & badPattern
                     );

void getBadPattern(  const std::set<Word> &, 
                     const std::set<Word> &, 
                     const std::set<Word> &, 
                     const std::set<Word> &, 
                     const std::set<Word> &, 
                     const std::set<Word> &, 
                     const std::set<Word> &, 
                     const std::set<Word> &, 

                     std::set<Word> &,
                     std::set<Word> &,
                     std::set<Word> &,
                     std::set<Word> &,

                     std::set<Word> &,
                     std::set<Word> &,
                     std::set<Word> &,
                     std::set<Word> &,

                     std::set< std::pair<NS,NS> > &, int
                     );


void processBadPattern ( std::pair<NS, NS> & badPattern, 
                        const std::map<Word, set<Word>> & frontMapIS0, 
                        const std::map<Word, set<Word>> & frontMapIS1,
                        const std::map<Word, set<Word>> & frontMapIS2,
                        const std::map<Word, set<Word>> & frontMapIS3,
                        const std::map<Word, set<Word>> & backMapIS0,
                        const std::map<Word, set<Word>> & backMapIS1, 
                        const std::map<Word, set<Word>> & backMapIS2,
                        const std::map<Word, set<Word>> & backMapIS3,
                        std::set<std::pair<NS, NS> > & impossible, 
                        std::set<std::pair<NS, NS> > & truncated, 
                        std::set<std::pair<NS, NS> > & pTruncated, 
                        std::set<std::pair<NS, NS> > & cTruncated ); 


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
