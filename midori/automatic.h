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

void getBadPatternX( std::set<Word> &,
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

void processBadPattern ( pair<NS, NS> & badPattern, 
                        const map<Word, set<Word>> &,
                        const map<Word, set<Word>> &,
                        const map<Word, set<Word>> &,
                        const map<Word, set<Word>> &,
                        const map<Word, set<Word>> &,
                        const map<Word, set<Word>> &,
                        const map<Word, set<Word>> &,
                        const map<Word, set<Word>> &,
                         set<pair<NS, NS> > & impossible, 
                         set<pair<NS, NS> > & truncated );

void getIntersectionOfBadPattern ( const set< pair<NS, NS> > & bp,
                                    const map<Word, set<Word>> & fmap0,
                                    const map<Word, set<Word>> & fmap1,
                                    const map<Word, set<Word>> & fmap2,
                                    const map<Word, set<Word>> & fmap3,
                                    const map<Word, set<Word>> & bmap0,
                                    const map<Word, set<Word>> & bmap1,
                                    const map<Word, set<Word>> & bmap2,
                                    const map<Word, set<Word>> & bmap3,
                                    set<Word> & S0, 
                                    set<Word> & S1, 
                                    set<Word> & S2, 
                                    set<Word> & S3,  
                                    
                                    set<Word> & E0, 
                                    set<Word> & E1,  
                                    set<Word> & E2,
                                    set<Word> & E3 ); 
#endif
