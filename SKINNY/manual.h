#ifndef __MANUAL_H__
#define __MANUAL_H__

#include<set>
#include"basic.h"

using namespace std;

std::set<std::pair<NS, NS>> getTruncatedPatternFromSamples( const std::string &,
        std::pair<NS, NS> & );

void removeTruncated (   std::pair<NS, NS> &, 
                         const std::set<Word> &, 
                         const std::set<Word> &, 
                         const std::set<Word> &, 
                         const std::set<Word> &, 
                         const std::set<Word> &, 
                         const std::set<Word> &, 
                         const std::set<Word> &, 
                         const std::set<Word> &, 
                         const std::set<Word> &, 
                         const std::set<Word> &,
                         const std::set<Word> &,
                         const std::set<Word> &,
                         const std::set<Word> &,
                         const std::set<Word> &, 
                         const std::set<Word> &,
                         const std::set<Word> &,
                         std::set< std::pair<BS, BS> > &, 
                         const std::set<std::pair<NS, NS> > & );

std::set<std::pair<NS, NS>> getBadPatternSamples( std::string );

void processBadPatternManual (  
                         const set<Word> & start0, 
                         const set<Word> & start1, 
                         const set<Word> & start2, 
                         const set<Word> & start3, 
                         const set<Word> & end0, 
                         const set<Word> & end1, 
                         const set<Word> & end2, 
                         const set<Word> & end3, 
                         set<pair<BS, BS> > & impossible, 
                         set<pair<NS, NS> > & truncated );
#endif
