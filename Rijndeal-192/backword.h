#ifndef __BACKWORD_H__
#define __BACKWORD_H__

#include<set>
#include"forword.h"


void getInvPatterns(const std::set<Word> & E0, std::set<Word> & backIS0);

void InvPassSuperSbox( Word, std::set<Word>& );
Word Select_inner_to_back( const std::set<Word> &, const std::set<Word> & );

std::set<Word> getIS_back( const std::set<Word> & );

void NarrowSet_inner_to_back( std::set<Word> &, const std::set<Word> & total );

void InitializeMapBack( const std::set<Word> & S, std::map<Word,
        std::set<Word>> & Map, const std::set<Word> & );

#endif
