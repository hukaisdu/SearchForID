#ifndef __BACKWORD_H__
#define __BACKWORD_H__

#include<set>
#include"forword.h"


void getInvPatterns(const std::set<Word> & E0, std::set<Word> & backIS0, int select_number);
void InvPassSuperSbox( Word, int **, std::set<Word>& );
Word Select_inner_to_back( int **, const std::set<Word> &, const std::set<Word> & );
std::set<Word> getIS_back(int **, int **, const std::set<Word> &, int
        select_number = SELECT_NUMBER );
std::set<Word> findUnique_inner_to_back( Word, int**, std::set<Word>, const
        std::set<Word> & );
void NarrowSet_inner_to_back( std::set<Word> &, int **, const std::set<Word> & total );

void Reduce_inner_to_back( Word, int **, std::set<Word> & );

void InitializeMapBack( const std::set<Word> & S, int ** table, std::map<Word,
        std::set<Word>> & Map, const std::set<Word> & );

#endif
