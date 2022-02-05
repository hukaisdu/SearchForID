#ifndef __BACKWORD_H__
#define __BACKWORD_H__

#include<set>
#include"forword.h"

void generateInvSuperDDT( int ** Invtable );

void InvPassSuperSbox( Word, int **, std::set<Word>& );
Word Select_inner_to_back( int **, const std::set<Word> &, const std::set<Word> & );
std::set<Word> getIS_back(int **, int **, const std::set<Word> &, int
        select_number = SELECT_NUMBER );
std::set<Word> findUnique_inner_to_back( Word, int**, std::set<Word>, const
        std::set<Word> & );
void NarrowSet_inner_to_back( std::set<Word> &, int **, const std::set<Word> & total );

void Reduce_inner_to_back( Word, int **, std::set<Word> & );


void InitializeMapBack( const std::set<Word> &, int **, std::map<Word,
        std::set<Word>> &, const std::set<Word> & );

void getBackwardPattern( const std::set<Word> &, std::set<Word> &, int );

#endif
