#ifndef __FORWORD_H__
#define __FORWORD_H__

#include<set>
#include<map>
#include"basic.h"

const int SELECT_NUMBER = 1000;

void getPatterns ( const std::set<Word> & P, std::set<Word> & seed, int select );
void PassSuperSbox( Word,  int **, std::set<Word> & );
Word Select_inner_to_front( int **, const std::set<Word>&, const std::set<Word>&, int select_number ); // TODO
std::set<Word> getIS_front( int **, int **, const std::set<Word> &, int
        select_number = SELECT_NUMBER );
void NarrowSet_inner_to_front( std::set<Word> &, int **, const std::set<Word> & ); 
std::set<Word> findUnique_inner_to_front( Word, int**, std::set<Word>, const
        std::set<Word> & );
void Reduce_inner_to_front( Word, int ** , std::set<Word> & );

void InitializeMapFront( const std::set<Word> & S, int ** Invtable, std::map<Word, std::set<Word>> & Map, const std::set<Word> &  );

#endif
