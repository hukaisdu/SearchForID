#ifndef __FORWORD_H__
#define __FORWORD_H__

#include<set>
#include<map>
#include"basic.h"

const int SELECT_NUMBER = 1000;

void generateSuperDDT( int ** table );

void PassSuperSbox( Word,  int **, std::set<Word> & );
Word Select_inner_to_front( int **, const std::set<Word>&, const std::set<Word>&, int select_number ); // TODO

std::set<Word> getIS_front( int **, int **, const std::set<Word> &, int
        select_number = SELECT_NUMBER );

void NarrowSet_inner_to_front( std::set<Word> &, int **, const std::set<Word> & ); 

std::set<Word> findUnique_inner_to_front( Word, int**, std::set<Word>, const
        std::set<Word> & );

void Reduce_inner_to_front( Word, int ** , std::set<Word> & );

void InitializeMapFront( const std::set<Word> &, int **, std::map<Word,
        std::set<Word>> &, const std::set<Word> & );

void getForwardPattern( const std::set<Word> &, std::set<Word> &, int );



#endif
