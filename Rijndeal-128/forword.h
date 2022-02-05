#ifndef __FORWORD_H__
#define __FORWORD_H__

#include<set>
#include<map>
#include"basic.h"

const int SELECT_NUMBER = 1000;

void getPatterns ( const std::set<Word> & P, std::set<Word> & seed );

void PassSuperSbox( Word,  std::set<Word> & );

Word Select_inner_to_front( const std::set<Word>&, const std::set<Word>& ); // TODO

std::set<Word> getIS_front( const std::set<Word> &  );

void NarrowSet_inner_to_front( std::set<Word> &, const std::set<Word> & ); 

void InitializeMapFront( const std::set<Word> & S, std::map<Word, std::set<Word>> & Map, const std::set<Word> &  );

#endif
