#ifndef __BASIC_H__
#define __BASIC_H__

#include<set>
#include<iostream>
#include<array>
#include<tuple>
#include<map>
extern int ROUND;
extern int ** table;
extern int ** Invtable;

typedef unsigned short int Word;
typedef unsigned char Nibble;
typedef unsigned char Bit;
typedef std::array<Nibble, 16> NS; // state as Nibbles
typedef std::array<Bit, 64> BS; // state as Bits

void RemoveSymmetry( std::set<NS> & S );
using std::set;
using std::map;

/*
void getBadWord( const set< pair<NS, NS>> & bad, 
        set<Word> & H0, 
        set<Word> & H1, 
        set<Word> & H2, 
        set<Word> & H3, 
        set<Word> & E0, 
        set<Word> & E1, 
        set<Word> & E2, 
        set<Word> & E3 ); 
*/

Word Nibble_to_Word( Nibble, Nibble, Nibble, Nibble );
void Word_to_Nibble( Word, Nibble&, Nibble&, Nibble&, Nibble& );
void Nibble_to_Bit( const NS & , BS & );
void Bit_to_Nibble( const BS &, NS & );
void State_to_Word( const NS &, Word&, Word&, Word&, Word& );
void Word_to_State( Word, Word, Word, Word, NS &  );

NS getRandom( const std::set<NS>& );
Word getRandom( const std::set<Word>& );
std::pair<NS, NS> getRandom( const std::set< std::pair<NS, NS> >& );

void genDDT( const int *, int ** table );
void printNibbleState( const NS &, std::ostream & os = std::cout );
void printS( const NS & );
std::set<Nibble>* InvSboxDDT( const NS & );

// is truncated
bool sat( const std::pair< NS, NS > &, const std::pair<NS, NS> & );

bool satSet( const std::pair< NS, NS > &,  const std::set< std::pair<NS, NS> > & ); 
void reduceTotalSetFromTruncated( const std::pair<NS, NS> & x, std::set< std::pair<NS, NS> > & totalSet );
void reduceTotalSetFromInner( const std::pair<NS, NS> & x, std::set< std::pair<NS, NS> > & totalSet );

void ReduceSet( std::set<Word> &, const std::set<Word> & );

void reduceNSFromTruncated( const NS &, std::set<NS> & );

void reduceNSFromInner_to_front( const NS & x, std::set< NS > & totalSet );
void reduceNSFromInner_to_back( const NS & x, std::set< NS > & totalSet );
#endif

