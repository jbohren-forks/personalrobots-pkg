#ifndef __HEAP_H_
#define __HEAP_H_

//the maximum size of the heap
#define HEAPSIZE 20000000 
#define HEAPSIZE_INIT 5000

struct HEAPELEMENT
{
  AbstractSearchState *heapstate;
  CKey key;
};
typedef struct HEAPELEMENT heapelement;

class CHeap
{

//data
public:
  int percolates;		//for counting purposes
  heapelement* heap;
  int currentsize;
  int allocated;

//constructors
public:
  CHeap();
  ~CHeap();

//functions
public:

  bool emptyheap(); 
  bool fullheap();
  bool inheap(AbstractSearchState *AbstractSearchState);
  CKey getkeyheap(AbstractSearchState *AbstractSearchState);
  void makeemptyheap();
  void insertheap(AbstractSearchState *AbstractSearchState, CKey key);
  void deleteheap(AbstractSearchState *AbstractSearchState);
  void updateheap(AbstractSearchState *AbstractSearchState, CKey NewKey);
  AbstractSearchState *getminheap();
  AbstractSearchState *getminheap(CKey& ReturnKey);
  CKey getminkeyheap();
  AbstractSearchState *deleteminheap();
  void makeheap();

private:
  void percolatedown(int hole, heapelement tmp);
  void percolateup(int hole, heapelement tmp);
  void percolateupordown(int hole, heapelement tmp);

  void growheap();
  void sizecheck();


//operators
public:

};


#endif



