#include "../headers.h"


void heaperror(char* ErrorString)
{
  //need to send a message from here somehow
	printf("%s\n", ErrorString);
	exit(1);
}


//returns an infinite key
CKey InfiniteKey()
{
	CKey key;
	key.SetKeytoInfinity();
	return key;

}


//constructors and destructors
CHeap::CHeap()
{
  percolates = 0;
  currentsize = 0;
  allocated = HEAPSIZE_INIT;

  heap = new heapelement[allocated];

}

CHeap::~CHeap()
{
  int i;
  for (i=1; i<=currentsize; ++i)
    heap[i].heapstate->heapindex = 0;

  delete [] heap;
}


void CHeap::percolatedown(int hole, heapelement tmp)
{
  int child;

  if (currentsize != 0)
  {

    for (; 2*hole <= currentsize; hole = child)
	{
	  child = 2*hole;

	  if (child != currentsize && heap[child+1].key < heap[child].key)
	    ++child;
	  if (heap[child].key < tmp.key)
	    {
	      percolates += 1;
	      heap[hole] = heap[child];
	      heap[hole].heapstate->heapindex = hole;
	    }
	  else
	    break;
	}
      heap[hole] = tmp;
      heap[hole].heapstate->heapindex = hole;
   }
}

void CHeap::percolateup(int hole, heapelement tmp)
{
  if (currentsize != 0)
    {
      for (; hole > 1 && tmp.key < heap[hole/2].key; hole /= 2)
	  {
		percolates += 1;
		heap[hole] = heap[hole/2];
		heap[hole].heapstate->heapindex = hole;
	  }  
      heap[hole] = tmp;
      heap[hole].heapstate->heapindex = hole;
    }
}

void CHeap::percolateupordown(int hole, heapelement tmp)
{
  if (currentsize != 0)
    {
      if (hole > 1 && heap[hole/2].key > tmp.key)
		percolateup(hole, tmp);
      else
		percolatedown(hole, tmp);
    }
}

bool CHeap::emptyheap()
{
  return currentsize == 0;
}


bool CHeap::fullheap()
{
  return currentsize == HEAPSIZE-1;
}

bool CHeap::inheap(AbstractSearchState *AbstractSearchState)
{
  return (AbstractSearchState->heapindex != 0);
}


CKey CHeap::getkeyheap(AbstractSearchState *AbstractSearchState)
{
  if (AbstractSearchState->heapindex == 0)
    heaperror("GetKey: AbstractSearchState is not in heap");

  return heap[AbstractSearchState->heapindex].key;
}

void CHeap::makeemptyheap()
{
  int i;

  for (i=1; i<=currentsize; ++i)
    heap[i].heapstate->heapindex = 0;
  currentsize = 0;
}

void CHeap::makeheap()
{
  int i;

  for (i = currentsize / 2; i > 0; i--)
    {
      percolatedown(i, heap[i]);
    }
}

void CHeap::growheap()
{
  heapelement* newheap;
  int i;

  printf("growing heap size from %d ", allocated);

  allocated = 2*allocated;
  if(allocated > HEAPSIZE)
	  allocated = HEAPSIZE;

  printf("to %d\n", allocated);

  newheap = new heapelement[allocated];

  for (i=0; i<=currentsize; ++i)
    newheap[i] = heap[i];

  delete [] heap;

  heap = newheap;
}


void CHeap::sizecheck()
{

  if (fullheap())
    heaperror("insertheap: heap is full");
  else if(currentsize == allocated-1)
  {
	growheap();
  }
}



void CHeap::insertheap(AbstractSearchState *AbstractSearchState, CKey key)
{
  heapelement tmp;
  char strTemp[100];

  sizecheck();

  if (AbstractSearchState->heapindex != 0)
    {
      sprintf(strTemp, "insertheap: AbstractSearchState is already in heap");
      heaperror(strTemp);
    }
  tmp.heapstate = AbstractSearchState;
  tmp.key = key;
  percolateup(++currentsize, tmp); 
}

void CHeap::deleteheap(AbstractSearchState *AbstractSearchState)
{
  if (AbstractSearchState->heapindex == 0)
    heaperror("deleteheap: AbstractSearchState is not in heap");
  percolateupordown(AbstractSearchState->heapindex, heap[currentsize--]);
  AbstractSearchState->heapindex = 0;
}

void CHeap::updateheap(AbstractSearchState *AbstractSearchState, CKey NewKey)
{
  if (AbstractSearchState->heapindex == 0)
    heaperror("Updateheap: AbstractSearchState is not in heap");
  if (heap[AbstractSearchState->heapindex].key != NewKey)
    {
      heap[AbstractSearchState->heapindex].key = NewKey;
      percolateupordown(AbstractSearchState->heapindex, heap[AbstractSearchState->heapindex]);
    }
}

AbstractSearchState* CHeap::getminheap()
{
  if (currentsize == 0)
    heaperror("GetMinheap: heap is empty");
  return heap[1].heapstate;
}

AbstractSearchState* CHeap::getminheap(CKey& ReturnKey)
{
  if (currentsize == 0)
    {
      heaperror("GetMinheap: heap is empty");
      ReturnKey = InfiniteKey();
    }
  ReturnKey = heap[1].key;
  return heap[1].heapstate;
}

CKey CHeap::getminkeyheap()
{  
  CKey ReturnKey;
  if (currentsize == 0)
    return InfiniteKey();
  ReturnKey = heap[1].key;
  return ReturnKey;
}

AbstractSearchState* CHeap::deleteminheap()
{
  AbstractSearchState *AbstractSearchState;

  if (currentsize == 0)
    heaperror("DeleteMin: heap is empty");

  AbstractSearchState = heap[1].heapstate;
  AbstractSearchState->heapindex = 0;
  percolatedown(1, heap[currentsize--]);
  return AbstractSearchState;
}

