#ifndef __LIST_H_
#define __LIST_H_

//the maximum size of the heap
#define LISTSIZE 5000000

struct listelement
{
  AbstractSearchState *liststate;
  struct listelement *prev;
  struct listelement *next;
};
typedef struct listelement listelement; 

class CList
{

//data
public:

	listelement* firstelement;
	int currentsize;

//constructors
public:
	CList() 
	  {
	    firstelement = NULL;
	    currentsize = 0;
	  };
	~CList()
	{};

//functions
public:
  bool empty()
    {return (currentsize == 0);};
  bool full()
    {return (currentsize >= LISTSIZE);};
  bool in(AbstractSearchState *AbstractSearchState1, int listindex)
    { 
      return (AbstractSearchState1->listelem[listindex] != NULL);
    };
  void makeemptylist(int listindex)
    {
      while(firstelement != NULL)
	remove(firstelement->liststate, listindex);
    };
  void insert(AbstractSearchState *AbstractSearchState1, int listindex)
    {
      if(currentsize >= LISTSIZE)
	{
	  printf("ERROR: list is full\n");
	  exit(1);
	}
      if(AbstractSearchState1->listelem[listindex] != NULL)
	{
	  printf("ERROR: insert: element is already in the list\n");
	  exit(1);
	}
      listelement *insertelem = (listelement*)malloc(sizeof(listelement));
      insertelem->liststate = AbstractSearchState1;
      insertelem->prev = NULL;
      insertelem->next = firstelement;
      if(firstelement != NULL)
	firstelement->prev = insertelem;
      firstelement = insertelem;
      AbstractSearchState1->listelem[listindex] = insertelem;
      currentsize++;
    };
 void insertinfront(AbstractSearchState *AbstractSearchState1, int listindex)
    {
      insert(AbstractSearchState1, listindex);
    };
  void remove(AbstractSearchState *AbstractSearchState1, int listindex)
    {
      if(currentsize == 0 || AbstractSearchState1->listelem[listindex] == NULL)
	{
	  printf("ERROR: delete: list does not contain the element\n");
	  exit(1);
	}
      if(AbstractSearchState1->listelem[listindex]->prev != NULL && 
	 AbstractSearchState1->listelem[listindex]->next != NULL)
	{
	  //in the middle of the list
	  AbstractSearchState1->listelem[listindex]->prev->next = 
	    AbstractSearchState1->listelem[listindex]->next;
	  AbstractSearchState1->listelem[listindex]->next->prev = 
	    AbstractSearchState1->listelem[listindex]->prev;
	}
      else if(AbstractSearchState1->listelem[listindex]->prev != NULL)
	{
	  //at the end of the list
	  AbstractSearchState1->listelem[listindex]->prev->next = NULL;
	}
      else if(AbstractSearchState1->listelem[listindex]->next != NULL)
	{
	  //at the beginning of the list
	  AbstractSearchState1->listelem[listindex]->next->prev = NULL;
	  firstelement = AbstractSearchState1->listelem[listindex]->next;
	}
      else
	{
	  //the only element in the list
	  firstelement = NULL;
	}
      //delete
      free(AbstractSearchState1->listelem[listindex]);
      AbstractSearchState1->listelem[listindex] = NULL;
      currentsize--;
    };
  AbstractSearchState *getfirst()
    {
      if(firstelement == NULL)
	return NULL;
      else
	return firstelement->liststate;
    };
  AbstractSearchState *getnext(AbstractSearchState* AbstractSearchState1, int listindex)
    {
      if(AbstractSearchState1->listelem[listindex]->next == NULL)
	return NULL;
      else
	return AbstractSearchState1->listelem[listindex]->next->liststate;
    };

private:



//operators
public:


};




#endif
