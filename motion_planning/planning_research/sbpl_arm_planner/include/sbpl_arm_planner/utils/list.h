/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
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
