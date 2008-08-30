#ifndef __VEC_PERMUTATIONS_H__
#define __VEC_PERMUTATIONS_H__

#include <vector>
#include <iostream>

#include <stdlib.h>
#include <math.h>

using namespace std;

/**
   @brief Generates permutations of a label vector
 */
class VectorPermutations {
public:
  /**
     @param nLabels The number of distinct labels/states
     @param length The length of the permutation vector
   */
  VectorPermutations(int nLabels, int length) {
    perms = calcPerms(nLabels, length);
  }

  ~VectorPermutations() {
    freePerms(perms);
  }
  
  vector<vector<int>*>* getPerms() { 
    return perms; 
  }

  template<typename T>
  void map(T (*func)(vector<int>* perm),
	   vector<T> &out) {
    for (vector<vector<int>*>::iterator permPr = perms->begin();
	 permPr != perms->end();
	 permPr++) {
      out.push_back(func(*permPr));
    }
  }

  void print() {
    for (vector<vector<int>*>::iterator it = perms->begin();
	 it != perms->end();
	 it++) {
      printPerm(*it);
    }
  }

  void printPerm(vector<int>* perm) {
    for (vector<int>::iterator it = perm->begin();
	 it != perm->end(); 
	 it++) {
      cout << *it;
    }
    cout << endl;
  }

  // returns random permutation of length size
  template <class T> static
  vector<T> randomPermutation(const vector<T>& vec, int size) {
    vector<T> newlist;
    
    if (size == 0) 
      return newlist;
    
    // pick an element out at random
    int randind = int(ceil(drand48() * vec.size())) - 1;
    
    vector<T> sublist = vec;
    sublist.erase(sublist.begin() + randind);
    
    // get random permutation of rest of elements
    newlist = randomPermutation(sublist, size - 1);
    
    // append randomly picked-out element to new list
    newlist.push_back(vec[randind]);
    
    return newlist;
  }

private:
  vector<vector<int>*> *perms;

  vector<vector<int>*> *calcPerms(int nLabels, int length) {
    // base case: return list of one 0-length permutation
    if (length == 0) {
      vector<vector<int>* > *perms = new vector<vector<int>* >();
      perms->push_back(new vector<int>());
      return perms;
    }

    // get all length n-1 permutations
    vector<vector<int>* > *perms = calcPerms(nLabels, length - 1);

    // allocate new list of permutations
    vector<vector<int>* > *newperms = new vector<vector<int>* >();

    // append every label to every length n-1 permutation
    for (vector<vector<int>*>::iterator it = perms->begin();
	 it != perms->end();
	 it++) {
      vector<int> *perm = *it;
      for (int label = 0; label < nLabels; label++) {
	vector<int> *newperm = new vector<int>();
	*newperm = *perm;
	newperm->push_back(label);
	newperms->push_back(newperm);
      }
    }

    freePerms(perms);

    return newperms;
  }

  void freePerms(vector<vector<int>*> *perms) {
    for (vector<vector<int>*>::iterator it = perms->begin();
	 it != perms->end();
	 it++) {
      delete *it;
    }
    delete perms;
  }
};

#endif
