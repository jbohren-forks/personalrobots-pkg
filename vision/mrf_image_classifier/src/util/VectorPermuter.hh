#ifndef __VEC_PERMUTER_H__
#define __VEC_PERMUTER_H__

#include <vector>
#include <iostream>

using namespace std;

class VectorPermuter {
public:
  static vector<vector<int>* > *getPerms(int nLabels, int length) {
    // base case: return list of one 0-length permutation
    if (length == 0) {
      vector<vector<int>* > *perms = new vector<vector<int>* >();
      perms->push_back(new vector<int>());
      return perms;
    }

    // get all length n-1 permutations
    vector<vector<int>* > *perms = getPerms(nLabels, length - 1);

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

  static void freePerms(vector<vector<int>*> *perms) {
    for (vector<vector<int>*>::iterator it = perms->begin();
	 it != perms->end();
	 it++) {
      delete *it;
    }
    delete perms;
  }

  static void printPerms(vector<vector<int>*> *perms) {
    for (vector<vector<int>*>::iterator it = perms->begin();
	 it != perms->end();
	 it++) {
      printPerm(*it);
    }
  }

  static void printPerm(vector<int>* perm) {
    for (vector<int>::iterator it = perm->begin();
	 it != perm->end(); 
	 it++) {
      cout << *it;
    }
    cout << endl;
  }

  template<typename T>
  static void mapPerms(T (*func)(vector<int>* perm),
		       vector<vector<int>*> *perms, 
		       vector<T> &out) {
    for (vector<vector<int>*>::iterator permPr = perms->begin();
	 permPr != perms->end();
	 permPr++) {
      out.push_back(func(*permPr));
    }
  }
};

#endif
