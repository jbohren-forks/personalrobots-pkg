/* weights.h
 * Instance Weights:
 *
 * this is trying to be a memory "smart" container that will decide
 * whether or not to use sparse/dense representation yet
 * still provide a uniform interface.
 *
 * It is possible to do this because - we know how many training
 * instances there are apriori - and each time we split a node
 * we will know how many instances are going to the left/right
 * subtrees
 *
 * approx mem usage:
 *  sparse: density * 5 bytes + map overhead
 *  dense: num_instances bytes
 */
#ifndef _WEIGHTS_H_
#define _WEIGHTS_H_

#include <vector>
#include <iostream>
#include "types.h"

using namespace std;

namespace librf {

class weight_list {
  public:
   weight_list(int n) : num_instances_(n), sum_(0.0f){
      array_ = new float[n];
      for(int i =0; i < n; ++i) {
        array_[i] = 0;
      }
   }
   float operator[](int i) const {
    return array_[i];
   }
   void add(int i, float num){
    array_[i]+=num;
    sum_+=num;
   }
   int getnum() { return num_instances_; }
   int size() const { return num_instances_;}
   float sum() const {
      return sum_;
   }
   ~weight_list(){
     delete[] array_;
   }
  private:
   float sum_;
   int num_instances_;
   float* array_;
};

} // namespace
#endif
