/**
 * tree_node.h
 * @file
 * @brief structure for a single node in a decision tree
 */

#ifndef _TREE_NODE_H_
#include "types.h"
#include <fstream>
#include <vector>
#include "discrete_dist.h"
using namespace std;

namespace librf {

typedef enum {EMPTY, BUILD_ME, TERMINAL, SPLIT, BUILD_ME_BY_DIST} NodeStatusType;

struct tree_node {
/// default constructor initializes to garbage known state
  tree_node() :status(EMPTY),
               fgbglabel(99),
			   viewpartlabel(99),
               attr(99),
               start(99),
               size(99),
               split_point(-999.0),
               fgbgentropy(-9999.0),
               left(0),
               right(0),
			   leaf_no(0)
		       {
			   }

  NodeStatusType status;
  uchar fgbglabel;
  uchar viewpartlabel;
  int attr;
  int start;
  int size;
  int left;
  int right;
  float fgbgentropy;
  float viewentropy;
  float split_point;
  uchar depth;
  int leaf_no;
  vector<int> vecpatchid;
  vector<float> fgbgdist;
  vector<float> viewdist;
  int nodeid;
  float gain;

  void write(ostream& o) const;
  void writeverbose(ostream& o) const;

  bool read(istream& i);
};
} //namespace
#endif
