/**
 * @file
 * @brief A single decision tree
 * The implementation is a port of the Fortran implementation.
*
 */

#ifndef _TREE_H_
#define _TREE_H_
#include "types.h"
#include "tree_node.h"
#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <math.h>
// #include "FeatureExtractor.h"
using namespace std;

namespace librf {

class InstanceSet;
class weight_list;
class DiscreteDist;

/**
 * @brief
 * A single decision tree
 *
 *
 *
 * Strategy:
 *  - Binary tree in a fixed size array
 *    - #leaves is bounded by #instances!
 *  - Sorted instances kept in an special matrix such that:
 *    - Each node has a contiguous group of rows in the matrix
 *    - Each column gives the sorted instance order with respect
 *          to a feature
 *
 * Trees can only be created in two ways:
 *  -# load from a saved model
 *  -# grown from a certain bagging of a dataset

*/
class Tree {
    public:
        /// Construct a new tree by loading it from a file
        Tree(istream& in);
        /// Construct a new tree by training
        Tree(const InstanceSet& set,
			 weight_list* weights,
			 int number_instance,
             int K,
			 int min_size,
             float min_gain,
			 int max_depth,
			 bool brandomsplit,
			 int iSplitNum,
			 bool bTreeSplit);
         ~Tree();  // clean up
        /// predict an instance from a set
        int predict(const InstanceSet& set, int instance_no, int *leaf_no = NULL , int *terminal = NULL) const;
		int predictviewpart( const InstanceSet& set, int instance_no, int *leaf_no = NULL , int *terminal = NULL ) const;

		int predict_singlefeauture(float * feature, int * leaf_no, int *terminal = NULL) const;
		// int predict_singlefeauture(float * feature, int * leaf_no, CFeatureExtractor & fe, int *terminal = NULL) const;
        int terminal_node(const InstanceSet& set, int i) const;

        /// Return the accuracy for the training set
		void training_accuracy(float * accuracy) const;
		//compute confusion matrix
		void confusion_matrix() const;

        void print() const;
        // do all the work -- separated this from constructor to
        // facilitate threading
        void grow();
        void write(ostream& o) const;
		void writeverbose(ostream& o) const;

		void read(istream& i);

		int get_terminal_nodes();

		//Msun added
		vector<int>* get_vecpatchIds( int terminialLocalId);

    private:
        void copy_instances();
        void move_data(tree_node* n, int split_attr, int split_idx);
		void find_best_split(tree_node* n, const vector<int>& attrs, int* split_attr, int* split_idx, float* split_point, float* split_gain, int SPLIT_METHOD);
		void find_best_split_for_attr(tree_node* n, int attr, float prior, int* split_idx, float *split_point, float* best_gain, int SPLIT_METHOD);

        void get_random_split_for_attr(tree_node* n,
                                      int attr,
                                      float prior,
                                      int* split_idx,
                                      float *split_point,
                                      float* best_gain);

        // Node marking
		void add_node(int start, int size, int depth, NodeStatusType nodestatus, float gain_parent);
        void mark_terminal(tree_node* n);
		void mark_split(tree_node* n, int split_attr, float split_point);

        void build_tree();
        void build_node(int node_num);
        void print_node(int n) const;
        vector<tree_node> vecnodes_;
        //Msun added:
        vector<int> TerminalNodeGlobalNodelIds_;

        set<int> vars_used_;
        int terminal_nodes_;
        int split_nodes_;
        // get sorted indices
        // Const reference to instance set -- we don't get to delete it
        const InstanceSet& set_;
        // array of instance nums sorted by attributes
        // this is the block array that stores which instances belong to
        // which node

        // Turns out there is not much of a gain in batch allocating the
        // 2d array (perhaps because, we only access a single column at a time)
        int** sorted_inum_;
        // A single weight list for all of the instances
        weight_list* weight_list_;
        // Depth of current tree
        int K_;
        int min_size_;
        float min_gain_;
		int max_depth_;
		bool brandomsplit_;
		int iSplitNum_;

        // scratch space
        int* temp;
        int* move_left;
        int num_instances_;
        int num_attributes_;
		//split
		bool bTreeSplit_;
		float min_gain_split_;

        // Constants
        static const int kLeft;
        static const int kRight;

};

} // namespace
#endif
