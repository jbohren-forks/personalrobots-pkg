/**
 * @file
 * @brief Randomforest interface
 * This is the interface to manage a random forest
 */

#ifndef _RANDOM_FOREST_H_
#define _RANDOM_FOREST_H_

#include <vector>
#include "instance_set.h"
#include "tree.h"
// #include "FeatureExtractor.h"
using namespace std;

namespace librf {
	class DiscreteDist;
	class InstanceSet;
	class Tree;
	/**
	 * @brief
	 * RandomForest class.  Interface for growing random forests from training
	 * data or loading a random forest from disk.
	 */
	class RandomForest {
	  public:
		/// Empty constructor

		RandomForest(void)  {}
		~RandomForest(void) {
			for (int i = 0; i < vectrees_.size(); ++i) {
				delete vectrees_.at(i);
			}
		}

		 /// Method to predict the label
		 // int predict(const Instance& c) const;
		 /// Method that returns the class probability
		 // float predict_prob(const Instance& c) const;
		 /// Method to predict the label
		 int predict(const InstanceSet& set, int instance_no) const;
		 // method to go throught the random forest to return the id list from leaf nodes
		 vector<int> predict_node(const InstanceSet& set, int instance_no) const;
		 int predictviewpart(const InstanceSet& set, int instance_no) const;

		//(Build from training data)
		int build(const InstanceSet& set,
					 int num_trees,
					 int K,
					 int min_size,
					 float min_gain,
					 int max_depth,
					 float fSampleRatio,
					 bool bRandomSplit,
					 int iSplitNum,
					 bool bTreeSplit,
					 vector<float> & vecWeightList);

		 int get_trees_size()  {	 return (int)vectrees_.size();	 }

		 /// Predict probability of given label
		 float predict_prob_singlefeature(float * feature, int label,  int *leaflist) const;
		 // float predict_prob_singlefeature(float * feature, int label,  int *leaflist, CFeatureExtractor & fe) const;

		 /// Returns training accuracy
		 float * training_accuracy(const InstanceSet& set) const;
		 void confusion_matrix(int num_labels,
							   const vector<int>&,
							   const vector<int>&) const;
		 void destroy();
		 /// Load random forest
		 void read(istream& i);
		 /// Save random forest
		 void write(ostream& o);
		 void writeverbose(ostream& o);

		 int get_tree_size(int t);
		 int get_dim();
		 void print(const InstanceSet& set);

		 //Msun added
		 vector<int>* get_vecpatchIds( int treeId, int terminialLocalId);

	  private:
		//InstanceSet& set_;  // training data set
		vector<Tree*> vectrees_;     // component trees in the forest
		int max_depth_;           // maximum depth of trees (DEPRECATED)
		int K_;                   // random vars to try per split
		int min_size_;				//minimum size of a node in a tree
		float min_gain_;				//minimum gain of a split in building the trees
		vector<int> vecclass_weights_;
		int dim_;
		bool bRandomSplit_;
		int iSplitNum_;
		float fSampleRatio_;
		bool bTreeSplit_;
	};
} // namespace
#endif
