/**
* @file
* @brief random forest implementation
*/
//#include "stdafx.h"
#include "random_forest.h"
#include "tree.h"
#include "instance_set.h"
#include "weights.h"
#include <fstream>
#include <algorithm>
#include <time.h>
extern int NView;
namespace librf {

	/**
	* @param set training data
	* @param num_trees #trees to train
	* @param K #random vars to consider at each split
	* number of instances)
	*/
	int RandomForest::build(const InstanceSet& set,
		int num_trees,
		int K,
		int min_size,
		float min_gain,
		int max_depth,
		float fSampleRatio,
		bool bRandomSplit,
		int iSplitNum,
		bool bTreeSplit,
		vector<float> & vecWeightList)
	{
		K_ = K;
		min_size_ = min_size;
		min_gain_ = min_gain;
		max_depth_ = max_depth;
		dim_ = set.num_attributes();
		bRandomSplit_ = bRandomSplit;
		iSplitNum_ = iSplitNum;
		fSampleRatio_ = fSampleRatio;
		bTreeSplit_ = bTreeSplit;

		//print random forest infomation
		cout << "RandomForest Constructor " << num_trees << endl;
		cout << "K " << K_ << endl;
		cout << "min_size " << min_size_ << endl;
		cout << "min_gain " << min_gain_ << endl;
		cout << "max_depth " << max_depth_ << endl;
		cout << "Sample_ratio " << fSampleRatio_ << endl;
		cout << "Unsupervised Split " << bTreeSplit_ << endl;

		for (int i = 0; i < num_trees; ++i) {
			weight_list* w = new weight_list(set.size());
			float fRatio = num_trees == 1 ? 1.0f : fSampleRatio_;
			int iCurSampleCount = int(set.size() * fRatio);
			vector<bool> vecbHit(set.size(), false);
			int iSampleGetted = 0;
			if (iCurSampleCount == set.size())
			{
				for (int j = 0; j < set.size(); j++)
				{
					w->add(j, 1.0f);
				}
			}
			else
			{
				while (iSampleGetted < iCurSampleCount)
				{
					int j = (rand() * rand()) % set.size();
					if (!vecbHit[j])
					{
						w->add(j, 1.0f);
						vecbHit[j] = true;
						iSampleGetted ++;
					}
				}
			}

			printf("start to build tree...");
			Tree* tree = new Tree(set, w, iCurSampleCount, K, min_size, min_gain, max_depth_, bRandomSplit_, iSplitNum_, bTreeSplit_);
			printf("start to grow tree...");
			tree->grow();
			printf("good!\n");
			vectrees_.push_back(tree);
			cout << "Tree " << i << endl;
			//print the tree information
			tree->print();
		}
		//print the forest information
		print(set);
		return 0;
	}
	void RandomForest::destroy()
	{
		for (int i = 0; i < vectrees_.size(); ++i) {
			delete vectrees_.at(i);
		}
		vectrees_.resize(0);
	}
	int RandomForest::get_tree_size(int t)
	{
		return vectrees_.at(t)->get_terminal_nodes();
	}

	void RandomForest::write(ostream& o) {
		o << vectrees_.size() << " " << dim_ << " " << K_ << endl;
		for (int i = 0; i < vectrees_.size(); ++i) {
			vectrees_.at(i)->write(o);
		}
	}

	void RandomForest::writeverbose(ostream& o) {
		o << vectrees_.size() << " " << dim_ << " " << K_ << endl;
		for (int i = 0; i < vectrees_.size(); ++i) {
			vectrees_.at(i)->writeverbose(o);
		}
	}

	void RandomForest::read(istream& in) {
		int num_trees, K;
		in >> num_trees >> dim_ >> K;
		vectrees_.resize(0);
		for (int i = 0; i < num_trees; ++i) {
			vectrees_.push_back(new Tree(in));
		}
	}

	int RandomForest::predict(const InstanceSet& set, int instance_no) const {
		// Gather the votes from each tree
		DiscreteDist votes;
		for (int i = 0; i < vectrees_.size(); ++i) {
			int predict = vectrees_.at(i)->predict(set, instance_no);
			votes.add(predict);
		}
		return votes.mode();
	}
	vector<int> RandomForest::predict_node( const InstanceSet& set, int instance_no ) const
	{
		vector<int> predictnodelist;
		int leaf_no;
		for (int i = 0; i < vectrees_.size(); ++i) {
			vectrees_.at(i)->predict(set, instance_no, &leaf_no);
			predictnodelist.push_back(leaf_no);
		}
		return predictnodelist;
	}
	int RandomForest::predictviewpart(const InstanceSet& set, int instance_no) const {
		// Gather the votes from each tree
		DiscreteDist votes(NView);
		for (int i = 0; i < vectrees_.size(); ++i) {
			int predict = vectrees_.at(i)->predictviewpart(set, instance_no);
			votes.add(predict);
		}
		return votes.mode();
	}
	float RandomForest::predict_prob_singlefeature(float *feature, int label, int *leaflist) const {
		// Gather the votes from each tree
		DiscreteDist votes;
		for (int i = 0; i < vectrees_.size(); ++i) {
			int predict = vectrees_.at(i)->predict_singlefeauture(feature, leaflist + i);
			votes.add(predict);
		}
		return votes.percentage(label);
	}
	//float RandomForest::predict_prob_singlefeature(float *feature, int label, int *leaflist, CFeatureExtractor & fe) const {
	//	// Gather the votes from each tree
	//	DiscreteDist votes;
	//	for (int i = 0; i < trees_.size(); ++i) {
	//		int predict = trees_[i]->predict_singlefeauture(feature, leaflist + i, fe);
	//		votes.add(predict);
	//	}
	//	return votes.percentage(label);
	//}
	int RandomForest::get_dim()
	{
		return dim_;
	}

	void RandomForest::confusion_matrix(int num_labels,
		const vector<int>& predicts,
		const vector<int>& actual) const {
			assert(actual.size() == predicts.size());
			vector< vector<int> > matrix;
			matrix.resize(num_labels);
			for (int i = 0; i < num_labels; ++i) {
				matrix[i].resize(num_labels, 0);
			}
			for (int i = 0; i < actual.size(); ++i) {
				matrix[actual[i]][predicts[i]]++;
			}
			for (int i = 0; i < num_labels; ++i) {
				for (int j = 0; j < num_labels; ++j) {
					cout << matrix[i][j] << " ";
				}
				cout <<endl;
			}
	}

	float * RandomForest::training_accuracy(const InstanceSet& set) const {
		int correctfgbg = 0;
		int correctviewpart = 0;
		int nfg = 0;
		for (int i =0; i < set.size(); ++i) {
			if (predict(set, i) == set.fgbglabel(i))
				correctfgbg++;
			if (set.fgbglabel(i) == 1 && predictviewpart(set, i) == set.viewpartlabel(i))
				correctviewpart++;
			if (set.fgbglabel(i) == 1)
				nfg++;
		}
		float * accuracy = new float[2];
		accuracy[0] = float(correctfgbg) / set.size();
		accuracy[1] = float(correctviewpart) / nfg;
		return accuracy;
	}
	void RandomForest::print(const InstanceSet& set)
	{
		cout << "Trees Number: " << vectrees_.size() << endl;
		float * accuracy = training_accuracy(set);
		cout << "Training acc: " << accuracy[0] << "/" << accuracy[1] << endl;
		delete[] accuracy;
		vector<int> predicts(set.size());
		vector<int> labels(set.size());
		for (int i = 0; i < set.size(); i++)
		{
			predicts[i] = predict(set, i);
			labels[i] = set.fgbglabel(i);
		}
		//hard coded, fix it
		confusion_matrix(NClass, predicts, labels);
	}

	vector<int>* RandomForest::get_vecpatchIds( int treeId, int terminialLocalId){
        return vectrees_.at(treeId)->get_vecpatchIds( terminialLocalId);
    }


} // namespace
