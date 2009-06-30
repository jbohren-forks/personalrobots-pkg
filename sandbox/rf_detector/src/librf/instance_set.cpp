#include "instance_set.h"
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <float.h>
#include "weights.h"
#include "types.h"
#include "stringutils.h"
#include <vector>
#include <algorithm>

//#include "stdlib.h"

using namespace std;

namespace librf {
	//InstanceSet::InstanceSet(){}

	InstanceSet* InstanceSet::load_data_and_labels( IN vector< vector<float> > & vecUniverse, IN vector<int> & vecFgbgLabel, IN vector<int> & vecViewLabel)
	{
		return new InstanceSet(vecUniverse, vecFgbgLabel, vecViewLabel);
	}

	void InstanceSet::save_data_and_labels( IN vector< vector<float> > & vecUniverse, IN vector<int> & vecFgbgLabels, IN vector<int> & vecViewLabels )
	{
		vecsorted_indices_.resize(0);
		vecUniverse = attributes_;
		vecFgbgLabels = vecfgbglabels_;
		vecViewLabels = vecviewpartlabels_;
	}
	/**
	* Named constructor for creating a subset from an existing set
	*
	* @param set existing set
	* @param wl non-zeroes weights
	*/
	InstanceSet* InstanceSet::create_subset(const InstanceSet& set,
		const weight_list& wl) {
			return new InstanceSet(set, wl);
	}
	InstanceSet::InstanceSet(IN vector< vector<float> > &vecUniverse,
							 IN vector<int> & vecFgbgLabel,
							 IN vector<int> & vecViewPartLabel)
	{
		int num_features = vecUniverse.size();
		int num_instances = vecUniverse.at(0).size();
		create_dummy_var_names(num_features);

		attributes_ = vecUniverse;
		vecUniverse.resize(0);
		vecfgbglabels_ = vecFgbgLabel;
		vecFgbgLabel.resize(0);
		this->vecviewpartlabels_ = vecViewPartLabel;
		vecViewPartLabel.resize(0);
		this->vecpatchid_.resize(vecViewPartLabel.size());
		for (int i = 0; i < vecViewPartLabel.size(); i++)
			this->vecpatchid_.at(i) = i;
		printf("Sorting attributes...");
		create_sorted_indices();
		printf("Done.\n");
		assert(attributes_.size() > 0);
		assert(attributes_.at(0).size() == vecfgbglabels_.size());
	}

	void InstanceSet::create_dummy_var_names(int n) {
		for (int i = 0; i <  n; ++i) {
			stringstream ss;
			ss << i;
			vecvar_names_.push_back(ss.str());
		}
	}
	void InstanceSet::create_sorted_indices() {
		// allocate sorted_indices_
		vecsorted_indices_.resize(attributes_.size());
		// sort
		for (int i = 0; i < attributes_.size(); ++i) {
			sort_attribute(attributes_.at(i), &vecsorted_indices_.at(i));
		}
	}

	void InstanceSet::sort_attribute(const vector<float>& attribute,
		vector<int>*indices) {
			vector<pair<float, int> > pairs;
			for (int i = 0; i < attribute.size(); ++i) {
				pairs.push_back(make_pair(attribute.at(i),i));
			}
			std::sort(pairs.begin(), pairs.end());
			for (int i = 0; i < pairs.size(); ++i) {
				indices->push_back(pairs.at(i).second);
			}
	}

	// Grab a subset of the instance (for getting OOB data
	InstanceSet::InstanceSet(const InstanceSet& set,
		const weight_list& weights) : attributes_(set.num_attributes()){
			// Calculate the number of OOB cases
			//cout << "creating OOB subset for weight list of size "
			//     << weights.size() << endl;
			for (int i = 0; i < weights.size(); ++i) {
				if (weights[i] == 0) {
					// append instance
					for (int j = 0; j < set.num_attributes(); ++j) {
						attributes_.at(j).push_back(set.get_attribute(i, j));
					}
					vecfgbglabels_.push_back(set.fgbglabel(i));
					vecviewpartlabels_.push_back(set.viewpartlabel(i));
					vecpatchid_.push_back(set.patchid(i));
				}
			}
	}

	/**
	* Permute method
	* Used for variable importance
	*/
	void InstanceSet::permute(int var, unsigned int *seed) {
		vector<float>& attr = attributes_.at(var);
		for (int i = 0; i < attr.size(); ++i) {
			int idx = rand() % vecfgbglabels_.size(); // randomly select an index
			float tmp = attr.at(i);  // swap last value with random index value
			attr.at(i) = attr.at(idx);
			attr.at(idx) =  tmp;
		}
	}

	void InstanceSet::load_var(int var, const vector<float>& source) {
		// use the STL built-in copy/assignment
		attributes_.at(var) = source;
	}

	void InstanceSet::save_var(int var, vector<float>* target) {
		const vector<float>& attr = attributes_.at(var);
		// use the STL built-in copy/assignment
		*target = attr;
	}
} // namespace
