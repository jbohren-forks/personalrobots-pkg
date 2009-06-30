/**
* @file
* @brief Instance Set
* This is the abstraction for a data set
* -- Currently libSVM, CSV
* -- want to support ARFF
*/
#ifndef _INSTANCE_SET_H_
#define _INSTANCE_SET_H_
#include "stdafx.h"
#include <string>
#include <vector>
#include <fstream>
#include "discrete_dist.h"
#include <time.h>

using namespace std;

namespace librf {

	class weight_list;
	/**
	* @brief
	* InstanceSet class. Interface for loading training/testing data.
	*/
	class InstanceSet {
	public:
		/// Empty constructor
		InstanceSet() {};
		/// Named constructor - create a subset from an existing instanceset
		static InstanceSet* create_subset(const InstanceSet&, const weight_list&);
		static InstanceSet* load_data_and_labels( IN vector<vector<float> > & vecUniverse, IN vector<int> & vecFgbgLabel, IN vector<int> & vecViewLabel);
		void save_data_and_labels(IN vector< vector<float> > & vecUniverse, IN vector<int> & vecFgbgLabels, IN vector<int> & vecViewLabels);
		/// copy a variable array out
		void save_var(int var, vector<float> *target);
		/// load a variable array in
		void load_var(int var, const vector<float>&);
		void permute(int var, unsigned int * seed);
		/// sort the variables
		void create_sorted_indices();

		const vector<int>& get_sorted_indices(int attribute) const{
			return vecsorted_indices_[attribute];
		}
		/// Most common label
		int mode_label() const {
			return distribution_.mode();
		}
		/// Get a particular instance's label
		unsigned char fgbglabel(int i) const{
			return vecfgbglabels_[i];
		}
		unsigned char viewpartlabel(int i) const{
			return vecviewpartlabels_[i];
		}
		unsigned int patchid(int i) const{
			return vecpatchid_[i];
		}
		/// Number of instances
		unsigned int size() const {
			return (unsigned int) (vecfgbglabels_.size());
		}
		/// Number of attributes
		unsigned int num_attributes() const {
			return (unsigned int) (attributes_.size());
		}
		/// Get a particular instance's attribute
		float get_attribute(int i, int attr) const {
			return attributes_.at(attr).at(i);
		}
		/// Get a variable name (useful if there is a header with var
		//names)
		string get_varname(int i) const {
			return vecvar_names_.at(i);
		}
		void write_csv(ostream& out, bool header, const string& delim);
		void write_transposed_csv(ostream& out, const string& delim);
		//void destroy(){

		//float class_entropy() const{
		//  return distribution_.entropy_over_classes();
		//}
	private:
		/// Get a subset of an existing instance set
		InstanceSet(const InstanceSet&, const weight_list&);
		InstanceSet(IN vector< vector<float> > &vecUniverse,
			IN vector<int> & vecFgbgLabel,
			IN vector<int> & vecViewPartLabel);
		/// Feature select from existing instance set
		InstanceSet(const InstanceSet&, const vector<int>&);
		void create_dummy_var_names(int n);
		void sort_attribute(const vector<float>&attribute, vector<int>*indices);
		DiscreteDist distribution_;
		// List of Attribute Lists
		// Thus access is attributes_ [attribute] [ instance]
		vector< vector<float> > attributes_;
		// List of true labels
		// access is labels_ [instance]

		vector<int>  vecfgbglabels_;
		vector<int>  vecviewpartlabels_;
		vector<int>  vecpatchid_;

		vector<string> vecvar_names_;
		vector< vector<int> > vecsorted_indices_;
	};

}  // namespace
#endif
