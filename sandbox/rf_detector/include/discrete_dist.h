/*******************
Discrete distribution
********************/
#pragma once
#include "assert.h"
#ifndef _DISCRETE_DIST_H_
#define _DISCRETE_DIST_H_

#include <vector>
#include <iostream>
#include <math.h>
#include "types.h"

using namespace std;

namespace librf {

	class DiscreteDist {
	public:
		DiscreteDist(int size = NClass) : sum_(0.0f), size_(size),counter_(size, 0.0f)
		{

		}
		void add(int value, float weight = 1.0f) {
			counter_[value] += weight;
			sum_ += weight;
		}
		void remove(int value, float weight = 1.0f) {
			counter_[value] -= weight;
			sum_ -= weight;
		}
		float sum() const {
			return sum_;
		}
		void copydist(vector<float> & dist)
		{
			dist.resize(counter_.size());
			assert(dist.size() == counter_.size());
			for (int i = 0; i < dist.size(); i++)
			{
				dist[i] = counter_[i] / (float)sum_;
			}
		}
		unsigned int mode() const {
			float max = -1;
			int mode = -10;
			for (unsigned int i = 0; i < size_; ++i) {
				float val = counter_[i];
				if (val > max) {
					max = val;
					mode = i;
				}
			}
			return mode;
		}
		void print() {
			for (unsigned int i = 0; i < size_; ++i) {
				cout << i << ":" << float(counter_[i]) << endl;
			}
		}
		unsigned int num_labels() const {
			return size_;
		}
		float weight(int i) const {
			return counter_[i];
		}
		float percentage(int i) const {
			return float(weight(i)) / sum();
		}
		static const double kLog2;
		static float entropy_conditioned_naive(const DiscreteDist* sets,
			int num_dists) {
				float H = 0;
				// H(Y |X) = Sum Prob(X=x) H(Y | x = x)
				float total = 0;
				for (int i = 0; i < num_dists; ++i) {
					float split_entropy = 0;
					float split_total = 0;
					for (unsigned int j = 0; j< sets[i].num_labels(); ++j) {
						float weight = (float)sets[i].weight(j);
						split_entropy -= lnFunc(weight);
						split_total += weight;
						total += weight;
						cerr << j << ":" << weight <<endl;
					}
					if (split_total == 0) {
						split_entropy = 0;
					} else {
						split_entropy = (split_entropy + lnFunc(split_total) ) /
							(split_total * (float)kLog2);
					}
					cerr << "Split " << i << ":" << split_entropy <<endl
						;
					H += split_total * split_entropy;
				}
				return H / (total);
		}
		//il/i * e(il) + ir/i * e(ir)
		// il is the number of nodes in the left subset, while ir is the number of nodes in the right subset
		static float entropy_conditioned(const DiscreteDist* sets, int num_dists) {
			float returnValue = 0;
			float total = 0;
			float sumForSet;

			for (int i = 0; i < num_dists; ++i ) {
				sumForSet = 0;
				for (unsigned int j = 0; j < sets[i].num_labels(); ++j) {
					float weight = (float)sets[i].weight(j);
					returnValue += lnFunc(weight);
					sumForSet += weight;
				}
				returnValue -= lnFunc(sumForSet);
				total += sumForSet;
			}
			if (total == 0){
				return 0;
			}
			returnValue = -returnValue /(total * (float)kLog2);
			assert(returnValue == returnValue);
			return returnValue;
		}

		// Adapted from ContingencyTables.java: entropyOverColumns
		// min:0, max:log(size_)/log(2)
		float entropy_over_classes() const{
			float returnValue = 0;
			float total = 0;
			for (unsigned int i = 0; i < size_; ++i) {
				returnValue -= lnFunc((float)counter_[i]);
				total += counter_[i];
			}
			if (total == 0) {
				return 0;
			}
			return (returnValue + lnFunc(total)) / (total * (float)kLog2);
		}
	private:
		float sum_;
		unsigned int size_;
		static float lnFunc(float num) {
			if (num  < 1e-6) {
				return 0;
			} else {
				return num * log(num);
			}
		}
		vector<float> counter_;
	};
} // namespace
#endif
