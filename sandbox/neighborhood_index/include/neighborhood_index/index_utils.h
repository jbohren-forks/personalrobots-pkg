/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Author: mariusm

#ifndef INDEX_UTILS_H_
#define INDEX_UTILS_H_

#include <vector>
#include <geometry_msgs/Point32.h>

namespace neighborhood_index {

template<typename T,typename U>
float euclidean_dist(T a, U b)
{
	return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z);
}


class ResultSet
{
public:
	virtual void init(const geometry_msgs::Point32& p) = 0;

	virtual void add(const geometry_msgs::Point32& p, int ind) = 0;
};

class KNNResultSet : public ResultSet
{
public:
	std::vector<int>& indices_;
	std::vector<float>& distances_;

	size_t count_;
	geometry_msgs::Point32 query_;

	KNNResultSet(std::vector<int>& indices, std::vector<float>& distances) : indices_(indices), distances_(distances), count_(0) {};

	void init(const geometry_msgs::Point32& p)
	{
		query_ = p;
	}

	void add(const geometry_msgs::Point32& p, int ind)
	{
		float dist = euclidean_dist(query_,p);

		if (count_<indices_.size()) {
			indices_[count_] = ind;
			distances_[count_] = dist;
			count_++;
			return;
		}
		else if (distances_[count_-1]>dist) {
			int k = count_ - 1;
			distances_[k] = dist;
			indices_[k] = ind;

			while (k>0 && distances_[k]<distances_[k-1]) {
				std::swap(distances_[k], distances_[k-1]);
				std::swap(indices_[k], indices_[k-1]);
			}
		}
	}
};


class RadiusResultSet : public ResultSet
{
public:
	std::vector<int>& indices_;
	std::vector<float>& distances_;
	float radius_;
	int max_nn_;

	geometry_msgs::Point32 query_;

	RadiusResultSet(std::vector<int>& indices, std::vector<float>& distances, float radius, int max_nn) : indices_(indices), distances_(distances),
		radius_(radius), max_nn_(max_nn)
	{
		indices_.resize(0);
		distances_.resize(0);
	};

	void init(const geometry_msgs::Point32& p)
	{
		query_ = p;
	}


	void add(const geometry_msgs::Point32& p, int ind)
	{
		float dist = euclidean_dist(query_,p);

		if (dist<radius_) {
			if (indices_.size()<(size_t)max_nn_) {
				indices_.push_back(ind);
				distances_.push_back(dist);
			}
			else if (distances_.back()>dist) {
				distances_.back() = dist;
				indices_.back() = ind;
			}
			else {
				return;
			}

			// bubble up
			int k = indices_.size() - 1;
			while (k>0 && distances_[k]<distances_[k-1]) {
				std::swap(distances_[k], distances_[k-1]);
				std::swap(indices_[k], indices_[k-1]);
			}
		}
	}
};


}


#endif /* INDEX_UTILS_H_ */
