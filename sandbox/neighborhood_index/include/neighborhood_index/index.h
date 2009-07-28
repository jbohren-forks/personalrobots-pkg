/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Marius Muja and Ethan Dreyfuss */

#ifndef _NEIGHBORHOOD_INDEX_
#define _NEIGHBORHOOD_INDEX_

#include <vector>
#include <robot_msgs/PointCloud.h>

namespace neighborhood_index
{

  class Index {
    virtual void knnSearch(const robot_msgs::Point32 &point, int k,
                           std::vector<int> &k_indices, std::vector<float> &k_distances) = 0;
    virtual void knnSearch(const robot_msgs::PointCloud &pointsToFind, int k,
                           std::vector<std::vector<int> > &k_indices_array, std::vector<std::vector<float> > &k_distances_array) = 0;

    virtual bool radiusSearch (const robot_msgs::Point32 &p_q, double radius,
                               std::vector<int> &k_indices, std::vector<float> &k_distances, int max_nn = INT_MAX) = 0;
    virtual bool radiusSearch (const robot_msgs::PointCloud &points, double radius,
                               std::vector<std::vector<int> > &k_indices_array, std::vector<std::vector<float> > &k_distances_array, int max_nn = INT_MAX) = 0;

  };

  //Version of Index which supports dynamic resizing
  class DynamicIndex : public Index {
    virtual void add(const robot_msgs::Point32 &point) = 0;
    virtual void add(const robot_msgs::PointCloud &points) = 0;

    virtual void remove(const robot_msgs::Point32 &point) = 0;
    virtual void remove(const robot_msgs::PointCloud &points) = 0;

    virtual bool canRemove() = 0;
    virtual bool optimize(int optimizationLevel = 0) = 0;
  };

}

#endif
