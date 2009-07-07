#ifndef __DESCRIPTORS_3D_H__
#define __DESCRIPTORS_3D_H__
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Daniel Munoz
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

#include <iostream>
#include <string>
#include <list>
#include <vector>

#include <ros/console.h>

#include <Eigen/Core>

#include <robot_msgs/PointCloud.h>

#include <point_cloud_mapping/kdtree/kdtree.h>
#include <point_cloud_mapping/geometry/nearest.h>

using namespace std;

class Descriptor3D
{
  public:
    Descriptor3D() :
      result_size_(0), data_set_(false), data_(NULL), data_kdtree_(NULL), interest_pt_set_(false),
          interest_pt_idx_(0), interest_region_set_(false), interest_region_indices_(NULL)
    {
    }

    virtual ~Descriptor3D()
    {
    }
    ;

    virtual bool compute(Eigen::MatrixXf** result, bool debug) = 0;

    void setData(const robot_msgs::PointCloud* data, cloud_kdtree::KdTree* data_kdtree)
    {
      data_ = data;
      data_kdtree_ = data_kdtree;
      data_set_ = true;
    }

    void setInterestPoint(unsigned int interest_pt_idx)
    {
      interest_pt_idx_ = interest_pt_idx;
      interest_pt_set_ = true;

      interest_region_indices_ = NULL;
      interest_region_set_ = false;
    }

    // may or may not be used by descriptor
    void setInterestRegion(const vector<int>* interest_region_indices)
    {
      interest_region_indices_ = interest_region_indices;
      interest_region_set_ = true;

      interest_pt_idx_ = 0;
      interest_pt_set_ = false;
    }

    unsigned int getResultSize() const
    {
      return result_size_;
    }

  protected:
    // will check whether can use either interest pt or region
    virtual bool readyToCompute() = 0;

    unsigned int result_size_;

    bool data_set_;
    const robot_msgs::PointCloud* data_;
    cloud_kdtree::KdTree* data_kdtree_;

    bool interest_pt_set_;
    unsigned int interest_pt_idx_;

    bool interest_region_set_;
    const vector<int>* interest_region_indices_;
};

#endif
