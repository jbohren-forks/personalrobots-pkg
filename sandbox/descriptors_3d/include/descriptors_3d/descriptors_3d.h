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

#include <Eigen/Core>

#include <ros/console.h>

#include <robot_msgs/PointCloud.h>

#include <point_cloud_mapping/kdtree/kdtree.h>
#include <point_cloud_mapping/geometry/nearest.h>

using namespace std;

class Descriptor3D
{
  public:
    Descriptor3D() :
      result_size_(0), data_defined_(false), data_(NULL), data_kdtree_(NULL), interest_pt_set_(false)
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
      data_defined_ = true;
    }

    void setInterestPoint(unsigned int interest_pt_idx)
    {
      interest_pt_idx_ = interest_pt_idx;
      interest_pt_set_ = true;
    }

    unsigned int getResultSize()
    {
      return result_size_;
    }

  protected:
    virtual bool readyToCompute() = 0;

    unsigned int result_size_;

    bool data_defined_;
    const robot_msgs::PointCloud* data_;
    cloud_kdtree::KdTree* data_kdtree_;

    bool interest_pt_set_;
    unsigned int interest_pt_idx_;

};

class LocalGeometry: public Descriptor3D
{
  public:
    LocalGeometry() :
      radius_(-1.0), indices_(NULL), ref_tangent_defined_(false), ref_normal_defined_(false),
          origin_defined_(false), use_elevation_(false), use_curvature_(false)
    {
    }

    virtual bool compute(Eigen::MatrixXf** result, bool debug);

    virtual bool readyToCompute();

    // ----------------------------------------------------------

    int defineNeighborhood(double radius)
    {
      if (radius > 0.0)
      {
        radius_ = radius;
        indices_ = NULL;
        return 0;
      }
      return -1;
    }

    void defineNeighborhood(vector<int>* indices)
    {
      indices_ = indices;
      radius_ = -1.0;
    }

    void defineOrigin(double x, double y, double z)
    {
      origin_x_ = x;
      origin_y_ = y;
      origin_z_ = z;
      origin_defined_ = true;
    }

    void useCurvature()
    {
      use_curvature_ = true;
    }

    void useElevation()
    {
      use_elevation_ = true;
    }

    void useTangentOrientation(double ref_x, double ref_y, double ref_z)
    {
      ref_tangent_[0] = ref_x;
      ref_tangent_[1] = ref_y;
      ref_tangent_[2] = ref_z;
      ref_tangent_flipped_[0] = -ref_x;
      ref_tangent_flipped_[1] = -ref_y;
      ref_tangent_flipped_[2] = -ref_z;
      ref_tangent_defined_ = true;
    }
    void useNormalOrientation(double ref_x, double ref_y, double ref_z)
    {
      ref_normal_[0] = ref_x;
      ref_normal_[1] = ref_y;
      ref_normal_[2] = ref_z;
      ref_normal_flipped_[0] = -ref_x;
      ref_normal_flipped_[1] = -ref_y;
      ref_normal_flipped_[2] = -ref_z;
      ref_normal_defined_ = true;
    }

    void useRawBoundingBox()
    {
      // TODO
    }
    void usePCABoundingBox()
    {
      // TODO
    }

  private:
    double radius_;
    vector<int>* indices_;

    bool ref_tangent_defined_;
    Eigen::Vector3d ref_tangent_;
    Eigen::Vector3d ref_tangent_flipped_;

    bool ref_normal_defined_;
    Eigen::Vector3d ref_normal_;
    Eigen::Vector3d ref_normal_flipped_;

    bool origin_defined_;
    double origin_x_;
    double origin_y_;
    double origin_z_;

    bool use_elevation_;
    bool use_curvature_;
};

#endif
