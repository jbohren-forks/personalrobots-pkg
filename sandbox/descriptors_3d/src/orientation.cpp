/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage
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

#include <descriptors_3d/orientation.h>

using namespace std;

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void Orientation::useTangentOrientation(double ref_x, double ref_y, double ref_z)
{
  if (ref_tangent_defined_ == false)
  {
    ref_tangent_defined_ = true;
    result_size_++;
  }

  ref_tangent_[0] = ref_x;
  ref_tangent_[1] = ref_y;
  ref_tangent_[2] = ref_z;
  ref_tangent_.normalize();

  ref_tangent_flipped_[0] = -ref_tangent_[0];
  ref_tangent_flipped_[1] = -ref_tangent_[1];
  ref_tangent_flipped_[2] = -ref_tangent_[2];
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void Orientation::useNormalOrientation(double ref_x, double ref_y, double ref_z)
{
  if (ref_normal_defined_ == false)
  {
    ref_normal_defined_ = true;
    result_size_++;
  }

  ref_normal_[0] = ref_x;
  ref_normal_[1] = ref_y;
  ref_normal_[2] = ref_z;
  ref_normal_.normalize();

  ref_normal_flipped_[0] = -ref_normal_[0];
  ref_normal_flipped_[1] = -ref_normal_[1];
  ref_normal_flipped_[2] = -ref_normal_[2];
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void Orientation::compute(const robot_msgs::PointCloud& data,
                          cloud_kdtree::KdTree& data_kdtree,
                          const cv::Vector<const robot_msgs::Point32*>& interest_pts,
                          cv::Vector<cv::Vector<float> >& results)
{
  results.clear();
  results.resize(interest_pts.size());

  if (spectral_info_ == NULL)
  {
    if (analyzeInterestPoints(data, data_kdtree, interest_pts) < 0)
    {
      return;
    }
  }

  computeFeatures(results);
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void Orientation::compute(const robot_msgs::PointCloud& data,
                          cloud_kdtree::KdTree& data_kdtree,
                          const cv::Vector<const vector<int>*>& interest_region_indices,
                          cv::Vector<cv::Vector<float> >& results)
{
  results.clear();
  results.resize(interest_region_indices.size());

  if (spectral_info_ == NULL)
  {
    if (analyzeInterestRegions(data, data_kdtree, interest_region_indices) < 0)
    {
      return;
    }
  }

  computeFeatures(results);
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void Orientation::computeFeatures(cv::Vector<cv::Vector<float> >& results)
{
  // Assumes it has been correctly resized in compute() above
  unsigned int nbr_interest_samples = results.size();

  const vector<Eigen::Vector3d*>& tangents = spectral_info_->getTangents();
  const vector<Eigen::Vector3d*>& normals = spectral_info_->getNormals();
  if (nbr_interest_samples != tangents.size())
  {
    ROS_ERROR("Orientation::computeFeatures inconsistent spectral information");
    return;
  }

  for (size_t i = 0 ; i < nbr_interest_samples ; i++)
  {
    // Can compute feature only when spectral info available
    if (tangents[i] != NULL)
    {
      results[i].resize(result_size_);
      size_t feature_idx = 0;

      if (ref_tangent_defined_)
      {
        double tangent_dot = (tangents[i])->dot(ref_tangent_);
        if (tangent_dot < 0.0)
        {
          tangent_dot = (tangents[i])->dot(ref_tangent_flipped_);
        }
        results[i][feature_idx++] = tangent_dot;
      }

      if (ref_normal_defined_)
      {
        double normal_dot = (normals[i])->dot(ref_normal_);
        if (normal_dot < 0.0)
        {
          normal_dot = (normals[i])->dot(ref_normal_flipped_);
        }
        results[i][feature_idx++] = normal_dot;
      }
    }
  }
}
