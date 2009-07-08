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

#include <descriptors_3d/local_geometry.h>

using namespace std;

// --------------------------------------------------------------
/*! Ready to compute features when:
 *    data is set
 *    interest point with radius OR interest region set
 */
// --------------------------------------------------------------
inline bool LocalGeometry::readyToCompute()
{
  return data_set_ && ((interest_pt_set_ && radius_ > 0.0) || interest_region_set_);
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
bool LocalGeometry::compute(Eigen::MatrixXf** result, bool debug)
{
  if (readyToCompute() == false)
  {
    ROS_ERROR("LocalGeometry::compute not ready");
    return false;
  }

  // ----------------------------------------
  // find neighbors
  const vector<int>* neighbor_indices = NULL;

  vector<int> range_search_indices;
  if (interest_pt_set_)
  {
    neighbor_indices = &range_search_indices;
    vector<float> neighbor_distances; // unused
    if (data_kdtree_->radiusSearch(interest_pt_idx_, radius_, range_search_indices, neighbor_distances)
        == false)
    {
      ROS_ERROR("LocalGeometry::compute radius search failed");
      return false;
    }
  }
  else
  {
    neighbor_indices = interest_region_indices_;
  }

  // ----------------------------------------
  if (neighbor_indices->size() < 3)
  {
    ROS_ERROR("did not have enough neighbors");
    return false;
  }

  // ----------------------------------------
  Eigen::Matrix3d eigen_vectors;
  Eigen::Vector3d eigen_values;
  Eigen::Vector3d tangent;
  Eigen::Vector3d normal;
  cloud_geometry::nearest::computePatchEigen(*data_, *neighbor_indices, eigen_vectors, eigen_values);
  // smallest eigenvalue = index 0
  for (int i = 0 ; i < 3 ; i++)
  {
    normal[i] = eigen_vectors(i, 0);
    tangent[i] = eigen_vectors(i, 2);
  }

  // ----------------------------------------
  *result = new Eigen::MatrixXf(result_size_, 1);

  unsigned int idx = 0;
  (**result)[idx++] = eigen_values[2];
  (**result)[idx++] = eigen_values[2] - eigen_values[1];
  (**result)[idx++] = eigen_values[1] - eigen_values[0];

  if (ref_tangent_defined_)
  {
    double tangent_dot = tangent.dot(ref_tangent_);
    if (tangent_dot < 0.0)
    {
      tangent_dot = tangent.dot(ref_tangent_flipped_);
    }
    (**result)[idx++] = tangent_dot;
  }

  if (ref_normal_defined_)
  {
    double normal_dot = normal.dot(ref_normal_);
    if (normal_dot < 0.0)
    {
      normal_dot = normal.dot(ref_normal_flipped_);
    }
    (**result)[idx++] = normal_dot;
  }

  if (use_elevation_)
  {
    (**result)[idx++] = data_->pts[interest_pt_idx_].z;
  }

  return true;
}
