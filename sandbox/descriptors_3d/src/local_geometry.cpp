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
/*! See function definition */
// --------------------------------------------------------------
void LocalGeometry::compute(const robot_msgs::PointCloud& data,
                            cloud_kdtree::KdTree& data_kdtree,
                            const cv::Vector<robot_msgs::Point32*>& interest_pts,
                            cv::Vector<cv::Vector<float> >& results)
{
  robot_msgs::Point32 centroid;
  Eigen::Matrix3d eigen_vectors;
  Eigen::Vector3d eigen_values;
  Eigen::Vector3d tangent;
  Eigen::Vector3d normal;

  unsigned int nbr_pts = interest_pts.size();
  results.resize(nbr_pts);

  // If using spectral features, then crete neighborhoods
  if (use_spectral_)
  {
    if (spectral_radius_ < 0.0)
    {
      ROS_ERROR("You should call compute() with region indices");
      results.clear();
      results.resize(nbr_pts);
      return;
    }

    vector<int> neighbor_indices;
    vector<float> neighbor_distances; // unused
    for (unsigned int i = 0 ; i < nbr_pts ; i++)
    {
      // ----------------------------------------
      // Find neighboring points within radius
      neighbor_indices.clear();
      neighbor_distances.clear();
      if (data_kdtree.radiusSearch(*(interest_pts[i]), spectral_radius_, neighbor_indices, neighbor_distances)
          == false)
      {
        ROS_ERROR("LocalGeometry::compute radius search failed");
        results[i].clear();
        continue;
      }

      // ----------------------------------------
      // Need 3-by-3 matrix to have full rank
      if (neighbor_indices.size() < 3)
      {
        ROS_ERROR("did not have enough neighbors");
        results[i].clear();
        continue;
      }

      // ----------------------------------------
      // Eigen-analysis of support volume
      // smallest eigenvalue = index 0
      cloud_geometry::nearest::computePatchEigenNormalized(data, neighbor_indices, eigen_vectors,
          eigen_values, centroid);
      for (int j = 0 ; j < 3 ; j++)
      {
        normal[j] = eigen_vectors(j, 0);
        tangent[j] = eigen_vectors(j, 2);
      }

      // ----------------------------------------
      // Compute features
      populateFeaturesWithSpectral(eigen_values, tangent, normal, *(interest_pts[i]), data, neighbor_indices,
          results[i]);
    }
  }
  // Otherwise compute features that do not require normal/tangent information
  else
  {
    for (unsigned int i = 0 ; i < nbr_pts ; i++)
    {
      populateFeatures(0, *(interest_pts[i]), results[i]);
    }
  }
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
/*
void LocalGeometry::compute(const robot_msgs::PointCloud& data,
                            cloud_kdtree::KdTree& data_kdtree,
                            cv::Vector<cv::Vector<float> >& results)
{
  robot_msgs::Point32 centroid;
  Eigen::Matrix3d eigen_vectors;
  Eigen::Vector3d eigen_values;
  Eigen::Vector3d tangent;
  Eigen::Vector3d normal;

  unsigned int nbr_pts = data.pts.size();
  results.resize(nbr_pts);

  // If using spectral features, then crete neighborhoods
  if (use_spectral_)
  {
    if (spectral_radius_ < 0.0)
    {
      ROS_ERROR("You should call compute() with region indices");
      results.clear();
      results.resize(nbr_pts);
      return;
    }

    vector<int> neighbor_indices;
    vector<float> neighbor_distances; // unused
    for (unsigned int i = 0 ; i < nbr_pts ; i++)
    {
      // -------------------------------
      if (i % 1000 == 0)
      {
        ROS_INFO("sample: %u / %u", i, nbr_pts);
      }

      // ----------------------------------------
      // Find neighboring points within radius
      neighbor_indices.clear();
      neighbor_distances.clear();
      if (data_kdtree.radiusSearch(i, spectral_radius_, neighbor_indices, neighbor_distances) == false)
      {
        results[i].clear();
        ROS_ERROR("LocalGeometry::compute radius search failed, vector has size: %u", results[i].size());
        continue;
      }

      // ----------------------------------------
      // Need 3-by-3 matrix to have full rank
      if (neighbor_indices.size() < 3)
      {
        results[i].clear();
        ROS_ERROR("LocalGeometry::compute not enough neighbors at pt %u, vector has size: %u", i, results[i].size());
        continue;
      }

      // ----------------------------------------
      // Eigen-analysis of support volume
      // smallest eigenvalue = index 0
      cloud_geometry::nearest::computePatchEigenNormalized(data, neighbor_indices, eigen_vectors,
          eigen_values, centroid);
      for (int j = 0 ; j < 3 ; j++)
      {
        normal[j] = eigen_vectors(j, 0);
        tangent[j] = eigen_vectors(j, 2);
      }

      // ----------------------------------------
      // Compute features
      populateFeaturesWithSpectral(eigen_values, tangent, normal, data.pts[i], data, neighbor_indices,
          results[i]);
    }
  }
  // Otherwise compute features that do not require normal/tangent information
  else
  {
    for (unsigned int i = 0 ; i < nbr_pts ; i++)
    {
      populateFeatures(0, data.pts[i], results[i]);
    }
  }
}
*/

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
void LocalGeometry::compute(const robot_msgs::PointCloud& data,
                            cloud_kdtree::KdTree& data_kdtree,
                            const cv::Vector<vector<int>*>& interest_region_indices,
                            cv::Vector<cv::Vector<float> >& results)
{
  robot_msgs::Point32 centroid;
  Eigen::Matrix3d eigen_vectors;
  Eigen::Vector3d eigen_values;
  Eigen::Vector3d tangent;
  Eigen::Vector3d normal;

  unsigned int nbr_regions = interest_region_indices.size();
  results.resize(nbr_regions);
  vector<int>* curr_region_indices = NULL;

  // used when doing range search around centroid
  vector<int> neighbor_indices;
  vector<float> neighbor_distances; // unused

  // If using spectral features, then crete neighborhoods
  if (use_spectral_)
  {

    for (unsigned int i = 0 ; i < nbr_regions ; i++)
    {
      if (spectral_radius_ < 0.0)
      {
        curr_region_indices = interest_region_indices[i];
      }
      else
      {
        // ----------------------------------------
        // Find neighboring points within radius around CENTROID
        neighbor_indices.clear();
        neighbor_distances.clear();
        curr_region_indices = &neighbor_indices;
        cloud_geometry::nearest::computeCentroid(data, *(curr_region_indices), centroid);
        if (data_kdtree.radiusSearch(centroid, spectral_radius_, neighbor_indices, neighbor_distances)
            == false)
        {
          ROS_ERROR("LocalGeometry::compute radius search failed");
          results[i].clear();
          continue;
        }
      }

      // ----------------------------------------
      // Need 3-by-3 matrix to have full rank
      if (curr_region_indices->size() < 3)
      {
        ROS_ERROR("did not have enough neighbors");
        results[i].clear();
        continue;
      }

      // ----------------------------------------
      // Eigen-analysis of support volume
      // smallest eigenvalue = index 0
      cloud_geometry::nearest::computePatchEigenNormalized(data, *(curr_region_indices), eigen_vectors,
          eigen_values, centroid);
      for (int j = 0 ; j < 3 ; j++)
      {
        normal[j] = eigen_vectors(j, 0);
        tangent[j] = eigen_vectors(j, 2);
      }

      // ----------------------------------------
      // Compute features
      populateFeaturesWithSpectral(eigen_values, tangent, normal, centroid, data, *curr_region_indices,
          results[i]);
    }

  }
  // Otherwise compute features that do not require normal/tangent information
  else
  {
    for (unsigned int i = 0 ; i < nbr_regions ; i++)
    {
      cloud_geometry::nearest::computeCentroid(data, *(curr_region_indices), centroid);
      populateFeatures(0, centroid, results[i]);
    }
  }
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
void LocalGeometry::populateFeaturesWithSpectral(const Eigen::Vector3d& eigen_values,
                                                 const Eigen::Vector3d& tangent,
                                                 const Eigen::Vector3d& normal,
                                                 const robot_msgs::Point32& interest_pt,
                                                 const robot_msgs::PointCloud& data,
                                                 const vector<int>& neighborhood_indices,
                                                 cv::Vector<float>& result)
{
  result.resize(result_size_);

  unsigned int idx = 0;
  result[idx++] = eigen_values[0]; // scatter
  result[idx++] = eigen_values[2] - eigen_values[1]; // linear
  result[idx++] = eigen_values[1] - eigen_values[0]; // surface

  if (ref_tangent_defined_)
  {
    double tangent_dot = tangent.dot(ref_tangent_);
    if (tangent_dot < 0.0)
    {
      tangent_dot = tangent.dot(ref_tangent_flipped_);
    }
    result[idx++] = tangent_dot;
  }

  if (ref_normal_defined_)
  {
    double normal_dot = normal.dot(ref_normal_);
    if (normal_dot < 0.0)
    {
      normal_dot = normal.dot(ref_normal_flipped_);
    }
    result[idx++] = normal_dot;
  }

  populateFeatures(idx, interest_pt, result);
}

// --------------------------------------------------------------
/*! See function definition */
// --------------------------------------------------------------
void LocalGeometry::populateFeatures(unsigned int start_idx,
                                     const robot_msgs::Point32& interest_pt,
                                     cv::Vector<float>& result)
{
  if (start_idx == 0)
  {
    result.resize(result_size_);
  }

  if (use_elevation_)
  {
    result[start_idx++] = interest_pt.z;
  }
}
