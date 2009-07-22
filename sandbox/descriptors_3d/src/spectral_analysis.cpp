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

#include <descriptors_3d/spectral_analysis.h>

using namespace std;

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
SpectralAnalysis::~SpectralAnalysis()
{
  clear();
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void SpectralAnalysis::clear()
{
  unsigned int nbr_data = normals_.size();
  for (unsigned int i = 0 ; i < nbr_data ; i++)
  {
    if (normals_[i] != NULL)
    {
      delete normals_[i];
      delete middle_eig_vecs_[i];
      delete tangents_[i];
      delete eigen_values_[i];
      delete centroids_[i];
    }
  }
  normals_.clear();
  middle_eig_vecs_.clear();
  tangents_.clear();
  eigen_values_.clear();
  centroids_.clear();

  support_radius_defined_ = false;
  spectral_info_ = NULL;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int SpectralAnalysis::useSpectralInformation(SpectralAnalysis* spectral_info)
{
  if (support_radius_defined_)
  {
    ROS_ERROR("Cannot use spectral information after the radius had been defined");
    return -1;
  }
  else
  {
    spectral_info_ = spectral_info;
    return 0;
  }
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int SpectralAnalysis::setSpectralRadius(double support_radius)
{
  if (spectral_info_ == NULL)
  {
    support_radius_ = support_radius;
    support_radius_defined_ = true;
    return 0;
  }
  else
  {
    ROS_ERROR("Cannot change support radius when spectral information has been defined");
    return -1;
  }
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int SpectralAnalysis::analyzeInterestPoints(const robot_msgs::PointCloud& data,
                                            cloud_kdtree::KdTree& data_kdtree,
                                            const cv::Vector<robot_msgs::Point32*>& interest_pts)
{
  // ----------------------------------------
  // Ensure some regions are provided
  unsigned int nbr_interest_pts = interest_pts.size();

  // ----------------------------------------
  // Ensure radius is valid
  if (support_radius_defined_ == false || support_radius_ < 1e-6)
  {
    ROS_ERROR("SpectralShape::compute() support radius must be set to a positive value");
    return -1;
  }

  // ----------------------------------------
  // Clear out any previous computations
  clear();

  // ----------------------------------------
  // Allocate accordingly
  normals_.assign(nbr_interest_pts, NULL);
  middle_eig_vecs_.assign(nbr_interest_pts, NULL);
  tangents_.assign(nbr_interest_pts, NULL);
  eigen_values_.assign(nbr_interest_pts, NULL);
  centroids_.assign(nbr_interest_pts, NULL);

  // ----------------------------------------
  // Find neighboring points within radius for each interest point
  for (size_t i = 0 ; i < nbr_interest_pts ; i++)
  {
    // ---------------------
    // Retrieve next interest point
    const robot_msgs::Point32* curr_interest_pt = interest_pts[i];
    if (curr_interest_pt == NULL)
    {
      ROS_WARN("SpectralAnalysis::analyzeInterestPoints() passed NULL interest point");
      continue;
    }

    // ---------------------
    // Retrieve neighboring points around the interest point
    vector<int> neighbor_indices;
    vector<float> neighbor_distances; // unused
    // radiusSearch returning false (0 neighbors) is okay
    data_kdtree.radiusSearch(*curr_interest_pt, support_radius_, neighbor_indices, neighbor_distances);

    // ---------------------
    // Compute spectral information for interest point
    computeSpectralInfo(data, neighbor_indices, i);
  }

  // ----------------------------------------
  spectral_info_ = this;
  return 0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int SpectralAnalysis::analyzeInterestRegions(const robot_msgs::PointCloud& data,
                                             cloud_kdtree::KdTree& data_kdtree,
                                             const cv::Vector<vector<int>*>& interest_region_indices)
{
  // ----------------------------------------
  // Ensure some regions are provided
  unsigned int nbr_regions = interest_region_indices.size();

  // ----------------------------------------
  // Ensure radius has been defined (negative value is okay if using regions)
  if (support_radius_defined_ == false)
  {
    ROS_ERROR("SpectralShape::compute() support radius must be set");
    return -1;
  }

  // ----------------------------------------
  // Clear out any previous computations
  clear();

  // ----------------------------------------
  // Allocate accordingly
  normals_.assign(nbr_regions, NULL);
  middle_eig_vecs_.assign(nbr_regions, NULL);
  tangents_.assign(nbr_regions, NULL);
  eigen_values_.assign(nbr_regions, NULL);
  centroids_.assign(nbr_regions, NULL);

  // ----------------------------------------
  // For each interest region, either:
  //   Use the region itself as the support volume
  //   Find a support volume within a radius from the region's centroid
  robot_msgs::Point32 region_centroid;
  for (size_t i = 0 ; i < nbr_regions ; i++)
  {
    // ---------------------
    // Retrieve next interest region
    // (By default, use the interest region as the support volume)
    const vector<int>* curr_interest_region = interest_region_indices[i];
    if (curr_interest_region == NULL)
    {
      ROS_WARN("SpectralAnalysis::analyzeInterestPoints() passed NULL interest region");
      continue;
    }

    // Do a range search around the interest region's CENTROID if indicated
    vector<int> neighbor_indices;
    if (support_radius_ > 1e-6)
    {
      // Compute centroid of interest region
      cloud_geometry::nearest::computeCentroid(data, *curr_interest_region, region_centroid);

      vector<float> neighbor_distances; // unused
      // radiusSearch returning false (0 neighbors) is okay
      data_kdtree.radiusSearch(region_centroid, support_radius_, neighbor_indices, neighbor_distances);

      // Now point to the neighboring points from radiusSearch
      curr_interest_region = &neighbor_indices;
    }

    // ---------------------
    // Compute spectral information for interest region
    computeSpectralInfo(data, *curr_interest_region, i);
  }

  // ----------------------------------------
  spectral_info_ = this;
  return 0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void SpectralAnalysis::computeSpectralInfo(const robot_msgs::PointCloud& data,
                                           const vector<int>& support_volume_indices,
                                           size_t idx)
{
  // ----------------------------------------
  // Need 3-by-3 matrix to have full rank
  if (support_volume_indices.size() < 3)
  {
    ROS_WARN("SpectralAnalysis::computeSpectralInfo() not enough neighbors for interest sample %u", idx);
    return;
  }

  // ----------------------------------------
  // Allocate for new data
  normals_[idx] = new Eigen::Vector3d();
  middle_eig_vecs_[idx] = new Eigen::Vector3d();
  tangents_[idx] = new Eigen::Vector3d();
  eigen_values_[idx] = new Eigen::Vector3d();
  centroids_[idx] = new Eigen::Vector3d();

  // ----------------------------------------
  // Eigen-analysis of support volume
  // smallest eigenvalue = index 0
  robot_msgs::Point32 centroid;
  Eigen::Matrix3d eigen_vectors;
  cloud_geometry::nearest::computePatchEigenNormalized(data, support_volume_indices, eigen_vectors,
      *(eigen_values_[idx]), centroid);

  // ----------------------------------------
  // Populate containers
  for (unsigned int j = 0 ; j < 3 ; j++)
  {
    (*(normals_[idx]))[j] = eigen_vectors(j, 0);
    (*(middle_eig_vecs_[idx]))[j] = eigen_vectors(j, 1);
    (*(tangents_[idx]))[j] = eigen_vectors(j, 2);
  }

  // Make unit length
  (normals_[idx])->normalize();
  (middle_eig_vecs_[idx])->normalize();
  (tangents_[idx])->normalize();

  (*(centroids_[idx]))[0] = centroid.x;
  (*(centroids_[idx]))[1] = centroid.y;
  (*(centroids_[idx]))[2] = centroid.z;
}

