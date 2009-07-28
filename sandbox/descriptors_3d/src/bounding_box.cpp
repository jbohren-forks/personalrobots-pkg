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

#include <descriptors_3d/bounding_box.h>

using namespace std;

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
BoundingBox::BoundingBox(bool use_pca_bbox, bool use_raw_bbox)
{
  use_pca_bbox_ = use_pca_bbox;
  use_raw_bbox_ = use_raw_bbox;

  result_size_ = 0;
  if (use_pca_bbox_)
  {
    result_size_ += 3;
  }
  if (use_raw_bbox_)
  {
    result_size_ += 3;
  }

  bbox_radius_ = -1.0;
  bbox_radius_set_ = false;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void BoundingBox::setBoundingBoxRadius(float bbox_radius)
{
  bbox_radius_ = bbox_radius;
  bbox_radius_set_ = true;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void BoundingBox::compute(const robot_msgs::PointCloud& data,
                          cloud_kdtree::KdTree& data_kdtree,
                          const cv::Vector<const robot_msgs::Point32*>& interest_pts,
                          cv::Vector<cv::Vector<float> >& results)
{
  // ----------------------------------------
  // Allocate the results to be 0 vectors for each interest point
  unsigned int nbr_interest_pts = interest_pts.size();
  results.resize(nbr_interest_pts);

  // ----------------------------------------
  // Verify valid radius has been set
  if (bbox_radius_set_ == false)
  {
    ROS_ERROR("BoundingBox::compute() the bounding box radius not set");
    return;
  }
  if (bbox_radius_ < 0.0)
  {
    ROS_ERROR("BoundingBox::compute() the radius %f is invalid", bbox_radius_);
    return;
  }

  // ----------------------------------------
  // Extract principle components if specified
  const vector<Eigen::Vector3d*>* eig_vecs_max = NULL;
  const vector<Eigen::Vector3d*>* eig_vecs_mid = NULL;
  const vector<Eigen::Vector3d*>* eig_vecs_min = NULL;
  if (use_pca_bbox_)
  {
    if (spectral_info_ == NULL)
    {
      if (analyzeInterestPoints(data, data_kdtree, interest_pts) < 0)
      {
        return;
      }
    }
    eig_vecs_max = &(spectral_info_->getTangents());
    eig_vecs_mid = &(spectral_info_->getMiddleEigenVectors());
    eig_vecs_min = &(spectral_info_->getNormals());
    if (eig_vecs_max->size() != nbr_interest_pts)
    {
      ROS_ERROR("BoundingBox::compute inconsistent spectral information");
      return;
    }
  }

  // ----------------------------------------
  // Iterate over each interest point, grab neighbors within bbox radius,
  // then compute bounding box
  for (size_t i = 0 ; i < nbr_interest_pts ; i++)
  {
    // Retrieve interest point
    const robot_msgs::Point32* curr_interest_pt = interest_pts[i];
    if (curr_interest_pt == NULL)
    {
      ROS_WARN("BoundingBox::compute() passed NULL interest point");
      continue;
    }

    // Grab neighbors around interest point
    vector<int> neighbor_indices;
    vector<float> neighbor_distances; // unused
    // radiusSearch returning false (0 points) is handled by computeBoundingBoxFeatures
    data_kdtree.radiusSearch(*curr_interest_pt, bbox_radius_, neighbor_indices, neighbor_distances);

    // Compute features
    if (use_pca_bbox_)
    {
      computeBoundingBoxFeatures(data, neighbor_indices, (*eig_vecs_max)[i], (*eig_vecs_mid)[i],
          (*eig_vecs_min)[i], results[i]);
    }
    else
    {
      computeBoundingBoxFeatures(data, neighbor_indices, NULL, NULL, NULL, results[i]);
    }
  }
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void BoundingBox::compute(const robot_msgs::PointCloud& data,
                          cloud_kdtree::KdTree& data_kdtree,
                          const cv::Vector<const vector<int>*>& interest_region_indices,
                          cv::Vector<cv::Vector<float> >& results)
{
  // ----------------------------------------
  // Allocate the results to be 0 vectors for each interest region
  unsigned int nbr_interest_regions = interest_region_indices.size();
  results.resize(nbr_interest_regions);

  // ----------------------------------------
  // Verify valid radius has been set
  if (bbox_radius_set_ == false)
  {
    ROS_ERROR("BoundingBox::compute() the bounding box radius not set");
    return;
  }

  // ----------------------------------------
  // Extract principle components if specified
  const vector<Eigen::Vector3d*>* eig_vecs_max = NULL;
  const vector<Eigen::Vector3d*>* eig_vecs_mid = NULL;
  const vector<Eigen::Vector3d*>* eig_vecs_min = NULL;
  if (use_pca_bbox_)
  {
    if (spectral_info_ == NULL)
    {
      if (analyzeInterestRegions(data, data_kdtree, interest_region_indices) < 0)
      {
        return;
      }
    }
    eig_vecs_max = &(spectral_info_->getTangents());
    eig_vecs_mid = &(spectral_info_->getMiddleEigenVectors());
    eig_vecs_min = &(spectral_info_->getNormals());
    if (eig_vecs_max->size() != nbr_interest_regions)
    {
      ROS_ERROR("BoundingBox::compute inconsistent spectral information");
      return;
    }
  }

  // ----------------------------------------
  // Compute the dimensions of the bounding box around the points
  for (size_t i = 0 ; i < nbr_interest_regions ; i++)
  {
    // Retrieve interest region
    const vector<int>* curr_interest_region = interest_region_indices[i];
    if (curr_interest_region == NULL)
    {
      ROS_WARN("BoundingBox::compute() passed NULL interest region");
      continue;
    }

    // Find the neighborhood around the region's centroid if indicated to.
    vector<int> neighbor_indices;
    if (bbox_radius_ > 1e-6)
    {
      // Compute centroid of interest region
      robot_msgs::Point32 region_centroid;
      cloud_geometry::nearest::computeCentroid(data, *curr_interest_region, region_centroid);

      vector<float> neighbor_distances; // unused
      // radiusSearch returning false (0 points) is handled by computeBoundingBoxFeatures
      data_kdtree.radiusSearch(region_centroid, bbox_radius_, neighbor_indices, neighbor_distances);

      // Now point to the neighboring points from radiusSearch
      curr_interest_region = &neighbor_indices;
    }

    // Compute features
    if (use_pca_bbox_)
    {
      computeBoundingBoxFeatures(data, *curr_interest_region, (*eig_vecs_max)[i], (*eig_vecs_mid)[i],
          (*eig_vecs_min)[i], results[i]);
    }
    else
    {
      computeBoundingBoxFeatures(data, *curr_interest_region, NULL, NULL, NULL, results[i]);
    }
  }
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void BoundingBox::computeBoundingBoxFeatures(const robot_msgs::PointCloud& data,
                                             const vector<int>& neighbor_indices,
                                             const Eigen::Vector3d* eig_vec_max,
                                             const Eigen::Vector3d* eig_vec_mid,
                                             const Eigen::Vector3d* eig_vec_min,
                                             cv::Vector<float>& result)
{
  result.resize(result_size_);
  unsigned int nbr_pts = neighbor_indices.size();

  // --------------------------
  // Check for special case when no points in the bounding box as will initialize
  // the min/max extremas using the first point below
  if (nbr_pts == 0)
  {
    ROS_WARN("BoundingBox::computeBoundingBoxFeatures() No points to form bounding box");
    for (size_t i = 0 ; i < result_size_ ; i++)
    {
      result[i] = 0.0;
    }
    return;
  }

  // --------------------------
  // Index into result
  size_t result_idx = 0;

  // --------------------------
  // Compute bounding box in the principle components of the point cloud
  if (use_pca_bbox_)
  {
    // NULL indicates no eigenvector could be extracted
    if (eig_vec_max == NULL)
    {
      ROS_WARN("BoundingBox::computeBoundingBoxFeatures() No spectral information...skipping it");
      result.clear();
      return;
    }

    // Get the norms of the current principle component vectors in order to
    // compute scalar projections
    double norm_ev_max = eig_vec_max->norm();
    double norm_ev_mid = eig_vec_mid->norm();
    double norm_ev_min = eig_vec_min->norm();

    // Initialize extrema values of the first point's scalar projections
    // onto the principle components
    Eigen::Vector3d curr_pt;
    curr_pt[0] = data.pts[neighbor_indices[0]].x;
    curr_pt[1] = data.pts[neighbor_indices[0]].y;
    curr_pt[2] = data.pts[neighbor_indices[0]].z;
    float min_v1 = curr_pt.dot(*eig_vec_max) / norm_ev_max;
    float min_v2 = curr_pt.dot(*eig_vec_mid) / norm_ev_mid;
    float min_v3 = curr_pt.dot(*eig_vec_min) / norm_ev_min;
    float max_v1 = min_v1;
    float max_v2 = min_v2;
    float max_v3 = min_v3;

    // Loop over remaining points in region and update projection extremas
    for (unsigned int i = 1 ; i < nbr_pts ; i++)
    {
      curr_pt[0] = data.pts[neighbor_indices[i]].x;
      curr_pt[1] = data.pts[neighbor_indices[i]].y;
      curr_pt[2] = data.pts[neighbor_indices[i]].z;

      // biggest eigenvector
      float curr_projection = curr_pt.dot(*eig_vec_max) / norm_ev_max;
      if (curr_projection < min_v1)
      {
        min_v1 = curr_projection;
      }
      if (curr_projection > max_v1)
      {
        max_v1 = curr_projection;
      }
      // middle eigenvector
      curr_projection = curr_pt.dot(*eig_vec_mid) / norm_ev_mid;
      if (curr_projection < min_v2)
      {
        min_v2 = curr_projection;
      }
      if (curr_projection > max_v2)
      {
        max_v2 = curr_projection;
      }
      // smallest eigenvector
      curr_projection = curr_pt.dot(*eig_vec_min) / norm_ev_min;
      if (curr_projection < min_v3)
      {
        min_v3 = curr_projection;
      }
      if (curr_projection > max_v3)
      {
        max_v3 = curr_projection;
      }
    }

    // --------------------------
    result[result_idx++] = max_v1 - min_v1;
    result[result_idx++] = max_v2 - min_v2;
    result[result_idx++] = max_v3 - min_v3;
  }

  // --------------------------
  // Compute bounding box of the xyz point cloud
  if (use_raw_bbox_)
  {
    // Initialize extrema values to the first coordinate in the interest region
    float min_x = data.pts[neighbor_indices[0]].x;
    float min_y = data.pts[neighbor_indices[0]].y;
    float min_z = data.pts[neighbor_indices[0]].z;
    float max_x = min_x;
    float max_y = min_y;
    float max_z = min_z;

    // Loop over remaining points in region and update extremas
    for (unsigned int i = 1 ; i < nbr_pts ; i++)
    {
      // x
      float curr_coord = data.pts[neighbor_indices[i]].x;
      if (curr_coord < min_x)
      {
        min_x = curr_coord;
      }
      if (curr_coord > max_x)
      {
        max_x = curr_coord;
      }
      // y
      curr_coord = data.pts[neighbor_indices[i]].y;
      if (curr_coord < min_y)
      {
        min_y = curr_coord;
      }
      if (curr_coord > max_y)
      {
        max_y = curr_coord;
      }
      // z
      curr_coord = data.pts[neighbor_indices[i]].z;
      if (curr_coord < min_z)
      {
        min_z = curr_coord;
      }
      if (curr_coord > max_z)
      {
        max_z = curr_coord;
      }
    }

    // --------------------------
    result[result_idx++] = max_x - min_x;
    result[result_idx++] = max_y - min_y;
    result[result_idx++] = max_z - min_z;
  }
}
