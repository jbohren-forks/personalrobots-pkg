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
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void BoundingBox::compute(const robot_msgs::PointCloud& data,
                          cloud_kdtree::KdTree& data_kdtree,
                          const cv::Vector<robot_msgs::Point32*>& interest_pts,
                          cv::Vector<cv::Vector<float> >& results)
{
  ROS_ERROR("BoundingBox can only be used on a region of points");
  results.resize(interest_pts.size());
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void BoundingBox::compute(const robot_msgs::PointCloud& data,
                          cloud_kdtree::KdTree& data_kdtree,
                          const cv::Vector<vector<int>*>& interest_region_indices,
                          cv::Vector<cv::Vector<float> >& results)
{
  // ----------------------------------------
  // Allocate the results to be 0 vectors for each interest region
  unsigned int nbr_interest_regions = interest_region_indices.size();
  results.resize(nbr_interest_regions);

  // ----------------------------------------
  // Extract principle components if specified
  const vector<Eigen::Vector3d*>* eig_vec_max = NULL;
  const vector<Eigen::Vector3d*>* eig_vec_mid = NULL;
  const vector<Eigen::Vector3d*>* eig_vec_min = NULL;
  if (use_pca_bbox_)
  {
    if (spectral_info_ == NULL)
    {
      if (analyzeInterestRegions(data, data_kdtree, interest_region_indices) < 0)
      {
        return;
      }
    }
    eig_vec_max = &(spectral_info_->getTangents());
    eig_vec_mid = &(spectral_info_->getMiddleEigenVectors());
    eig_vec_min = &(spectral_info_->getNormals());
  }

  // ----------------------------------------
  // For each interest region, compute its bounding box
  for (size_t i = 0 ; i < nbr_interest_regions ; i++)
  {
    // --------------------------
    const vector<int>* curr_indices = interest_region_indices[i];
    if (curr_indices == NULL || curr_indices->size() == 0)
    {
      ROS_ERROR("BoudningBox::compute() Passed NULL or empty region indices");
      continue;
    }

    // --------------------------
    // Allocate for current region's features
    size_t results_idx = 0;
    results[i].resize(result_size_);

    // --------------------------
    // Compute bounding box of the xyz point cloud
    if (use_raw_bbox_)
    {
      // Initialize extrema values to the first coordinate in the interest region
      float min_x = data.pts[(*curr_indices)[0]].x;
      float min_y = data.pts[(*curr_indices)[0]].y;
      float min_z = data.pts[(*curr_indices)[0]].z;
      float max_x = min_x;
      float max_y = min_y;
      float max_z = min_z;

      // Loop over remaining points in region and update extremas
      for (unsigned int j = 1 ; j < curr_indices->size() ; j++)
      {
        // x
        float curr_coord = data.pts[(*curr_indices)[j]].x;
        if (curr_coord < min_x)
        {
          min_x = curr_coord;
        }
        if (curr_coord > max_x)
        {
          max_x = curr_coord;
        }
        // y
        curr_coord = data.pts[(*curr_indices)[j]].y;
        if (curr_coord < min_y)
        {
          min_y = curr_coord;
        }
        if (curr_coord > max_y)
        {
          max_y = curr_coord;
        }
        // z
        curr_coord = data.pts[(*curr_indices)[j]].z;
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
      results[i][results_idx++] = max_x - min_x;
      results[i][results_idx++] = max_y - min_y;
      results[i][results_idx++] = max_z - min_z;
    }

    // --------------------------
    // Compute bounding box in the principle components of the point cloud
    if (use_pca_bbox_)
    {
      if ((*eig_vec_max)[i] == NULL)
      {
        ROS_WARN("No spectral information for interest region %u...skipping it", i);
        continue;
      }

      // Initialize extrema values to the first coordinate in the interest region
      Eigen::Vector3d curr_pt;
      curr_pt[0] = data.pts[(*curr_indices)[0]].x;
      curr_pt[1] = data.pts[(*curr_indices)[0]].y;
      curr_pt[2] = data.pts[(*curr_indices)[0]].z;
      float min_x = curr_pt.dot(*((*eig_vec_max)[0]));
      float min_y = curr_pt.dot(*((*eig_vec_mid)[0]));
      float min_z = curr_pt.dot(*((*eig_vec_min)[0]));
      float max_x = min_x;
      float max_y = min_y;
      float max_z = min_z;

      // Loop over remaining points in region and update extremas
      for (unsigned int j = 1 ; j < curr_indices->size() ; j++)
      {
        curr_pt[0] = data.pts[(*curr_indices)[j]].x;
        curr_pt[1] = data.pts[(*curr_indices)[j]].y;
        curr_pt[2] = data.pts[(*curr_indices)[j]].z;

        // biggest eigenvector
        float curr_coord = curr_pt.dot(*((*eig_vec_max)[i]));
        if (curr_coord < min_x)
        {
          min_x = curr_coord;
        }
        if (curr_coord > max_x)
        {
          max_x = curr_coord;
        }
        // middle eigenvector
        curr_coord = curr_pt.dot(*((*eig_vec_mid)[i]));
        if (curr_coord < min_y)
        {
          min_y = curr_coord;
        }
        if (curr_coord > max_y)
        {
          max_y = curr_coord;
        }
        // smallest eigenvector
        curr_coord = curr_pt.dot(*((*eig_vec_min)[i]));
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
      results[i][results_idx++] = max_x - min_x;
      results[i][results_idx++] = max_y - min_y;
      results[i][results_idx++] = max_z - min_z;
    }
  }
}
