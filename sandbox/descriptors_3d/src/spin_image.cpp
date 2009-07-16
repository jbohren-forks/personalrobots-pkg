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

#include <descriptors_3d/spin_image.h>

using namespace std;

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void SpinImage::setAxisCustom(double ref_x, double ref_y, double ref_z)
{
  spin_axis_[0] = ref_x;
  spin_axis_[1] = ref_y;
  spin_axis_[2] = ref_z;
  spin_axis_.normalize();
  spin_axis_type_ = SpinImage::CUSTOM;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
int SpinImage::setImageDimensions(double row_res,
                                  double col_res,
                                  unsigned int nbr_rows,
                                  unsigned int nbr_cols)
{
  if (nbr_rows % 2 != 1)
  {
    ROS_ERROR("Number of rows for spin image must be odd");
    return -1;
  }
  row_res_ = row_res;
  col_res_ = col_res;
  nbr_rows_ = nbr_rows;
  nbr_cols_ = nbr_cols;
  dimensions_defined_ = true;
  result_size_ = nbr_rows * nbr_cols;
  return 0;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void SpinImage::compute(const robot_msgs::PointCloud& data,
                        cloud_kdtree::KdTree& data_kdtree,
                        const cv::Vector<robot_msgs::Point32*>& interest_pts,
                        cv::Vector<cv::Vector<float> >& results)
{
  size_t nbr_interest_pts = interest_pts.size();
  results.resize(nbr_interest_pts);

  // ----------------------------------------
  // Ensure the axis and image dimensions are defined
  if (spin_axis_type_ == SpinImage::UNDEFINED || dimensions_defined_ == false)
  {
    ROS_ERROR("The spin axis and image dimensions must be defined first");
    return;
  }

  // ----------------------------------------
  // Extract normals/tangents if specified
  if (spin_axis_type_ != SpinImage::CUSTOM && spectral_info_ == NULL)
  {
    if (analyzeInterestPoints(data, data_kdtree, interest_pts) < 0)
    {
      return;
    }
  }

  // ----------------------------------------
  // Recover the normals/tangents if specified
  const vector<Eigen::Vector3d*>* normals = NULL;
  const vector<Eigen::Vector3d*>* tangents = NULL;
  if (spin_axis_type_ == NORMAL)
  {
    normals = &(spectral_info_->getNormals());
  }
  else if (spin_axis_type_ == TANGENT)
  {
    tangents = &(spectral_info_->getTangents());
  }
  else
  {
    // must be CUSTOM
    spin_axis_.normalize();
  }

  // ----------------------------------------
  // Compute the radius to the corner square that will be needed for radius search
  double spin_radius = sqrt(pow(row_res_ * nbr_rows_, 2.0) + pow(col_res_ * nbr_cols_, 2.0));

  // ----------------------------------------
  // For each interest point, look at neighbors and compute the spin image
  Eigen::Vector3d center_pt;
  vector<int> neighbor_indices;
  vector<float> neighbor_distances; // unused
  for (size_t i = 0 ; i < nbr_interest_pts ; i++)
  {
    // Spin around the interest point
    center_pt[0] = (*(interest_pts[i])).x;
    center_pt[1] = (*(interest_pts[i])).y;
    center_pt[2] = (*(interest_pts[i])).z;

    // Find neighboring points
    neighbor_indices.clear();
    neighbor_distances.clear();
    if (!data_kdtree.radiusSearch(*(interest_pts[i]), spin_radius, neighbor_indices, neighbor_distances))
    {
      ROS_WARN("SpinImage::compute() radius search failed.  This should not happen.");
      continue;
    }

    // Make spin axis be unit
    if (spin_axis_type_ == NORMAL)
    {
      for (unsigned int j = 0 ; j < 3 ; j++)
      {
        spin_axis_[j] = (*((*normals)[i]))[j];
      }
      spin_axis_.normalize();
    }
    else if (spin_axis_type_ == TANGENT)
    {
      for (unsigned int j = 0 ; j < 3 ; j++)
      {
        spin_axis_[j] = (*((*tangents)[i]))[j];
      }
      spin_axis_.normalize();
    }

    computeSpinImage(data, neighbor_indices, center_pt, results[i]);
  }
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void SpinImage::compute(const robot_msgs::PointCloud& data,
                        cloud_kdtree::KdTree& data_kdtree,
                        const cv::Vector<vector<int>*>& interest_region_indices,
                        cv::Vector<cv::Vector<float> >& results)
{
  size_t nbr_interest_regions = interest_region_indices.size();
  results.resize(nbr_interest_regions);

  // ----------------------------------------
  // Ensure the axis and image dimensions are defined
  if (spin_axis_type_ == SpinImage::UNDEFINED || dimensions_defined_ == false)
  {
    ROS_ERROR("The spin axis and image dimensions must be defined first");
    return;
  }

  // ----------------------------------------
  // Extract normals/tangents if specified
  if (spin_axis_type_ != SpinImage::CUSTOM && spectral_info_ == NULL)
  {
    if (analyzeInterestRegions(data, data_kdtree, interest_region_indices) < 0)
    {
      return;
    }
  }

  // ----------------------------------------
  // Recover the normals/tangents if specified
  const vector<Eigen::Vector3d*>* normals = NULL;
  const vector<Eigen::Vector3d*>* tangents = NULL;
  if (spin_axis_type_ == NORMAL)
  {
    normals = &(spectral_info_->getNormals());
  }
  else if (spin_axis_type_ == TANGENT)
  {
    tangents = &(spectral_info_->getTangents());
  }
  else
  {
    // must be CUSTOM
    spin_axis_.normalize();
  }

  // ----------------------------------------
  // Compute the radius to the corner square that will be needed for radius search
  double spin_radius = -1.0;
  if (use_interest_region_only_ == false)
  {
    spin_radius = sqrt(pow(row_res_ * nbr_rows_, 2.0) + pow(col_res_ * nbr_cols_, 2.0));
  }

  // ----------------------------------------
  // For each region, compute centroid as the center of spin image, then
  // look at neighbors and compute the spin image
  Eigen::Vector3d center_pt;
  robot_msgs::Point32 region_centroid;
  vector<int> radius_search_neighbors;
  vector<float> neighbor_distances; // unused
  vector<int>* neighbors_for_spin_img = &radius_search_neighbors;
  for (size_t i = 0 ; i < nbr_interest_regions ; i++)
  {
    // Spin around the centroid of the interest region
    cloud_geometry::nearest::computeCentroid(data, *(interest_region_indices[i]), region_centroid);
    center_pt[0] = region_centroid.x;
    center_pt[1] = region_centroid.y;
    center_pt[2] = region_centroid.z;

    // Find neighboring points
    if (use_interest_region_only_)
    {
      neighbors_for_spin_img = interest_region_indices[i];
    }
    else
    {
      radius_search_neighbors.clear();
      neighbor_distances.clear();
      if (!data_kdtree.radiusSearch(region_centroid, spin_radius, radius_search_neighbors, neighbor_distances))
      {
        ROS_WARN("SpinImage::compute() radius search failed.  This should not happen.");
        continue;
      }
    }

    // Make spin axis be unit
    if (spin_axis_type_ == NORMAL)
    {
      for (unsigned int j = 0 ; j < 3 ; j++)
      {
        spin_axis_[j] = (*((*normals)[i]))[j];
      }
      spin_axis_.normalize();
    }
    else if (spin_axis_type_ == TANGENT)
    {
      for (unsigned int j = 0 ; j < 3 ; j++)
      {
        spin_axis_[j] = (*((*tangents)[i]))[j];
      }
      spin_axis_.normalize();
    }

    computeSpinImage(data, *neighbors_for_spin_img, center_pt, results[i]);
  }
}

// --------------------------------------------------------------
/*
 * See function definition
 * This method uses spin_axis_ and assumes it is unit length
 */
// --------------------------------------------------------------
void SpinImage::computeSpinImage(const robot_msgs::PointCloud& data,
                                 const vector<int>& neighbor_indices,
                                 const Eigen::Vector3d& center_pt,
                                 cv::Vector<float>& spin_image)
{
  spin_image.resize(result_size_);
  unsigned int row_offset = nbr_rows_ / 2;
  //static_cast<unsigned int> (floor(0.5 * static_cast<double> (nbr_rows_)));

  Eigen::Vector3d neighbor_vec; // vector from center to neighbor point
  double neighbor_vec_norm = 0.0;
  double axis_projection = 0.0;
  int signed_row_nbr = 0;
  int curr_row = 0;
  unsigned int curr_col = 0;
  float max_bin_count = 1.0; // init to 1.0 so avoid divide by 0 if no neighbor points
  size_t curr_spin_img_idx = 0;
  for (unsigned int i = 0 ; i < neighbor_indices.size() ; i++)
  {
    // Create vector from center point to neighboring point
    neighbor_vec[0] = data.pts[neighbor_indices[i]].x - center_pt[0];
    neighbor_vec[1] = data.pts[neighbor_indices[i]].y - center_pt[1];
    neighbor_vec[2] = data.pts[neighbor_indices[i]].z - center_pt[2];
    neighbor_vec_norm = neighbor_vec.norm();

    // ----------------------------------------
    // Scalar projection of neighbor_vec onto spin axis (unit length)
    axis_projection = neighbor_vec_norm * neighbor_vec.dot(spin_axis_);
    // Computed signed bin along the axis
    signed_row_nbr = static_cast<int> (floor(axis_projection / row_res_));
    // Offset the bin
    curr_row = signed_row_nbr + row_offset;

    // ----------------------------------------
    // Two vectors a and b originating from the same point form a parallelogram
    // |a x b| is the area Q of a parallelogram with base |a| and height h (Q = |a|h)
    // http://en.wikipedia.org/wiki/Cross_product
    // a = spin axis (unit length)
    // b = neighbor_vec
    // h = Q / |a| = |a x b| / |a| = |a x b|
    curr_col = static_cast<unsigned int> (floor((spin_axis_.cross(neighbor_vec)).norm() / col_res_));

    // Increment appropriate spin image cell
    if (curr_row >= 0 && static_cast<unsigned int> (curr_row) < nbr_rows_ && curr_col < nbr_cols_)
    {
      curr_spin_img_idx = static_cast<size_t> (curr_row * nbr_cols_ + curr_col);
      spin_image[curr_spin_img_idx] += 1.0;
      if (spin_image[curr_spin_img_idx] > max_bin_count)
      {
        max_bin_count = spin_image[curr_spin_img_idx];
      }
    }
  }

  for (size_t i = 0 ; i < result_size_ ; i++)
  {
    spin_image[i] /= max_bin_count;
  }
}
