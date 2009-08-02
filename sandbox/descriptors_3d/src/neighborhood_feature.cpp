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

#include <descriptors_3d/generic/neighborhood_feature.h>

using namespace std;

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
NeighborhoodFeature::NeighborhoodFeature()
{
  neighborhood_radius_ = -1.0;
  neighborhood_radius_defined_ = false;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
NeighborhoodFeature::~NeighborhoodFeature()
{
}

// --------------------------------------------------------------
/* See function definition.
 * Invariant: neighborhood_radius_ is positive */
// --------------------------------------------------------------
void NeighborhoodFeature::doComputation(const sensor_msgs::PointCloud& data,
                                        cloud_kdtree::KdTree& data_kdtree,
                                        const cv::Vector<const geometry_msgs::Point32*>& interest_pts,
                                        cv::Vector<cv::Vector<float> >& results)
{
  // ----------------------------------------
  if (neighborhood_radius_defined_ == false)
  {
    ROS_ERROR("NeighborhoodFeature::compute radius not defined yet");
    return;
  }

  // ----------------------------------------
  // Iterate over each interest point, grab neighbors within bbox radius,
  // then compute bounding box
  size_t nbr_interest_pts = interest_pts.size();
  for (size_t i = 0 ; i < nbr_interest_pts ; i++)
  {
    // Retrieve interest point
    const geometry_msgs::Point32* curr_interest_pt = interest_pts[i];
    if (curr_interest_pt == NULL)
    {
      ROS_WARN("BoundingBox::compute() passed NULL interest point");
      continue;
    }

    // Grab neighbors around interest point
    vector<int> neighbor_indices;
    vector<float> neighbor_distances; // unused
    // radiusSearch returning false (0 points) is handled by computeBoundingBoxFeatures
    data_kdtree.radiusSearch(*curr_interest_pt, neighborhood_radius_, neighbor_indices, neighbor_distances);

    computeNeighborhoodFeature(data, neighbor_indices, i, results[i]);
  }
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void NeighborhoodFeature::doComputation(const sensor_msgs::PointCloud& data,
                                        cloud_kdtree::KdTree& data_kdtree,
                                        const cv::Vector<const vector<int>*>& interest_region_indices,
                                        cv::Vector<cv::Vector<float> >& results)
{
  // ----------------------------------------
  if (neighborhood_radius_defined_ == false)
  {
    ROS_ERROR("NeighborhoodFeature::compute radius not defined yet");
    return;
  }

  // ----------------------------------------
  // Compute the dimensions of the bounding box around the points
  size_t nbr_interest_regions = interest_region_indices.size();
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
    if (neighborhood_radius_ > 1e-6)
    {
      // Compute centroid of interest region
      geometry_msgs::Point32 region_centroid;
      cloud_geometry::nearest::computeCentroid(data, *curr_interest_region, region_centroid);

      vector<float> neighbor_distances; // unused
      // radiusSearch returning false (0 points) is handled by computeBoundingBoxFeatures
      data_kdtree.radiusSearch(region_centroid, neighborhood_radius_, neighbor_indices, neighbor_distances);

      // Now point to the neighboring points from radiusSearch
      curr_interest_region = &neighbor_indices;
    }

    computeNeighborhoodFeature(data, *curr_interest_region, i, results[i]);
  }
}
