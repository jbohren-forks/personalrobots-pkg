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

#include <descriptors_3d/position.h>

using namespace std;

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void Position::compute(const sensor_msgs::PointCloud& data,
                       cloud_kdtree::KdTree& data_kdtree,
                       const cv::Vector<const geometry_msgs::Point32*>& interest_pts,
                       cv::Vector<cv::Vector<float> >& results)
{
  size_t nbr_interest_pts = interest_pts.size();
  results.resize(nbr_interest_pts);
  for (size_t i = 0 ; i < nbr_interest_pts ; i++)
  {
    results[i].resize(1);
    results[i][0] = (interest_pts[i])->z;
  }
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void Position::compute(const sensor_msgs::PointCloud& data,
                       cloud_kdtree::KdTree& data_kdtree,
                       const cv::Vector<const vector<int>*>& interest_region_indices,
                       cv::Vector<cv::Vector<float> >& results)
{
  size_t nbr_interest_regions = interest_region_indices.size();
  results.resize(nbr_interest_regions);

  geometry_msgs::Point32 region_centroid;
  for (size_t i = 0 ; i < nbr_interest_regions ; i++)
  {
    cloud_geometry::nearest::computeCentroid(data, *(interest_region_indices[i]), region_centroid);
    results[i].resize(1);
    results[i][0] = region_centroid.z;
  }
}
