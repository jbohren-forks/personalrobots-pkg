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

#include <descriptors_3d/spectral_shape.h>

using namespace std;

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void SpectralShape::useCurvature()
{
  if (!use_curvature_)
  {
    result_size_++;
    use_curvature_ = true;
  }
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void SpectralShape::compute(const sensor_msgs::PointCloud& data,
                            cloud_kdtree::KdTree& data_kdtree,
                            const cv::Vector<const geometry_msgs::Point32*>& interest_pts,
                            cv::Vector<cv::Vector<float> >& results)
{
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
void SpectralShape::compute(const sensor_msgs::PointCloud& data,
                            cloud_kdtree::KdTree& data_kdtree,
                            const cv::Vector<const vector<int>*>& interest_region_indices,
                            cv::Vector<cv::Vector<float> >& results)
{
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
void SpectralShape::computeFeatures(cv::Vector<cv::Vector<float> >& results)
{
  // Assumes it has been correctly resized in compute() above
  unsigned int nbr_interest_pts = results.size();

  // Verify the retrieved spectral information contains the same number of interest points/regions
  const vector<Eigen::Vector3d*>& eigen_values = spectral_info_->getEigenValues();
  if (eigen_values.size() != nbr_interest_pts)
  {
    ROS_ERROR("SpectralShape::computeFeatures inconsistent spectral information");
    return;
  }

  // Iterate over each interest sample and compute its features
  for (size_t i = 0 ; i < nbr_interest_pts ; i++)
  {
    size_t feature_idx = 0;

    // NULL indicates couldnt compute spectral information for interest sample
    if (eigen_values[i] != NULL)
    {
      results[i].resize(result_size_);

      results[i][feature_idx++] = static_cast<float> ((*(eigen_values[i]))[0]); // scatter
      results[i][feature_idx++] = static_cast<float> ((*(eigen_values[i]))[2] - (*(eigen_values[i]))[1]); // linear
      results[i][feature_idx++] = static_cast<float> ((*(eigen_values[i]))[1] - (*(eigen_values[i]))[0]); // surface

      if (use_curvature_)
      {
        results[i][feature_idx++] = static_cast<float> ((*(eigen_values[i]))[0] / ((*(eigen_values[i]))[0]
            + (*(eigen_values[i]))[1] + (*(eigen_values[i]))[2]));
      }
    }
  }
}
