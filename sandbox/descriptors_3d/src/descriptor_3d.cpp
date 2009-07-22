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

#include <descriptors_3d/descriptor_3d.h>

using namespace std;

Descriptor3D::~Descriptor3D()
{
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void Descriptor3D::concatenateFeatures(const vector<cv::Vector<cv::Vector<float> > >& all_descriptor_results,
                                       const unsigned int nbr_samples,
                                       const unsigned int nbr_concatenated_vals,
                                       vector<float*>& concatenated_features,
                                       set<unsigned int>& failed_indices)
{
  failed_indices.clear();
  concatenated_features.assign(nbr_samples, NULL);
  unsigned int nbr_descriptors = all_descriptor_results.size();

  // ----------------------------------------------
  // Iterate over each interest point and compute all feature descriptors
  // If all descriptor computations are successful, then concatenate all values into 1 array
  for (size_t i = 0 ; i < nbr_samples ; i++)
  {
    // --------------------------------
    // Verify all features for the point were computed successfully
    bool all_features_success = true; // flag if all descriptors were computed correctly
    for (unsigned int j = 0 ; all_features_success && j < nbr_descriptors ; j++)
    {
      // Get the computed features vector for all interest samples
      const cv::Vector<cv::Vector<float> >& curr_descriptor_for_cloud = all_descriptor_results[j];

      // Get the computed feature for current sample
      const cv::Vector<float>& curr_feature_vals = curr_descriptor_for_cloud[i];

      // non-zero descriptor length indicates computed successfully
      unsigned int curr_nbr_feature_vals = curr_feature_vals.size();
      all_features_success = curr_nbr_feature_vals != 0;
    }

    // --------------------------------
    // If all successful, then concatenate feature values into one vector
    if (all_features_success)
    {
      // allocate
      float* curr_concat_feats = static_cast<float*> (malloc(sizeof(float) * nbr_concatenated_vals));

      unsigned int prev_val_idx = 0; // offset when copying into concat_features
      for (unsigned int j = 0 ; j < nbr_descriptors ; j++)
      {
        // retrieve descriptor values for current point
        const cv::Vector<cv::Vector<float> >& curr_descriptor_for_cloud = all_descriptor_results[j];
        const cv::Vector<float>& curr_feature_vals = curr_descriptor_for_cloud[i];
        unsigned int curr_nbr_feature_vals = curr_feature_vals.size();

        // copy descriptor values into concatenated vector at correct location
        memcpy(curr_concat_feats + prev_val_idx, curr_feature_vals.begin(), sizeof(float)
            * curr_nbr_feature_vals);

        // skip over all computed features so far
        prev_val_idx += curr_nbr_feature_vals;
      }

      // Save it
      concatenated_features[i] = curr_concat_feats;
    }
    // Otherwise features not successful, so note them
    else
    {
      ROS_WARN("skipping concatenation of sample %u", i);
      failed_indices.insert(i);
    }
  }

  return;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
unsigned int Descriptor3D::computeAndConcatFeatures(const robot_msgs::PointCloud& data,
                                                    cloud_kdtree::KdTree& data_kdtree,
                                                    const cv::Vector<robot_msgs::Point32*>& interest_pts,
                                                    vector<Descriptor3D*>& descriptors_3d,
                                                    vector<float*>& concatenated_features,
                                                    set<unsigned int>& failed_indices)

{
  // Allocate feature computation for each descriptor
  unsigned int nbr_descriptors = descriptors_3d.size();
  vector<cv::Vector<cv::Vector<float> > > all_descriptor_results(nbr_descriptors);

  // ----------------------------------------------
  // Iterate over each descriptor and compute features for each point in the point cloud
  unsigned int nbr_concatenated_vals = 0;
  for (unsigned int i = 0 ; i < nbr_descriptors ; i++)
  {
    if (descriptors_3d[i] == NULL)
    {
      return 0;
    }

    descriptors_3d[i]->compute(data, data_kdtree, interest_pts, all_descriptor_results[i]);
    nbr_concatenated_vals += (descriptors_3d[i])->getResultSize();
    ROS_INFO("Descriptor will have this many features: %u", descriptors_3d[i]->getResultSize());
  }

  concatenateFeatures(all_descriptor_results, interest_pts.size(), nbr_concatenated_vals,
      concatenated_features, failed_indices);
  return nbr_concatenated_vals;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
unsigned int Descriptor3D::computeAndConcatFeatures(const robot_msgs::PointCloud& data,
                                                    cloud_kdtree::KdTree& data_kdtree,
                                                    const cv::Vector<vector<int>*>& interest_region_indices,
                                                    vector<Descriptor3D*>& descriptors_3d,
                                                    vector<float*>& concatenated_features,
                                                    set<unsigned int>& failed_indices)

{
  // Allocate feature computation for each descriptor
  unsigned int nbr_descriptors = descriptors_3d.size();
  vector<cv::Vector<cv::Vector<float> > > all_descriptor_results(nbr_descriptors);

  // ----------------------------------------------
  // Iterate over each descriptor and compute features for each point in the point cloud
  unsigned int nbr_concatenated_vals = 0;
  for (unsigned int i = 0 ; i < nbr_descriptors ; i++)
  {
    if (descriptors_3d[i] == NULL)
    {
      return 0;
    }

    descriptors_3d[i]->compute(data, data_kdtree, interest_region_indices, all_descriptor_results[i]);
    nbr_concatenated_vals += (descriptors_3d[i])->getResultSize();
    ROS_INFO("Descriptor will have this many features: %u", descriptors_3d[i]->getResultSize());
  }

  concatenateFeatures(all_descriptor_results, interest_region_indices.size(), nbr_concatenated_vals,
      concatenated_features, failed_indices);
  return nbr_concatenated_vals;
}

