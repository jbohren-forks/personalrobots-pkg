#ifndef __D3D_DESCRIPTORS_3D_H__
#define __D3D_DESCRIPTORS_3D_H__
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

#include <vector>
#include <set>

#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/cvaux.hpp>

#include <ros/console.h>

#include <robot_msgs/PointCloud.h>

#include <point_cloud_mapping/kdtree/kdtree.h>

using namespace std;

// --------------------------------------------------------------
//* Descriptor3D
/*!
 * \brief An abstract class representing a descriptor that can
 *        compute feature values from 3-D data
 */
// --------------------------------------------------------------
class Descriptor3D
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief Instantiates a descriptor with 0 feature values
     */
    // --------------------------------------------------------------
    Descriptor3D() :
      result_size_(0)
    {
    }

    virtual ~Descriptor3D()
    {
    }

    static unsigned int computeAndConcatFeatures(const robot_msgs::PointCloud& pt_cloud,
                                                 cloud_kdtree::KdTree& pt_cloud_kdtree,
                                                 const cv::Vector<robot_msgs::Point32*>& interest_pts,
                                                 vector<Descriptor3D*>& descriptors_3d,
                                                 vector<float*>& concatenated_features,
                                                 set<unsigned int>& failed_indices)

    {
      failed_indices.clear();

      // Allocate results for each INTEREST point
      unsigned int nbr_interest_pts = interest_pts.size();
      concatenated_features.assign(nbr_interest_pts, NULL);

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

        descriptors_3d[i]->compute(pt_cloud, pt_cloud_kdtree, interest_pts, all_descriptor_results[i]);
        nbr_concatenated_vals += descriptors_3d[i]->getResultSize();
        ROS_INFO("Descriptor will have this many features: %u", descriptors_3d[i]->getResultSize());
      }

      // ----------------------------------------------
      // Iterate over each interest point and compute all feature descriptors
      // If all descriptor computations are successful, then concatenate all values into 1 array
      float* curr_concat_feats = NULL; // concatenated features from all descriptors
      unsigned int prev_val_idx = 0; // offset when copying into concat_features
      unsigned int curr_nbr_feature_vals = 0; // length of current descriptor
      bool all_features_success = true; // flag if all descriptors were computed correctly
      for (unsigned int i = 0 ; i < nbr_interest_pts ; i++)
      {
        // --------------------------------
        // Verify all features for the point were computed successfully
        all_features_success = true;
        for (unsigned int j = 0 ; all_features_success && j < nbr_descriptors ; j++)
        {
          cv::Vector<cv::Vector<float> >& curr_descriptor_for_cloud = all_descriptor_results[j];
          cv::Vector<float>& curr_feature_vals = curr_descriptor_for_cloud[(size_t) i];

          // non-zero descriptor length indicates computed successfully
          all_features_success = curr_feature_vals.size() != 0;
        }

        // --------------------------------
        // If all successful, then concatenate feature values
        if (all_features_success)
        {
          // --------------------------------
          // Concatenate all descriptors into one feature vector
          curr_concat_feats = static_cast<float*> (malloc(sizeof(float) * nbr_concatenated_vals));
          prev_val_idx = 0;
          for (unsigned int j = 0 ; j < nbr_descriptors ; j++)
          {
            // retrieve descriptor values for current point
            cv::Vector<cv::Vector<float> >& curr_descriptor_for_cloud = all_descriptor_results[j];
            cv::Vector<float>& curr_feature_vals = curr_descriptor_for_cloud[(size_t) i];
            curr_nbr_feature_vals = curr_feature_vals.size();

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

      return nbr_concatenated_vals;
    }

    // --------------------------------------------------------------
    /*!
     * \brief Computes feature values for each specified interest point
     *
     * See the inherited class for the type of features computed
     *
     * \param data Point cloud of the data
     * \param data_kdtree K-D tree representation of data
     * \param interest_pts List of points to compute features for
     * \param results Vector to hold computed vector of features for each interest point.
     *                If the features could not be computed for an interest point i, then
     *                results[i].size() = 0
     */
    // --------------------------------------------------------------
    virtual void compute(const robot_msgs::PointCloud& data,
                         cloud_kdtree::KdTree& data_kdtree,
                         const cv::Vector<robot_msgs::Point32*>& interest_pts,
                         cv::Vector<cv::Vector<float> >& results) = 0;

    // --------------------------------------------------------------
    /*!
     * \brief Computes feature values for each interest region of points
     *
     * See the inherited class for the type of features computed
     *
     * \param data Point cloud of the data
     * \param data_kdtree K-D tree representation of data
     * \param interest_region_indices List of groups of indices into data that represent an interest region
     * \param results Vector to hold computed vector of features for each interest point.
     *                If the features could not be computed for an interest point i, then
     *                results[i].size() = 0
     */
    // --------------------------------------------------------------
    virtual void compute(const robot_msgs::PointCloud& data,
                         cloud_kdtree::KdTree& data_kdtree,
                         const cv::Vector<vector<int>*>& interest_region_indices,
                         cv::Vector<cv::Vector<float> >& results) = 0;

    // --------------------------------------------------------------
    /*!
     * \brief Returns the number of feature values this descriptor computes on success
     *
     * \return the number of feature values this descriptor computes on success
     */
    // --------------------------------------------------------------
    inline unsigned int getResultSize() const
    {
      return result_size_;
    }

  protected:
    unsigned int result_size_;
};

#endif
