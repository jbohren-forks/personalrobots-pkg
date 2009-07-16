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
                                                 set<unsigned int>& failed_indices);

    static unsigned int computeAndConcatFeatures(const robot_msgs::PointCloud& pt_cloud,
                                                 cloud_kdtree::KdTree& pt_cloud_kdtree,
                                                 const cv::Vector<vector<int>*>& interest_region_indices,
                                                 vector<Descriptor3D*>& descriptors_3d,
                                                 vector<float*>& concatenated_features,
                                                 set<unsigned int>& failed_indices);

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

  private:
    static unsigned int
    concatenateFeatures(const vector<cv::Vector<cv::Vector<float> > >& all_descriptor_results,
                        const unsigned int nbr_samples,
                        const unsigned int nbr_concatenated_vals,
                        vector<float*>& concatenated_features,
                        set<unsigned int>& failed_indices);
};

#endif
