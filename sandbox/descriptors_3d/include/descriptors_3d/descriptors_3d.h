#ifndef __DESCRIPTORS_3D_H__
#define __DESCRIPTORS_3D_H__
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Daniel Munoz
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

#include <iostream>
#include <string>
#include <list>
#include <vector>

#include <ros/console.h>

#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/cvaux.hpp>

#include <robot_msgs/PointCloud.h>

#include <point_cloud_mapping/kdtree/kdtree.h>

using namespace std;

// --------------------------------------------------------------
//* Descriptor3D
/**
 * \brief An abstract class representing a descriptor that can
 *        compute feature values from 3-D data
 */
// --------------------------------------------------------------
class Descriptor3D
{
  public:
    Descriptor3D() :
      result_size_(0)
    {
    }

    virtual ~Descriptor3D()
    {
    }

    // --------------------------------------------------------------
    /**
     * \brief Computes feature values using the previously set data and
     *        interest point/region and any descriptor-specific parameters
     *        defined in the inherited class
     *
     * \param result Pointer to store compute feature values. This method will
     *               allocate memory and the caller is responsible for freeing it.
     *
     * \return true on success, otherwise false
     */
    // --------------------------------------------------------------

    virtual void compute(const robot_msgs::PointCloud& data,
                         cloud_kdtree::KdTree& data_kdtree,
                         const cv::Vector<robot_msgs::Point32*>& interest_pts,
                         cv::Vector<cv::Vector<float> >& results) = 0;

    virtual void compute(const robot_msgs::PointCloud& data, cloud_kdtree::KdTree& data_kdtree, cv::Vector<
        cv::Vector<float> >& results) = 0;

    virtual void compute(const robot_msgs::PointCloud& data,
                         cloud_kdtree::KdTree& data_kdtree,
                         const cv::Vector<vector<int>*>& interest_region_indices,
                         cv::Vector<cv::Vector<float> >& results) = 0;

    unsigned int getResultSize() const;

  protected:
    unsigned int result_size_;
};

#endif
