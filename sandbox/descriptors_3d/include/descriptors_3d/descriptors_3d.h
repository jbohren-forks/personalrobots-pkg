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

#include <Eigen/Core>

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
      result_size_(0), data_set_(false), data_(NULL), data_kdtree_(NULL), interest_pt_set_(false),
          interest_pt_idx_(0), interest_region_set_(false), interest_region_indices_(NULL)
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
    virtual bool compute(Eigen::MatrixXf** result) const = 0;

    // --------------------------------------------------------------
    /**
     * \brief Sets the data this descriptor will operate on
     *
     * \param data Data structure of 3-d data
     * \param data_kdtree Kd-tree version of data for efficient neighbor lookup
     *
     * \return true on success, otherwise false
     */
    // --------------------------------------------------------------
    bool setData(const robot_msgs::PointCloud* data, cloud_kdtree::KdTree* data_kdtree);

    void setInterestPoint(unsigned int interest_pt_idx);

    // may or may not be used by descriptor
    void setInterestRegion(const vector<int>* interest_region_indices);

    unsigned int getResultSize() const;

  protected:
    // will check whether can use either interest pt or region
    virtual bool readyToCompute() const = 0;

    unsigned int result_size_;

    bool data_set_;
    const robot_msgs::PointCloud* data_;
    cloud_kdtree::KdTree* data_kdtree_;

    bool interest_pt_set_;
    unsigned int interest_pt_idx_;

    bool interest_region_set_;
    const vector<int>* interest_region_indices_;
};

#endif
