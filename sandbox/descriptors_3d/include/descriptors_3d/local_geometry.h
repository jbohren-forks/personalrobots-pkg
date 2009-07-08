#ifndef __LOCAL_GEOMETRY_H__
#define __LOCAL_GEOMETRY_H__
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

#include <Eigen/Core>

#include <point_cloud_mapping/geometry/nearest.h>

#include <descriptors_3d/descriptors_3d.h>

// --------------------------------------------------------------
//* LocalGeometry
/**
 * \brief Implements 3-D descriptors that describe local point cloud geometry
 *
 *
 */
// --------------------------------------------------------------
class LocalGeometry: public Descriptor3D
{
  public:
    LocalGeometry() :
      radius_(-1.0), ref_tangent_defined_(false), ref_normal_defined_(false), use_elevation_(false),
          use_curvature_(false)
    {
      result_size_ = 3;
    }

    virtual bool compute(Eigen::MatrixXf** result, bool debug);

    // ----------------------------------------------------------

    int setInterestRadius(double radius)
    {
      if (radius > 0.0)
      {
        radius_ = radius;
        return 0;
      }
      return -1;
    }

    // ===================================================================
    /*! \name Spectral analysis related */
    // ===================================================================
    //@{
    void useSpectral()
    {
      //
    }

    void useTangentOrientation(double ref_x, double ref_y, double ref_z)
    {
      if (ref_tangent_defined_ == false)
      {
        ref_tangent_defined_ = true;
        result_size_++;
      }

      ref_tangent_[0] = ref_x;
      ref_tangent_[1] = ref_y;
      ref_tangent_[2] = ref_z;
      ref_tangent_flipped_[0] = -ref_x;
      ref_tangent_flipped_[1] = -ref_y;
      ref_tangent_flipped_[2] = -ref_z;
    }

    void useNormalOrientation(double ref_x, double ref_y, double ref_z)
    {
      if (ref_normal_defined_ == false)
      {
        ref_normal_defined_ = true;
        result_size_++;
      }

      ref_normal_[0] = ref_x;
      ref_normal_[1] = ref_y;
      ref_normal_[2] = ref_z;
      ref_normal_flipped_[0] = -ref_x;
      ref_normal_flipped_[1] = -ref_y;
      ref_normal_flipped_[2] = -ref_z;
      ref_normal_defined_ = true;
    }

    void useCurvature()
    {
      // TODO
    }

    void usePCABoundingBox()
    {
      // TODO
    }

    void useSpinImageNormal()
    {
      // TODO
    }

    void useSpinImageTangent()
    {
      // TODO
    }
    //@}

    void useSpinImage(double ref_x, double ref_y, double ref_z)
    {
      // TODO
    }

    void useElevation()
    {
      if (use_elevation_ == false)
      {
        use_elevation_ = true;
        result_size_++;
      }
    }

    void useRawBoundingBox()
    {
      // TODO
    }

  protected:
    virtual bool readyToCompute();

  private:
    void computeSpinImage(Eigen::Vector3d& axis);

    double radius_;

    bool ref_tangent_defined_;
    Eigen::Vector3d ref_tangent_;
    Eigen::Vector3d ref_tangent_flipped_;

    bool ref_normal_defined_;
    Eigen::Vector3d ref_normal_;
    Eigen::Vector3d ref_normal_flipped_;

    bool use_elevation_;
    bool use_curvature_;
};

#endif
