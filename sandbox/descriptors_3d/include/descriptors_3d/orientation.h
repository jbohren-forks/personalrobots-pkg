#ifndef __D3D_ORIENTATION_H__
#define __D3D_ORIENTATION_H__
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

#include <Eigen/Core>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.hpp>

#include <descriptors_3d/spectral_analysis.h>

using namespace std;

// --------------------------------------------------------------
//* Orientation
/*!
 * \brief An Orientation feature looks at the angle of an interest
 *        point/region's neighborhood principle directions with respect
 *        to a given reference direction.
 *
 * The principle directions are the tangent (biggest eigenvector)
 * and normal (smallest eigenvector) vectors from the extracted spectral
 * information.
 */
// --------------------------------------------------------------
class Orientation: public SpectralAnalysis
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief Instantiates the Orientation descriptor
     *
     * The computed feature is the cosine of the angle between the extracted
     * tangent/normal vector against the specified reference directions.
     *
     * \warning Since the sign of the extracted vectors have no meaning, the
     *          computed feature is always between 0 and 1
     *
     * If computing using both tangent and normal vectors, the feature vector
     * format is: [a b] where a is the projection of the tangent vector and b
     * is the projection of the normal vector
     */
    // --------------------------------------------------------------
    Orientation() :
      ref_tangent_defined_(false), ref_normal_defined_(false)
    {
      result_size_ = 0;
    }

    // ===================================================================
    /*! \name Optional settings  */
    // ===================================================================
    //@{
    // --------------------------------------------------------------
    /**
     * \brief Sets this descriptor to compare extracted tangent vectors against
     *        the specified reference direction
     *
     * The reference direction does not need to be of unit length
     *
     * \param ref_x The x coordinate of the reference vector
     * \param ref_y The y coordinate of the reference vector
     * \param ref_z The z coordinate of the reference vector
     */
    // --------------------------------------------------------------
    void useTangentOrientation(double ref_x, double ref_y, double ref_z);

    // --------------------------------------------------------------
    /**
     * \brief Sets this descriptor to compare extracted normal vectors against
     *        the specified reference direction
     *
     * The reference direction does not need to be of unit length
     *
     * \param ref_x The x coordinate of the reference vector
     * \param ref_y The y coordinate of the reference vector
     * \param ref_z The z coordinate of the reference vector
     */
    // --------------------------------------------------------------
    void useNormalOrientation(double ref_x, double ref_y, double ref_z);

    // TODO use sensor location so the extracted directions have signs
    //void useSensorLocation();
    //@}

    // --------------------------------------------------------------
    /*!
     * \brief Computes the scalar projection of the extracted direction(s) around
     *        each interest point against the reference direction(s).
     *
     * \warning useTangentOrientation() and/or useNormalOrientation() must be called first
     * \warning setSpectralRadius() or useSpectralInformation() must be called first
     *
     * \see Descriptor3D::compute
     */
    // --------------------------------------------------------------
    virtual void compute(const robot_msgs::PointCloud& data,
                         cloud_kdtree::KdTree& data_kdtree,
                         const cv::Vector<robot_msgs::Point32*>& interest_pts,
                         cv::Vector<cv::Vector<float> >& results);

    // --------------------------------------------------------------
    /*!
     * \brief Computes the scalar projection of the extracted direction(s) of
     *        each interest region against the reference direction(s).
     *
     * \warning useTangentOrientation() and/or useNormalOrientation() must be called first
     * \warning setSpectralRadius() or useSpectralInformation() must be called first
     *
     * \see Descriptor3D::compute
     */
    // --------------------------------------------------------------
    virtual void compute(const robot_msgs::PointCloud& data,
                         cloud_kdtree::KdTree& data_kdtree,
                         const cv::Vector<vector<int>*>& interest_region_indices,
                         cv::Vector<cv::Vector<float> >& results);

  protected:
    // --------------------------------------------------------------
    /*!
     * \brief Computes the orientation features
     *
     * \param results Container to hold computed result for each interest point/region
     */
    // --------------------------------------------------------------
    void computeFeatures(cv::Vector<cv::Vector<float> >& results);

  private:
    /*! \brief Flag if useTangentOrientation() has been called */
    bool ref_tangent_defined_;
    /*! \brief Reference vector to compare extracted tangent vectors */
    Eigen::Vector3d ref_tangent_;
    /*! \brief The negative of ref_tangent_ */
    Eigen::Vector3d ref_tangent_flipped_;

    /*! \brief Flag if useNormalOrientation() has been called */
    bool ref_normal_defined_;
    /*! \brief Reference vector to compare extracted normal vectors */
    Eigen::Vector3d ref_normal_;
    /*! \brief The negative of ref_normal_ */
    Eigen::Vector3d ref_normal_flipped_;
};

#endif
