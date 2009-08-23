/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

//! \author Vijay Pradeep

#ifndef KINEMATIC_CALIBRATION_CHAIN_MODIFIER_H_
#define KINEMATIC_CALIBRATION_CHAIN_MODIFIER_H_

#include <vector>

#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "newmat10/newmat.h"

#include "kinematic_calibration/active_link_params.h"

namespace kinematic_calibration
{

/**
 * \brief Storing information about and performs incremental modification of a link.
 * A link modification consists of a rotation and translation of the link's tip. In our
 * framework, we assume that translational modifications occur in the base frame of the
 * link, whereas rotational modifcations occur in the tip frame of the link.
 */
class LinkModifier
{
public:

  /**
   * \brief Specifies how the link translation should be modified.
   * xyz defines a translation of the tip, in the root frame of the link.
   */
  void specifyTranslationParams(double x, double y, double z) ;

  /**
   * \brief Specifies how the link rotation should be modified.
   * xyz define an axis/angle rotation of the tip, in the frame of the tip.
   *  * The direction of [x y z] defines the rotation axis.
   *  * The magnitude of [x y z] defines the rotation angle, in radians
   */
  void specifyRotationParams(double x, double y, double z) ;

  /**
   * \brief Modifies a link given the specified modifications
   * \param segment The KDL segment that will be modified
   */
  void modifyLink(KDL::Segment& segment) const ;
private:
  KDL::Vector trans_ ;
  KDL::Rotation rot_ ;
} ;


/**
 * Modifies an existing chain, given an incremental modification.
 */
class ChainModifier
{
public:
  /**
   * \brief Defines the modification parameters of an entire chain, using a single newmat matrix
   * \param all_params Each column stores the modification parameters of a given link in a chain.
   *                     The first 3 rows specify translation, and the last 3 rows specify rotation
   * \return negative on error
   */
  int specifyAllParams(const NEWMAT::Matrix& all_params) ;

  /**
   * \brief Defines a sparse representation of the modification parameters, with the help of ActiveLinkParams
   * The active params vector is built into a newmat matrix using ActiveLinkParams as a lookup. The active table
   * is checked column by column, allowing us to populate the matrix accordingly.
   * \return negative on error
   */
  int specifyActiveParams(const std::vector<double>& active_params, const ActiveLinkParams& active) ;

  /**
   * \brief Grab the chain size that the ChainModifier is expecting
   */
  unsigned int getNumLinks() const
  {
    return link_modifiers_.size() ;
  }

  /**
   * \brief Modifies the specified chain, given a set of modification parameters
   * \param chain The chain to be modified
   * \return negative on error
   */
  int modifyChain(KDL::Chain& chain) const ;

  static int buildParamMatrix(NEWMAT::Matrix& mat, const std::vector<double>& params, const ActiveLinkParams& active) ;
private:


  std::vector<LinkModifier> link_modifiers_ ;

} ;

}

#endif // KINEMATIC_CALIBRATION_CHAIN_MODIFIER_H_
