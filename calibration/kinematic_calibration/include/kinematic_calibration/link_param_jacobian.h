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

#ifndef KINEMATIC_CALIBRATION_LINK_PARAM_JACOBIAN_H_
#define KINEMATIC_CALIBRATION_LINK_PARAM_JACOBIAN_H_

#include <vector>

#include "kdl/frames.hpp"

namespace kinematic_calibration
{

struct LinkTwists
{
  KDL::Twist trans_[3] ;
  KDL::Twist rot_[3] ;
} ;

class LinkParamJacobian
{
public:
  LinkParamJacobian() ;
  ~LinkParamJacobian() ;

  /**
   * Propagate jacobian to a point specified in the current frame's coordinates
   * \param cur_to_next vector specifying the point to propagate the jacobian to. Defined wrt the current coordinate frame.
   */
  void changeRefPoint(const KDL::Vector& cur_to_next) ;

  /**
   * Stores all of the individual twists for the system parameters.  Each element of links_ stores
   * the twists specific to the parameters of the link in the system with the same index.
   * ie. links_[0] corresponds to segment 0 in the associated chain.
   */
  std::vector<LinkTwists> links_ ;
};

}

#endif /* KINEMATIC_CALIBRATION_LINK_PARAM_JACOBIAN_H_ */
