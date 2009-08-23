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

#ifndef KINEMATIC_CALIBRATION_LINK_PARAM_JACOBIAN_SOLVER_H_
#define KINEMATIC_CALIBRATION_LINK_PARAM_JACOBIAN_SOLVER_H_


#include "kinematic_calibration/link_param_jacobian.h"
#include "kdl/chain.hpp"
#include "kdl/jntarray.hpp"

namespace kinematic_calibration
{

/**
 * Computes the end effector jacobian wrt all the link parameters along the kinematic chain. The eef motion is calculated for 6
 * differential perturbations: (dx, dy, dz, rx, ry, rz).
 * dx, dy, dz are the x,y & z translational displacements (respectively) of the end effector in the link's base frame
 * rx, ry, rz are the x y z rotations (respectively) of the end effector in the frame of the current link's tip.
 */
class LinkParamJacobianSolver
{
public:
  LinkParamJacobianSolver() ;

  ~LinkParamJacobianSolver() ;

  /**
   * Function that computes the link-parameter jacobian
   * \param chain The KDL datatype specifying the kinematic chain
   * \param joint_states stores the joint angles/displacements for the current configuration that we want to evaluate
   * \param jac (output) Stores the computed jacobian
   * \return negative on error
   */
  static int JointsToCartesian(const KDL::Chain& chain, const KDL::JntArray& joint_states, LinkParamJacobian& jac) ;
  
} ;

}

#endif /* KINEMATIC_CALIBRATION_LINK_PARAM_JACOBIAN_SOLVER_H_ */
