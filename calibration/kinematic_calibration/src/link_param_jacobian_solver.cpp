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

#include "kinematic_calibration/link_param_jacobian_solver.h"

#include "kdl/frames.hpp"

using namespace kinematic_calibration ;
using namespace KDL ;

LinkParamJacobianSolver::LinkParamJacobianSolver()
{
  
}

LinkParamJacobianSolver::~LinkParamJacobianSolver()
{

}


/*
 * To compute the jacobian, we start at the base, and iterate up the chain. We assume that the link we are currently on
 * is the last link in the chain, so the tip of our current link is effectively the end-effector of our chain. Once we compute
 * the jacobian for this pseudo-end effector, we move to the next link, and transform the previously computed jacobian to be for
 * the new end-effector.
 */
int LinkParamJacobianSolver::JointsToCartesian(const Chain& chain, const JntArray& joint_states, LinkParamJacobian& jac)
{
  const unsigned int N = chain.getNrOfJoints() ;
  
  if ( joint_states.rows() != N )                                 // Sanity check on input data
    return -1 ;
  
  jac.twists_.resize(N) ;
  
  Frame T_start_to_eef ;                                          // Defines transform from the start of the chain to end-effector [of the part of the chain we've seen so far]
  Frame T_start_to_prev_eef = Frame::Identity() ;
  
  unsigned int joint_num = 0 ;                                    // Keep track of which joint we're currently on. Since some joints are fixed,
                                                                  //    the joint # could be different than the segment #

  for (unsigned int i=0; i<chain.getNrOfSegments(); i++)                            // Walk up the chain
  {
    const bool movable = chain.getSegment(i).getJoint().getType() != Joint::None ;  // Not sure what the right answer is for dealing with fixed joints
    const bool process_segment = movable ;                                          //    For now, compute Jacobians only for movable joints. 
    
    if(process_segment)
      T_start_to_eef = T_start_to_prev_eef * chain.getSegment(i).pose(joint_states(joint_num)) ;
    else
      T_start_to_eef = T_start_to_prev_eef * chain.getSegment(i).pose(0.0) ;
    
    Vector cur_segment_length = T_start_to_eef.p - T_start_to_prev_eef.p ;  // Defines vector-length of the current segment in the global frame
    jac.changeRefPoint(cur_segment_length) ;                                // Walk the previously computed jacobians up the chain
    
    if (process_segment)
    {
      // Build Twists for the translation elements x,y,z.  These translations occur in the base frame of the current segment
      Twist trans_twist[3] ;
      trans_twist[0].vel[0] = 1.0 ;
      trans_twist[0].vel[1] = 0.0 ;
      trans_twist[0].vel[2] = 0.0 ;
      trans_twist[1].vel[0] = 0.0 ;
      trans_twist[1].vel[1] = 1.0 ;
      trans_twist[1].vel[2] = 0.0 ;
      trans_twist[2].vel[0] = 0.0 ;
      trans_twist[2].vel[1] = 0.0 ;
      trans_twist[2].vel[2] = 1.0 ;
      
      // Build rotation from start to base frame of current link. This also includes the rotation from the current link's joint.
      Rotation R_start_to_cur_base = T_start_to_prev_eef.M * chain.getSegment(i).getJoint().pose(joint_states(joint_num)).M ;
      for (int j=0; j<3; j++)
      {
        trans_twist[j] = R_start_to_cur_base*trans_twist[j] ;                 // Rotate translations from the base frame of the current link to the start frame
        jac.twists_[joint_num].trans_[j] = trans_twist[j] ;                   // Copy data into the output datatype
      }
      
      // Build Twists for incremental rotations at the end of the segment.  These translations occur after the segment transform.
      Twist rot_twist[3] ;
      
      // First build twists in the [current] end-effector frame
      rot_twist[0].rot[0] = 1.0 ;
      rot_twist[0].rot[1] = 0.0 ;
      rot_twist[0].rot[2] = 0.0 ;
      rot_twist[1].rot[0] = 0.0 ;
      rot_twist[1].rot[1] = 1.0 ;
      rot_twist[1].rot[2] = 0.0 ;
      rot_twist[2].rot[0] = 0.0 ;
      rot_twist[2].rot[1] = 0.0 ;
      rot_twist[2].rot[2] = 1.0 ;
      
      // Now we need to transform these rotation twists to the base (beginning-of-segment) frame
      for (int j=0; j<3; j++)
      {
        rot_twist[j] = T_start_to_eef.M*rot_twist[j] ;
        jac.twists_[joint_num].rot_[j] = rot_twist[j] ;                                   // Copy data into the output datatype
      }
    }
    if (movable)
      joint_num++ ;
    
    T_start_to_prev_eef = T_start_to_eef ;
  }
  return 0 ;
}
