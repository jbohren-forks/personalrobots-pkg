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

#include "kinematic_calibration/chain_modifier.h"

using namespace kinematic_calibration ;

void LinkModifier::specifyTranslationParams(double x, double y, double z)
{
  trans_[0] = x ;
  trans_[1] = y ;
  trans_[2] = z ;
}

void LinkModifier::specifyRotationParams(double x, double y, double z)
{
  KDL::Vector rot_axis(x,y,z) ;
  double rot_angle ;

  if (rot_axis.Norm() < 1e-6)           // Deal with the unstable case
  {
    rot_axis[0] = 0 ;
    rot_axis[1] = 0 ;
    rot_axis[2] = 1 ;
    rot_angle = 0.0 ;
  }
  else
  {
    rot_angle = rot_axis.Norm() ;
    rot_axis = rot_axis / rot_angle ;           // normalize the axis
  }

  rot_ = KDL::Rotation::Rot(rot_axis, rot_angle) ;
}

void LinkModifier::modifyLink(KDL::Segment& segment) const
{
  KDL::Frame f_tip = segment.getFrameToTip() ;
  f_tip.p = f_tip.p + trans_ ;
  f_tip.M = f_tip.M * rot_ ;
  segment = KDL::Segment(segment.getJoint(), f_tip, segment.getInertia(), segment.getCM() ) ;
}

int ChainModifier::specifyAllParams(const NEWMAT::Matrix& all_params)
{
  const int num_links = all_params.Ncols() ;
  link_modifiers_.resize(num_links) ;

  if (all_params.Nrows() != 6)
    return -1 ;

  for (int i=0; i<num_links; i++)
  {
    link_modifiers_[i].specifyTranslationParams(all_params(1,i+1), all_params(2,i+1), all_params(3,i+1)) ;
    link_modifiers_[i].specifyRotationParams(all_params(4,i+1), all_params(5,i+1), all_params(6,i+1)) ;
  }
  return 0 ;
}

int ChainModifier::specifyActiveParams(const std::vector<double>& params, const ActiveLinkParams& active)
{
  int result ;

  NEWMAT::Matrix mat ;

  result = buildParamMatrix(mat, params, active) ;
  if (result < 0)
    return result - 10 ;

  result = specifyAllParams(mat) ;

  if (result < 0)
    return result - 20 ;

  return result ;
}

int ChainModifier::modifyChain(KDL::Chain& chain) const
{
  if (chain.getNrOfSegments() != link_modifiers_.size())
  {
    return -1 ;
  }

  KDL::Chain next_chain ;
  KDL::Segment cur_segment ;

  for (unsigned int i=0; i<chain.getNrOfSegments(); i++)
  {
    cur_segment = chain.getSegment(i) ;
    link_modifiers_[i].modifyLink(cur_segment) ;
    next_chain.addSegment(cur_segment) ;
  }
  chain = next_chain ;

  return 0 ;
}

int ChainModifier::buildParamMatrix(NEWMAT::Matrix& mat, const std::vector<double>& params, const ActiveLinkParams& active)
{
  const unsigned int num_active = active.getNumActive() ;
  if (num_active != params.size())      // # of Params must be the same as the number of active link params
    return -1 ;

  mat.ReSize(6,active.getNumLinks()) ;

  mat = 0 ;     // Set all elems to zero

  int cur_index = 0 ;
  for (unsigned int i=0; i<active.getNumLinks(); i++)
  {
    for (unsigned int j=0; j<6; j++)
    {
      if(active(j,i))
      {
        mat(j+1,i+1) = params[cur_index] ;
        cur_index++ ;
      }
      else
        mat(j+1,i+1) = 0.0 ;
    }
  }
  return 0 ;
}
