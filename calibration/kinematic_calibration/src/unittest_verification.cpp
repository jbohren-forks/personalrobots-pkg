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

#include "kinematic_calibration/unittest_verification.h"

using namespace kinematic_calibration ;

int UnitTestVerification::ComputeChainError(const KDL::Chain& chain1, const KDL::Chain& chain2, double& max_error)
{
  max_error = 0.00 ;

  if (chain1.getNrOfSegments() != chain2.getNrOfSegments())
    return -1 ;

  for (unsigned int i=0; i<chain1.getNrOfSegments(); i++)
  {
    int result ;
    double cur_seg_error ;

    result = ComputeFrameError(chain1.getSegment(i).getFrameToTip(),  chain2.getSegment(i).getFrameToTip(), cur_seg_error) ;

    if (result < 0)
      return result ;

    if (cur_seg_error > max_error)
      max_error = cur_seg_error ;
  }
  return 0 ;
}

int UnitTestVerification::ComputeFrameError(const KDL::Frame& frame1, const KDL::Frame& frame2, double& error)
{
  double frame_error = 0.0 ;
  for (int i=0; i<3; i++)
  {
    frame_error += fabs( frame1.p.data[i] - frame2.p.data[i] ) ;
  }

  for (int i=0; i<9; i++)
  {
    frame_error += fabs( frame1.M.data[i] - frame2.M.data[i] ) ;
  }

  error = frame_error ;

  return 0 ;
}
