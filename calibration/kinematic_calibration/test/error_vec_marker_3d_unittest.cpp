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

#include <gtest/gtest.h>


#include "kinematic_calibration/parameter_estimator.h"
#include "kdl/chain.hpp"

using namespace std ;
using namespace kinematic_calibration ;

static const double epsilon = 1e-6 ;

/**
 * Builds a really basic chain, moves it into a few simple orientations,
 * and then makes sure the output error vector is correct. At the moment
 * this is definitely not a rigorous test. It is simply a sanity check.
 */
TEST(KINEMATIC_CALIBRATION_error_vec, marker_3d_easy1)
{
  KDL::Chain chain ;
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                                KDL::Frame(KDL::Rotation(), KDL::Vector(1,0,0) )
                               ) ) ;

  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                                KDL::Frame(KDL::Rotation(), KDL::Vector(1,0,0) )
                               ) ) ;

  KDL::JntArray jnt_array(2) ;
  std::vector<ParameterEstimator::MarkerData3d> input_data ;

  ParameterEstimator::MarkerData3d marker(2) ;

  // ***** Input Data 0 *****
  marker.joint_states(0) = 0.0 ;
  marker.joint_states(1) = 0.0 ;
  marker.marker_sensed = KDL::Vector(2.01, 0.02, 0.03) ;
  input_data.push_back(marker) ;

  // ***** Input Data 1 *****
  marker.joint_states(0) = M_PI/2 ;
  marker.joint_states(1) = 0.0 ;
  marker.marker_sensed = KDL::Vector(0.11, 2.12, 0.13) ;
  input_data.push_back(marker) ;

  // ***** Input Data 2 *****
  marker.joint_states(0) = M_PI/2 ;
  marker.joint_states(1) = M_PI/2 ;
  marker.marker_sensed = KDL::Vector(-0.79, 1.22, 0.23) ;
  input_data.push_back(marker) ;

  ParameterEstimator est ;
  NEWMAT::ColumnVector vec ;
  int result ;
  result = est.buildErrorVecMarker3d(chain, input_data, vec, JacNewmatBridge::JacTerms::TRANS) ;

  EXPECT_EQ(result, 0) ;
  EXPECT_NEAR(vec(1), .01, epsilon) ;
  EXPECT_NEAR(vec(2), .02, epsilon) ;
  EXPECT_NEAR(vec(3), .03, epsilon) ;
  EXPECT_NEAR(vec(4), .11, epsilon) ;
  EXPECT_NEAR(vec(5), .12, epsilon) ;
  EXPECT_NEAR(vec(6), .13, epsilon) ;
  EXPECT_NEAR(vec(7), .21, epsilon) ;
  EXPECT_NEAR(vec(8), .22, epsilon) ;
  EXPECT_NEAR(vec(9), .23, epsilon) ;
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
