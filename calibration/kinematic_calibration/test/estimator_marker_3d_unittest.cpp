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

#include "kinematic_calibration/unittest_io.h"
#include "kinematic_calibration/unittest_verification.h"
#include "kinematic_calibration/parameter_estimator.h"
#include "kdl/chain.hpp"

using namespace std ;
using namespace kinematic_calibration ;

static const double epsilon = 1e-6 ;

void RunTestMarker3d(string path, unsigned int num_links)
{
  int result ;

  JointStatesGetter js_getter ;
  result = js_getter.openFile(path + "/joint_states.txt") ;
  EXPECT_EQ(result, 0) ;

  KDLVectorGetter eef_getter ;
  result = eef_getter.openFile(path + "/eef_sensed.txt") ;
  EXPECT_EQ(result, 0) ;

  ActiveLinkParams active ;
  result = active.loadFromFile(path+"/active_params.txt", num_links) ;
  EXPECT_EQ(result, 0) ;

  ModelGetter model_getter ;
  KDL::Chain chain_cad, chain_actual ;
  result = model_getter.openFile(path + "/model_cad.txt") ;
  EXPECT_EQ(result, 0) ;
  chain_cad = model_getter.getModel() ;
  EXPECT_EQ(chain_cad.getNrOfSegments(), num_links) ;
  model_getter.closeFile() ;

  result = model_getter.openFile(path + "/model_actual.txt") ;
  EXPECT_EQ(result, 0) ;
  chain_actual = model_getter.getModel() ;
  EXPECT_EQ(chain_actual.getNrOfSegments(), num_links) ;
  model_getter.closeFile() ;

  ParameterEstimator::MarkerData3d marker(num_links) ;
  vector<ParameterEstimator::MarkerData3d> input_data ;

  // Build the MarkerData3d vector
  while(true)
  {
    int result_js =  js_getter.getNextJointArray(marker.joint_states) ;
    int result_eef = eef_getter.getNextVec(marker.marker_sensed) ;
    if (result_js < 0)
    {
      EXPECT_TRUE(result_eef < 0) ;
      break ;
    }
    else
      EXPECT_EQ(result_eef, 0) ;
    input_data.push_back(marker) ;
  }

  // Run the parameter estimator
  KDL::Chain chain_opt(chain_cad) ;
  ParameterEstimator est ;
  result = est.estimateParametersMarker3d(chain_cad, chain_opt, input_data, active) ;
  EXPECT_EQ(result, 0) ;
  //printf("Actual Chain:\n") ;
  KDL::Frame cur_frame = chain_actual.getSegment(0).getFrameToTip() ;
  //printf(" Translation:\n") ;
  //printf(" [ %15.12f %15.12f %15.12f ]\n", cur_frame.p.data[0], cur_frame.p.data[1], cur_frame.p.data[2]) ;
  //printf(" Rotation:\n") ;
  //printf(" [ %15.12f %15.12f %15.12f \n", cur_frame.M.data[0], cur_frame.M.data[1], cur_frame.M.data[2]) ;
  //printf("   %15.12f %15.12f %15.12f \n", cur_frame.M.data[3], cur_frame.M.data[4], cur_frame.M.data[5]) ;
  //printf("   %15.12f %15.12f %15.12f]\n", cur_frame.M.data[6], cur_frame.M.data[7], cur_frame.M.data[8]) ;

  double error = 5;
  result = UnitTestVerification::ComputeChainError(chain_actual, chain_opt, error) ;
  EXPECT_EQ(result, 0) ;
  EXPECT_NEAR(error, 0.0, epsilon)  ;
}

TEST(KINEMATIC_CALIBRATION_estimator, marker_3d_easy1_trans)
{
  RunTestMarker3d(string("./test/data/estimator_marker_3d_unittest/easy1_trans"),1) ;
}

TEST(KINEMATIC_CALIBRATION_estimator, marker_3d_easy2_rot)
{
  RunTestMarker3d(string("./test/data/estimator_marker_3d_unittest/easy2_rot"),2) ;
}

TEST(KINEMATIC_CALIBRATION_estimator, marker_3d_med4_all)
{
  RunTestMarker3d(string("./test/data/estimator_marker_3d_unittest/med4_all"),4) ;
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
