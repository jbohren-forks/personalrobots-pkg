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

#include <string>

#include "kinematic_calibration/verify_jacobian.h"

using namespace kinematic_calibration ;
using namespace std ;

static const double epsilon = 1e-6 ;


int RunVerifier(const string& path, double& max_error)
{
  VerifyJacobian verifier ;  
  int result ;  
  result = verifier.ComputeMaxError(path+"/model.txt", path+"/joint_states.txt", path+"/jacobians.txt", max_error) ;  
  return result ;
}

TEST(KINEMATIC_CALIBRATION_JACOBIAN, 2d_easy)
{
  double max_error ;
  int result = RunVerifier("./test/data/2d_easy", max_error) ;
  EXPECT_EQ(0, result) ;
  EXPECT_NEAR(max_error, 0, epsilon) ;
}

TEST(KINEMATIC_CALIBRATION_JACOBIAN, 2d_hard)
{
  double max_error ;
  int result = RunVerifier("./test/data/2d_hard", max_error) ;
  EXPECT_EQ(0, result) ;
  EXPECT_NEAR(max_error, 0, epsilon) ;
}

TEST(KINEMATIC_CALIBRATION_JACOBIAN, 3d_hard)
{
  double max_error ;
  int result = RunVerifier("./test/data/3d_hard", max_error) ;
  EXPECT_EQ(0, result) ;
  EXPECT_NEAR(max_error, 0, epsilon) ;
}

TEST(KINEMATIC_CALIBRATION_JACOBIAN, edge_case_1)
{
  double max_error ;
  int result = RunVerifier("./test/data/edge_case_1", max_error) ;
  EXPECT_EQ(0, result) ;
  EXPECT_NEAR(max_error, 0, epsilon) ;
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
