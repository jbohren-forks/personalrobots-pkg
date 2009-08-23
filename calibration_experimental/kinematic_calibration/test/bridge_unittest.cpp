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

#include "kinematic_calibration/jac_newmat_bridge.h"
#include "kinematic_calibration/active_link_params.h"

using namespace kinematic_calibration ;
using namespace std ;

static const double epsilon = 1e-6 ;

TEST(KINEMATIC_CALIBRATION_BRIDGE, active_params_1)
{
  ActiveLinkParams active ;
  active.setNumLinks(1) ;

  active.setAllInactive() ;
  active(1,0) = true ;
  active(2,0) = true ;
  active(3,0) = true ;
  active(4,0) = true ;
  active(5,0) = true ;

  EXPECT_FALSE(active(0,0)) ;
  EXPECT_TRUE(active(1,0)) ;
  EXPECT_TRUE(active(2,0)) ;
  EXPECT_TRUE(active(3,0)) ;
  EXPECT_TRUE(active(4,0)) ;
  EXPECT_TRUE(active(5,0)) ;

  EXPECT_EQ(active.getNumActive(), (unsigned int)5) ;
}


TEST(KINEMATIC_CALIBRATION_BRIDGE, one_link_easy_all)
{
  LinkParamJacobian jac ;
  jac.links_.resize(1) ;
  jac.links_[0].trans_[0].vel.data[0] = 1.23 ;

  ActiveLinkParams active ;
  active.setNumLinks(1) ;

  active.setAllInactive() ;
  active(0,0) = true ;

  NEWMAT::Matrix mat(6,1) ;

  int result ;

  result = JacNewmatBridge::jacToNewmat(jac, active, mat, JacNewmatBridge::JacTerms::ALL) ;

  EXPECT_EQ(0, result) ;
  EXPECT_NEAR(mat(1,1), 1.23, epsilon) ;
  EXPECT_NEAR(mat(2,1), 0.00, epsilon) ;
  EXPECT_EQ(6, mat.Nrows()) ;
  EXPECT_EQ(1, mat.Ncols()) ;
}

TEST(KINEMATIC_CALIBRATION_BRIDGE, one_link_medium_all)
{
  LinkParamJacobian jac ;
  jac.links_.resize(1) ;
  jac.links_[0].trans_[1].vel.data[0] = 1.01 ;
  jac.links_[0].trans_[2].rot.data[2] = 2.02 ;
  jac.links_[0].rot_[0].vel.data[0] = 3.03 ;
  jac.links_[0].rot_[2].rot.data[1] = 4.04 ;

  ActiveLinkParams active ;
  active.setNumLinks(1) ;

  active.setAllInactive() ;
  active(1,0) = true ;
  active(2,0) = true ;
  active(3,0) = true ;
  active(4,0) = true ;
  active(5,0) = true ;

  NEWMAT::Matrix mat ;

  int result ;

  result = JacNewmatBridge::jacToNewmat(jac, active, mat, JacNewmatBridge::JacTerms::ALL) ;

  EXPECT_EQ(0, result) ;
  EXPECT_EQ(6, mat.Nrows()) ;
  EXPECT_EQ(5, mat.Ncols()) ;
  EXPECT_NEAR(mat(1,1), 1.01, epsilon) ;
  EXPECT_NEAR(mat(6,2), 2.02, epsilon) ;
  EXPECT_NEAR(mat(1,3), 3.03, epsilon) ;
  EXPECT_NEAR(mat(1,4), 0.00, epsilon) ;
  EXPECT_NEAR(mat(5,5), 4.04, epsilon) ;
}

TEST(KINEMATIC_CALIBRATION_BRIDGE, one_link_medium_rot)
{
  LinkParamJacobian jac ;
  jac.links_.resize(1) ;
  jac.links_[0].trans_[1].vel.data[0] = 1.01 ;
  jac.links_[0].trans_[2].rot.data[2] = 2.02 ;
  jac.links_[0].rot_[0].vel.data[0] = 3.03 ;
  jac.links_[0].rot_[2].rot.data[1] = 4.04 ;

  ActiveLinkParams active ;
  active.setNumLinks(1) ;

  active.setAllInactive() ;
  active(1,0) = true ;
  active(2,0) = true ;
  active(3,0) = true ;
  active(4,0) = true ;
  active(5,0) = true ;

  NEWMAT::Matrix mat(6,5) ;

  int result ;

  result = JacNewmatBridge::jacToNewmat(jac, active, mat, JacNewmatBridge::JacTerms::ROT) ;

  EXPECT_EQ(0, result) ;
  EXPECT_EQ(3, mat.Nrows()) ;
  EXPECT_EQ(5, mat.Ncols()) ;
  EXPECT_NEAR(mat(3,2), 2.02, epsilon) ;
  EXPECT_NEAR(mat(2,5), 4.04, epsilon) ;
}

TEST(KINEMATIC_CALIBRATION_BRIDGE, multi_link_medium_rot)
{
  LinkParamJacobian jac ;
  jac.links_.resize(4) ;
  jac.links_[0].trans_[1].vel.data[0] = 1.01 ;
  jac.links_[1].trans_[2].rot.data[2] = 2.02 ;
  jac.links_[2].rot_[0].vel.data[0] = 3.03 ;
  jac.links_[3].rot_[2].rot.data[1] = 4.04 ;

  ActiveLinkParams active ;
  active.setNumLinks(4) ;

  active.setAllInactive() ;
  active(0,0) = true ;
  active(1,0) = true ;
  active(2,1) = true ;
  active(3,2) = true ;
  active(5,2) = true ;
  active(5,3) = true ;

  NEWMAT::Matrix mat(6,5) ;

  int result ;

  result = JacNewmatBridge::jacToNewmat(jac, active, mat, JacNewmatBridge::JacTerms::ROT) ;

  EXPECT_EQ(0, result) ;
  EXPECT_EQ(3, mat.Nrows()) ;
  EXPECT_EQ(6, mat.Ncols()) ;
  EXPECT_NEAR(mat(2,6), 4.04, epsilon) ;
}

TEST(KINEMATIC_CALIBRATION_BRIDGE, multi_link_medium_trans)
{
  LinkParamJacobian jac ;
  jac.links_.resize(4) ;
  jac.links_[0].trans_[1].vel.data[0] = 1.01 ;
  jac.links_[1].trans_[2].rot.data[2] = 2.02 ;
  jac.links_[2].rot_[0].vel.data[0] = 3.03 ;
  jac.links_[3].rot_[2].rot.data[1] = 4.04 ;

  ActiveLinkParams active ;
  active.setNumLinks(4) ;

  active.setAllInactive() ;
  active(0,0) = true ;
  active(1,0) = true ;
  active(2,1) = true ;
  active(3,2) = true ;
  active(5,2) = true ;
  active(5,3) = true ;

  NEWMAT::Matrix mat(6,5) ;

  int result ;

  result = JacNewmatBridge::jacToNewmat(jac, active, mat, JacNewmatBridge::JacTerms::TRANS) ;

  EXPECT_EQ(0, result) ;
  EXPECT_EQ(3, mat.Nrows()) ;
  EXPECT_EQ(6, mat.Ncols()) ;
  EXPECT_NEAR(mat(1,2), 1.01, epsilon) ;
  EXPECT_NEAR(mat(1,4), 3.03, epsilon) ;
}

void BuildJacVecTestSet(vector<LinkParamJacobian>& jacs, ActiveLinkParams& active)
{
  jacs.resize(2) ;
  jacs[0].links_.resize(4) ;
  jacs[0].links_[0].trans_[1].vel.data[0] = 11.21 ;
  jacs[0].links_[1].trans_[2].rot.data[2] = 12.33 ;
  jacs[0].links_[2].rot_[0].vel.data[0] = 13.41 ;
  jacs[0].links_[2].rot_[0].vel.data[1] = 13.42 ;
  jacs[0].links_[3].rot_[2].rot.data[1] = 14.63 ;

  jacs[1].links_.resize(4) ;
  jacs[1].links_[0].trans_[1].vel.data[0] = 21.21 ;
  jacs[1].links_[1].trans_[2].rot.data[2] = 22.33 ;
  jacs[1].links_[2].rot_[0].vel.data[0] = 23.41 ;
  jacs[1].links_[2].rot_[0].vel.data[1] = 23.42 ;
  jacs[1].links_[3].rot_[2].rot.data[1] = 24.63 ;

  active.setNumLinks(4) ;

  active.setAllInactive() ;
  active(0,0) = true ;
  active(1,0) = true ;
  active(2,1) = true ;
  active(3,2) = true ;
  active(5,2) = true ;
  active(5,3) = true ;
}

TEST(KINEMATIC_CALIBRATION_BRIDGE, jacvec_multi_link_translation)
{
  vector<LinkParamJacobian> jacs ;
  ActiveLinkParams active ;

  BuildJacVecTestSet(jacs, active) ;

  NEWMAT::Matrix mat(5,5) ;             // Initialize to the wrong size

  int result ;

  result = JacNewmatBridge::jacVectorToNewmat(jacs, active, mat, JacNewmatBridge::JacTerms::TRANS) ;

  EXPECT_EQ(0, result) ;
  EXPECT_EQ(6, mat.Nrows()) ;
  EXPECT_EQ(6, mat.Ncols()) ;
  EXPECT_NEAR(mat(1,1), 0.00, epsilon) ;
  EXPECT_NEAR(mat(1,2),11.21, epsilon) ;
  EXPECT_NEAR(mat(1,3), 0.00, epsilon) ;
  EXPECT_NEAR(mat(2,3), 0.00, epsilon) ;
  EXPECT_NEAR(mat(3,3), 0.00, epsilon) ;
  EXPECT_NEAR(mat(1,4),13.41, epsilon) ;
  EXPECT_NEAR(mat(2,4),13.42, epsilon) ;
  EXPECT_NEAR(mat(3,4), 0.00, epsilon) ;
  EXPECT_NEAR(mat(3,5), 0.00, epsilon) ;
  EXPECT_NEAR(mat(3,6), 0.00, epsilon) ;

  EXPECT_NEAR(mat(1+3,1), 0.00, epsilon) ;
  EXPECT_NEAR(mat(1+3,2),21.21, epsilon) ;
  EXPECT_NEAR(mat(1+3,3), 0.00, epsilon) ;
  EXPECT_NEAR(mat(2+3,3), 0.00, epsilon) ;
  EXPECT_NEAR(mat(3+3,3), 0.00, epsilon) ;
  EXPECT_NEAR(mat(1+3,4),23.41, epsilon) ;
  EXPECT_NEAR(mat(2+3,4),23.42, epsilon) ;
  EXPECT_NEAR(mat(3+3,4), 0.00, epsilon) ;
  EXPECT_NEAR(mat(3+3,5), 0.00, epsilon) ;
  EXPECT_NEAR(mat(3+3,6), 0.00, epsilon) ;
}

TEST(KINEMATIC_CALIBRATION_BRIDGE, jacvec_multi_link_all)
{
  vector<LinkParamJacobian> jacs ;
  ActiveLinkParams active ;

  BuildJacVecTestSet(jacs, active) ;

  NEWMAT::Matrix mat(5,5) ;             // Initialize to the wrong size

  int result ;

  result = JacNewmatBridge::jacVectorToNewmat(jacs, active, mat, JacNewmatBridge::JacTerms::ALL) ;

  EXPECT_EQ(0, result) ;
  EXPECT_EQ(12, mat.Nrows()) ;
  EXPECT_EQ(6, mat.Ncols()) ;

  EXPECT_NEAR(mat(1,1), 0.00, epsilon) ;
  EXPECT_NEAR(mat(1,2),11.21, epsilon) ;
  EXPECT_NEAR(mat(6,3),12.33, epsilon) ;
  EXPECT_NEAR(mat(1,4),13.41, epsilon) ;
  EXPECT_NEAR(mat(2,4),13.42, epsilon) ;
  EXPECT_NEAR(mat(3,4), 0.00, epsilon) ;
  EXPECT_NEAR(mat(3,5), 0.00, epsilon) ;
  EXPECT_NEAR(mat(5,6),14.63, epsilon) ;

  EXPECT_NEAR(mat(1+6,1), 0.00, epsilon) ;
  EXPECT_NEAR(mat(1+6,2),21.21, epsilon) ;
  EXPECT_NEAR(mat(6+6,3),22.33, epsilon) ;
  EXPECT_NEAR(mat(1+6,4),23.41, epsilon) ;
  EXPECT_NEAR(mat(2+6,4),23.42, epsilon) ;
  EXPECT_NEAR(mat(3+6,4), 0.00, epsilon) ;
  EXPECT_NEAR(mat(3+6,5), 0.00, epsilon) ;
  EXPECT_NEAR(mat(5+6,6),24.63, epsilon) ;
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
