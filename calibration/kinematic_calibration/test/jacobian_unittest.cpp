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

#include "kinematic_calibration/unittest_io.h"

using namespace kinematic_calibration ;
using namespace std ;

static const double epsilon = 1e-6 ;

/**
 * \brief The main piece of the LinkParamJacobian test harness
 * Combines the JacobiansGetter, JointStatesGetter, and ModelGetter to create a more streamlined way to manipulate
 * LinkParamJacobian stored in files.
 */
class VerifyJacobian
{
public:
  VerifyJacobian() {  }
  ~VerifyJacobian() {  }
  /**
   * \brief Computes the max error between the LinkParamJacobian in a text file and the LinkParamJacobian calculated by KDL
   */
  int ComputeMaxError(const std::string& model_file, const std::string& joint_params_file, const std::string& jacobians_file, double& max_error)
  {
    int result ;

    result = model_getter_.openFile(model_file) ;
    if (result < 0)
      return -1 ;

    result = joint_params_getter_.openFile(joint_params_file) ;
    if (result < 0)
      return -1 ;

    result = jacobians_getter_.openFile(jacobians_file) ;
    if (result < 0)
      return -1 ;

    KDL::Chain chain = model_getter_.getModel() ;

    const unsigned int J = chain.getNrOfJoints() ;
    KDL::JntArray joint_array(J) ;
    result = joint_params_getter_.getNextJointArray(joint_array) ;
    if (result < 0)
      return -1 ;

    LinkParamJacobian jac_actual ;
    jac_actual.links_.resize(J) ;
    result = jacobians_getter_.getNextJacobian(jac_actual) ;

    // Compute the jacobian using KDL functions

    LinkParamJacobian jac_computed ;
    jac_computed.links_.resize(J) ;

    LinkParamJacobianSolver jac_solver ;
    jac_solver.JointsToCartesian(chain, joint_array, jac_computed) ;

    max_error = 0.0 ;
    for (unsigned int i=0; i < jac_computed.links_.size(); i++)
    {
      for (unsigned int j=0; j<3; j++)
      {
        for (unsigned int k=0; k<3; k++)
        {
          double cur_error ;
          cur_error = fabs( jac_computed.links_[i].trans_[j].vel(k) - jac_actual.links_[i].trans_[j].vel(k) ) ;
          if (cur_error > max_error)
            max_error = cur_error ;

          cur_error = fabs( jac_computed.links_[i].rot_[j].vel(k) - jac_actual.links_[i].rot_[j].vel(k) ) ;
          if (cur_error > max_error)
            max_error = cur_error ;
        }
      }
    }

    return 0 ;
  }

private:
  ModelGetter model_getter_ ;
  JointStatesGetter joint_params_getter_ ;
  JacobiansGetter jacobians_getter_ ;
} ;

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
  int result = RunVerifier("./test/data/jacobian_unittest/2d_easy", max_error) ;
  EXPECT_EQ(0, result) ;
  EXPECT_NEAR(max_error, 0, epsilon) ;
}

TEST(KINEMATIC_CALIBRATION_JACOBIAN, 2d_hard)
{
  double max_error ;
  int result = RunVerifier("./test/data/jacobian_unittest/2d_hard", max_error) ;
  EXPECT_EQ(0, result) ;
  EXPECT_NEAR(max_error, 0, epsilon) ;
}

TEST(KINEMATIC_CALIBRATION_JACOBIAN, 3d_hard)
{
  double max_error ;
  int result = RunVerifier("./test/data/jacobian_unittest/3d_hard", max_error) ;
  EXPECT_EQ(0, result) ;
  EXPECT_NEAR(max_error, 0, epsilon) ;
}

TEST(KINEMATIC_CALIBRATION_JACOBIAN, edge_case_1)
{
  double max_error ;
  int result = RunVerifier("./test/data/jacobian_unittest/edge_case_1", max_error) ;
  EXPECT_EQ(0, result) ;
  EXPECT_NEAR(max_error, 0, epsilon) ;
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
