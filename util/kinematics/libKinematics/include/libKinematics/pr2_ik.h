//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of the Willow Garage nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

#ifndef PR2_KINEMATICS_HH
#define PR2_KINEMATICS_HH

#include <iostream>
#include <vector>
#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>
#include <libKinematics/kinematics.h>


namespace kinematics
{
#define NUM_JOINTS_ARM7DOF 7
#define IK_EPS 1e-6

  class arm7DOF : public kinematics::SerialRobot
  {
    public:

    arm7DOF(std::vector<NEWMAT::Matrix> anchors, std::vector<NEWMAT::Matrix> axis, std::vector<std::string> joint_type);
     
    std::vector<NEWMAT::Matrix> axis_;

    std::vector<NEWMAT::Matrix> anchors_;

    std::vector<NEWMAT::Matrix> xi_0_;

    std::vector<NEWMAT::Matrix> xi_0_inv_;

    std::vector<std::vector<double> > solution_ik_;
       
    void ComputeIK(NEWMAT::Matrix g, double theta1);

    void ComputeIKEfficient(NEWMAT::Matrix g, double t1);

    void ComputeIKEfficientTheta3(NEWMAT::Matrix g, double t3);

    double increment_;

    bool computeIKFast(NEWMAT::Matrix g, int joint_num, double initial_guess);

    bool computeIKFastWithConstraints(NEWMAT::Matrix g, int joint_num, double initial_guess);

    bool setAngleMultipliers(std::vector<double> angle_mult);

    private:

    bool use_joint_limits_;

    double ap_[5];

    double a1_,a2_,a4_;

    int solveCosineEqn(const double &a, const double &b, const double &c, double &soln1, double &soln2);

    std::vector<double> solution_;

    std::vector<double> angle_multipliers_;

    NEWMAT::Matrix matInv(const NEWMAT::Matrix &g);

    NEWMAT::Matrix gf_;

    NEWMAT::Matrix home_inv_;

    NEWMAT::Matrix grhs_;

    int solve_quadratic(double a, double b, double c, double *x1, double *x2);

    bool computeNewGuess(const double &initial_value, double &return_value, int joint_num);

    int num_positive_increments_;

    int num_negative_increments_;

    bool positive_increment_valid_;

    bool negative_increment_valid_;

    bool last_increment_positive_;

    bool last_increment_negative_;

  };
}

#endif
