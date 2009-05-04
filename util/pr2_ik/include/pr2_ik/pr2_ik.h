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
// * Neither the name of Willow Garage, Inc. nor the names of its
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

#include <angles/angles.h>
#include <Eigen/Array>
#include <vector>
namespace pr2_ik
{
/**** List of angles (for reference) *******
      th1 = shoulder/turret pan
      th2 = shoulder/turret lift/pitch
      th3 = shoulder/turret roll
      th4 = elbow pitch
      th5 = elbow roll 
      th6 = wrist pitch
      th7 = wrist roll 
****/
  static const int NUM_JOINTS_ARM7DOF = 7;

  static const double IK_EPS = 1e-5;

  class PR2IK
  {
    public:

    PR2IK(const double &shoulder_offset_distance, 
          const double &shoulder_elbow_distance, 
          const double &elbow_wrist_distance, 
          const double &torso_shoulder_x, 
          const double &torso_shoulder_y, 
          const double &torso_shoulder_z, 
          const int &free_angle);
  
    ~PR2IK();

    bool setAngleMultipliers(std::vector<double> angle_mult);

    bool setFreeAngle(int angle_index);

    bool setJointLimits(std::vector<double> min_angles, std::vector<double> max_angles);

    int free_angle_;/** Index of free parameter that is specified externally, can be either 0 or 2 */

    std::vector<std::vector<double> > solution_ik_;

    void computeIKEfficient(const Eigen::Matrix4f &g, const double &t1);

    void computeIKEfficientTheta3(const Eigen::Matrix4f &g, const double &t3);

    Eigen::Vector3f torso_shoulder_offset_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::vector<double> min_angles_;

    std::vector<double> max_angles_;

    private:

    Eigen::Matrix4f matrixInverse(const Eigen::Matrix4f &g);

    bool checkJointLimits(const std::vector<double> &angles);

    bool checkJointLimits(const double &angle, const int &angle_index);

    bool solveQuadratic(const double &a, const double &b, const double &c, double *x1, double *x2);

    bool solveCosineEqn(const double &a, const double &b, const double &c, double &soln1, double &soln2);

    Eigen::Matrix4f grhs_;

    Eigen::Matrix4f gf_;

    Eigen::Matrix4f home_inv_;

    std::vector<double> angle_multipliers_;

    std::vector<double> solution_;

    double a1_,a2_,a4_;

    double ap_[5];
  };
}
