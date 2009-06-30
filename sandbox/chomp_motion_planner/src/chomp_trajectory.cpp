/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Mrinal Kalakrishnan */

#include <chomp_motion_planner/chomp_trajectory.h>

namespace chomp
{

ChompTrajectory::ChompTrajectory(const ChompRobotModel* robot_model, double duration, double discretization):
  robot_model_(robot_model),
  num_points_((duration/discretization)+1),
  discretization_(discretization),
  duration_(duration)
{
  init();
}

ChompTrajectory::ChompTrajectory(const ChompRobotModel* robot_model, int num_points, double discretization):
  robot_model_(robot_model),
  num_points_(num_points),
  discretization_(discretization),
  duration_((num_points-1)*discretization)
{
  init();
}

ChompTrajectory::~ChompTrajectory()
{
}

void ChompTrajectory::init()
{
  num_joints_ = robot_model_->getNumKDLJoints();
  trajectory_.resize(num_points_, KDL::JntArray(num_joints_));
}

void ChompTrajectory::fillInMinJerk(int startIndex, int endIndex)
{
  double T[6]; // powers of the time duration
  T[0] = 1.0;
  T[1] = (endIndex - startIndex)*discretization_;

  for (int i=2; i<=5; i++)
    T[i] = T[i-1]*T[1];

  // calculate the spline coefficients for each joint:
  // (these are for the special case of zero start and end vel and acc)
  double coeff[num_joints_][6];
  for (int i=0; i<num_joints_; i++)
  {
    double x0 = trajectory_[startIndex](i);
    double x1 = trajectory_[endIndex](i);
    coeff[i][0] = x0;
    coeff[i][1] = 0;
    coeff[i][2] = 0;
    coeff[i][3] = (-20*x0 + 20*x1) / (2*T[3]);
    coeff[i][4] = (30*x0 - 30*x1) / (2*T[4]);
    coeff[i][5] = (-12*x0 + 12*x1) / (2*T[5]);

  }

  // now fill in the joint positions at each time step
  for (int i=startIndex+1; i<endIndex; i++)
  {
    double t[6]; // powers of the time index point
    t[0] = 1.0;
    t[1] = (i - startIndex)*discretization_;
    for (int k=2; k<=5; k++)
      t[k] = t[k-1]*t[1];

    for (int j=0; j<num_joints_; j++)
    {
      trajectory_[i](j) = 0.0;
      for (int k=0; k<=5; k++)
      {
        trajectory_[i](j) += t[k]*coeff[j][k];
      }
    }
  }

}

}
