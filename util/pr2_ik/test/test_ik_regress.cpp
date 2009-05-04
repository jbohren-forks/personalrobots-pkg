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

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <ros/node.h>
#include <pr2_ik/pr2_ik_solver.h>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <gtest/gtest.h>

#define IK_NEAR 0.1
using namespace pr2_ik;
using namespace KDL;

double gen_rand(double min, double max)
{
  int rand_num = rand()%100+1;
  double result = min + (double)((max-min)*rand_num)/101.0;
  return result;
}

TEST(PR2IK, initialize)
{
  PR2IKSolver ik;
  ASSERT_TRUE(ik.active_);
}

bool NOT_NEAR(const double &v1, const double &v2, const double &NEAR)
{
   if(fabs(v1-v2) > NEAR)
      return true;
   return false;
}

TEST(PR2IK, inverseKinematics)
{
  int num_tests = 1000000;
  PR2IKSolver ik;
  if(!ik.active_)
  {
    ROS_FATAL("ik not initialized");
    exit(1);
  }
  KDL::Chain kdl_chain;
  KDL::ChainFkSolverPos_recursive *jnt_to_pose_solver;
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  KDL::Frame p_out;
  KDL::Frame p_ik;

/* initialize random seed: */
  srand ( time(NULL) );

  ik.chain_.toKDL(kdl_chain);

  jnt_to_pose_solver = new KDL::ChainFkSolverPos_recursive(kdl_chain);
  jnt_pos_in.resize(7);
  jnt_pos_out.resize(7);

  int num_solutions(0);

  for(int kk = 0; kk < num_tests; kk++)
  {
    for(int i=0; i < 7; i++)
    {
      jnt_pos_in(i) = gen_rand(std::max(ik.pr2_ik_->min_angles_[i],-M_PI),std::min(ik.pr2_ik_->max_angles_[i],M_PI));
      EXPECT_TRUE((jnt_pos_in(i) <= ik.pr2_ik_->max_angles_[i]));
      EXPECT_TRUE((jnt_pos_in(i) >= ik.pr2_ik_->min_angles_[i]));
    }

    if(jnt_to_pose_solver->JntToCart(jnt_pos_in,p_out) >=0)
    {
      bool ik_valid = (ik.CartToJnt(jnt_pos_in,p_out,jnt_pos_out) >= 0);
      EXPECT_TRUE(ik_valid);
      if(ik_valid)
      {
        num_solutions++;
        jnt_to_pose_solver->JntToCart(jnt_pos_out,p_ik);
        for(int j=0; j< 3; j++)
        {
          EXPECT_NEAR(p_ik.M(j,0),p_out.M(j,0),IK_NEAR);
          EXPECT_NEAR(p_ik.M(j,1),p_out.M(j,1),IK_NEAR); 
          EXPECT_NEAR(p_ik.M(j,2),p_out.M(j,2),IK_NEAR); 
          EXPECT_NEAR(p_ik.p(j),p_out.p(j),IK_NEAR);
        }
      }
    }
  }
  EXPECT_EQ(num_solutions,num_tests);
  delete jnt_to_pose_solver;
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init (argc, argv);
  ros::Node n("pr2_ik_regression_test");
  return RUN_ALL_TESTS();
}
