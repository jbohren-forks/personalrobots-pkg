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

#include <ros/node.h>
#include <pr2_ik/pr2_ik_solver.h>
#include <kdl/chainfksolverpos_recursive.hpp>

int main(int argc, char **argv)
{
   ros::init (argc, argv);
   ros::Node n("pr2_ik_node");

   pr2_ik::PR2IKSolver ik;
   if(!ik.active_)
   {
      ROS_FATAL("ik not initialized");
      exit(1);
   }

   KDL::JntArray input;
   input.resize(7);
   input(0) = 0.5;
   input(1) = 1.3;
   input(2) = -2.0;
   input(3) = -0.9;
   input(4) = -1.8;
   input(5) = 1.2;
   input(6) = 1.6;

   KDL::Chain kdl_chain;
   KDL::ChainFkSolverPos_recursive *jnt_to_pose_solver;

   ik.chain_.toKDL(kdl_chain);
   jnt_to_pose_solver = new KDL::ChainFkSolverPos_recursive(kdl_chain);

   KDL::JntArray jnt_pos_out;
   KDL::Frame p_out;

   jnt_pos_out.resize(7);
   bool ik_valid = false;
   if(jnt_to_pose_solver->JntToCart(input,p_out) >=0)
      ik_valid = (ik.CartToJnt(input,p_out,jnt_pos_out) >= 0);

   if(ik_valid)
   {
     for(int i=0; i<7; i++)
     {
       ROS_INFO("%d %f",i,jnt_pos_out(i));
     }
   }

   delete jnt_to_pose_solver;
}
