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

#define IK_NEAR 0.001
using namespace pr2_ik;
using namespace KDL;

int main(int argc, char **argv)
{
   ros::init (argc, argv);
   ros::Node n("pr2_ik_node");
   KDL::JntArray input;
   input.resize(7);

//   const double ja[7] = {-0.087379,1.225220,-2.439131,-1.610891,-2.395076,1.585149,0.031105};
   const double ja[7] = {0.566087,1.358282,-2.673285,-0.969307,-1.835188,1.585149,1.648559};
  
//   double init =  -1.970823;
   double init =  -2.670823;

   input(0) = ja[0];
   input(1) = ja[1];
   input(2) = ja[2];
   input(3) = ja[3];

   input(4) = ja[4];
   input(5) = ja[5];
   input(6) = ja[6];

   KDL::JntArray guess = input;
   guess(2) = init;

   PR2IKSolver ik;
   if(!ik.active_)
   {
      ROS_FATAL("ik not initialized");
      exit(1);
   }

   KDL::Chain kdl_chain;
   KDL::ChainFkSolverPos_recursive *jnt_to_pose_solver;
   std::vector<KDL::JntArray> jnt_pos_out;
   KDL::Frame p_out;
   KDL::Frame p_ik;

   srand ( time(NULL) );

   ik.chain_.toKDL(kdl_chain);

   jnt_to_pose_solver = new KDL::ChainFkSolverPos_recursive(kdl_chain);
//   jnt_pos_out.resize(7);

   if(jnt_to_pose_solver->JntToCart(input,p_out) >=0)
   {

     bool ik_valid = (ik.CartToJntSearch(guess,p_out,jnt_pos_out,10) >= 0);
      if(ik_valid) 
         ROS_INFO("True");
      for(int j=0; j<7; j++)
      {
         printf("%f ",input(j));
      }
      printf("\n");
      for(int m=0; m < 3; m++)
      {
         printf("%f %f %f %f\n",p_out.M(m,0),p_out.M(m,1),p_out.M(m,2),p_out.p(m));
      }

      ROS_INFO("%d solutions",ik.pr2_ik_->solution_ik_.size());
      for(int i=0; i < (int) ik.pr2_ik_->solution_ik_.size(); i++)
      {
         for(int j=0; j<7; j++)
         {
            printf("%f ",ik.pr2_ik_->solution_ik_[i][j]);
            input(j) = ik.pr2_ik_->solution_ik_[i][j];
         }
         printf("\n");
         if(jnt_to_pose_solver->JntToCart(input,p_ik) >=0)
         {
            for(int m=0; m < 3; m++)
            {
               printf("%f %f %f %f\n",p_ik.M(m,0),p_ik.M(m,1),p_ik.M(m,2),p_ik.p(m));
            }
         }

      }
   }
   delete jnt_to_pose_solver;
}

