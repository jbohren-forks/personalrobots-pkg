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

/** \author Ioan Sucan */

#include <planning_environment/robot_models.h>
#include <ros/time.h>
#include <gtest/gtest.h>
#include <iostream>
#include <sstream>

TEST(Loading, Simple)
{
    planning_environment::RobotModels m("robotdesc/pr2");

    EXPECT_TRUE(m.getKinematicModel().get() != NULL);
}

TEST(ForwardKinematics, RuntimeArm)
{
    planning_environment::RobotModels m("robotdesc/pr2");
    planning_models::KinematicModel* kmodel = m.getKinematicModel().get();
    
    int gid = kmodel->getGroupID("right_arm");
    unsigned int dim = kmodel->getGroupDimension(gid);
    double params[dim];
    for (unsigned int i = 0 ; i < dim ; ++i)
	params[dim] = 0.1;

    ros::WallTime tm = ros::WallTime::now();
    const unsigned int NT = 100000;  
    for (unsigned int i = 0 ; i < NT ; ++i)
	kmodel->computeTransformsGroup(params, gid);
    double fps = (double)NT / (ros::WallTime::now() - tm).toSec();
    ROS_INFO("%f forward kinematics steps per second", fps);
    
    EXPECT_TRUE(fps > 10000.0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_robot_models");
    
    return RUN_ALL_TESTS();
}
