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

/**

@mainpage

@htmlinclude manifest.html

@b test_kinematic_planning is a node that can be used to test a
kinematic planning node.

<hr>

@section usage Usage
@verbatim
$ test_kinematic_planning [standard ROS args]
@endverbatim

@par Example

@verbatim
$ test_kinematic_planning
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "left_pr2arm_pos"/PR2Arm : retrieve current left arm position
- @b "right_pr2arm_pos"/PR2Arm : retrieve current right arm position

Publishes to (name/type):
- @b "cmd_leftarmconfig"/PR2Arm : issue joint angle commands to the left arm
- @b "cmd_rightarmconfig"/PR2Arm : issue joint angle commands to the right arm

<hr>

@section services ROS services

Uses (name/type):
- @b "plan_kinematic_path"/KinematicMotionPlan : given a robot model, starting ang goal states, this service computes a collision free path

Provides (name/type):
- None

<hr>

@section parameters ROS parameters
- None


**/

#include <ros/node.h>
#include <std_msgs/PR2Arm.h>

using namespace std_msgs;

class TestTestKinematicPlanning : public ros::node
{
public:

    TestTestKinematicPlanning(void) : ros::node("test_test_kinematic_planning")
    {
      advertise<PR2Arm>("right_pr2arm_set_position");
      advertise<PR2Arm>("left_pr2arm_set_position");
    }    
};


int main(int argc, char **argv)
{  
    ros::init(argc, argv);
    
    TestTestKinematicPlanning test;

    sleep(3);

    //for(int i = 0; i < 5; i++) {
    std_msgs::PR2Arm armGoal;
    armGoal.turretAngle = -.8;
    armGoal.shoulderLiftAngle = 0;
    armGoal.upperarmRollAngle = 0;
    armGoal.elbowAngle = 0;
    armGoal.forearmRollAngle = 0; 
    armGoal.wristPitchAngle = 0;
    armGoal.wristRollAngle = 0;
    armGoal.gripperForceCmd = 0;
    armGoal.gripperGapCmd = 0;
    test.publish("right_pr2arm_set_position",armGoal);
    //sleep(1);
    //test.publish("left_pr2arm_set_position",armGoal);
    sleep(3);
        
    test.shutdown();
    
    return 0;    
}
