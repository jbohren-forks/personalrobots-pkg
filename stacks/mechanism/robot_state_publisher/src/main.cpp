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

/* Author: Wim Meeussen */

#include <kdl/tree.hpp>
#include <ros/ros.h>
#include <kdl_parser/tree_parser.hpp>
#include <mechanism_msgs/MechanismState.h>
#include "robot_state_publisher/robot_state_publisher.h"

using namespace std;
using namespace ros;
using namespace KDL;
using namespace robot_state_publisher;

class MechStatePublisher{
public:
  MechStatePublisher(const string& robot_desc)
    : publish_rate_(0.0)
  {
    // create kinematic tree
    Tree tree;
    if (!treeFromString(robot_desc, tree))
      ROS_ERROR("Failed to construct robot model from xml string");
    state_publisher_ = new RobotStatePublisher(tree);

    // set publish frequency
    double publish_freq;
    n_.param("~publish_frequency", publish_freq, 50.0);
    publish_rate_ = Rate(publish_freq);

    // subscribe to mechanism state
    mech_state_sub_ = n_.subscribe("/mechanism_state", 1, &MechStatePublisher::callbackMechState, this);;

  };
  ~MechStatePublisher()
  {
    delete state_publisher_;
  };

private:
  void callbackMechState(const mechanism_msgs::MechanismStateConstPtr& state)
  {
    // get joint positions from state message
    map<string, double> joint_positions;
    for (unsigned int i=0; i<state->joint_states.size(); i++)
      joint_positions.insert(make_pair(state->joint_states[i].name, state->joint_states[i].position));
    state_publisher_->publishTransforms(joint_positions, state->header.stamp);
    publish_rate_.sleep();
  }

  NodeHandle n_;
  robot_state_publisher::RobotStatePublisher* state_publisher_;
  Rate publish_rate_;
  Subscriber mech_state_sub_;
  map<string, string> joint_segment_mapping_;
};




// ----------------------------------
// ----- MAIN -----------------------
// ----------------------------------
int main(int argc, char** argv)
{
  // Initialize ros
  ros::init(argc, argv, "robot_state_publisher");
  NodeHandle node;

  // build robot model
  string robot_desc;
  node.param("/robotdesc/pr2", robot_desc, string());
  MechStatePublisher publisher(robot_desc);

  ros::spin();
  return 0;
}
