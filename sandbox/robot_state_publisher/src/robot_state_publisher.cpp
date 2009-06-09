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

#include "robot_state_publisher/robot_state_publisher.h"
#include <kdl/frames_io.hpp>

using namespace std;
using namespace ros;
using namespace KDL;



namespace robot_state_publisher{

RobotStatePublisher::RobotStatePublisher()
  : n_("robot_state_publisher"),
    tf_(*(Node::instance())),
    publish_rate_(0.0)
{
  // set publish frequency
  double publish_freq;
  n_.param("~publish_frequency", publish_freq, 1.0);
  publish_rate_ = Rate(publish_freq);

  // build robot model
  string robot_desc;
  n_.param("/robotdesc/pr2", robot_desc, string());
  if (!treeFromString(robot_desc, tree_))
    ROS_ERROR("Failed to construct robot model from xml string");

  // build tree solver
  solver_.reset(new TreeFkSolverPosFull_recursive(tree_));

  // subscribe to mechanism_state
  mech_state_subscr_ =  n_.subscribe("/mechanism_state", 1, &RobotStatePublisher::callback, this);
}


void RobotStatePublisher::callback(const MechanismStateConstPtr& state)
{
  // get joint positions from state message
  map<string, double> joint_positions;
  for (unsigned int i=0; i<state->joint_states.size(); i++){
    string tmp = state->joint_states[i].name;
    tmp.resize(tmp.size()-6);
    tmp = tmp + "_link";
    joint_positions[tmp] = state->joint_states[i].position;
    cout << "Joint " << tmp << " at position " << state->joint_states[i].position << endl;
  }


  // calculate transforms form root to every segment in tree
  map<string, Frame> frames;
  solver_->JntToCart(joint_positions, frames);

  // send transforms to tf
  for (map<string, Frame>::const_iterator f=frames.begin(); f!=frames.end(); f++){
    cout << "frame " << f->first << " = " << f->second.p << endl;
    tf_.sendTransform(tf::Transform(tf::Quaternion(0,0,0), tf::Vector3(f->second.p(0), f->second.p(1), f->second.p(2))), state->header.stamp, f->first + "_test", "root");
  }

  publish_rate_.sleep();
}

}
