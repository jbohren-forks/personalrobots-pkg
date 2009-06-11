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
#include <tf_conversions/tf_kdl.h>

using namespace std;
using namespace ros;
using namespace KDL;



namespace robot_state_publisher{

RobotStatePublisher::RobotStatePublisher(const Tree& tree)
  : n_("robot_state_publisher"),
    publish_rate_(0.0),
    tree_(tree)
{
  // set publish frequency
  double publish_freq;
  n_.param("~publish_frequency", publish_freq, 50.0);
  publish_rate_ = Rate(publish_freq);

  // get tf prefix
  n_.param("~tf_prefix", tf_prefix_, string());

  // build tree solver
  solver_.reset(new TreeFkSolverPosFull_recursive(tree_));

  // subscribe to mechanism_state
  mech_state_subscr_ =  n_.subscribe("/mechanism_state", 1, &RobotStatePublisher::callback, this);

  // advertise tf message
  tf_publisher_ = n_.advertise<tf::tfMessage>("/tf_message", 5);

  // get the 'real' root segment of the tree, which is the first child of "root"
  SegmentMap::const_iterator root = tree.getSegment("root");
  assert(root->second.children.begin() != root->second.children.end());  // every tree has root element
  root_ = (*root->second.children.begin())->first;
}


void RobotStatePublisher::callback(const MechanismStateConstPtr& state)
{
  // get joint positions from state message
  map<string, double> joint_positions;
  for (unsigned int i=0; i<state->joint_states.size(); i++){
    // @TODO: What to do with this 'hack' to replace _joint and _frame by _link?
    string tmp = state->joint_states[i].name;
    tmp.replace(tmp.size()-6, tmp.size(), "_link");
    joint_positions.insert(make_pair(tmp, state->joint_states[i].position));
    //cout << "Joint " << tmp << " at position " << state->joint_states[i].position << endl;
  }

  // calculate transforms form root to every segment in tree
  map<string, Frame> frames;
  solver_->JntToCart(joint_positions, frames);
  tf::tfMessage tf_msg; tf_msg.transforms.resize(frames.size()-1);
  robot_msgs::TransformStamped trans; 

  // publish the transforms to tf, converting the transforms from "root" to the 'real' root 
  map<string, Frame>::const_iterator root = frames.find(root_);
  if (root != frames.end()){
    // remove root from published poses
    Frame offset = root->second.Inverse();
    unsigned int i = 0;
    // send transforms to tf
    for (map<string, Frame>::const_iterator f=frames.begin(); f!=frames.end(); f++){
      if (f != root){
	Frame frame = offset * f->second;
	tf::Transform tf_frame;
	tf::TransformKDLToTF(frame, tf_frame);
	trans.header.stamp = state->header.stamp;
	trans.header.frame_id = tf::remap(tf_prefix_, f->first);
	trans.parent_id = root->first;
	tf::TransformTFToMsg(tf_frame, trans.transform);
	tf_msg.transforms[i++] = trans;
      }
    }
    tf_publisher_.publish(tf_msg);
  }
  else
    cout << "failed to find root" << endl;
  publish_rate_.sleep();
}

}
