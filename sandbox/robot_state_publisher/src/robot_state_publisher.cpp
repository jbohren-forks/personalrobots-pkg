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

RobotStatePublisher::RobotStatePublisher(const Tree& tree)
  : n_("robot_state_publisher"),
    publish_rate_(0.0),
    tree_(tree),
    use_tf_broadcaster_(true)
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
  if (!use_tf_broadcaster_)
    tf_publisher_ = n_.advertise<tf::tfMessage>("/tf_message", 5);

  // get root segment
  SegmentMap::const_iterator root = tree.getSegment("root");
  if (root->second.children.begin() != root->second.children.end())
    root_ = (*root->second.children.begin())->first;
}


void RobotStatePublisher::callback(const MechanismStateConstPtr& state)
{
  // get joint positions from state message
  map<string, double> joint_positions;
  for (unsigned int i=0; i<state->joint_states.size(); i++){
    // @TODO: What to do with this 'hack' to replace _joint by _link?
    string tmp = state->joint_states[i].name;
    tmp.replace(tmp.size()-6, tmp.size(), "_link");
    joint_positions[tmp] = state->joint_states[i].position;
    //cout << "Joint " << tmp << " at position " << state->joint_states[i].position << endl;
  }

  // calculate transforms form root to every segment in tree
  map<string, Frame> frames;
  solver_->JntToCart(joint_positions, frames);
  tf::tfMessage tf_msg; tf_msg.transforms.resize(frames.size()-1);
  robot_msgs::TransformStamped trans; 

  map<string, Frame>::const_iterator root = frames.find(root_);
  if (root != frames.end()){
    // remove root from published poses
    Frame offset = root->second.Inverse();
    unsigned int i = 0;
    // send transforms to tf
    for (map<string, Frame>::const_iterator f=frames.begin(); f!=frames.end(); f++){
      if (f != root){
	Frame frame = offset * f->second;
	if (use_tf_broadcaster_){
	  double z, y, x;
	  frame.M.GetEulerZYX(z,y,x);
	  tf_.sendTransform(tf::Transform(tf::Quaternion(z, y, x), tf::Vector3(frame.p(0), frame.p(1), frame.p(2))), state->header.stamp, tf::remap(tf_prefix_, f->first), root->first);
	}
	else{
	  double qx, qy, qz, qw;
	  frame.M.GetQuaternion(qx, qy, qz, qw);
	  trans.header.stamp = state->header.stamp;
	  trans.header.frame_id = tf::remap(tf_prefix_, f->first);
	  trans.parent_id = root->first;
	  trans.transform.translation.x = frame.p(0);
	  trans.transform.translation.y = frame.p(1);
	  trans.transform.translation.z = frame.p(2);
	  trans.transform.rotation.w = qw;
	  trans.transform.rotation.x = qx;
	  trans.transform.rotation.y = qy;
	  trans.transform.rotation.z = qz;
	  tf_msg.transforms[i++] = trans;
	}
      }
    }
    if (!use_tf_broadcaster_)
      tf_publisher_.publish(tf_msg);
  }
  else
    cout << "failed to find root" << endl;
  publish_rate_.sleep();
}

}
