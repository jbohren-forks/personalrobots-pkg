/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Sachin Chitta
 */

#include <algorithm>
#include <pr2_ik/pr2_ik_node.h>

using namespace KDL;
using namespace tf;
using namespace std;
using namespace ros;

namespace pr2_ik {

  PR2IKNode::PR2IKNode()
      : tf_(*ros::Node::instance()),
        dimension_(7)
  {
    node_handle_.param<std::string>("~root_name", root_name_, "torso_lift_link");
    node_handle_.param<bool>("~free_angle_constraint",free_angle_constraint_,false);
    node_handle_.param<int>("~dimension",dimension_,7);
    node_handle_.param<std::string>("~ik_service_name",ik_service_name_,"~ik_service");
    node_handle_.param<std::string>("~ik_query_name",ik_query_name_,"~ik_query");

    if(free_angle_constraint_)
    {
      double free_angle_constraint_min, free_angle_constraint_max;
      node_handle_.param<double>("~free_angle_constraint_min",free_angle_constraint_min,-M_PI);
      node_handle_.param<double>("~free_angle_constraint_max",free_angle_constraint_max,M_PI);
      pr2_ik_solver_.pr2_ik_->min_angles_[pr2_ik_solver_.pr2_ik_->free_angle_] = free_angle_constraint_min;
      pr2_ik_solver_.pr2_ik_->max_angles_[pr2_ik_solver_.pr2_ik_->free_angle_] = free_angle_constraint_max;
    }
    this->init();
    ik_service_ = node_handle_.advertiseService(ik_service_name_,&PR2IKNode::ikService,this);
    ik_query_ = node_handle_.advertiseService(ik_query_name_,&PR2IKNode::ikQuery,this);
  }

  PR2IKNode::~PR2IKNode()
  {
  }

  bool PR2IKNode::init()
  {
    ROS_DEBUG("Initialized IK controller");
    return true;
  }

  //  void PR2IKNode::command(const tf::MessageNotifier<pr2_ik::PoseCmd>::MessagePtr& pose_msg)

  int PR2IKNode::getJointIndex(const std::string &name)
  {
    for(int i=0; i< dimension_; i++)
      {
	if(pr2_ik_solver_.chain_.getJoint(i)->name_ == name)
	  {
	    return i;
	  }
      }
    return -1;   
  }

  bool PR2IKNode::checkJointNames(const manipulation_srvs::IKService::Request &request)
  {
      for(int i=0; i< dimension_; i++)
      {
	if(!getJointIndex(request.data.joint_names[i]))
	  {
	    return false;
	  }
      }
    return true;   
  }

  bool PR2IKNode::ikService(manipulation_srvs::IKService::Request &request, manipulation_srvs::IKService::Response &response)
  {
    tf::Stamped<tf::Pose> pose_stamped;
    if(!checkJointNames(request))
      {
	ROS_ERROR("Joint names in service request do not match joint names for IK node.");
	return false;
      }

    ROS_DEBUG("Got pose command");
    PoseStampedMsgToTF(request.data.pose_stamped, pose_stamped);
    ROS_DEBUG("Converted pose command to tf");
    // convert to reference frame of root link of the chain
    tf_.transformPose(root_name_, pose_stamped, pose_stamped);
    ROS_DEBUG("Converted tf command to root name");
    poseToFrame(pose_stamped, pose_desired_);
    ROS_DEBUG("Converted tf command to KDL");

    //Do the IK
    KDL::JntArray jnt_pos_in;
    KDL::JntArray jnt_pos_out;
    jnt_pos_in.resize(dimension_);
    for(int i=0; i < dimension_; i++)
      jnt_pos_in(i) = 0.0;
    jnt_pos_in(pr2_ik_solver_.pr2_ik_->free_angle_) = request.data.positions[pr2_ik_solver_.pr2_ik_->free_angle_];
    bool ik_valid = (pr2_ik_solver_.CartToJntSearch(jnt_pos_in,pose_desired_,jnt_pos_out,10) >= 0);

    if(ik_valid)
    {
      response.solution.resize(dimension_);
      for(int i=0; i < dimension_; i++)
      {
        response.solution[i] = jnt_pos_out(getJointIndex(request.data.joint_names[i]));
        ROS_DEBUG("IK Joint %s %d: %f",request.data.joint_names[i].c_str(),i,jnt_pos_out(i));
      }
      return true;
    }
    else
    {
      ROS_ERROR("IK invalid");   
      return false;
    }
  }

  bool PR2IKNode::ikQuery(manipulation_srvs::IKQuery::Request &request, manipulation_srvs::IKQuery::Response &response)
  {
    response.names.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      response.names[i] = pr2_ik_solver_.chain_.getJoint(i)->name_;
    }
    return true;
  }

  void PR2IKNode::poseToFrame(const tf::Pose& pose, KDL::Frame& frame)
  {
    frame.p(0) = pose.getOrigin().x();
    frame.p(1) = pose.getOrigin().y();
    frame.p(2) = pose.getOrigin().z();

    double Rz, Ry, Rx;
    pose.getBasis().getEulerZYX(Rz, Ry, Rx);
    frame.M = Rotation::EulerZYX(Rz, Ry, Rx);
  }
} // namespace

int main(int argc, char** argv){
  ros::init(argc, argv, "pr2_ik_node");
  pr2_ik::PR2IKNode pr2_ik_node;
  ros::spin();
  return(0);
}
