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
 * Author: Wim Meeussen
 */



#include "urdf/parser.h"
#include <algorithm>
#include "robot_kinematics/robot_kinematics.h"
#include "robot_mechanism_controllers/cartesian_trajectory_controller.h"


using namespace KDL;
using namespace tf;



namespace controller {

ROS_REGISTER_CONTROLLER(CartesianTrajectoryController)


CartesianTrajectoryController::CartesianTrajectoryController()
: jnt_to_pose_solver_(NULL),
  joints_(0,(mechanism::JointState*)NULL),
  motion_profile_(6, VelocityProfile_Trap(0,0))
{}

CartesianTrajectoryController::~CartesianTrajectoryController()
{
  if (jnt_to_pose_solver_) delete jnt_to_pose_solver_;
}



bool CartesianTrajectoryController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  fprintf(stderr, "initializing pose controller\n");

  // test if we got robot pointer
  assert(robot);
  robot_ = robot;

  // time
  last_time_ = robot->hw_->current_time_;

  // create twist controller
  pose_controller_.initXml(robot, config);

  // parse robot description from xml file
  ros::Node *node = ros::Node::instance();
  robot_kinematics::RobotKinematics robot_kinematics ;
  string robot_desc;
  node->param("robotdesc/pr2", robot_desc, string("")) ;
  printf("RobotDesc.length() = %u\n", robot_desc.length());
  robot_kinematics.loadString(robot_desc.c_str()) ;
  robot_kinematics::SerialChain* serial_chain = robot_kinematics.getSerialChain("right_arm");
  if (serial_chain == NULL)  
    fprintf(stderr, "Got NULL Chain\n") ;

  // convert description to KDL chain
  chain_        = serial_chain->chain;
  num_joints_   = chain_.getNrOfJoints();
  num_segments_ = chain_.getNrOfSegments();
  printf("Extracted KDL Chain with %u Joints and %u segments\n", num_joints_, num_segments_ );
  jnt_to_pose_solver_ = new ChainFkSolverPos_recursive(chain_);

  // get chain
  TiXmlElement *chain = config->FirstChildElement("chain");
  if (!chain) {
    fprintf(stderr, "Error: CartesianTrajectoryController was not given a chain\n");
    return false;
  }

  // get names for root and tip of robot
  const char *root_name = chain->Attribute("root");
  root_link_ = root_name;
  const char *tip_name = chain->Attribute("tip");
  if (!root_name) {
    fprintf(stderr, "Error: Chain element for CartesianTrajectoryController must specify the root\n");
    return false;
  }
  if (!tip_name)  {
    fprintf(stderr, "Error: Chain element for CartesianTrajectoryController must specify the tip\n");
    return false;
  }

  // test if we can get root from robot
  if (!robot->getLinkState(root_name)) {
    fprintf(stderr, "Error: link \"%s\" does not exist (CartesianTrajectoryController)\n", root_name);
    return false;
  }

  // get tip from robot
  mechanism::LinkState *current = robot->getLinkState(tip_name);
  if (!current)  {
    fprintf(stderr, "Error: link \"%s\" does not exist (CartesianTrajectoryController)\n", tip_name);
    return false;

  }

  // Works up the chain, from the tip to the root, and get joints
  while (current->link_->name_ != std::string(root_name)) 
    {
      // get joint from current link
      joints_.push_back(robot->getJointState(current->link_->joint_name_));
      assert(joints_[joints_.size()-1]);
      
      // get parent link
      current = robot->getLinkState(current->link_->parent_name_);
      
      if (!current) {
	  fprintf(stderr, "Error: for CartesianTrajectoryController, tip is not connected to root\n");
	  return false;
	}
    }
  // reverse order of joint vector
  std::reverse(joints_.begin(), joints_.end());

  // initialize motion profile
  double max_vel_trans, max_vel_rot, max_acc_trans, max_acc_rot;
  node->param("arm_trajectory/max_vel_trans", max_vel_trans, 0.0) ;
  node->param("arm_trajectory/max_vel_rot", max_vel_rot, 0.0) ;
  node->param("arm_trajectory/max_acc_trans", max_acc_trans, 0.0) ;
  node->param("arm_trajectory/max_acc_rot", max_acc_rot, 0.0) ;

  for (unsigned int i=0; i<3; i++){
    motion_profile_[i  ].SetMax(max_vel_trans, max_acc_trans);
    motion_profile_[i+3].SetMax(max_vel_rot,   max_acc_rot);
  }

  // set desired pose to current pose
  pose_current_ = getPose();
  twist_current_ = Twist::Zero();

  // start not moving
  is_moving_ = false;

  return true;
}




bool CartesianTrajectoryController::moveTo(const Frame& pose_desi, double duration)
{
  // don't do anything when still moving
  if (is_moving_) return false;

  // trajectory from pose_begin to pose_end
  pose_end_ = pose_desi;
  pose_begin_ = getPose();

  max_duration_ = 0;
  Twist twist_move = diff(pose_begin_, pose_end_);

  // Set motion profiles
  for (unsigned int i=0; i<6; i++){
    motion_profile_[i].SetProfileDuration( 0, twist_move(i), duration);
    max_duration_ = max( max_duration_, motion_profile_[i].Duration() );
  }

  // Rescale trajectories to maximal duration
  for (unsigned int i=0; i<6; i++)
    motion_profile_[i].SetProfileDuration( 0, twist_move(i), max_duration_ );

  cout << "will move to new pose in " << max_duration_ << " seconds" << endl;

  time_passed_ = 0;
  is_moving_ = true;

  return true;
}




void CartesianTrajectoryController::update()
{
  // get time
  double time = robot_->hw_->current_time_;
  double dt = time - last_time_;
  last_time_ = time;

  if (is_moving_){
    time_passed_ += dt;

    // ended trajectory
    if (time_passed_ > max_duration_){
      twist_current_ = Twist::Zero();
      pose_current_  = pose_end_;
      is_moving_ = false;
    }
    // still in trajectory
    else{
      // pose
      Twist twist_begin_current = Twist(Vector(motion_profile_[0].Pos(time_passed_),motion_profile_[1].Pos(time_passed_),motion_profile_[2].Pos(time_passed_)),
					Vector(motion_profile_[3].Pos(time_passed_),motion_profile_[4].Pos(time_passed_),motion_profile_[5].Pos(time_passed_)) );
      pose_current_ = Frame( pose_begin_.M * Rot( pose_begin_.M.Inverse( twist_begin_current.rot ) ), 
			     pose_begin_.p + twist_begin_current.vel);

      // twist
      for(unsigned int i=0; i<6; i++)
	twist_current_(i) = motion_profile_[i].Vel( time_passed_ );
    }
  }

  // send output to pose controller
  pose_controller_.pose_desi_ = pose_current_;
  pose_controller_.twist_ff_ = twist_current_;

  // update pose controller
  pose_controller_.update();
}





Frame CartesianTrajectoryController::getPose()
{
  // check if joints are calibrated
  for (unsigned int i = 0; i < joints_.size(); ++i) {
    if (!joints_[i]->calibrated_)
      fprintf(stderr,"Joint not calibrated\n");
  }

  // get the joint positions 
  JntArray jnt_pos(num_joints_);
  unsigned int i_corr = 0;
  for (unsigned int i=0; i<num_joints_; i++){
    while (joints_[i_corr]->joint_->type_ ==  mechanism::JOINT_FIXED)
      i_corr++;
    jnt_pos(i) = joints_[i_corr]->position_;
    i_corr++;
  }

  // get cartesian pose
  Frame result;
  jnt_to_pose_solver_->JntToCart(jnt_pos, result);

  return result;
}







ROS_REGISTER_CONTROLLER(CartesianTrajectoryControllerNode)

CartesianTrajectoryControllerNode::CartesianTrajectoryControllerNode()
: node_(ros::Node::instance()),
  robot_state_(*node_, true),
  command_notifier_(NULL)
{}

CartesianTrajectoryControllerNode::~CartesianTrajectoryControllerNode()
{
  if (command_notifier_) delete command_notifier_;
}


bool CartesianTrajectoryControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  // get name of topic to listen to
  topic_ = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic_ == "") {
    fprintf(stderr, "No topic given to CartesianTrajectoryControllerNode\n");
    return false;
  }

  // initialize controller  
  if (!controller_.initXml(robot, config))
    return false;
  
  // subscribe to pose commands
  command_notifier_ = new MessageNotifier<std_msgs::PoseStamped>(&robot_state_, node_,  boost::bind(&CartesianTrajectoryControllerNode::command, this, _1), topic_ + "/command", controller_.root_link_, 1);

  return true;
}


void CartesianTrajectoryControllerNode::update()
{
  controller_.update();
}


void CartesianTrajectoryControllerNode::command(const MessageNotifier<std_msgs::PoseStamped>::MessagePtr& pose_msg)
{
  // convert message to transform
  Stamped<Pose> pose_stamped;
  PoseStampedMsgToTF(*pose_msg, pose_stamped);

  // convert to reference frame of root link of the controller chain  
  robot_state_.transformPose(controller_.root_link_, pose_stamped, pose_stamped);

  // tell controller where to move to
  Frame pose_desi;
  TransformToFrame(pose_stamped, pose_desi);
  controller_.moveTo(pose_desi);
}


void CartesianTrajectoryControllerNode::TransformToFrame(const Transform& trans, Frame& frame)
{
  frame.p(0) = trans.getOrigin().x();
  frame.p(1) = trans.getOrigin().y();
  frame.p(2) = trans.getOrigin().z();

  double Rz, Ry, Rx;
  trans.getBasis().getEulerZYX(Rz, Ry, Rx);
  frame.M = Rotation::EulerZYX(Rz, Ry, Rx);
}



}; // namespace

