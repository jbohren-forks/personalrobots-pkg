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
using namespace ros;




namespace controller {


CartesianTrajectoryController::CartesianTrajectoryController()
: node_(ros::Node::instance()),
  jnt_to_pose_solver_(NULL),
  motion_profile_(6, VelocityProfile_Trap(0,0))
{}

CartesianTrajectoryController::~CartesianTrajectoryController()
{
  if (jnt_to_pose_solver_) delete jnt_to_pose_solver_;
}



bool CartesianTrajectoryController::init(mechanism::RobotState *robot_state, 
                                         const string& root_name, 
                                         const string& tip_name, 
                                         const string controller_name)
{
  cout << "initializing " << controller_name << " between " << root_name << " and " << tip_name << endl;
  controller_name_ = controller_name;

  // test if we got robot pointer
  assert(robot_state);
  robot_state_ = robot_state;

  // create robot chain from root to tip
  if (!robot_.init(robot_state->model_, root_name, tip_name))
    return false;
  robot_.toKDL(chain_);

  // create solver
  num_joints_   = chain_.getNrOfJoints();
  num_segments_ = chain_.getNrOfSegments();
  jnt_to_pose_solver_ = new ChainFkSolverPos_recursive(chain_);
  jnt_pos_.resize(num_joints_);

  // initialize motion profile
  double max_vel_trans, max_vel_rot, max_acc_trans, max_acc_rot;
  node_->param(controller_name_+"/max_vel_trans", max_vel_trans, 0.0) ;
  node_->param(controller_name_+"/max_vel_rot", max_vel_rot, 0.0) ;
  node_->param(controller_name_+"/max_acc_trans", max_acc_trans, 0.0) ;
  node_->param(controller_name_+"/max_acc_rot", max_acc_rot, 0.0) ;
  for (unsigned int i=0; i<3; i++){
    motion_profile_[i  ].SetMax(max_vel_trans, max_acc_trans);
    motion_profile_[i+3].SetMax(max_vel_rot,   max_acc_rot);
  }

  // initialize pose controller
  pose_controller_.init(robot_state, root_name, tip_name, controller_name_+"/pose");

  return true;
}




bool CartesianTrajectoryController::moveTo(const Frame& pose_desi, double duration)
{
  // don't do anything when still moving
  if (is_moving_) return false;

  // trajectory from pose_begin to pose_end
  request_preempt_ = false;
  pose_end_ = pose_desi;
  pose_begin_ = pose_current_;

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

  cout << controller_name_ << " will move to new pose in " << max_duration_ << " seconds" << endl;

  time_passed_ = 0;
  is_moving_ = true;

  return true;
}



bool CartesianTrajectoryController::starting()
{
  // time
  last_time_ = robot_state_->hw_->current_time_;

  // set desired pose to current pose
  pose_current_ = getPose();
  twist_current_ = Twist::Zero();

  // start not moving
  is_moving_ = false;
  request_preempt_ = false;

  return pose_controller_.starting();
}




void CartesianTrajectoryController::update()
{
  // get time
  double time = robot_state_->hw_->current_time_;
  double dt = time - last_time_;
  last_time_ = time;

  // preempt trajectory
  if (request_preempt_){
    twist_current_ = Twist::Zero();
    is_moving_ = false;
  }

  // if we are moving
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
      Twist twist_begin_current = Twist(Vector(motion_profile_[0].Pos(time_passed_),
					       motion_profile_[1].Pos(time_passed_),
					       motion_profile_[2].Pos(time_passed_)),
					Vector(motion_profile_[3].Pos(time_passed_),
					       motion_profile_[4].Pos(time_passed_),
					       motion_profile_[5].Pos(time_passed_)) );
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
  // get the joint positions and velocities
  robot_.getPositions(robot_state_->joint_states_, jnt_pos_);

  // get cartesian pose
  Frame result;
  jnt_to_pose_solver_->JntToCart(jnt_pos_, result);

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
  node_->unadvertiseService(controller_name_+"/move_to");
}


bool CartesianTrajectoryControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  // get the controller name
  controller_name_ = config->Attribute("name");

  // get name of root and tip
  string tip_name;
  node_->param(controller_name_+"/root_name", root_name_, string("no_root_name_given"));
  node_->param(controller_name_+"/tip_name", tip_name, string("no_tip_name_given"));

  // initialize controller
  if (!controller_.init(robot, root_name_, tip_name, controller_name_))
    return false;

  // subscribe to pose commands
  command_notifier_ = new MessageNotifier<robot_msgs::PoseStamped>(&robot_state_, node_,
								 boost::bind(&CartesianTrajectoryControllerNode::command, this, _1),
								 controller_name_ + "/command", root_name_, 1);

  // advertise moveto service
  node_->advertiseService(controller_name_+"/move_to", &CartesianTrajectoryControllerNode::moveTo, this);

  return true;
}


bool CartesianTrajectoryControllerNode::starting()
{
  return controller_.starting();
}

void CartesianTrajectoryControllerNode::update()
{
  controller_.update();
}



bool CartesianTrajectoryControllerNode::moveTo(robot_srvs::MoveToPose::Request &req,
                                               robot_srvs::MoveToPose::Response &resp)
{
  if (!moveTo(req.pose))
    return false;

  cout << "start moveto service " << endl;
  while (controller_.isMoving())
    Duration().fromSec(0.01).sleep();

  cout << "end moveto service " << endl;

  if (controller_.isPreempted())
    return false;
  else
    return true;
}


bool CartesianTrajectoryControllerNode::preempt(std_srvs::Empty::Request &req,
                                                std_srvs::Empty::Response &resp)
{
  // you can only preempt is the robot is moving
  if (!controller_.isMoving())
    return false;

  controller_.preempt();

  // wait for robot to stop moving
  Duration sleep_time = Duration().fromSec(0.01);
  while (controller_.isMoving())
    sleep_time.sleep();

  return true;
}



void CartesianTrajectoryControllerNode::command(const MessageNotifier<robot_msgs::PoseStamped>::MessagePtr& pose_msg)
{
  moveTo(*pose_msg);
}











// ----------------------------------
// helper functions
// ----------------------------------
bool CartesianTrajectoryControllerNode::moveTo(robot_msgs::PoseStamped& pose)
{
  // convert message to transform
  Stamped<Pose> pose_stamped;
  PoseStampedMsgToTF(pose, pose_stamped);

  // convert to reference frame of root link of the controller chain
  Duration sleeptime = Duration().fromSec(0.001);
  Duration timeout = Duration().fromSec(5.0);
  Time starttime = Time().now();
  try
  {
    while (!robot_state_.canTransform(root_name_, pose.header.frame_id, pose.header.stamp) &&
           (Time().now() - starttime) < timeout )
      sleeptime.sleep();
    robot_state_.transformPose(root_name_, pose_stamped, pose_stamped);
  }
  catch (const tf::TransformException &ex)
  {
    ROS_ERROR("Transform exception: %s", ex.what());
    return false;
  }

  // tell controller where to move to
  Frame pose_desi;
  TransformToFrame(pose_stamped, pose_desi);
  return controller_.moveTo(pose_desi);
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

