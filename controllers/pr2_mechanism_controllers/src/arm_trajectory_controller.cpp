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

// Original version: Sachin Chitta <sachinc@willowgarage.com>

#include "pr2_mechanism_controllers/arm_trajectory_controller.h"
#include "misc_utils/realtime_publisher.h"
#include "std_msgs/String.h"

using namespace controller;
using namespace std;

ROS_REGISTER_CONTROLLER(ArmTrajectoryController);

static inline double now()
{
  struct timespec n;
  clock_gettime(CLOCK_MONOTONIC, &n);
  return double(n.tv_nsec) / 1.0e9 + n.tv_sec;
}

ArmTrajectoryController::ArmTrajectoryController() :
  refresh_rt_vals_(false)
{
}

ArmTrajectoryController::~ArmTrajectoryController()
{
  // Assumes the joint controllers are owned by this class.
  for(unsigned int i=0; i<joint_position_controllers_.size();++i)
    delete joint_position_controllers_[i];
}

bool ArmTrajectoryController::initXml(mechanism::RobotState * robot, TiXmlElement * config)
{
  ROS_INFO("Initializing trajectory controller");

  robot_ = robot->model_;
  mechanism::Joint *joint;
  std::vector<double> joint_velocity_limits;
  std::vector<trajectory::Trajectory::TPoint> trajectory_points_vector;
  std::string trajectory_type = "linear";

  TiXmlElement *elt = config->FirstChildElement("controller");
  while (elt)
  {
    JointPositionController * jpc = new JointPositionController();
//    std::cout<<elt->Attribute("type")<<elt->Attribute("name")<<std::endl;
    ROS_INFO("Joint Position Controller: %s , %s",(elt->Attribute("type")),(elt->Attribute("name")));
    assert(static_cast<std::string>(elt->Attribute("type")) == std::string("JointPositionController"));

    joint_position_controllers_.push_back(jpc);
    if(!jpc->initXml(robot, elt))
      return false;

    joint = (robot->getJointState(jpc->getJointName()))->joint_;
    if(joint)
      joint_velocity_limits.push_back(joint->velocity_limit_/1.0);

    elt = elt->NextSiblingElement("controller");
  }

  ROS_INFO("ArmTrajectoryController:: Done loading controllers");

  elt = config->FirstChildElement("trajectory");

  if(!elt)
    ROS_WARN("No trajectory information in xml file. ");
  else
  {
    trajectory_type = std::string(elt->Attribute("interpolation"));
    ROS_INFO("ArmTrajectoryController:: interpolation type:: %s",trajectory_type.c_str());    
  }
  joint_cmd_rt_.resize(joint_position_controllers_.size()); 

  joint_trajectory_ = new trajectory::Trajectory((int) joint_position_controllers_.size());

  joint_trajectory_->setMaxRates(joint_velocity_limits);
  joint_trajectory_->setInterpolationMethod(trajectory_type);

  trajectory_point_.setDimension((int) joint_position_controllers_.size());
  dimension_ = (int) joint_position_controllers_.size();


// Add two points since every good trajectory must have at least two points, otherwise its just a point :-)
  for(int j=0; j < dimension_; j++)
    trajectory_point_.q_[j] = joint_position_controllers_[j]->joint_state_->position_;
  trajectory_point_.time_ = 0.0;
  trajectory_points_vector.push_back(trajectory_point_);

  for(int i=0; i < dimension_; i++)
     trajectory_point_.q_[i] = 0.0;
  trajectory_point_.time_ = 0.0;
  trajectory_points_vector.push_back(trajectory_point_);

  joint_trajectory_->autocalc_timing_ = true;

  ROS_INFO("Size of trajectory points vector : %d",trajectory_points_vector.size());
  if(!joint_trajectory_->setTrajectory(trajectory_points_vector))
    ROS_WARN("Trajectory not set correctly");

  trajectory_start_time_ = robot_->hw_->current_time_;

  return true;
}

void ArmTrajectoryController::setTrajectoryCmd(const std::vector<trajectory::Trajectory::TPoint>& joint_trajectory)
{
  if(joint_trajectory.size() < 1)
  {
    ROS_WARN("ArmTrajectoryController:: No points in trajectory");
    return;
  }

  joint_trajectory_->setTrajectory(joint_trajectory);
  joint_trajectory_->write("foo.txt",0.01);
  arm_controller_lock_.lock();
  refresh_rt_vals_ = true;
  arm_controller_lock_.unlock();
}

void ArmTrajectoryController::getJointPosCmd(pr2_mechanism_controllers::JointPosCmd & cmd) const
{
}

controller::JointPositionController* ArmTrajectoryController::getJointControllerByName(std::string name)
{
  for(int i=0; i< (int) joint_position_controllers_.size(); i++)
  {
    if(joint_position_controllers_[i]->getJointName() == name)
    {
      return joint_position_controllers_[i];
    }
  }
    return NULL;
}

int ArmTrajectoryController::getJointControllerPosByName(std::string name)
{
  for(int i=0; i< (int) joint_position_controllers_.size(); i++)
  {
    if(joint_position_controllers_[i]->getJointName() == name)
    {
      return i;
    }
  }
  return -1;
}

void ArmTrajectoryController::update(void)
{
  double start_time = now();

  double sample_time(0.0);
  if(refresh_rt_vals_)
  {
    trajectory_start_time_ = robot_->hw_->current_time_;
    refresh_rt_vals_ = false;
  }
  if(arm_controller_lock_.trylock())
  {
    sample_time = robot_->hw_->current_time_ - trajectory_start_time_;
    joint_trajectory_->sample(trajectory_point_,sample_time);

//    ROS_INFO("sample_time: %f", sample_time); 
    for(unsigned int i=0; i < joint_cmd_rt_.size(); ++i)
    {
      joint_cmd_rt_[i] = trajectory_point_.q_[i];
//      cout << " " << joint_cmd_rt_[i];
    }
//    cout << endl;
    arm_controller_lock_.unlock();
  }

  for(unsigned int i=0;i<joint_position_controllers_.size();++i)
    joint_position_controllers_[i]->setCommand(joint_cmd_rt_[i]);

  updateJointControllers();

  double end_time = now();

  static misc_utils::RealtimePublisher<std_msgs::String> p("/s", 1);
  if (p.trylock()) {
    char buf[1000];       
    sprintf(buf, "Time = %15.6lf\n", end_time - start_time);
    p.msg_.data = std::string(buf);
    p.unlockAndPublish();
  }
}

void ArmTrajectoryController::updateJointControllers(void)
{
  for(unsigned int i=0;i<joint_position_controllers_.size();++i)
    joint_position_controllers_[i]->update();
}

//------ Arm controller node --------

ROS_REGISTER_CONTROLLER(ArmTrajectoryControllerNode)

ArmTrajectoryControllerNode::ArmTrajectoryControllerNode()
  : Controller(), node_(ros::node::instance())
{
  std::cout<<"Controller node created"<<endl;
  c_ = new ArmTrajectoryController();
}

ArmTrajectoryControllerNode::~ArmTrajectoryControllerNode()
{
  /* node_->unadvertise_service(service_prefix_ + "/set_command");
  node_->unadvertise_service(service_prefix_ + "/set_command_array");
  node_->unadvertise_service(service_prefix_ + "/get_command");
  node_->unadvertise_service(service_prefix_ + "/set_target");
  */
   if(topic_name_ptr_ && topic_name_.c_str())
  {
    std::cout << "unsub arm controller" << topic_name_ << std::endl;
    node_->unsubscribe(topic_name_);
  }

  delete c_;
}

void ArmTrajectoryControllerNode::update()
{
  c_->update();
}

bool ArmTrajectoryControllerNode::initXml(mechanism::RobotState * robot, TiXmlElement * config)
{
  ROS_INFO("Loading ArmTrajectoryControllerNode.");
  service_prefix_ = config->Attribute("name");
  ROS_INFO("The service_prefix_ is %s",service_prefix_.c_str());

  if(c_->initXml(robot, config))  // Parses subcontroller configuration
  {
/*    node_->advertise_service(service_prefix_ + "/set_command", &ArmTrajectoryControllerNode::setJointPosHeadless, this);
    node_->advertise_service(service_prefix_ + "/set_command_array", &ArmTrajectoryControllerNode::setJointPosSrv, this);
    node_->advertise_service(service_prefix_ + "/get_command", &ArmTrajectoryControllerNode::getJointPosCmd, this);
    node_->advertise_service(service_prefix_ + "/set_target", &ArmTrajectoryControllerNode::setJointPosTarget, this);
*/
    topic_name_ptr_ = config->FirstChildElement("listen_topic");
    if(topic_name_ptr_)
    {
      topic_name_= topic_name_ptr_->Attribute("name");
      if(!topic_name_.c_str())
      {
        std::cout<<" A listen _topic is present in the xml file but no name is specified\n";
        return false;
      }
      node_->subscribe(topic_name_, traj_msg_, &ArmTrajectoryControllerNode::CmdTrajectoryReceived, this, 1);
      ROS_INFO("Listening to topic: %s",topic_name_.c_str());
    }
    ROS_INFO("Initialized controller");
    return true;
  }
  ROS_INFO("Could not initialize controller");
  return false;
}

void ArmTrajectoryControllerNode::CmdTrajectoryReceived()
{
  std::vector<trajectory::Trajectory::TPoint> tp;

  this->ros_lock_.lock();
  tp.resize((int)traj_msg_.get_points_size()+1);

  //set first point in trajectory to current position of the arm
    tp[0].setDimension((int) c_->dimension_);
  for(int j=0; j < c_->dimension_; j++)
  {
    tp[0].q_[j] = c_->joint_position_controllers_[j]->joint_state_->position_;
    tp[0].time_ = 0.0;
  }

  if(traj_msg_.get_points_size() > 0)
  {
     if((int) traj_msg_.points[0].get_positions_size() != (int) c_->dimension_)
     {
       ROS_WARN("Dimension of input trajectory = %d does not match number of controlled joints = %d",(int) traj_msg_.points[0].get_positions_size(), (int) c_->dimension_);
        return;
     }
  }
  else
  {
    ROS_WARN("Trajectory message has no way points");
     return;
  }

  for(int i=0; i < (int) traj_msg_.get_points_size(); i++)
  {
     tp[i+1].setDimension((int) c_->dimension_);
     for(int j=0; j < (int) c_->dimension_; j++)
     {
        tp[i+1].q_[j] = traj_msg_.points[i].positions[j];
        tp[i+1].time_ = traj_msg_.points[i].time;
     }
  }

  this->c_->setTrajectoryCmd(tp);
  this->ros_lock_.unlock();
}


bool ArmTrajectoryControllerNode::getJointPosCmd(pr2_mechanism_controllers::GetJointPosCmd::request &req,
                    pr2_mechanism_controllers::GetJointPosCmd::response &resp)
{
  pr2_mechanism_controllers::JointPosCmd cmd;
  c_->getJointPosCmd(cmd);
  resp.command = cmd;
  return true;
}
