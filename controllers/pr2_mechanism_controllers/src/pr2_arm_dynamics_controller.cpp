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

#include "pr2_mechanism_controllers/pr2_arm_dynamics_controller.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/String.h"

// Math utils
#include <angles/angles.h>

using namespace controller;
using namespace std;

ROS_REGISTER_CONTROLLER(PR2ArmDynamicsController);

PR2ArmDynamicsController::PR2ArmDynamicsController():num_joints_(0),last_time_(0)
{
}

PR2ArmDynamicsController::~PR2ArmDynamicsController()
{
  // Assumes the joint controllers are owned by this class.
  for(unsigned int i=0; i < num_joints_;++i)
  {
    delete joint_effort_controllers_[i];
  }
  delete gravity_torque_;
  delete gravity_torque_uncompensated_;
}

bool PR2ArmDynamicsController::initXml(mechanism::RobotState * robot, TiXmlElement * config)
{
  robot_ = robot->model_;
  TiXmlElement *elt = config->FirstChildElement("controller");
  while (elt)
  {
    JointEffortController * jec = new JointEffortController();
    std::cout<<elt->Attribute("type")<<elt->Attribute("name")<<std::endl;
    assert(static_cast<std::string>(elt->Attribute("type")) == std::string("JointEffortController"));
    joint_effort_controllers_.push_back(jec);
    if(!jec->initXml(robot, elt))
      return false;

    TiXmlElement *p = elt->FirstChildElement("joint")->FirstChildElement("pid");
    if (p)
    {
      control_toolbox::Pid  pid_controller;
      pid_controller.initPid(0, 0, 0, 0, 0);
      pid_controller.initXml(p);
      pid_controllers_.push_back(pid_controller);
   }
    else
      fprintf(stderr, "JointEffortController's config did not specify the default pid parameters.\n");

    elt = elt->NextSiblingElement("controller");
  }

  num_joints_ = joint_effort_controllers_.size();

  fprintf(stderr,"PR2ArmDynamicsController:: num_joints_: %d\n",num_joints_);

  goals_.resize(num_joints_);
  goals_rt_.resize(num_joints_);

  goals_dot_.resize(num_joints_);
  goals_rt_dot_.resize(num_joints_);

  goals_dot_dot_.resize(num_joints_);
  goals_rt_dot_dot_.resize(num_joints_);

  control_torque_.resize(num_joints_);

  gravity_torque_ = new KDL::Vector[num_joints_+1];
  gravity_torque_uncompensated_ = new KDL::Vector[num_joints_+1];

  last_time_ = robot->hw_->current_time_;

  return true;
}

void PR2ArmDynamicsController::setJointCmd(const std::vector<double> &j_values, const std::vector<double> &j_values_dot,  const std::vector<double> &j_values_dot_dot, const std::vector<std::string> & j_names)
{
  assert(j_values.size() == j_names.size());
  for(uint i = 0; i < j_values.size(); ++i)
  {
    const std::string & name = j_names[i];
    const int id = getJointControllerByName(name);
    if(id >= 0)
    {
      goals_[id] = j_values[i];
      goals_dot_[id] = j_values_dot[i];
      goals_dot_dot_[id] = j_values_dot_dot[i];
    }
  }
  goal_achieved_ = false;

  arm_controller_lock_.lock();
  refresh_rt_vals_ = true;
  arm_controller_lock_.unlock();
}


void PR2ArmDynamicsController::getJointCmd(robot_msgs::JointCmd & cmd) const
{
  const unsigned int n = joint_effort_controllers_.size();
  cmd.set_names_size(n);
  for(unsigned int i=0; i<n; ++i)
      cmd.names[i] = joint_effort_controllers_[i]->getJointName();

  cmd.set_positions_vec(goals_);
}


controller::JointEffortController* PR2ArmDynamicsController::getJointEffortControllerByName(std::string name)
{
  for(int i=0; i< (int) num_joints_; i++)
  {
    if(joint_effort_controllers_[i]->getJointName() == name)
    {
      return joint_effort_controllers_[i];
    }
  }
    return NULL;
}


int PR2ArmDynamicsController::getJointControllerByName(std::string name)
{
  for(int i=0; i< (int) num_joints_; i++)
  {
    if(joint_effort_controllers_[i]->getJointName() == name)
    {
      return i;
    }
  }
  return -1;
}


void PR2ArmDynamicsController::update(void)
{
//  cout << "Updating dynamics controller " << std::endl;
  double time = robot_->hw_->current_time_;

  if(refresh_rt_vals_ && arm_controller_lock_.try_lock())
  {
    for(unsigned int i=0; i < num_joints_; ++i)
    {
      goals_rt_[i] = goals_[i];
      goals_rt_dot_[i] = goals_dot_[i];
      goals_rt_dot_dot_[i] = goals_dot_dot_[i];
    }
    arm_controller_lock_.unlock();
  }

  computeControlTorque(time);

  for(unsigned int i=0; i < num_joints_;++i)
//  for(unsigned int i=5; i < num_joints_; ++i)
  {
   joint_effort_controllers_[i]->setCommand(control_torque_[i]);
//    joint_effort_controllers_[i]->setCommand(0.0);
  }

  updateJointControllers();

  for(unsigned int i=0; i < num_joints_; ++i)
  {
//    fprintf(stderr,"Effort: %d %f %f %f\n",i,control_torque_[i],gravity_torque_[i][2],joint_effort_controllers_[i]->joint_->commanded_effort_);
  }
}

void PR2ArmDynamicsController::computeControlTorque(const double &time)
{
  KDL::JntArray kdl_q(num_joints_);
  KDL::JntArray kdl_q_uncompensated(num_joints_);
  KDL::JntArray kdl_q_dot(num_joints_);
  KDL::JntArray kdl_q_dot_dot(num_joints_);
  double error(0),command(0),actual(0),pid_torque(0);
  int j_type;

  for(unsigned int i=0; i < num_joints_; ++i)
  {
    kdl_q(i) = joint_effort_controllers_[i]->joint_state_->position_;
    kdl_q_uncompensated(i) = kdl_q(i);
  }
  kdl_q_uncompensated(4) = 0.0;
  kdl_q_uncompensated(5) = 0.0;
  kdl_q_uncompensated(6) = 0.0;

  arm_chain_->computeGravityTerms(kdl_q,gravity_torque_);
  arm_chain_uncompensated_->computeGravityTerms(kdl_q_uncompensated,gravity_torque_uncompensated_);

  for(unsigned int i=0; i < num_joints_; ++i)
  {
    command = goals_rt_[i];
    actual = joint_effort_controllers_[i]->joint_state_->position_;
    j_type = joint_effort_controllers_[i]->joint_state_->joint_->type_;
    if(j_type == mechanism::JOINT_ROTARY)
    {
      if(!angles::shortest_angular_distance_with_limits(command, actual, joint_effort_controllers_[i]->joint_state_->joint_->joint_limit_min_, joint_effort_controllers_[i]->joint_state_->joint_->joint_limit_max_,error))
        error = 0;
    }
    else if(j_type == mechanism::JOINT_CONTINUOUS)
    {
      error = angles::shortest_angular_distance(command, actual);
    }
    else
    {
      error = actual - command;
    }
    pid_torque = pid_controllers_[i].updatePid(error, time - last_time_);
    if( i < 4)
    {
//      if(i == 2)
//        control_torque_[i] = 0.0;
//      else
        //control_torque_[i] = gravity_torque_[i][2] + pid_torque;
        control_torque_[i] = gravity_torque_[i][2] - gravity_torque_uncompensated_[i][2] + pid_torque;
    }
    else
      control_torque_[i] = gravity_torque_[i][2] + pid_torque;
//    fprintf(stderr,"%d:: %f, %f, %f, %f, %f, %f, %f\n",i,actual,command,error,pid_torque,gravity_torque_[i][2],control_torque_[i],time-last_time_);

    static realtime_tools::RealtimePublisher<std_msgs::String> p("/s", 1);
    if (p.trylock()) {
      char buf[1000];
      sprintf(buf, "Joint torques %d:: %s, %15.6lf %15.61f %15.61f %15.61f\n", i, joint_effort_controllers_[i]->joint_state_->joint_->name_.c_str(),control_torque_[i],gravity_torque_[i][2],gravity_torque_uncompensated_[i][2],pid_torque);
      p.msg_.data = std::string(buf);
      p.unlockAndPublish();
    }



  }


  last_time_ = time;
}


void PR2ArmDynamicsController::updateJointControllers(void)
{
  for(unsigned int i=0;i<num_joints_;++i)
    joint_effort_controllers_[i]->update();
}


void PR2ArmDynamicsController::checkForGoalAchieved_(void)
{
/*  goal_achieved_ = true;
  for(unsigned int i=0;i<goals_rt_.size();++i)
    goal_achieved_=goal_achieved_&&(error_margins_[i]<=0||std::abs(joint_effort_controllers_[i]->getMeasuredPosition()-goals_rt_[i])<error_margins_[i]);
*/
}


//------ Arm controller node --------

ROS_REGISTER_CONTROLLER(PR2ArmDynamicsControllerNode)

PR2ArmDynamicsControllerNode::PR2ArmDynamicsControllerNode()
  : Controller()
{
  ROS_INFO("PR2 arm gravity compensation controller node created");
  c_ = new PR2ArmDynamicsController();
}

PR2ArmDynamicsControllerNode::~PR2ArmDynamicsControllerNode()
{
  delete c_;
}

void PR2ArmDynamicsControllerNode::update()
{
  c_->update();
}

bool PR2ArmDynamicsControllerNode::initXml(mechanism::RobotState * robot, TiXmlElement * config)
{
  ROS_INFO("LOADING PR2 ARM DYNAMICS CONTROLLER NODE");
  ros::Node * const node = ros::Node::instance();
  string prefix = config->Attribute("name");
  std::cout<<"the prefix is "<<prefix<<std::endl;

  // Parses controller configuration.
  std::string kdl_chain_name="";
  TiXmlElement *j = config->FirstChildElement("kinematics");
  assert(j);
  j = j->FirstChildElement("elem");
  assert(j);
  while(j!=NULL)
  {
    if(j->Attribute("key")==std::string("kdl_chain_name"))
    {
      kdl_chain_name = j->GetText();
      break;
    }
    j = j->NextSiblingElement("elem");
  }


  // Parses subcontroller configuration
  if(c_->initXml(robot, config))
  {
    node->advertiseService(prefix + "/set_command_array", &PR2ArmDynamicsControllerNode::setJointSrv, this);
    node->advertiseService(prefix + "/get_command", &PR2ArmDynamicsControllerNode::getJointCmd, this);

// Parses kinematics description
    std::string pr2Contents;
    std::string pr2_uncompensated;

    node->getParam("robotdesc/pr2_uncompensated", pr2_uncompensated);
    node->getParam("robotdesc/pr2", pr2Contents);

    c_->pr2_kin_.loadString(pr2Contents.c_str());
    c_->pr2_kin_uncompensated_.loadString(pr2_uncompensated.c_str());

    c_->arm_chain_ = c_->pr2_kin_.getSerialChain(kdl_chain_name.c_str());
    c_->arm_chain_uncompensated_ = c_->pr2_kin_uncompensated_.getSerialChain(kdl_chain_name.c_str());

    fprintf(stderr,"Got arm chain %s\n",kdl_chain_name.c_str());
    assert(c_->arm_chain_);
    assert(c_->arm_chain_uncompensated_);
    return true;
  }
  return false;
}

bool PR2ArmDynamicsControllerNode::setJointSrv(robot_srvs::SetJointCmd::request &req,
                                   robot_srvs::SetJointCmd::response &resp)
{
  std::vector<double> pos;
  std::vector<double> vel;
  std::vector<double> acc;
  std::vector<std::string> names;

  req.set_positions_vec(pos);
  req.set_velocity_vec(vel);
  req.set_acc_vec(acc);
  req.set_names_vec(names);

  c_->setJointCmd(pos,vel,acc,names);
  return true;
}

bool PR2ArmDynamicsControllerNode::getJointCmd(robot_srvs::GetJointCmd::request &req,
                    robot_srvs::GetJointCmd::response &resp)
{
  robot_msgs::JointCmd cmd;
  c_->getJointCmd(cmd);
  resp.command = cmd;
  return true;
}
