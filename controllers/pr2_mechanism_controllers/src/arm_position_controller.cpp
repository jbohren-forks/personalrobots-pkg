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

// Original version: Timothy Hunter <tjhunter@willowgarage.com>

#include "pr2_mechanism_controllers/arm_position_controller.h"

using namespace controller;
using namespace std;

ROS_REGISTER_CONTROLLER(ArmPositionController);

ArmPositionController::ArmPositionController() :
  goal_achieved_(false),
  refresh_rt_vals_(false)
{
}

ArmPositionController::~ArmPositionController()
{
  // Assumes the joint controllers are owned by this class.
  for(unsigned int i=0; i<joint_position_controllers_.size();++i)
    delete joint_position_controllers_[i];
}

bool ArmPositionController::initXml(mechanism::RobotState * robot, TiXmlElement * config)
{
  robot_ = robot->model_;
  TiXmlElement *elt = config->FirstChildElement("controller");
  while (elt)
  {
    JointPositionController * jpc = new JointPositionController();
    std::cout<<elt->Attribute("type")<<elt->Attribute("name")<<std::endl;
    assert(static_cast<std::string>(elt->Attribute("type")) == std::string("JointPositionController"));
    joint_position_controllers_.push_back(jpc);
    if(!jpc->initXml(robot, elt))
      return false;

    elt = elt->NextSiblingElement("controller");
  }
  goals_.resize(joint_position_controllers_.size());
  error_margins_.resize(joint_position_controllers_.size());

  goals_rt_.resize(joint_position_controllers_.size());
  error_margins_rt_.resize(joint_position_controllers_.size());
  return true;
}

void ArmPositionController::setJointPosCmd(const std::vector<double> &j_values, const std::vector<std::string> & j_names)
{
  assert(j_values.size() == j_names.size());
  for(uint i=0;i<j_values.size();++i)
  {
    const std::string & name = j_names[i];
    const int id = getJointControllerPosByName(name);
    if(id>=0)
      goals_[id] = j_values[i];
  }
  goal_achieved_ = false;

  arm_controller_lock_.lock();
  refresh_rt_vals_ = true;
  arm_controller_lock_.unlock();
}

void ArmPositionController::setJointPosCmd(const std::vector<double> &j_values)
{
  assert(j_values.size() == joint_position_controllers_.size());
  for(uint i=0;i<j_values.size();++i)
  {
      goals_[i] = j_values[i];
      error_margins_[i] = -1;
  }

  goal_achieved_ = false;

  arm_controller_lock_.lock();
  refresh_rt_vals_ = true;
  arm_controller_lock_.unlock();
}

void ArmPositionController::getJointPosCmd(pr2_mechanism_controllers::JointPosCmd & cmd) const
{
  const unsigned int n = joint_position_controllers_.size();
  cmd.set_names_size(n);
  for(unsigned int i=0; i<n; ++i)
      cmd.names[i] = joint_position_controllers_[i]->getJointName();

  cmd.set_positions_vec(goals_);
  cmd.set_margins_vec(error_margins_);
}

void ArmPositionController::setJointPosCmd(const pr2_mechanism_controllers::JointPosCmd & cmd)
{
  assert(cmd.get_names_size()==cmd.get_positions_size());

  for(uint i=0;i<error_margins_.size();++i)
    error_margins_[i]=-1;

  for(uint i=0;i<cmd.get_names_size();++i)
  {
    const std::string & name = cmd.names[i];
    const int id = getJointControllerPosByName(name);
    if(id>=0)
    {
      goals_[id] = cmd.positions[i];
      error_margins_[id] = cmd.margins[i];
    }
  }
  goal_achieved_ = false;

  arm_controller_lock_.lock();
  refresh_rt_vals_ = true;
  arm_controller_lock_.unlock();
}


controller::JointPositionController* ArmPositionController::getJointControllerByName(std::string name)
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


int ArmPositionController::getJointControllerPosByName(std::string name)
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

//void ArmPositionController::getJointPosCmd(pr2_mechanism_controllers::GetJointPosCmd::response &resp)
//{
//  arm_controller_lock_.lock();
//  resp.set_positions_size(goals_.size());
//  resp.set_names_size(joint_position_controllers_.size());
//  assert(resp.get_positions_size() == goals_.size()); // TODO:Check intended behaviour here.
//  resp.set_positions_vec(goals_);
//  for(unsigned int i=0;i<joint_position_controllers_.size();++i)
//    resp.names[i] = joint_position_controllers_[i]->getJointName();
//  arm_controller_lock_.unlock();
//}

void ArmPositionController::update(void)
{
  if(refresh_rt_vals_ && arm_controller_lock_.trylock())
  {
    assert(goals_.size() == goals_rt_.size());
    for(unsigned int i=0; i<goals_.size(); ++i)
      goals_rt_[i] = goals_[i];
    for(unsigned int i=0; i<error_margins_.size(); ++i)
      error_margins_rt_[i] = error_margins_[i];
    arm_controller_lock_.unlock();
  }

  for(unsigned int i=0;i<goals_rt_.size();++i)
    joint_position_controllers_[i]->setCommand(goals_rt_[i]);

  updateJointControllers();
  checkForGoalAchieved_();
}

void ArmPositionController::updateJointControllers(void)
{
  for(unsigned int i=0;i<goals_rt_.size();++i)
    joint_position_controllers_[i]->update();
}


void ArmPositionController::checkForGoalAchieved_(void)
{
  goal_achieved_ = true;
  for(unsigned int i=0;i<goals_rt_.size();++i)
    goal_achieved_=goal_achieved_&&(error_margins_[i]<=0||std::abs(joint_position_controllers_[i]->joint_state_->position_-goals_rt_[i])<error_margins_[i]);
}


//------ Arm controller node --------

ROS_REGISTER_CONTROLLER(ArmPositionControllerNode)

ArmPositionControllerNode::ArmPositionControllerNode()
  : Controller(), node_(ros::node::instance())
{
  std::cout<<"Controller node created"<<endl;
  c_ = new ArmPositionController();
}

ArmPositionControllerNode::~ArmPositionControllerNode()
{
  node_->unadvertise_service(service_prefix_ + "/set_command");
  node_->unadvertise_service(service_prefix_ + "/set_command_array");
  node_->unadvertise_service(service_prefix_ + "/get_command");
  node_->unadvertise_service(service_prefix_ + "/set_target");

  if(ros_cb_ && topic_name_.c_str())
  {
    std::cout << "unsub arm controller" << topic_name_ << std::endl;
    node_->unsubscribe(topic_name_, &ArmPositionControllerNode::setJointPosSingleHeadless_cb, this);
  }

  delete c_;
}

void ArmPositionControllerNode::update()
{
  c_->update();
}

bool ArmPositionControllerNode::initXml(mechanism::RobotState * robot, TiXmlElement * config)
{
  std::cout<<"LOADING ARMCONTROLLERNODE"<<std::endl;
  service_prefix_ = config->Attribute("name");
  std::cout<<"the service_prefix_ is "<<service_prefix_<<std::endl;
  // Parses subcontroller configuration
  if(c_->initXml(robot, config))
  {
    node_->advertise_service(service_prefix_ + "/set_command", &ArmPositionControllerNode::setJointPosHeadless, this);
    node_->advertise_service(service_prefix_ + "/set_command_array", &ArmPositionControllerNode::setJointPosSrv, this);
    node_->advertise_service(service_prefix_ + "/get_command", &ArmPositionControllerNode::getJointPosCmd, this);
    node_->advertise_service(service_prefix_ + "/set_target", &ArmPositionControllerNode::setJointPosTarget, this);

    ros_cb_ = config->FirstChildElement("listen_topic");
    if(ros_cb_)
    {
      topic_name_=ros_cb_->Attribute("name");
      if(!topic_name_.c_str())
      {
        std::cout<<" A listen _topic is present in the xml file but no name is specified\n";
        return false;
      }
      node_->subscribe(topic_name_, msg_, &ArmPositionControllerNode::setJointPosSingleHeadless_cb, this, 1);
      std::cout<<"Listening to topic: "<<topic_name_<<std::endl;
    }

    return true;
  }
  return false;
}

bool ArmPositionControllerNode::setJointPosSrv(pr2_mechanism_controllers::SetJointPosCmd::request &req,
                                   pr2_mechanism_controllers::SetJointPosCmd::response &resp)
{
  std::vector<double> pos;
  req.get_positions_vec(pos);
  c_->setJointPosCmd(pos);
  return true;
}

void ArmPositionControllerNode::setJointPosArray(const std::vector<double> & joint_pos)
{
  c_->setJointPosCmd(joint_pos);
}

//bool ArmPositionControllerNode::getJointPosCmd(pr2_mechanism_controllers::GetJointPosCmd::request &req,
//                                   pr2_mechanism_controllers::GetJointPosCmd::response &resp)
//{
//  c_->getJointPosCmd(resp);
//  return true;
//}


bool ArmPositionControllerNode::setJointPosSingle(const pr2_mechanism_controllers::JointPosCmd & cmd)
{
  for(unsigned int i=0;i<cmd.get_positions_size();++i)
    std::cout<<cmd.positions[i]<<' ';
  std::cout<<std::flush;
  c_->setJointPosCmd(cmd);
  int ticks=0;
  static const double interval=1e-2;
  const int max_ticks = int(cmd.timeout/interval);
  ros::Duration d=ros::Duration().fromSec(interval);
  while(!(c_->goalAchieved() ||  ticks >= max_ticks))
  {
    d.sleep();
    ticks++;
  }
  if(ticks>=max_ticks)
  {
    std::cout<<" failed"<<std::endl;
    return false;
  }
  std::cout<<" reached"<<std::endl;
  return true;
}

bool ArmPositionControllerNode::setJointPosSingleHeadless(pr2_mechanism_controllers::JointPosCmd & cmd)
{
  std::cout<<"Implementing callback"<<std::endl;

  // msg
  std::cout<<"waypoint "<<std::flush;
  for(unsigned int i=0;i<cmd.get_positions_size();++i)
    std::cout << cmd.names[i] << "==" << cmd.positions[i] <<' ';
  std::cout<<std::flush;
  std::cout<<" headless"<<std::endl;

  // Removes any error margin info so that the controller returns true on target reached and frees any blocking call
  cmd.set_margins_size(cmd.get_positions_size());
  for(unsigned int i=0;i<cmd.get_positions_size();++i)
    cmd.margins[i]=-1;

  c_->setJointPosCmd(cmd);
  return true;
}

bool ArmPositionControllerNode::setJointPosHeadless(pr2_mechanism_controllers::SetJointTarget::request &req,
                  pr2_mechanism_controllers::SetJointTarget::response &resp)
{
  if(req.get_positions_size()!=1)
  {
    std::cout<<"setting a target in headless mode requires only one configuration in the list\n";
    return false;
  }
  setJointPosSingleHeadless(req.positions[0]);
  return true;
}

bool ArmPositionControllerNode::setJointPosTarget(pr2_mechanism_controllers::SetJointTarget::request &req,
                  pr2_mechanism_controllers::SetJointTarget::response &resp)
{
  bool reached=true;
  for(unsigned int i=0;i<req.get_positions_size();++i)
    reached = reached&&setJointPosSingle(req.positions[i]);
//  std::vector<double> end_pos;
//  c_->getCurrentConfiguration(end_pos);
//  resp.set_end_positions_vec(end_pos);
  return reached;

//   const pr2_mechanism_controllers::JointPosCmd cmd=req.positions;
//   c_->setJointPosCmd(cmd);
//   int ticks=0;
//   const int max_ticks = int(100*cmd.timeout);
//   ros::Duration d=ros::Duration(0,1000000);
//   while(!(c_->goal_achieved_ ||  ticks >= max_ticks))
//   {
//     d.sleep();
//     ticks++;
//   }
//   //TODO: advertise end position
//   std::vector<double> end_pos;
//   c_->getCurrentConfiguration(end_pos);
//   resp.set_end_positions_vec(end_pos);
//   if(ticks>=max_ticks)
//     return false;
//
//   return true;

}

bool ArmPositionControllerNode::getJointPosCmd(pr2_mechanism_controllers::GetJointPosCmd::request &req,
                    pr2_mechanism_controllers::GetJointPosCmd::response &resp)
{
  pr2_mechanism_controllers::JointPosCmd cmd;
  c_->getJointPosCmd(cmd);
  resp.command = cmd;
  return true;
}

void ArmPositionControllerNode::setJointPosSingleHeadless_cb()
{
  setJointPosSingleHeadless(msg_);
}



