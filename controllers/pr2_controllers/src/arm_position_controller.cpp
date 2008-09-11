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

#include "pr2_controllers/arm_position_controller.h"

using namespace controller;
using namespace std;

ROS_REGISTER_CONTROLLER(ArmPositionController)

ArmPositionController::ArmPositionController()
{
//   cout<<"new dummy"<<endl;
}

ArmPositionController::~ArmPositionController()
{
  // Assumes the joint controllers are owned by this class.
  for(unsigned int i=0; i<joint_position_controllers_.size();++i)
    delete joint_position_controllers_[i];
//   cout<<"deleted dummy"<<endl;
}

bool ArmPositionController::initXml(mechanism::RobotState * robot, TiXmlElement * config)
{
//   cout<<"CONFIGURE DUMMY "<<joint_position_controllers_.size()<<endl;
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
  goals_rt_.resize(joint_position_controllers_.size());
  error_margins_.resize(joint_position_controllers_.size());
  
//   cout<<"CONFIGURE DUMMY "<<joint_position_controllers_.size()<<endl;
  return true;
}

void ArmPositionController::setJointPosCmd(std::vector<double> &req_goals_)
{
//   std::cout<<req_goals_.size()<< " = " <<goals_.size()<<std::endl;
  assert(req_goals_.size() == goals_.size());
  arm_controller_lock_.lock();
  for (uint i=0; i<req_goals_.size(); i++) goals_[i] = req_goals_[i];
  for (uint i=0; i<goals_.size(); ++i) error_margins_[i] = -1;
  goalAchieved = true;
  arm_controller_lock_.unlock();
}

void ArmPositionController::setJointPosCmd(pr2_controllers::SetJointPosCmd::request &req)
{
//   cout<<"SET COMMANDS"<<endl;
  arm_controller_lock_.lock();
  std::cout<<req.get_positions_size()<<std::endl;
  std::cout<<goals_.size()<<std::endl;
  assert(req.get_positions_size() == goals_.size());
  req.get_positions_vec(goals_);
  for (uint i=0; i<goals_.size(); ++i) error_margins_[i] = -1;
  goalAchieved = true;
  arm_controller_lock_.unlock();
}

void ArmPositionController::setJointPosCmd(const pr2_controllers::JointPosCmd & cmd)
{
//   cout<<"SET COMMANDS JT"<<endl;
  arm_controller_lock_.lock();
  
  for(uint i=0;i<error_margins_.size();++i)
    error_margins_[i]=-1;
  assert(cmd.get_names_size()==cmd.get_positions_size());
  for(uint i=0;i<cmd.get_names_size();++i)
  {
    const std::string & name = cmd.names[i];
    const int id = getJointControllerPosByName(name);
    if(id>=0)
    {
      goals_[id] = cmd.positions[i];
      error_margins_[id] = cmd.margins[i];
//       std::cout<<">"<<name<<"("<<id<<")"<<goals_[id]<<" "<<error_margins_[id]<<std::endl;
    }
  }
  goalAchieved = false;
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

void ArmPositionController::getJointPosCmd(pr2_controllers::GetJointPosCmd::response &resp)
{
  arm_controller_lock_.lock();
  resp.set_positions_size(goals_.size());
  resp.set_names_size(joint_position_controllers_.size());
  assert(resp.get_positions_size() == goals_.size()); // TODO:Check intended behaviour here.
  resp.set_positions_vec(goals_);
  for(unsigned int i=0;i<joint_position_controllers_.size();++i)
    resp.names[i] = joint_position_controllers_[i]->getJointName();
  arm_controller_lock_.unlock();
}

void ArmPositionController::getCurrentConfiguration(std::vector<double> &vec)
{
  //TODO: warning: this read is not safe, since we cannot lock the realtime loop
  // A more complex mechanism is necessary here
  vec.resize(joint_position_controllers_.size());
  arm_controller_lock_.lock();
  for(unsigned int i=0; i<joint_position_controllers_.size(); ++i)
  {
    vec[i] = joint_position_controllers_[i]->getMeasuredPosition();
  }
  arm_controller_lock_.unlock();
}

void ArmPositionController::update(void)
{
  if(arm_controller_lock_.trylock())
  {
    assert(goals_.size() == goals_rt_.size());
    for(unsigned int i=0; i<goals_.size(); ++i)
      goals_rt_[i] = goals_[i];
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
  if(goals_.size()==0)
    return;
  goalAchieved = true;
  for(unsigned int i=0;i<goals_rt_.size();++i)
      goalAchieved=goalAchieved&&(error_margins_[i]<=0||std::abs(joint_position_controllers_[i]->getMeasuredPosition()-goals_rt_[i])<error_margins_[i]);
}


//------ Arm controller node --------

ROS_REGISTER_CONTROLLER(ArmPositionControllerNode)

ArmPositionControllerNode::ArmPositionControllerNode()
  : Controller()
{
  std::cout<<"Controller node created"<<endl;
  c_ = new ArmPositionController();
}

ArmPositionControllerNode::~ArmPositionControllerNode()
{
  delete c_;
}

void ArmPositionControllerNode::update()
{
  c_->update();
}

bool ArmPositionControllerNode::initXml(mechanism::RobotState * robot, TiXmlElement * config)
{
  std::cout<<"LOADING ARMCONTROLLERNODE"<<std::endl;
  ros::node * const node = ros::node::instance();
  string prefix = config->Attribute("name");
  
  // Parses subcontroller configuration
  if(c_->initXml(robot, config))
  {
    node->advertise_service(prefix + "/set_command", &ArmPositionControllerNode::setJointPosCmd, this);
    node->advertise_service(prefix + "/get_command", &ArmPositionControllerNode::getJointPosCmd, this);
    
    node->advertise_service(prefix + "/set_target", &ArmPositionControllerNode::jointPosCmd, this);

    return true;
  }
  return false;  
}

bool ArmPositionControllerNode::setJointPosCmd(pr2_controllers::SetJointPosCmd::request &req,
                                   pr2_controllers::SetJointPosCmd::response &resp)
{
  c_->setJointPosCmd(req);
  return true;
}

void ArmPositionControllerNode::setJointPosCmd(std::vector<double> &req_goals_)
{
  c_->setJointPosCmd(req_goals_);
}
bool ArmPositionControllerNode::getJointPosCmd(pr2_controllers::GetJointPosCmd::request &req,
                                   pr2_controllers::GetJointPosCmd::response &resp)
{
  c_->getJointPosCmd(resp);
  return true;
}

bool ArmPositionControllerNode::setSingleTargetCmd(const pr2_controllers::JointPosCmd & cmd)
{
  c_->setJointPosCmd(cmd);
  std::cout<<"waypoint"<<std::flush;
  int ticks=0;
  double interval=1e-2;
  const int max_ticks = int(cmd.timeout/interval);
  ros::Duration d=ros::Duration(interval);
  while(!(c_->goalAchieved ||  ticks >= max_ticks))
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

bool ArmPositionControllerNode::jointPosCmd(pr2_controllers::SetJointTarget::request &req,
                  pr2_controllers::SetJointTarget::response &resp)
{
  bool reached=true;
  for(unsigned int i=0;i<req.get_positions_size();++i)
    reached = reached&&setSingleTargetCmd(req.positions[i]);
  std::vector<double> end_pos;
  c_->getCurrentConfiguration(end_pos);
  resp.set_end_positions_vec(end_pos);
  return reached;

//   const pr2_controllers::JointPosCmd cmd=req.positions;
//   c_->setJointPosCmd(cmd);
//   int ticks=0;
//   const int max_ticks = int(100*cmd.timeout);
//   ros::Duration d=ros::Duration(0,1000000);
//   while(!(c_->goalAchieved ||  ticks >= max_ticks))
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


