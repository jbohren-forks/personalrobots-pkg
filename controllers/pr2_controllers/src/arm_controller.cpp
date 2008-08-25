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

#include "pr2_controllers/arm_controller.h"

using namespace controller;
using namespace std;

ROS_REGISTER_CONTROLLER(ArmDummyController)

    
ArmDummyController::ArmDummyController()
{
  cout<<"new dummy"<<endl;
}

ArmDummyController::~ArmDummyController()
{
  // Assumes the joint controllers are owned by this class.
  for(unsigned int i=0; i<joint_position_controllers_.size();++i)
    delete joint_position_controllers_[i];
  cout<<"deleted dummy"<<endl;
}

bool ArmDummyController::initXml(mechanism::Robot * robot, TiXmlElement * config)
{
  robot_ = robot;
  TiXmlElement *elt = config->FirstChildElement("controller");
  while (elt)
  {
    JointPositionController * jpc = new JointPositionController();
    std::cout<<elt->Attribute("type")<<elt->Attribute("name")<<std::endl;
    assert(static_cast<std::string>(elt->Attribute("type")) == std::string("JointPositionController"));
    joint_position_controllers_.push_back(jpc);
    if(!jpc->initXml(robot_, elt))
      return false;
    
    elt = elt->NextSiblingElement("controller");
  }
  goals_.resize(joint_position_controllers_.size());
  goals_rt_.resize(joint_position_controllers_.size());
  
  cout<<"CONFIGURE DUMMY "<<joint_position_controllers_.size()<<endl;
  return true;
}

void ArmDummyController::setCommand(pr2_controllers::SetArmCommand::request &req)
{
  cout<<"SET COMMANDS"<<endl;
  arm_controller_lock_.lock();
  std::cout<<req.get_positions_size()<<std::endl;
  std::cout<<goals_.size()<<std::endl;
  assert(req.get_positions_size() == goals_.size());
  req.get_positions_vec(goals_);
  arm_controller_lock_.unlock();
}

void ArmDummyController::getCommand(                               pr2_controllers::GetArmCommand::response &resp)
{
  arm_controller_lock_.lock();
  resp.set_positions_size(goals_.size());
  assert(resp.get_positions_size() == goals_.size()); // TODO:Check intended behaviour here.
  resp.set_positions_vec(goals_);
  arm_controller_lock_.unlock();
}

void ArmDummyController::getCurrentConfiguration(std::vector<double> &vec)
{
  //TODO: warning: this read is not safe, since we cannot lock the realtime loop
  // A more complex mechanism is necessary here
  arm_controller_lock_.lock();
  assert(vec.size() == joint_position_controllers_.size());
  for(unsigned int i=0; i<joint_position_controllers_.size(); ++i)
  {
    vec[i] = joint_position_controllers_[i]->getMeasuredPosition();
  }
  arm_controller_lock_.unlock();
}

void ArmDummyController::update(void)
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
}

void ArmDummyController::updateJointControllers(void)
{
  for(unsigned int i=0;i<goals_rt_.size();++i)
    joint_position_controllers_[i]->update();
}

//------ Arm controller node --------

ROS_REGISTER_CONTROLLER(ArmControllerNode)

ArmControllerNode::ArmControllerNode()
  : Controller()
{
  std::cout<<"Controller node created"<<endl;
  c_ = new ArmDummyController();
}

ArmControllerNode::~ArmControllerNode()
{
  delete c_;
}

void ArmControllerNode::update()
{
  c_->update();
}

bool ArmControllerNode::initXml(mechanism::Robot * robot, TiXmlElement * config)
{
  std::cout<<"LOADING ARMCONTROLLERNODE"<<std::endl;
  ros::node * const node = ros::node::instance();
  string prefix = config->Attribute("name");
  
  // Parses controller configuration.
  std::string kdl_chain_name="";
  TiXmlElement *j = config->FirstChildElement("map");
  j = j->FirstChildElement("elem");
  assert(j);
  while(j!=NULL)
  {
    std::cout<<j->Attribute("key")<<std::flush;
    if(j->Attribute("key")==std::string("kdl_chain_name"))
    {
      std::cout<<"XX"<<std::flush;
      kdl_chain_name = j->GetText();
    }
    j = j->NextSiblingElement("elem");
  }
  std::cout<<"ARM = "<<kdl_chain_name<<std::endl;
  
  // Parses kinematics description
  std::string pr2Contents;
  node->get_param("robotdesc/pr2", pr2Contents);
  pr2_kin_.loadString(pr2Contents.c_str());
  arm_chain_ = pr2_kin_.getSerialChain(kdl_chain_name.c_str());
      
  assert(arm_chain_);
  
  // Parses subcontroller configuration
  if(c_->initXml(robot, config))
  {
    node->advertise_service(prefix + "/set_command", &ArmControllerNode::setCommand, this);
    node->advertise_service(prefix + "/get_command", &ArmControllerNode::getCommand, this);
    node->advertise_service(prefix + "/set_cartesian_pos", &ArmControllerNode::setCommandCP, this);
    node->advertise_service(prefix + "/get_cartesian_pos", &ArmControllerNode::getCP, this);
    return true;
  }
  return false;  
}

bool ArmControllerNode::setCommand(pr2_controllers::SetArmCommand::request &req,
                                   pr2_controllers::SetArmCommand::response &resp)
{
  c_->setCommand(req);
  return true;
}

bool ArmControllerNode::getCommand(pr2_controllers::GetArmCommand::request &req,
                                   pr2_controllers::GetArmCommand::response &resp)
{
  c_->getCommand(resp);
  return true;
}


bool ArmControllerNode::setCommandCP(pr2_controllers::SetArmCartesianPos::request &req,
                  pr2_controllers::SetArmCartesianPos::response &resp)
{
  KDL::Frame targetFrame;
  targetFrame.p = KDL::Vector(req.x, req.y, req.z);
  KDL::Rotation targetRot;
  targetRot = targetRot.RPY(req.roll, req.pitch, req.yaw);
  targetFrame.M = targetRot;
  
  
  // Stores the current state in a KDL frame in a realtime safe way
  const int size = arm_chain_->num_joints_;
  KDL::JntArray cur_cfg = KDL::JntArray(size);
  std::vector<double> cur_reads(size);
  c_->getCurrentConfiguration(cur_reads);
  for(int i=0;i<size;++i)
    cur_cfg(i) = cur_reads[i];
  KDL::Frame cur_frame;
  arm_chain_->computeFK(cur_cfg, cur_frame);
  
  // Setting guess of inverse kinematics
  for(int i=0;i<size;++i)
    (*arm_chain_->q_IK_guess)(i) = cur_cfg(i);  
  // Inverse kinematics:
  KDL::JntArray target_cfg = KDL::JntArray(size);
  if(!arm_chain_->computeIK(targetFrame))
    return false;
  target_cfg = *(arm_chain_->q_IK_result);
  std::cout<<"TARGET\n"<<target_cfg<<std::endl;
  
  
  //TODO: check IK result
  if(!arm_chain_->computeFK(target_cfg, targetFrame))
    return false;
  std::cout<<"Kinematics seems correct!"<<std::endl;
  
  // Sends commands to the controllers
  pr2_controllers::SetArmCommand::request commands;
  commands.set_positions_size(size);
  for(int i=0;i<size;++i)
    commands.positions[i] = target_cfg(i);
  c_->setCommand(commands);
  return true;
}

bool ArmControllerNode::getCP(pr2_controllers::GetArmCartesianPos::request &req,
                                     pr2_controllers::GetArmCartesianPos::response &resp)
{
  std::cout<<"Called"<<std::endl;
  // Stores the current state in a KDL frame in a realtime safe way
  const int size = arm_chain_->num_joints_;
  KDL::JntArray cur_cfg = KDL::JntArray(size);
  std::vector<double> cur_reads(size);
  c_->getCurrentConfiguration(cur_reads);
  for(int i=0;i<size;++i)
    cur_cfg(i) = cur_reads[i];
  KDL::Frame cur_frame;
  arm_chain_->computeFK(cur_cfg, cur_frame);
  resp.x= cur_frame.p.x();
  resp.y= cur_frame.p.y();
  resp.z= cur_frame.p.z();
  cur_frame.M.GetRPY(resp.roll, resp.pitch, resp.yaw);
  return true;
}