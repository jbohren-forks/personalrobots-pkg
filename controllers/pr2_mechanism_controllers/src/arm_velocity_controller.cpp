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

#include "pr2_mechanism_controllers/arm_velocity_controller.h"

using namespace controller;
using namespace std;

ROS_REGISTER_CONTROLLER(ArmVelocityController)

ArmVelocityController::ArmVelocityController()
{
  cout<<"new dummy"<<endl;
}

ArmVelocityController::~ArmVelocityController()
{
  // Assumes the joint controllers are owned by this class.
  for(unsigned int i=0; i<joint_velocity_controllers_.size();++i)
    delete joint_velocity_controllers_[i];
  cout<<"deleted dummy"<<endl;
}

bool ArmVelocityController::initXml(mechanism::RobotState * robot, TiXmlElement * config)
{
  robot_ = robot;
  TiXmlElement *elt = config->FirstChildElement("controller");
  while (elt)
  {
    JointVelocityController * jpc = new JointVelocityController();
    std::cout<<elt->Attribute("type")<<elt->Attribute("name")<<std::endl;
    assert(static_cast<std::string>(elt->Attribute("type")) == std::string("JointVelocityController"));
    joint_velocity_controllers_.push_back(jpc);
    if(!jpc->initXml(robot_, elt))
      return false;

    elt = elt->NextSiblingElement("controller");
  }
  goals_.resize(joint_velocity_controllers_.size());
  goals_rt_.resize(joint_velocity_controllers_.size());

  cout<<"CONFIGURE DUMMY "<<joint_velocity_controllers_.size()<<endl;
  return true;
}

void ArmVelocityController::setJointVelCmd(pr2_mechanism_controllers::SetJointVelCmd::request &req)
{
  cout<<"SET COMMANDS"<<endl;
  arm_controller_lock_.lock();
  std::cout<<req.get_velocity_size()<<std::endl;
  std::cout<<goals_.size()<<std::endl;
  assert(req.get_velocity_size() == goals_.size());
  req.get_velocity_vec(goals_);
  arm_controller_lock_.unlock();
}

void ArmVelocityController::getJointVelCmd(pr2_mechanism_controllers::GetJointVelCmd::response &resp)
{
  arm_controller_lock_.lock();
  resp.set_velocity_size(goals_.size());
  assert(resp.get_velocity_size() == goals_.size()); // TODO:Check intended behaviour here.
  resp.set_velocity_vec(goals_);
  arm_controller_lock_.unlock();
}

void ArmVelocityController::setJointGains(const pr2_mechanism_controllers::SetJointGains::request &req)
{
  cout<<"SET GAINS"<<endl;
  arm_controller_lock_.lock();
  JointVelocityController *jpc = getJointControllerByName(req.name);
  if(jpc)
    jpc->setGains(req.p,req.i,req.d,req.i_min,req.i_max);
  arm_controller_lock_.unlock();
}

void ArmVelocityController::getJointGains(pr2_mechanism_controllers::GetJointGains::response &req)
{
  cout<<"GET GAINS"<<endl;
  arm_controller_lock_.lock();
  JointVelocityController *jpc = getJointControllerByName(req.name);
  if(jpc)
  {
    jpc->getGains(req.p,req.i,req.d,req.i_max,req.i_min);
  }
  arm_controller_lock_.unlock();
}

controller::JointVelocityController* ArmVelocityController::getJointControllerByName(std::string name)
{
  for(int i=0; i< (int) joint_velocity_controllers_.size(); i++)
  {
    if(joint_velocity_controllers_[i]->getJointName() == name)
    {
      return joint_velocity_controllers_[i];
    }
  }
    return NULL;
}

void ArmVelocityController::getCurrentConfiguration(std::vector<double> &vec)
{
  //TODO: warning: this read is not safe, since we cannot lock the realtime loop
  // A more complex mechanism is necessary here
  arm_controller_lock_.lock();
  assert(vec.size() == joint_velocity_controllers_.size());
  for(unsigned int i=0; i<joint_velocity_controllers_.size(); ++i)
  {
    vec[i] = joint_velocity_controllers_[i]->joint_state_->velocity_;
  }
  arm_controller_lock_.unlock();
}

void ArmVelocityController::update(void)
{
  if(arm_controller_lock_.trylock())
  {
    assert(goals_.size() == goals_rt_.size());
    for(unsigned int i=0; i<goals_.size(); ++i)
      goals_rt_[i] = goals_[i];
    arm_controller_lock_.unlock();
  }

  for(unsigned int i=0;i<goals_rt_.size();++i)
  {
    joint_velocity_controllers_[i]->setCommand(goals_rt_[i]);
  }
  updateJointControllers();
}

void ArmVelocityController::updateJointControllers(void)
{
  for(unsigned int i=0;i<goals_rt_.size();++i)
    joint_velocity_controllers_[i]->update();
}

//------ Arm controller node --------

ROS_REGISTER_CONTROLLER(ArmVelocityControllerNode)

ArmVelocityControllerNode::ArmVelocityControllerNode()
  : Controller()
{
  std::cout<<"Controller node created"<<endl;
  c_ = new ArmVelocityController();
}

ArmVelocityControllerNode::~ArmVelocityControllerNode()
{
  delete c_;
}

void ArmVelocityControllerNode::update()
{
  c_->update();
}

bool ArmVelocityControllerNode::initXml(mechanism::RobotState * robot, TiXmlElement * config)
{
  std::cout<<"LOADING ARMCONTROLLERNODE"<<std::endl;
  ros::Node * const node = ros::Node::instance();
  string prefix = config->Attribute("name");

  // Parses controller configuration.
  std::string kdl_chain_name="";
  TiXmlElement *j = config->FirstChildElement("map");
  j = j->FirstChildElement("elem");
  assert(j);
  while(j!=NULL)
  {
    if(j->Attribute("key")==std::string("kdl_chain_name"))
    {
      kdl_chain_name = j->GetText();
    }
    j = j->NextSiblingElement("elem");
  }

  // Parses kinematics description
  std::string pr2Contents;
  node->get_param("robotdesc/pr2", pr2Contents);
  while (!pr2_kin_.loadString(pr2Contents.c_str())) // retry if load fails
  {
    std::cout << "WARNING: waitig for robotdesc/pr2 xml string on param server.  run roslaunch send.xml or similar." << std::endl;
    usleep(100000);
  }
  arm_chain_ = pr2_kin_.getSerialChain(kdl_chain_name.c_str());

  assert(arm_chain_);

  // Parses subcontroller configuration
  if(c_->initXml(robot, config))
  {
    node->advertise_service(prefix + "/set_command", &ArmVelocityControllerNode::setJointVelCmd, this);
    node->advertise_service(prefix + "/get_command", &ArmVelocityControllerNode::getJointVelCmd, this);

    node->advertise_service(prefix + "/set_joint_gains", &ArmVelocityControllerNode::setJointGains, this);
    node->advertise_service(prefix + "/get_joint_gains", &ArmVelocityControllerNode::getJointGains, this);

    node->advertise_service(prefix + "/set_cartesian_vel", &ArmVelocityControllerNode::setCartesianVelCmd, this);
    node->advertise_service(prefix + "/get_cartesian_vel", &ArmVelocityControllerNode::getCartesianVelCmd, this);
    return true;
  }
  return false;
}



bool ArmVelocityControllerNode::setJointVelCmd(pr2_mechanism_controllers::SetJointVelCmd::request &req,
                                   pr2_mechanism_controllers::SetJointVelCmd::response &resp)
{
  c_->setJointVelCmd(req);
  return true;
}

bool ArmVelocityControllerNode::getJointVelCmd(pr2_mechanism_controllers::GetJointVelCmd::request &req,
                                   pr2_mechanism_controllers::GetJointVelCmd::response &resp)
{
  c_->getJointVelCmd(resp);
  return true;
}

bool ArmVelocityControllerNode::setJointGains(pr2_mechanism_controllers::SetJointGains::request &req,
                                   pr2_mechanism_controllers::SetJointGains::response &resp)
{
  c_->setJointGains(req);
  return true;
}

bool ArmVelocityControllerNode::getJointGains(pr2_mechanism_controllers::GetJointGains::request &req,
                                   pr2_mechanism_controllers::GetJointGains::response &resp)
{
  resp.name = req.name;
  c_->getJointGains(resp);
  return true;
}

bool ArmVelocityControllerNode::setCartesianVelCmd(pr2_mechanism_controllers::SetCartesianVelCmd::request &req,
                  pr2_mechanism_controllers::SetCartesianVelCmd::response &resp)
{
  const int size =       arm_chain_->num_joints_;

  KDL::Vector linear_vel = KDL::Vector(req.vx, req.vy, req.vz);
  KDL::Vector angular_vel = KDL::Vector(req.wx, req.wy, req.wz);
  KDL::Twist end_effector_twist = KDL::Twist(linear_vel,angular_vel);

  KDL::JntArray target_joint_vel = KDL::JntArray(size);
  KDL::JntArray cur_joint_pos = KDL::JntArray(size);

  std::vector<double> cur_reads(size);
  c_->getCurrentConfiguration(cur_reads);
  for(int i=0;i<size;++i)
    cur_joint_pos(i) = cur_reads[i];

  arm_chain_->computeDKInv(cur_joint_pos,end_effector_twist,target_joint_vel);
  std::cout << "TARGET VEL\n" << target_joint_vel << std::endl;

  // Sends commands to the controllers
  pr2_mechanism_controllers::SetJointVelCmd::request commands;
  commands.set_velocity_size(size);
  for(int i=0;i<size;++i)
    commands.velocity[i] = target_joint_vel(i);
  c_->setJointVelCmd(commands);

  return true;
}


bool ArmVelocityControllerNode::getCartesianVelCmd(pr2_mechanism_controllers::GetCartesianVelCmd::request &req, pr2_mechanism_controllers::GetCartesianVelCmd::response &resp)
{
  return true;
}
