/*
 *  rosgazebo
 *  Copyright (c) 2008, Willow Garage, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <rosControllers/RosJointController.h>

#include <cassert>
#include <iostream>

using namespace controller;
using ros::node;

RosJointController::RosJointController(controller::JointController *jointc, std::string jointName) : 
    jc(jointc),
    mJointName(jointName)
{
  assert(jointc);
  // The user did not set a name for the joint, let's set it
  if(jointc->getName() == "")
    jc->setName(jointName);
  mCmdBusName=mJointName+std::string("_cmd");
  mStateBusName=mJointName+std::string("_state");
  // Actually, there is a static poitner to the process's ros node:
  publisherNode = ros::node::instance();

}


RosJointController::RosJointController(controller::JointController *jointc) : 
    jc(jointc)
{
  mCmdBusName=mJointName+std::string("_cmd");
  mStateBusName=mJointName+std::string("_state");
  // Actually, there is a static poitner to the process's ros node:
  publisherNode = ros::node::instance();
  assert(jointc);
  mJointName = jointc->getName();

}


int RosJointController::AdvertiseSubscribeMessages()
{
  if(!publisherNode)
    publisherNode = ros::node::instance();  
  if(publisherNode)
  {
    publisherNode->advertise<rosControllers::RotaryJointState>(mStateBusName);
    std::cout<<"Publishing "<<mStateBusName<<std::endl;
  }
  else
    std::cout<<"No ROS node for "<<mStateBusName<<std::endl;
  return(0);
}

RosJointController::~RosJointController()
{
}


/*void
RosJointController::init(controller::JointController *jc)
{
  assert(jc);
  //FIXME: the joint controller does not initialize its name properly
//   assert(jc->name() == mJointName);
  this->jc = jc;
}*/


void
RosJointController::Update()
{
  this->lock.lock();
  // get the state from the joint and advertise it
  //TODO: add information from the pid controller
  jointStateMsg.ControlMode = jc->getMode();
  jointStateMsg.JointName = mJointName;
  // FIXME: does this function change the state of the controller?
  jointStateMsg.Saturated = jc->checkForSaturation();
  jc->getParam(std::string("PGain"), &(jointStateMsg.PGain));
  jc->getParam("IGain", &(jointStateMsg.IGain));
  jc->getParam("DGain", &(jointStateMsg.DGain));
  jc->getParam("IMax", &(jointStateMsg.IMax));
  jc->getParam("IMin", &(jointStateMsg.IMin));
  //TODO: add Time, SaturationEffort, MaxEffort from the joint
  
  //The commands:
  jc->getTorqueCmd(&(jointStateMsg.TorqueCmd));
  jc->getTorqueAct(&(jointStateMsg.TorqueAct));
  jc->getPosCmd(&(jointStateMsg.PosCmd));
  jc->getPosAct(&(jointStateMsg.PosAct));
  jc->getVelCmd(&(jointStateMsg.VelCmd));
  jc->getVelAct(&(jointStateMsg.VelAct));
  
  if(publisherNode)
    publisherNode->publish(mStateBusName, jointStateMsg);
  std::cout<<"Publishing "<<mStateBusName<<std::endl;
  this->lock.unlock();
}



