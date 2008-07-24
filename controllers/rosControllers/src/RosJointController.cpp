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

void
RosJointController::cmdReceived()
{
  assert(jc);
  std::cout<<"Received command:"<<velMsg.vx<<std::endl;
  // update the controller
  //FIXME: shouldn't we have a mutex to protect the joint controller?
  //FIXME: velMsg has 2 fields: is it the proper structure to use here?
  jc->setVelCmd(velMsg.vx);
}



RosJointController::RosJointController(std::string jointName) : 
    ros::node(jointName),
    tf(*this),
    jc(NULL),
    mJointName(jointName)
{
  mCmdBusName=mJointName+std::string("_cmd");
  mStateBusName=mJointName+std::string("_state");

}

int
RosJointController::advertiseSubscribeMessages()
{
  advertise<rosControllers::RotaryJointState>(mStateBusName);
  subscribe(mCmdBusName, velMsg, &RosJointController::cmdReceived);
  return(0);
}

RosJointController::~RosJointController()
{
}


void
RosJointController::init(controller::JointController *jc)
{
  assert(jc);
  assert(jc->getName() == mJointName);
  this->jc = jc;
}


void
RosJointController::update()
{
  this->lock.lock();
  // Get the state from the joint and advertise it
  // FIXME: how to express an enum in a message?
  jointStateMsg.ControlMode = jc->getMode();
  jointStateMsg.JointName = mJointName;
  // FIXME: does this function change the state of the controller?
  jointStateMsg.Saturated = jc->checkForSaturation();
  jc->getParam(std::string("PGain"), &(jointStateMsg.PGain));
//   jc->getParam("IGain", &(jointStateMsg.IGain));
//   jc->getParam("DGain", &(jointStateMsg.DGain));
//   jc->getParam("IMax", &(jointStateMsg.IMax));
//   jc->getParam("IMin", &(jointStateMsg.IMin));
  //TODO: add Time, SaturationEffort, MaxEffort from the joint
  
  //The commands:
  jc->getTorqueCmd(&(jointStateMsg.TorqueCmd));
  jc->getTorqueAct(&(jointStateMsg.TorqueAct));
  jc->getPosCmd(&(jointStateMsg.PosCmd));
  jc->getPosAct(&(jointStateMsg.PosAct));
  jc->getVelCmd(&(jointStateMsg.VelCmd));
  jc->getVelAct(&(jointStateMsg.VelAct));
  
  publish(mStateBusName, jointStateMsg);
  this->lock.unlock();
}



