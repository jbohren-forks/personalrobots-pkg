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

#include <rosControllers/RosJoint.h>

#include <cassert>
#include <iostream>

using namespace controller;
using namespace mechanism;
using ros::node;

 RosJoint::RosJoint(mechanism::Joint *j, std::string jointName) : 
    joint(j),
    mJointName(jointName)
    ,
    hw_(NULL)
{
  assert(joint);
  // The user did not set a name for the joint, let's set it
//   if(joint->name == "")
//     joint->name = jointName.c_str();
  mCmdBusName=mJointName+std::string("_cmd");
  mStateBusName=mJointName+std::string("_JOINT");
  // Actually, there is a static poitner to the process's ros node:
  publisherNode = ros::node::instance();
}

 RosJoint::RosJoint(PR2::PR2HW * hw, int joint_id, std::string jointName):
    joint(NULL),
    mJointName(jointName)
    ,
    hw_(hw),
    joint_id_(joint_id)
{
  assert(hw_);
  // The user did not set a name for the joint, let's set it
//   if(joint->name == "")
//     joint->name = jointName.c_str();
  mCmdBusName=mJointName+std::string("_cmd");
  mStateBusName=mJointName+std::string("_JOINT");
  // Actually, there is a static poitner to the process's ros node:
  publisherNode = ros::node::instance();
}


int RosJoint::AdvertiseSubscribeMessages()
{
  if(!publisherNode)
    publisherNode = ros::node::instance();  
  if(publisherNode)
  {
    publisherNode->advertise<rosControllers::JointState>(mStateBusName);
    std::cout<<"Publishing "<<mStateBusName<<std::endl;
  }
  else
    std::cout<<"No ROS node for "<<mStateBusName<<std::endl;
  return(0);
}

RosJoint::~RosJoint()
{
}

void
RosJoint::Update()
{
  this->lock.lock();
  // get the state from the joint and advertise it
  //TODO: add information from the pid controller
//   jointStateMsg.ControlMode = jc->getMode();
  jointStateMsg.JointName = mJointName;
  // FIXME: does this function change the state of the controller?
//   jointStateMsg.Saturated = jc->checkForSaturation();
  assert(joint||hw_);
  if(joint)
  {
    jointStateMsg.Initialized = joint->initialized;
    jointStateMsg.AppliedEffort = joint->appliedEffort;
    jointStateMsg.CommandedEffort = joint->commandedEffort;
    jointStateMsg.Pos = joint->position;
    jointStateMsg.Vel = joint->velocity;
  }
  
  if(hw_)
  {
    jointStateMsg.Initialized = hw_->jointData[joint_id_].controlMode;
    jointStateMsg.AppliedEffort = hw_->jointData[joint_id_].actualEffectorForce;
    jointStateMsg.CommandedEffort = hw_->jointData[joint_id_].cmdEffectorForce;
    jointStateMsg.Pos = hw_->jointData[joint_id_].actualPosition;
    jointStateMsg.Vel = hw_->jointData[joint_id_].actualSpeed;
  }

  if(publisherNode)
    publisherNode->publish(mStateBusName, jointStateMsg);
  std::cout<<"Publishing "<<mStateBusName<<std::endl;
  this->lock.unlock();
}



