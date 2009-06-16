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

/** \author Ioan Sucan */

#include "planning_environment/planning_monitor.h"
#include "planning_environment/kinematic_state_constraint_evaluator.h"

void planning_environment::PlanningMonitor::setPathConstraints(const motion_planning_msgs::KinematicConstraints &kc)
{
    kcPath_ = kc;
    transformConstraintsToFrame(kcPath_, getFrameId());
}

void planning_environment::PlanningMonitor::setGoalConstraints(const motion_planning_msgs::KinematicConstraints &kc)
{
    kcGoal_ = kc;
    transformConstraintsToFrame(kcPath_, getFrameId());
}

void planning_environment::PlanningMonitor::transformConstraintsToFrame(motion_planning_msgs::KinematicConstraints &kc, const std::string &target) const
{
    for (unsigned int i = 0; i < kc.pose_constraint.size() ; ++i)
	tf_->transformPose(target, kc.pose_constraint[i].pose, kc.pose_constraint[i].pose);
    
    // if there are any floating or planar joints, transform them
    if (getKinematicModel()->getModelInfo().planarJoints.size() > 0 || getKinematicModel()->getModelInfo().floatingJoints.size() > 0)
    {
	for (unsigned int i = 0; i < kc.joint_constraint.size() ; ++i)
	    transformJoint(kc.joint_constraint[i].joint_name, 0, kc.joint_constraint[i].value, kc.joint_constraint[i].header, target);
    }
}

void planning_environment::PlanningMonitor::transformPathToFrame(motion_planning_msgs::KinematicPath &kp, const std::string &target) const
{    
    // if there are no planar of floating transforms, there is nothing to do
    if (getKinematicModel()->getModelInfo().planarJoints.empty() && getKinematicModel()->getModelInfo().floatingJoints.empty())
    {
	kp.header.frame_id = target;
	return;
    }
    
    roslib::Header updatedHeader = kp.header;

    // transform start state
    std::vector<planning_models::KinematicModel::Joint*> joints;
    getKinematicModel()->getJoints(joints);
    unsigned int u = 0;
    for (unsigned int i = 0 ; i < joints.size() ; ++i)
	if (joints[i]->usedParams > 0)
	{
	    roslib::Header header = kp.header;
	    transformJoint(joints[i]->name, u, kp.start_state.vals, header, target);
	    updatedHeader = header;	    
	    u += joints[i]->usedParams;
	}


    // transform the rest of the states

    // get the joints this path is for
    joints.resize(kp.names.size());
    for (unsigned int j = 0 ; j < joints.size() ; ++j)
	joints[j] = getKinematicModel()->getJoint(kp.names[j]);
    
    // iterate through the states
    for (unsigned int i = 0 ; i < kp.states.size() ; ++i)
    {
	unsigned int u = 0;
	for (unsigned int j = 0 ; j < joints.size() ; ++j)
	{
	    roslib::Header header = kp.header;
	    transformJoint(joints[j]->name, u, kp.states[i].vals, header, target);
	    updatedHeader = header;
	    u += joints[j]->usedParams;
	}
    }
    
    kp.header = updatedHeader;
}

void planning_environment::PlanningMonitor::transformJointToFrame(motion_planning_msgs::KinematicJoint &kj, const std::string &target) const
{
    transformJoint(kj.joint_name, 0, kj.value, kj.header, target);
}

void planning_environment::PlanningMonitor::transformJoint(const std::string &name, unsigned int index, std::vector<double> &params, roslib::Header &header, const std::string& target) const
{
    // planar joints and floating joints may need to be transformed 
    planning_models::KinematicModel::Joint *joint = getKinematicModel()->getJoint(name);
    
    if (dynamic_cast<planning_models::KinematicModel::PlanarJoint*>(joint))
    {
	tf::Stamped<tf::Pose> pose;
	pose.setOrigin(btVector3(params[index + 0], params[index + 1], 0.0));
	pose.getBasis().setEulerYPR(params[index + 2], 0.0, 0.0);
	pose.stamp_ = header.stamp;
	pose.frame_id_ = header.frame_id;
	tf_->transformPose(target, pose, pose);
	params[index + 0] = pose.getOrigin().getX();
	params[index + 1] = pose.getOrigin().getY();
	btScalar dummy;
	pose.getBasis().getEulerYPR(params[index + 2], dummy, dummy);
	header.stamp = pose.stamp_;
	header.frame_id = pose.frame_id_;
    }
    else
    if (dynamic_cast<planning_models::KinematicModel::FloatingJoint*>(joint))
    {
	tf::Stamped<tf::Pose> pose;
	pose.setOrigin(btVector3(params[index + 0], params[index + 1], params[index + 2]));
	btQuaternion q(params[index + 3], params[index + 4], params[index + 5], params[index + 6]);
	pose.setRotation(q);
	pose.stamp_ = header.stamp;
	pose.frame_id_ = header.frame_id;
	tf_->transformPose(target, pose, pose);
	params[index + 0] = pose.getOrigin().getX();
	params[index + 1] = pose.getOrigin().getY();
	params[index + 2] = pose.getOrigin().getZ();
	q = pose.getRotation();
	params[index + 3] = q.getX();
	params[index + 4] = q.getY();
	params[index + 5] = q.getZ();
	params[index + 6] = q.getW();
	header.stamp = pose.stamp_;
	header.frame_id = pose.frame_id_;
    }   
    else
	header.frame_id = target;
}

bool planning_environment::PlanningMonitor::isStateValidOnPath(const planning_models::KinematicModel::StateParams *state) const
{
    getEnvironmentModel()->lock();
    getKinematicModel()->lock();
    
    // figure out the poses of the robot model
    getKinematicModel()->computeTransforms(state->getParams());
    // update the collision space
    getEnvironmentModel()->updateRobotModel();

    // check for collision
    bool valid = !getEnvironmentModel()->isCollision();
    
    if (valid)
    {	    
	KinematicConstraintEvaluatorSet ks;
	ks.add(getKinematicModel(), kcPath_.joint_constraint);
	ks.add(getKinematicModel(), kcPath_.pose_constraint);
	valid = ks.decide(state->getParams());
    }
    
    getKinematicModel()->unlock();
    getEnvironmentModel()->unlock();
    
    return valid;
}

bool planning_environment::PlanningMonitor::isStateValidAtGoal(const planning_models::KinematicModel::StateParams *state) const
{   
    getEnvironmentModel()->lock();
    getKinematicModel()->lock();
    
    // figure out the poses of the robot model
    getKinematicModel()->computeTransforms(state->getParams());
    // update the collision space
    getEnvironmentModel()->updateRobotModel();
    
    // check for collision
    bool valid = !getEnvironmentModel()->isCollision();
    
    if (valid)
    {	    
	KinematicConstraintEvaluatorSet ks;
	ks.add(getKinematicModel(), kcPath_.joint_constraint);
	ks.add(getKinematicModel(), kcPath_.pose_constraint);
	ks.add(getKinematicModel(), kcGoal_.joint_constraint);
	ks.add(getKinematicModel(), kcGoal_.pose_constraint);
	valid = ks.decide(state->getParams());
    }
    
    getKinematicModel()->unlock();
    getEnvironmentModel()->unlock();
    
    return valid;    
}


bool planning_environment::PlanningMonitor::isPathValid(const motion_planning_msgs::KinematicPath &path) const
{
    if (path.header.frame_id != getFrameId())
    {
	motion_planning_msgs::KinematicPath pathT = path;
	transformPathToFrame(pathT, getFrameId());
	return isPathValidAux(pathT);
    }
    else
	return isPathValidAux(path);
}

bool planning_environment::PlanningMonitor::isPathValidAux(const motion_planning_msgs::KinematicPath &path) const
{    
    planning_models::KinematicModel::StateParams *sp = getKinematicModel()->newStateParams();
    sp->setParams(path.start_state.vals);

    KinematicConstraintEvaluatorSet ks;
    ks.add(getKinematicModel(), kcPath_.joint_constraint);
    ks.add(getKinematicModel(), kcPath_.pose_constraint);
    
    getEnvironmentModel()->lock();
    getKinematicModel()->lock();
    
    // figure out the poses of the robot model
    getKinematicModel()->computeTransforms(sp->getParams());
    // update the collision space
    getEnvironmentModel()->updateRobotModel();
    
    bool valid = true;
    
    // get the joints this path is for
    std::vector<planning_models::KinematicModel::Joint*> joints(path.names.size());
    for (unsigned int j = 0 ; j < joints.size() ; ++j)
	joints[j] = getKinematicModel()->getJoint(path.names[j]);
    
    // check every state
    for (unsigned int i = 0 ; valid && i < path.states.size() ; ++i)
    {
	unsigned int u = 0;
	for (unsigned int j = 0 ; j < joints.size() ; ++j)
	{
	    /* copy the parameters that describe the joint */
	    std::vector<double> params;
	    for (unsigned int k = 0 ; k < joints[j]->usedParams ; ++k)
		params.push_back(path.states[i].vals[u + k]);
	    u += joints[j]->usedParams;
	    
	    /* set the parameters */
	    sp->setParamsJoint(params, joints[j]->name);
	}
	getKinematicModel()->computeTransforms(sp->getParams());
	getEnvironmentModel()->updateRobotModel();
	
	// check for collision
	valid = !getEnvironmentModel()->isCollision();
    
	// check for validity
	if (valid)
	    valid = ks.decide(sp->getParams());
    }

    // if we got to the last state, we also check the goal constraints
    if (valid)
    {
	ks.add(getKinematicModel(), kcGoal_.joint_constraint);
	ks.add(getKinematicModel(), kcGoal_.pose_constraint);
	valid = ks.decide(sp->getParams());
    }
    
    getKinematicModel()->unlock();
    getEnvironmentModel()->unlock();
    
    delete sp;
    return valid;
}
