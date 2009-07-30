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

#include "planning_environment/monitors/planning_monitor.h"
#include "planning_environment/util/kinematic_state_constraint_evaluator.h"
#include <boost/scoped_ptr.hpp>

void planning_environment::PlanningMonitor::loadParams(void)
{
    nh_.param<double>("~refresh_interval_collision_map", intervalCollisionMap_, 0.0);
    nh_.param<double>("~refresh_interval_kinematic_state", intervalState_, 0.0);
    nh_.param<double>("~refresh_interval_pose", intervalPose_, 0.0);
}

bool planning_environment::PlanningMonitor::isEnvironmentSafe(void) const
{
    if (!isMapUpdated(intervalCollisionMap_))
    {
        ROS_WARN("Planning is not safe: map not updated in the last %f seconds", intervalCollisionMap_);
        return false;
    }
  
    if (!isMechanismStateUpdated(intervalState_))
    {
        ROS_WARN("Planning is not safe: robot state not updated in the last %f seconds", intervalState_);
        return false;
    }  

    if (includePose_)
	if (!isPoseUpdated(intervalPose_))
	{
	    ROS_WARN("Planning is not safe: robot pose not updated in the last %f seconds", intervalPose_);
	    return false;
	}
    return true;
}

void planning_environment::PlanningMonitor::clearConstraints(void)
{
    kcPath_.joint_constraint.clear();
    kcPath_.pose_constraint.clear();
    kcGoal_.joint_constraint.clear();
    kcGoal_.pose_constraint.clear();
}

void planning_environment::PlanningMonitor::setPathConstraints(const motion_planning_msgs::KinematicConstraints &kc)
{
    kcPath_ = kc;
    transformConstraintsToFrame(kcPath_, getFrameId());
}

void planning_environment::PlanningMonitor::setGoalConstraints(const motion_planning_msgs::KinematicConstraints &kc)
{
    kcGoal_ = kc;
    transformConstraintsToFrame(kcGoal_, getFrameId());
}

bool planning_environment::PlanningMonitor::transformConstraintsToFrame(motion_planning_msgs::KinematicConstraints &kc, const std::string &target) const
{
    bool res = true;
    for (unsigned int i = 0; i < kc.pose_constraint.size() ; ++i)
    {
	bool ok = false;
	if (tf_->canTransform(target, kc.pose_constraint[i].pose.header.frame_id, kc.pose_constraint[i].pose.header.stamp, tfWait_))
	{
	    try
	    {
		tf_->transformPose(target, kc.pose_constraint[i].pose, kc.pose_constraint[i].pose);
		ok = true;
	    }
	    catch(...)
	    {
	    }
	}
	if (!ok)
	{
	    ROS_ERROR("Unable to transform pose constraint on link '%s' to frame '%s'", kc.pose_constraint[i].link_name.c_str(), target.c_str());
	    res = false;
	}
    }
    
    // if there are any floating or planar joints, transform them
    if (getKinematicModel()->getModelInfo().planarJoints.size() > 0 || getKinematicModel()->getModelInfo().floatingJoints.size() > 0)
    {
	for (unsigned int i = 0; i < kc.joint_constraint.size() ; ++i)
	    if (!transformJoint(kc.joint_constraint[i].joint_name, 0, kc.joint_constraint[i].value, kc.joint_constraint[i].header, target))
		res = false;
    }
    return res;
}

bool planning_environment::PlanningMonitor::transformPathToFrame(motion_planning_msgs::KinematicPath &kp, const std::string &target) const
{    
    // if there are no planar of floating transforms, there is nothing to do
    if (getKinematicModel()->getModelInfo().planarJoints.empty() && getKinematicModel()->getModelInfo().floatingJoints.empty())
    {
	kp.header.frame_id = target;
	return true;
    }
    
    roslib::Header updatedHeader = kp.header;
    updatedHeader.frame_id = target;
    
    // transform start state
    for (unsigned int i = 0 ; i < kp.start_state.size() ; ++i)
	if (!transformJointToFrame(kp.start_state[i], target))
	    return false;
    
    
    // transform the rest of the states

    // get the joints this path is for
    std::vector<planning_models::KinematicModel::Joint*> joints;
    joints.resize(kp.names.size());
    for (unsigned int j = 0 ; j < joints.size() ; ++j)
    {
	joints[j] = getKinematicModel()->getJoint(kp.names[j]);
	if (joints[j] == NULL)
	{
	    ROS_ERROR("Unknown joint '%s' found on path", kp.names[j].c_str());
	    return false;
	}
    }
    
    // iterate through the states
    for (unsigned int i = 0 ; i < kp.states.size() ; ++i)
    {
	unsigned int u = 0;
	for (unsigned int j = 0 ; j < joints.size() ; ++j)
	{
	    roslib::Header header = kp.header;
	    if (!transformJoint(joints[j]->name, u, kp.states[i].vals, header, target))
		return false;
	    updatedHeader = header;
	    u += joints[j]->usedParams;
	}
    }
    
    kp.header = updatedHeader;
    return true;
}

bool planning_environment::PlanningMonitor::transformJointToFrame(motion_planning_msgs::KinematicJoint &kj, const std::string &target) const
{
    return transformJoint(kj.joint_name, 0, kj.value, kj.header, target);
}

bool planning_environment::PlanningMonitor::transformJoint(const std::string &name, unsigned int index, std::vector<double> &params, roslib::Header &header, const std::string& target) const
{
    // planar joints and floating joints may need to be transformed 
    planning_models::KinematicModel::Joint *joint = getKinematicModel()->getJoint(name);
    if (joint == NULL)
    {
	ROS_ERROR("Unknown joint '%s'", name.c_str());
	return false;
    }
    else
	if (params.size() < index + joint->usedParams)
	{
	    ROS_ERROR("Insufficient parameters for joint '%s'", name.c_str());
	    return false;
	}
    
    if (dynamic_cast<planning_models::KinematicModel::PlanarJoint*>(joint))
    {
	tf::Stamped<tf::Pose> pose;
	pose.setOrigin(btVector3(params[index + 0], params[index + 1], 0.0));
	pose.getBasis().setEulerYPR(params[index + 2], 0.0, 0.0);
	pose.stamp_ = header.stamp;
	pose.frame_id_ = header.frame_id;
	bool ok = false;
	if (tf_->canTransform(target, pose.frame_id_, pose.stamp_, tfWait_))
	{
	    try
	    {
		tf_->transformPose(target, pose, pose);
		ok = true;
	    }
	    catch(...)
	    {
	    }
	}
	if (!ok)
	{
	    ROS_ERROR("Unable to transform planar joint '%s' to frame '%s'", name.c_str(), target.c_str());
	    return false;
	}
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
	bool ok = false;
	if (tf_->canTransform(target, pose.frame_id_, pose.stamp_, tfWait_))
	{
	    try
	    {
		tf_->transformPose(target, pose, pose);
		ok = true;
	    }
	    catch(...)
	    {
	    }
	}
	if (!ok)
	{
	    ROS_ERROR("Unable to transform floating joint '%s' to frame '%s'", name.c_str(), target.c_str());
	    return false;
	}
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
    return true;
}

bool planning_environment::PlanningMonitor::isStateCollisionFree(const planning_models::StateParams *state) const
{
    getEnvironmentModel()->lock();
    getKinematicModel()->lock();
    
    // figure out the poses of the robot model
    getKinematicModel()->computeTransforms(state->getParams());
    // update the collision space
    getEnvironmentModel()->updateRobotModel();

    // check for collision
    std::vector<collision_space::EnvironmentModel::Contact> contacts;
    bool valid = !getEnvironmentModel()->getCollisionContacts(contacts, 1);
    
    getKinematicModel()->unlock();
    getEnvironmentModel()->unlock();
    
    if (onCollisionContact_)
	for (unsigned int i = 0 ; i < contacts.size() ; ++i)
	    onCollisionContact_(contacts[i]);
    
    return valid;
}

bool planning_environment::PlanningMonitor::isStateValidOnPath(const planning_models::StateParams *state) const
{
    getEnvironmentModel()->lock();
    getKinematicModel()->lock();
    
    // figure out the poses of the robot model
    getKinematicModel()->computeTransforms(state->getParams());
    // update the collision space
    getEnvironmentModel()->updateRobotModel();

    // check for collision
    std::vector<collision_space::EnvironmentModel::Contact> contacts;
    bool valid = !getEnvironmentModel()->getCollisionContacts(contacts, 1);
    
    if (valid)
    {	    
	KinematicConstraintEvaluatorSet ks;
	ks.add(getKinematicModel(), kcPath_.joint_constraint);
	ks.add(getKinematicModel(), kcPath_.pose_constraint);
	valid = ks.decide(state->getParams());
    }
    
    getKinematicModel()->unlock();
    getEnvironmentModel()->unlock();

    if (onCollisionContact_)
	for (unsigned int i = 0 ; i < contacts.size() ; ++i)
	    onCollisionContact_(contacts[i]);

    return valid;
}

bool planning_environment::PlanningMonitor::isStateValidAtGoal(const planning_models::StateParams *state) const
{   
    getEnvironmentModel()->lock();
    getKinematicModel()->lock();
    
    // figure out the poses of the robot model
    getKinematicModel()->computeTransforms(state->getParams());
    // update the collision space
    getEnvironmentModel()->updateRobotModel();
    
    // check for collision
    std::vector<collision_space::EnvironmentModel::Contact> contacts;
    bool valid = !getEnvironmentModel()->getCollisionContacts(contacts, 1);
    
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
    
    if (onCollisionContact_)
	for (unsigned int i = 0 ; i < contacts.size() ; ++i)
	    onCollisionContact_(contacts[i]);
    
    return valid;    
}

int planning_environment::PlanningMonitor::closestStateOnPath(const motion_planning_msgs::KinematicPath &path, const planning_models::StateParams *state) const
{
    return closestStateOnPath(path, 0, path.states.size() - 1, state);
}

int planning_environment::PlanningMonitor::closestStateOnPath(const motion_planning_msgs::KinematicPath &path, unsigned int start, unsigned int end, const planning_models::StateParams *state) const
{
    if (end >= path.states.size())
	end = path.states.size() - 1;
    if (start > end)
	return -1;
    
    if (path.header.frame_id != getFrameId())
    {
	motion_planning_msgs::KinematicPath pathT = path;
	if (transformPathToFrame(pathT, getFrameId()))
	    return closestStateOnPathAux(pathT, start, end, state);
	else
	    return -1;
    }
    else
	return closestStateOnPathAux(path, start, end, state);  
}

int planning_environment::PlanningMonitor::closestStateOnPathAux(const motion_planning_msgs::KinematicPath &path, unsigned int start, unsigned int end, const planning_models::StateParams *state) const
{
    double dist = 0.0;
    int    pos  = -1;

    // get the joints this path is for
    std::vector<planning_models::KinematicModel::Joint*> joints(path.names.size());
    for (unsigned int j = 0 ; j < joints.size() ; ++j)
    {
	joints[j] = getKinematicModel()->getJoint(path.names[j]);
	if (joints[j] == NULL)
	{
	    ROS_ERROR("Unknown joint '%s' found on path", path.names[j].c_str());
	    return -1;
	}
    }
    
    for (unsigned int i = start ; i <= end ; ++i)
    {
	unsigned int u = 0;
	double       d = 0.0;
	for (unsigned int j = 0 ; j < joints.size() ; ++j)
	{
	    if (path.states[i].vals.size() < u + joints[j]->usedParams)
	    {
		ROS_ERROR("Incorrect state specification on path");
		return -1;
	    }
	    
	    const double *jparams = state->getParamsJoint(joints[j]->name);	    
	    for (unsigned int k = 0 ; k < joints[j]->usedParams ; ++k)
	    {
		double diff = fabs(path.states[i].vals[u + k] - jparams[k]);
		d += diff * diff;
	    }
	    u += joints[j]->usedParams;
	}
	
	if (pos < 0 || d < dist)
	{
	    pos = i;
	    dist = d;
	}
    }
    
    return pos;
}

bool planning_environment::PlanningMonitor::isPathValid(const motion_planning_msgs::KinematicPath &path, bool verbose) const
{
    return isPathValid(path, 0, path.states.size() - 1, verbose);
}

bool planning_environment::PlanningMonitor::isPathValid(const motion_planning_msgs::KinematicPath &path, unsigned int start, unsigned int end, bool verbose) const
{
    if (end >= path.states.size())
	end = path.states.size() - 1;
    if (start > end)
	return true;
    if (path.header.frame_id != getFrameId())
    {
	motion_planning_msgs::KinematicPath pathT = path;
	if (transformPathToFrame(pathT, getFrameId()))
	    return isPathValidAux(pathT, start, end, verbose);
	else
	    return false;
    }
    else
	return isPathValidAux(path, start, end, verbose); 
}

bool planning_environment::PlanningMonitor::isPathValidAux(const motion_planning_msgs::KinematicPath &path, unsigned int start, unsigned int end, bool verbose) const
{    
    boost::scoped_ptr<planning_models::StateParams> sp(getKinematicModel()->newStateParams());
    
    for (unsigned int i = 0 ; i < path.start_state.size() ; ++i)
	sp->setParamsJoint(path.start_state[i].value, path.start_state[i].joint_name);
    
    if (!sp->seenAll())
    {
	ROS_WARN("Incomplete start state specification in path");
	return false;
    }
    
    KinematicConstraintEvaluatorSet ks;
    ks.add(getKinematicModel(), kcPath_.joint_constraint);
    ks.add(getKinematicModel(), kcPath_.pose_constraint);
    
    getEnvironmentModel()->lock();
    getKinematicModel()->lock();

    bool vlevel = getEnvironmentModel()->getVerbose();
    getEnvironmentModel()->setVerbose(verbose);
       
    // figure out the poses of the robot model
    getKinematicModel()->computeTransforms(sp->getParams());
    // update the collision space
    getEnvironmentModel()->updateRobotModel();
    
    bool valid = true;
    
    // get the joints this path is for
    std::vector<planning_models::KinematicModel::Joint*> joints(path.names.size());
    for (unsigned int j = 0 ; j < joints.size() ; ++j)
    {
	joints[j] = getKinematicModel()->getJoint(path.names[j]);
	if (joints[j] == NULL)
	{
	    ROS_ERROR("Unknown joint '%s' found on path", path.names[j].c_str());
	    valid = false;
	    break;
	}
    }

    unsigned int sdim = getKinematicModel()->getJointsDimension(path.names);
    
    // check every state
    for (unsigned int i = start ; valid && i <= end ; ++i)
    {
	if (path.states[i].vals.size() != sdim)
	{
	    ROS_ERROR("Incorrect state specification on path");
	    valid = false;
	    break;
	}
	sp->setParamsJoints(path.states[i].vals, path.names);	

	getKinematicModel()->computeTransforms(sp->getParams());
	getEnvironmentModel()->updateRobotModel();
	
	// check for collision
	std::vector<collision_space::EnvironmentModel::Contact> contacts;
	valid = !getEnvironmentModel()->getCollisionContacts(contacts, 1);
	
	if (onCollisionContact_)
	    for (unsigned int i = 0 ; i < contacts.size() ; ++i)
		onCollisionContact_(contacts[i]);
	
	if (verbose && !valid)
	    ROS_INFO("isPathValid: State %d is in collision", i);

	// check for validity
	if (valid)
	{
	    valid = ks.decide(sp->getParams());
	    if (verbose && !valid)
	        ROS_INFO("isPathValid: State %d does not satisfy constraints", i);
	}
    }

    getEnvironmentModel()->setVerbose(vlevel);
    getKinematicModel()->unlock();
    getEnvironmentModel()->unlock();
    
    return valid;
}
