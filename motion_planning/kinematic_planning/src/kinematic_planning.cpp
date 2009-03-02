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

/**

@mainpage

@htmlinclude ../manifest.html

@b KinematicPlanning is a node capable of planning kinematic paths for
a set of robot models. Each robot model is a complete model specified
in URDF or consists of an URDF group.

Organization:
 - there are multiple models
 - there are multiple planners that can be used for each model
 - there are multiple types of planning requests

The code is mostly implemented in the included RKP* files (ROS
Kinematic Planning). There exists one basic class (RKPBasicRequest)
that can handle different requests. However, since the type of a
request may vary, the code for this basic request is templated. The
functions that vary with the type of request have a simple default
implementation (this implementation should not be used since it is
incomplete) and specializations for the different types of
requests. For new requests to be added, only specialization is
needed. Inheritance is also possible, on top of RKPBasicRequest, but
is not needed for the types of requets currently handled.

A model is defined for each loaded URDF model, and for each of the
URDF groups marked for planning. This model includes a kinematic
model, a collision space (shared between models) and a set of
planners. If a planner is used for different models, it is
instantiated each time. Since planners may require different
setup/configuration code, there exists a base class that defines the
functionality and an inherited class for each type of planner that can
be instantiated. The planners are associated to string names: RRT,
LazyRRT, EST, SBL, IKSBL. These string names can be used for the 
planner_id component of the planning request.

When checking states for validity, a resolution at which paths are
check needs to be defined. To make things easier for the user, this
parameter is computed by default by the SpaceInformationRKPModel
class. The current settings work fine for the PR2, but if another
robot is to be used, different settings man need to be used.

\todo
- Find a better way to specify resolution for state validity
checking.

When using replanning, this node monitors the current state of the
robot and that of the environment. When a change is detected, the
currently executed path is checked for validity. If the current path
is no longer valid, the validity status is set to false and a new path
is computed. When the new path is computed, it is sent in the status
messsage and validity is set to true (unless no solution is found, in
which case the status remains invalid; a change in the map or a call
to force_replanning will make the planner try again). When the planner
detects that the robot reached the desired position, it stops the
replanning thread and sets the done flag to true.

If the monitored state of the robot and of the environment is not
updated for a while (see the @ref parameters section), the unsafe flag
of the planning status is set to true.

<hr>

@section usage Usage
@verbatim
$ kinematic_planning [standard ROS args]
@endverbatim

@par Example

@verbatim
$ kinematic_planning robot_description:=robotdesc/pr2
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- None

Publishes to (name/type):
- @b "kinematic_planning_status"/KinematicPlanStatus : the current path to goal (published when replanning) and the 
  status of the motion planner (whether the path is valid, complete, etc)

<hr>

@section notes Notes

This node inherits from CollisionSpaceMonitor. Additional relevant
topics, services and parameters are documented in
CollisionSpaceMonitor.

<hr>

@section services ROS services

Uses (name/type):
- None

Provides (name/type):

- @b "plan_kinematic_path_state"/KinematicPlanState : given a robot model, starting and goal 
  states, this service computes a collision free path

- @b "plan_kinematic_path_position"/KinematicPlanLinkPosition : given a robot model, starting
  state and goal poses of certain links, this service computes a collision free path

- @b "replan_kinematic_path_state"/KinematicReplanState : given a robot model, starting and goal states,
  this service computes and recomputes a collision free path until the monitored state is actually at the goal or
  stopping is requested. Changes in the collision model trigger replanning. The computed path is published
  as part of the status message.

- @b "replan_kinematic_path_position"/KinematicReplanLinkPosition : given a robot model, starting state and goal
  poses of certain links, this service computes a collision free path until the monitored state is actually at the 
  goal or stopping is requested. Changes in the collision model trigger replanning.

- @b "replan_force"/Empty : signal the planner to replan one more step

- @b "replan_stop"/Empty : signal the planner to stop replanning


<hr>

@section parameters ROS parameters
- @b "refresh_interval_collision_map"/double : if more than this interval passes when receiving a request for motion planning,
  the unsafe flag is set to true

- @b "refresh_interval_kinematic_state"/double : if more than this interval passes when receiving a request for motion planning,
  the unsafe flag is set to true

- @b "refresh_interval_base_pose"/double : if more than this interval passes when receiving a request for motion planning,
  the unsafe flag is set to true, unless we are planning in the robot frame, in which case, this the pase pose does not matter
  (the collision map would be in the same frame)

**/

#include "kinematic_planning/CollisionSpaceMonitor.h"
#include "kinematic_planning/RKPModel.h"
#include "kinematic_planning/RKPBasicRequestState.h"
#include "kinematic_planning/RKPBasicRequestLinkPosition.h"

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include <std_srvs/Empty.h>

using namespace kinematic_planning;

class KinematicPlanning : public CollisionSpaceMonitor
{
public:
    
    KinematicPlanning(ros::Node *node) : CollisionSpaceMonitor(node)
    {
	m_node->advertiseService("plan_kinematic_path_state",    &KinematicPlanning::planToState, this);
	m_node->advertiseService("plan_kinematic_path_position", &KinematicPlanning::planToPosition, this);
	
	m_replanID = 0;
	m_replanningThread = NULL;
	m_collisionMonitorChange = false;
	m_currentRequestType = R_NONE;
	
	m_currentPlanStatus.id = -1;
	m_currentPlanStatus.new_id = 0;
	m_currentPlanStatus.distance = -1.0;
	m_currentPlanStatus.done = 1;
	m_currentPlanStatus.approximate = 0;
	m_currentPlanStatus.valid = 1;
	m_currentPlanStatus.unsafe = 0;
	m_currentlyExecutedPath.set_states_size(0);
	
	m_node->advertiseService("replan_kinematic_path_state",    &KinematicPlanning::replanToState, this);
	m_node->advertiseService("replan_kinematic_path_position", &KinematicPlanning::replanToPosition, this);
	m_node->advertiseService("replan_force",                   &KinematicPlanning::forceReplanning, this);
	m_node->advertiseService("replan_stop",                    &KinematicPlanning::stopReplanning, this);

	m_node->advertise<robot_msgs::KinematicPlanStatus>("kinematic_planning_status", 1);

	// determine intervals; a value of 0 means forever
	m_node->param("refresh_interval_collision_map", m_intervalCollisionMap, 3.0);
	m_node->param("refresh_interval_kinematic_state", m_intervalKinematicState, 0.5);
	m_node->param("refresh_interval_base_pose", m_intervalBasePose, 0.5);
    }
    
    /** Free the memory */
    virtual ~KinematicPlanning(void)
    {
	stopReplanning();
	stopPublishingStatus();
	for (std::map<std::string, RKPModel*>::iterator i = m_models.begin() ; i != m_models.end() ; i++)
	    delete i->second;
    }
    
    void run(void)
    {
	loadRobotDescription();
        std::vector<std::string> mlist;    
	knownModels(mlist);
	ROS_INFO("Known models:");    
	for (unsigned int i = 0 ; i < mlist.size() ; ++i)
	    ROS_INFO("  * %s", mlist[i].c_str());    
	
	waitForState();
	startPublishingStatus();
	
	if (mlist.size() > 0)
	    m_node->spin();
	else
	    ROS_ERROR("No robot models loaded. OMPL planning node cannot start.");
    }
    
    bool isSafeToPlan(bool report)
    {
	if (!isMapUpdated(m_intervalCollisionMap))
	{
	    if (report)
		ROS_WARN("Planning is not safe: map is not up to date");
	    return false;
	}
	
	if (!isStateUpdated(m_intervalKinematicState))
	{
	    if (report)
		ROS_WARN("Planning is not safe: kinematic state is not up to date");
	    return false;
	}
	
	return true;
    }
    
    bool forceReplanning(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
	ROS_INFO("Received request to force replanning");
	notifyReplanning();
	return true;
    }
    
    void notifyReplanning(void)
    {
	m_continueReplanningLock.lock();
	m_collisionMonitorChange = true;
	m_continueReplanningLock.unlock();
	m_collisionMonitorCondition.notify_all();
    }
    
    void stopReplanning(void)
    {	
	m_replanningLock.lock();
	bool stop = false;
	m_continueReplanningLock.lock();
	if (m_currentRequestType != R_NONE)
	{
	    if (m_currentRequestType == R_STATE)
		m_requestState.release();
	    else
		m_requestLinkPosition.release();
	    
	    /* make sure the working thread knows it is time to stop */
	    m_currentRequestType = R_NONE;	 
	    m_collisionMonitorCondition.notify_all();
	    stop = true;
	}
	m_continueReplanningLock.unlock();
	
	if (stop)
	{
	    /* wait for the thread to stop & clean up*/
	    m_replanningThread->join();
	    delete m_replanningThread;
	    m_replanningThread = false;
	}
	
	m_replanningLock.unlock();

	if (stop)
	    ROS_INFO("Replanning stopped");	
    }
    
    bool stopReplanning(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
	ROS_INFO("Received request to stop replanning");	
	stopReplanning();
	return true;
    }
    
    void startPublishingStatus(void)
    {
	if (loadedRobot())
	{
	    m_publishStatus = true;	
	    m_statusThread = new boost::thread(boost::bind(&KinematicPlanning::publishStatus, this));
	}
    }
    
    void stopPublishingStatus(void)
    {
	if (m_publishStatus)
	{
	    m_publishStatus = false;
	    m_statusThread->join();
	    delete m_statusThread;
	}	
    }

    bool replanToState(robot_srvs::KinematicReplanState::Request &req, robot_srvs::KinematicReplanState::Response &res)
    {
	ROS_INFO("Request for replanning to a state");
	bool st = false;
	res.id = -1;
	
	stopReplanning();
	
	if (m_robotState)
	{	    
	    currentState(req.value.start_state);
	    st = m_requestState.configure(m_models, req.value);

	    if (st)
	    {
		// start planning thread
		m_replanningLock.lock();
		m_currentRequestType = R_STATE;
		
		m_currentPlanStatus.id = ++m_replanID;
		m_currentPlanStatus.new_id = 1;
		m_currentPlanStatus.valid = 1;
		m_currentPlanStatus.path.set_states_size(0);
		m_currentPlanStatus.done = 0;
		m_currentPlanStatus.approximate = 1;
		m_currentPlanStatus.distance = -1.0;
		res.id = m_currentPlanStatus.id;
		m_statusLock.unlock();	    
		
		ROS_INFO("Start replanning with plan id %d", res.id);
		m_replanningThread = new boost::thread(boost::bind(&KinematicPlanning::replanToStateThread, this));
		m_replanningLock.unlock();
	    }
	    else
		ROS_ERROR("Received invalid request");
	}
	else
	    ROS_ERROR("Current robot state is unknown. Cannot start replanning.");
	
	return st;	
    }
    
    bool replanToPosition(robot_srvs::KinematicReplanLinkPosition::Request &req, robot_srvs::KinematicReplanLinkPosition::Response &res)
    {
	ROS_INFO("Request for replanning to a position");
	bool st = false;
	res.id = -1;
	
	stopReplanning();
	
	if (m_robotState)
	{
	    currentState(req.value.start_state);
	    st = m_requestLinkPosition.configure(m_models, req.value);

	    if (st)
	    {
		// start planning thread
		m_replanningLock.lock();
		m_currentRequestType = R_POSITION;
		
		m_currentPlanStatus.id = ++m_replanID;
		m_currentPlanStatus.new_id = 1;
		m_currentPlanStatus.valid = 1;
		m_currentPlanStatus.path.set_states_size(0);
		m_currentPlanStatus.done = 0;
		m_currentPlanStatus.approximate = 1;
		m_currentPlanStatus.distance = -1.0;
		res.id = m_currentPlanStatus.id;
		m_statusLock.unlock();	    
		
		ROS_INFO("Start replanning with plan id %d", res.id);
		m_replanningThread = new boost::thread(boost::bind(&KinematicPlanning::replanToPositionThread, this));
		m_replanningLock.unlock();	
	    }
	    else
		ROS_ERROR("Received invalid request");
	}
	else
	    ROS_ERROR("Current robot state is unknown. Cannot start replanning.");
	
	return st;
    }
    
    bool planToState(robot_srvs::KinematicPlanState::Request &req, robot_srvs::KinematicPlanState::Response &res)
    {
	ROS_INFO("Request for planning to a state");
	bool trivial = false;
	bool approximate = false;
	
	if (req.value.start_state.get_vals_size() == 0)
	{
	    currentState(req.value.start_state);
	    ROS_INFO("Using current state as starting point");
	}
	
	res.value.unsafe = isSafeToPlan(true) ? 0 : 1;	
	bool result = m_requestStateOpenLoop.configure(m_models, req.value);
	if (result)
	{
	    m_requestStateOpenLoop.execute(res.value.path, res.value.distance, trivial, approximate);
	    m_requestStateOpenLoop.release();
	}
	
	res.value.id = -1;
	res.value.new_id = 0;
	res.value.done = trivial ? 1 : 0;
	res.value.valid = res.value.path.get_states_size() > 0;
	res.value.approximate = approximate ? 1 : 0;
	
	return result;
    }

    bool planToPosition(robot_srvs::KinematicPlanLinkPosition::Request &req, robot_srvs::KinematicPlanLinkPosition::Response &res)
    {	
	ROS_INFO("Request for planning to a position");
	bool trivial = false;
	bool approximate = false;
	
	if (req.value.start_state.get_vals_size() == 0)
	{
	    currentState(req.value.start_state);
	    ROS_INFO("Using current state as starting point");
	}
	
	res.value.unsafe = isSafeToPlan(true) ? 0 : 1;
	bool result = m_requestLinkPositionOpenLoop.configure(m_models, req.value);
	if (result)
	{
	    m_requestLinkPositionOpenLoop.execute(res.value.path, res.value.distance, trivial, approximate);
	    m_requestLinkPositionOpenLoop.release();
	}
		
	res.value.id = -1;
	res.value.new_id = 0;
	res.value.done = trivial ? 1 : 0;
	res.value.valid = res.value.path.get_states_size() > 0;
	res.value.approximate = approximate ? 1 : 0;
	
	return result;
    }

    virtual void setRobotDescription(robot_desc::URDF *file)
    {
	CollisionSpaceMonitor::setRobotDescription(file);	
	defaultPosition();
	
	ROS_INFO("=======================================");	
	std::stringstream ss;
	m_kmodel->printModelInfo(ss);
	ROS_INFO("%s", ss.str().c_str());	
	ROS_INFO("=======================================");

	/* set the data for the model */
	RKPModel *model = new RKPModel();
	model->collisionSpaceID = 0;
	model->collisionSpace = m_collisionSpace;
        model->kmodel = m_kmodel;
	model->groupName = m_kmodel->getModelName();
	createMotionPlanningInstances(model);
	
	/* remember the model by the robot's name */
	m_models[model->groupName] = model;
	
	/* create a model for each group */
	std::vector<std::string> groups;
	m_kmodel->getGroups(groups);

	for (unsigned int i = 0 ; i < groups.size() ; ++i)
	{
	    RKPModel *model = new RKPModel();
	    model->collisionSpaceID = 0;
	    model->collisionSpace = m_collisionSpace;
	    model->kmodel = m_kmodel;
	    model->groupID = m_kmodel->getGroupID(groups[i]);
	    model->groupName = groups[i];
	    createMotionPlanningInstances(model);
	    m_models[model->groupName] = model;
	}
    }

    void currentState(robot_msgs::KinematicState &state)
    {
	state.set_vals_size(m_kmodel->getModelInfo().stateDimension);
	const double *params = m_robotState->getParams();
	for (unsigned int i = 0 ; i < state.get_vals_size() ; ++i)
            state.vals[i] = params[i];
    }
    
    void knownModels(std::vector<std::string> &model_ids)
    {
	for (std::map<std::string, RKPModel*>::const_iterator i = m_models.begin() ; i != m_models.end() ; ++i)
	    model_ids.push_back(i->first);
    }

protected:
    
    void publishStatus(void)
    {
	double seconds;
	m_node->param("kinematic_planning_status_interval", seconds, 0.02);
	ros::Duration duration(seconds);
	ros::Duration delta(std::min(0.01, seconds));
	
	while (m_publishStatus)
	{
	    ros::Time nextTime = ros::Time::now() + duration;
	    bool wait = true;
	    while (m_publishStatus && wait && ros::Time::now() < nextTime)
	    {
		delta.sleep();
		if (m_statusLock.try_lock())
		{
		    if (m_currentPlanStatus.path.get_states_size() > 0)
			wait = false;
		    m_statusLock.unlock();
		}
	    }

	    bool issueStop = false;	    
	    bool replan_inactive = m_continueReplanningLock.try_lock();
	    
	    m_statusLock.lock();

	    // replanning thread is waiting 
	    if (replan_inactive)
	    {
		// check if we reached the goal position
		if (m_currentRequestType == R_STATE)
		{
		    currentState(m_requestState.activeRequest().start_state);
		    m_currentPlanStatus.done = m_requestState.isTrivial(&m_currentPlanStatus.distance) ? 1  : 0;
		    issueStop = m_currentPlanStatus.done;
		}
		else
		    if (m_currentRequestType == R_POSITION)
		    {
			currentState(m_requestLinkPosition.activeRequest().start_state);
			m_currentPlanStatus.done = m_requestLinkPosition.isTrivial(&m_currentPlanStatus.distance) ? 1 : 0;
			issueStop = m_currentPlanStatus.done;
		    }
	    }

	    // we check the safety of the plan, unless we are
	    // reporting a path, in which case we want to make sure
	    // the sent safety value is the one before the motion
	    // planning started
	    if (m_currentPlanStatus.path.states.empty())
		m_currentPlanStatus.unsafe = isSafeToPlan(false) ? 0 : 1;
	    
	    m_node->publish("kinematic_planning_status", m_currentPlanStatus);
	    
	    // if we are sending a new path, remember it
	    // we will need to check it for validity when the ma changes
	    // we also make sure the next time the status is sent, we do not
	    // resend the path
	    if (m_currentPlanStatus.path.get_states_size() > 0)
	    {
		m_currentlyExecutedPath = m_currentPlanStatus.path;
		m_currentPlanStatus.path.set_states_size(0);
	    }
	    else
	    {
		// if we are not sending anything, and the path is not valid or done,
		// we mark the fact there is no path that we are currently monitoring
		if (!m_currentPlanStatus.valid || m_currentPlanStatus.done)
		    m_currentlyExecutedPath.set_states_size(0);
	    }
	    
	    // make sure new_id becomes 0, if it was 1
	    if (m_currentPlanStatus.new_id == 1)
		m_currentPlanStatus.new_id = 0;
	    
	    m_statusLock.unlock();
	    
	    if (replan_inactive)
		m_continueReplanningLock.unlock();
	    
	    if (issueStop)
	    {
		ROS_INFO("Motion plan was succesfully executed");
		stopReplanning();
	    }
	}
    }
        
    /** Wait for a change in the environment and recompute the motion plan */
    void replanToStateThread(void)
    {	
	ros::Duration eps(0.001);
	robot_msgs::KinematicPath solution;
	unsigned int step = 0;
	bool trivial = false;
	bool approximate = false;
	
	while (m_currentRequestType == R_STATE && !trivial)
	{    
	    step++;
	    ROS_INFO("Replanning step %d", step);
	    boost::mutex::scoped_lock lock(m_continueReplanningLock);
	    m_collisionMonitorChange = false;
	    double distance = 0.0;
	    bool safe = isSafeToPlan(true);
	    
	    currentState(m_requestState.activeRequest().start_state);
	    m_currentlyExecutedPathStart = m_requestState.activeRequest().start_state;
	    m_requestState.execute(solution, distance, trivial, approximate);
	    bool foundSolution = solution.get_states_size() > 0;
	    
	    m_statusLock.lock();	    
	    m_currentPlanStatus.path = solution;
	    m_currentPlanStatus.distance = distance;
	    m_currentPlanStatus.done = trivial ? 1 : 0;
	    m_currentPlanStatus.approximate = approximate ? 1 : 0;
	    m_currentPlanStatus.valid = foundSolution ? 1 : 0;
	    m_currentPlanStatus.unsafe = safe ? 0 : 1;
	    m_statusLock.unlock();	    
	    
	    if (trivial)
		break;
	    
	    if (foundSolution)
		// wait for a map update
		while (m_currentRequestType == R_STATE && !m_collisionMonitorChange)
		    m_collisionMonitorCondition.wait(m_continueReplanningLock);
	    else
	    {
		// give a chance to the map to update itself
		m_continueReplanningLock.unlock();
		eps.sleep();
		m_continueReplanningLock.lock();
	    }
	}
    }

    /** Wait for a change in the environment and recompute the motion plan */
    void replanToPositionThread(void)
    {	
	ros::Duration eps(0.001);
	robot_msgs::KinematicPath solution;
	unsigned int step = 0;
	bool trivial = false;
	bool approximate = false;

	while (m_currentRequestType == R_POSITION && !trivial)
	{
	    step++;
	    ROS_INFO("Replanning step %d", step);
	    boost::mutex::scoped_lock lock(m_continueReplanningLock);
	    m_collisionMonitorChange = false;
	    double distance = 0.0;
	    bool safe = isSafeToPlan(true);
	    
	    currentState(m_requestLinkPosition.activeRequest().start_state);
	    m_currentlyExecutedPathStart = m_requestLinkPosition.activeRequest().start_state;
	    m_requestLinkPosition.execute(solution, distance, trivial, approximate);
	    bool foundSolution = solution.get_states_size() > 0;

	    m_statusLock.lock();	    
	    m_currentPlanStatus.path = solution;
	    m_currentPlanStatus.distance = distance;
	    m_currentPlanStatus.done = trivial ? 1 : 0;
	    m_currentPlanStatus.approximate = approximate ? 1 : 0;
	    m_currentPlanStatus.valid = foundSolution ? 1 : 0;
	    m_currentPlanStatus.unsafe = safe ? 0 : 1;
	    m_statusLock.unlock();
	    
	    if (trivial)
		break;
	    
	    if (foundSolution)
		while (m_currentRequestType == R_POSITION && !m_collisionMonitorChange)
		    m_collisionMonitorCondition.wait(m_continueReplanningLock);
	    else
	    {
		// give a chance to the map to update itself
		m_continueReplanningLock.unlock();
		eps.sleep();
		m_continueReplanningLock.lock();
	    }
	}
    }
    
    /** Event executed after a change in the perceived world is observed */
    virtual void afterWorldUpdate(void)
    {
	CollisionSpaceMonitor::afterWorldUpdate();
	bool update = false;
	
	// notify the replanning thread of the change
	m_continueReplanningLock.lock();
	m_statusLock.lock();
	if (m_currentRequestType != R_NONE && m_currentPlanStatus.valid)
	{
	    if (m_currentRequestType == R_STATE)
	    {
		m_requestState.activeRequest().start_state = m_currentlyExecutedPathStart;
		update = !m_requestState.isStillValid(m_currentlyExecutedPath);
	    }
	    else
		if (m_currentRequestType == R_POSITION)
		{
		    m_requestLinkPosition.activeRequest().start_state = m_currentlyExecutedPathStart;
		    update = !m_requestLinkPosition.isStillValid(m_currentlyExecutedPath);
		} 
	    
	    if (update)
	    {
		// stop current plan, compute a new plan
		ROS_INFO("Currently executed path is no longer valid. Recomputing...");
		m_collisionMonitorChange = true;
		m_currentPlanStatus.valid = 0;
	    }
	    else
		ROS_INFO("Currently executed path is still valid");
	}
	else
	    if (!m_currentPlanStatus.valid)
	    {
		m_collisionMonitorChange = true;
		update = true;
	    }
	
	m_statusLock.unlock();
	m_continueReplanningLock.unlock();

	if (update)
	    m_collisionMonitorCondition.notify_all();
    }
    
private:
    
    /* instantiate the planners that can be used  */
    void createMotionPlanningInstances(RKPModel* model)
    {	
	std::map<std::string, std::string> options;
	robot_desc::URDF::Group *group = m_urdf->getGroup(model->kmodel->getURDFGroup(model->groupName));
	
	options.clear();
	if (group)
	{
	    const robot_desc::URDF::Map &data = group->data;
	    options = data.getMapTagValues("planning", "RRT");
	}
	
	model->addRRT(options);
	

	options.clear();
	if (group)
	{
	    const robot_desc::URDF::Map &data = group->data;
	    options = data.getMapTagValues("planning", "LazyRRT");
	}
	model->addLazyRRT(options);
	
	options.clear();
	if (group)
	{
	    const robot_desc::URDF::Map &data = group->data;
	    options = data.getMapTagValues("planning", "EST");
	}
	model->addEST(options);

	options.clear();
	if (group)
	{
	    const robot_desc::URDF::Map &data = group->data;
	    options = data.getMapTagValues("planning", "SBL");
	}
	model->addSBL(options); 

	options.clear();
	if (group)
	{
	    const robot_desc::URDF::Map &data = group->data;
	    options = data.getMapTagValues("planning", "IKSBL");
	}
	model->addIKSBL(options); 

	options.clear();
	if (group)
	{
	    const robot_desc::URDF::Map &data = group->data;
	    options = data.getMapTagValues("planning", "KPIECE");
	}
	model->addKPIECE(options); 

	options.clear();
	if (group)
	{
	    const robot_desc::URDF::Map &data = group->data;
	    options = data.getMapTagValues("planning", "IKKPIECE");
	}
	model->addIKKPIECE(options); 
    }

    ModelMap                                                        m_models;
    RKPBasicRequest<robot_msgs::KinematicPlanStateRequest>          m_requestStateOpenLoop;
    RKPBasicRequest<robot_msgs::KinematicPlanLinkPositionRequest>   m_requestLinkPositionOpenLoop;
    RKPBasicRequest<robot_msgs::KinematicPlanStateRequest>          m_requestState;
    RKPBasicRequest<robot_msgs::KinematicPlanLinkPositionRequest>   m_requestLinkPosition;
    

    // intervals for determining whether the monitored state & map are up to date
    double                                                          m_intervalCollisionMap;
    double                                                          m_intervalKinematicState;
    double                                                          m_intervalBasePose;


    /*********** DATA USED FOR REPLANNING ONLY ***********/
    
    // currently considered request
    int                                                             m_currentRequestType;
    
    // current status of the motion planner
    robot_msgs::KinematicPlanStatus                                 m_currentPlanStatus; 
    robot_msgs::KinematicPath                                       m_currentlyExecutedPath;
    robot_msgs::KinematicState                                      m_currentlyExecutedPathStart;
    
    // lock used for changing the motion planner status
    boost::mutex                                                    m_statusLock;
    // status publishing thread
    boost::thread                                                  *m_statusThread;
    // flag used to request stopping the publishing thread
    bool                                                            m_publishStatus;
    
    // the ID of the current replanning task
    int                                                             m_replanID;
    
    // pointer to the replanning thread (not NULL only when current request type is R_NONE)
    boost::thread                                                  *m_replanningThread;
    // lock used to synchronize access to the replanning flag and the replanning thread
    boost::mutex                                                    m_replanningLock;

    // flag set when the map was updated
    bool                                                            m_collisionMonitorChange;
    // condition being broadcasted when the map is updated
    boost::condition                                                m_collisionMonitorCondition;
    // lock used in conjuction with the condition
    boost::mutex                                                    m_continueReplanningLock;    
};

class OutputHandlerROScon : public ompl::msg::OutputHandler
{
public:
    
    OutputHandlerROScon(void) : OutputHandler()
    {
    }
    
    /** Issue a ROS error */
    virtual void error(const std::string &text)
    {
	ROS_ERROR("%s", text.c_str());
    }	    
    
    /** Issue a ROS warning */
    virtual void warn(const std::string &text)
    {
	ROS_WARN("%s", text.c_str());
    }
    
    /** Issue ROS info */
    virtual void inform(const std::string &text)
    {
	ROS_INFO("%s", text.c_str());
    }	    
    
    /** Issue ROS info */
    virtual void message(const std::string &text)
    {
	ROS_INFO("%s", text.c_str());
    }
    
};

int main(int argc, char **argv)
{ 
    ros::init(argc, argv);
    
    OutputHandlerROScon rosconOutputHandler;	
    ompl::msg::useOutputHandler(&rosconOutputHandler);
    
    ros::Node node("ompl_planning");
    KinematicPlanning planner(&node);
    planner.run();
    
    return 0;
}
