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
LazyRRT, EST, SBL. These string names can be used for the planner_id
component of the planning request.

When checking states for validity, a resolution at which paths are
check needs to be defined. To make things easier for the user, this
parameter is computed by default by the SpaceInformationRKPModel
class. The current settings work fine for the PR2, but if another
robot is to be used, different settings man need to be used.

\todo
- Find a better way to specify resolution for state validity
checking.
- Move code from header files to .cpp files (maybe define a library?)


<hr>

@section usage Usage
@verbatim
$ kinematic_planning robot_model [standard ROS args]
@endverbatim

@par Example

@verbatim
$ kinematic_planning robotdesc/pr2
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "replan_kinematic_path_state"/KinematicPlanStateRequest : given a robot model, starting and goal states, this service computes and recomputes a collision free path until the monitored state is actually at the goal or stopping is requested. Changes in the collision model trigger replanning.


- @b "replan_kinematic_path_position"/KinematicPlanStateRequest : given a robot model, starting state and goal poses of certain links, this service computes a collision free path until the monitored state is actually at the goal or stopping is requested. Changes in the collision model trigger replanning.  
  
- @b "replan_stop"/Empty : signal the planner to stop replanning

Additional subscriptions due to inheritance from CollisionSpaceMonitor:

Publishes to (name/type):
- @b "path_to_goal"/KinematicPath : the current path to goal (published when replanning)

<hr>

@section services ROS services

Uses (name/type):
- None

Provides (name/type):
- @b "plan_kinematic_path_state"/KinematicPlanState : given a robot model, starting and goal states, this service computes a collision free path
- @b "plan_kinematic_path_position"/KinematicPlanLinkPosition : given a robot model, starting state and goal poses of certain links, this service computes a collision free path


<hr>

@section parameters ROS parameters
- None

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

class KinematicPlanning : public ros::Node,
			  public CollisionSpaceMonitor
{
public:
    
    KinematicPlanning(const std::string &robot_model) : ros::Node("kinematic_planning"),
							CollisionSpaceMonitor(dynamic_cast<ros::Node*>(this),
									      robot_model)
    {
	advertiseService("plan_kinematic_path_state",    &KinematicPlanning::planToState);
	advertiseService("plan_kinematic_path_position", &KinematicPlanning::planToPosition);
	
	m_replanID = 0;
	m_replanningThread = NULL;
	m_collisionMonitorChange = false;
	m_currentRequestType = R_NONE;
	
	m_currentPlanStatus.id = -1;
	m_currentPlanStatus.distance = -1.0;
	m_currentPlanStatus.done = 1;
	m_currentPlanStatus.valid = 1;

	advertiseService("replan_kinematic_path_state",    &KinematicPlanning::replanToState);
	advertiseService("replan_kinematic_path_position", &KinematicPlanning::replanToPosition);
	advertiseService("replan_stop",                    &KinematicPlanning::stopReplanning);

	advertise<robot_msgs::KinematicPlanStatus>("kinematic_planning_status", 1);
    }
    
    /** Free the memory */
    virtual ~KinematicPlanning(void)
    {
	stopReplanning();
	stopPublishingStatus();
	for (std::map<std::string, RKPModel*>::iterator i = m_models.begin() ; i != m_models.end() ; i++)
	    delete i->second;
    }
    
    void currentState(robot_msgs::KinematicState &state)
    {
	state.set_vals_size(m_kmodel->stateDimension);
	const double *params = m_robotState->getParams();
	for (unsigned int i = 0 ; i < state.get_vals_size() ; ++i)
            state.vals[i] = params[i];
    }
    
    void stopReplanning(void)
    {
	std_srvs::Empty::request  dummy1;
	std_srvs::Empty::response dummy2;
	ROS_INFO("Auto-stopping replanning...");	
	stopReplanning(dummy1, dummy2);
    }
    
    bool stopReplanning(std_srvs::Empty::request &req, std_srvs::Empty::response &res)
    {
	m_replanningLock.lock();
	bool stop = false;
	m_continueReplanningLock.lock();
	if (m_currentRequestType != R_NONE)
	{
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
	ROS_INFO("Replanning stopped");	
	return true;
    }
    
    void startPublishingStatus(void)
    {
	m_publishStatus = true;	
	m_statusThread = new boost::thread(boost::bind(&KinematicPlanning::publishStatus, this));
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
    
    void publishStatus(void)
    {
	double seconds;
	param("kinematic_planning_status_interval", seconds, 0.5);
	ros::Duration duration(seconds);
	ros::Duration delta(0.05);

	while (m_publishStatus)
	{
	    ros::Time nextTime = ros::Time::now() + duration;
	    bool wait = true;
	    while (wait && ros::Time::now() < nextTime)
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
	    
	    if (replan_inactive)
	    {
		if (m_currentRequestType == R_STATE)
		{
		    currentState(m_currentPlanToStateRequest.start_state);
		    m_currentPlanStatus.done = m_requestState.isTrivial(m_models, m_currentPlanToStateRequest, &m_currentPlanStatus.distance) ? 1  : 0;
		    issueStop = m_currentPlanStatus.done;
		}
		else
		    if (m_currentRequestType == R_POSITION)
		    {
			currentState(m_currentPlanToPositionRequest.start_state);
			m_currentPlanStatus.done = m_requestLinkPosition.isTrivial(m_models, m_currentPlanToPositionRequest, &m_currentPlanStatus.distance) ? 1 : 0;
			issueStop = m_currentPlanStatus.done;
		    }
	    }
	    
	    publish("kinematic_planning_status", m_currentPlanStatus);
	    
	    if (m_currentPlanStatus.path.get_states_size() > 0)
		m_currentPlanStatus.path.set_states_size(0);
	    
	    m_statusLock.unlock();
	    
	    if (replan_inactive)
		m_continueReplanningLock.unlock();
	    
	    if (issueStop)
		stopReplanning();
	}
    }
    
    bool replanToState(robot_srvs::KinematicReplanState::request &req, robot_srvs::KinematicReplanState::response &res)
    {
	ROS_INFO("Request for replanning to a state");
	bool st = false;
	res.id = -1;
	
	stopReplanning();
	
	if (m_robotState)
	{
	    // back up the request
	    m_currentPlanToStateRequest = req.value;

	    // start planning thread
	    m_replanningLock.lock();
	    m_currentRequestType = R_STATE;
	    
	    m_statusLock.lock();	    
	    m_currentPlanStatus.id = ++m_replanID;
	    m_currentPlanStatus.valid = 1;
	    m_currentPlanStatus.path.set_states_size(0);
	    m_currentPlanStatus.done = 0;
	    m_currentPlanStatus.distance = -1.0;
	    res.id = m_currentPlanStatus.id;
	    m_statusLock.unlock();	    

	    m_replanningThread = new boost::thread(boost::bind(&KinematicPlanning::replanToStateThread, this));
	    m_replanningLock.unlock();
	    st = true;
	}
	else
	    ROS_ERROR("Current robot state is unknown. Cannot start replanning.");
	
	return st;	
    }
    
    bool replanToPosition(robot_srvs::KinematicReplanLinkPosition::request &req, robot_srvs::KinematicReplanLinkPosition::response &res)
    {
	ROS_INFO("Request for replanning to a position");
	bool st = false;
	res.id = -1;
	
	stopReplanning();
	
	if (m_robotState)
	{
	    // back up the request
	    m_currentPlanToPositionRequest = req.value;
	    
	    // start planning thread
	    m_replanningLock.lock();
	    m_currentRequestType = R_POSITION;

	    m_statusLock.lock();	    
	    m_currentPlanStatus.id = ++m_replanID;
	    m_currentPlanStatus.valid = 1;
	    m_currentPlanStatus.path.set_states_size(0);
	    m_currentPlanStatus.done = 0;
	    m_currentPlanStatus.distance = -1.0;
	    res.id = m_currentPlanStatus.id;
	    m_statusLock.unlock();	    

	    m_replanningThread = new boost::thread(boost::bind(&KinematicPlanning::replanToPositionThread, this));
	    m_replanningLock.unlock();	
	    st = true;
	}
	else
	    ROS_ERROR("Current robot state is unknown. Cannot start replanning.");
	
	return st;
    }
    
    bool planToState(robot_srvs::KinematicPlanState::request &req, robot_srvs::KinematicPlanState::response &res)
    {
	ROS_INFO("Request for planning to a state");
	bool trivial = false;
	if (req.value.start_state.get_vals_size() == 0)
	{
	    currentState(req.value.start_state);
	    ROS_INFO("Using current state as starting point");
	}
	
	bool result = m_requestState.execute(m_models, req.value, res.value.path, res.value.distance, trivial);

	res.value.id = -1;
	res.value.done = trivial ? 1 : 0;
	res.value.valid = res.value.path.get_states_size() > 0;
	return result;
    }

    /** Wait for a change in the environment and recompute the motion plan */
    void replanToStateThread(void)
    {
	robot_msgs::KinematicPath solution;
	unsigned int step = 0;
	bool trivial = false;

	while (m_currentRequestType == R_STATE && !trivial)
	{    
	    step++;
	    ROS_INFO("Replanning step %d", step);
	    boost::mutex::scoped_lock lock(m_continueReplanningLock);
	    m_collisionMonitorChange = false;
	    double distance = 0.0;
	    
	    currentState(m_currentPlanToStateRequest.start_state);
	    m_requestState.execute(m_models, m_currentPlanToStateRequest, solution, distance, trivial);
	    
	    m_statusLock.lock();	    
	    m_currentPlanStatus.path = solution;
	    m_currentPlanStatus.distance = distance;
	    m_currentPlanStatus.done = trivial ? 1 : 0;
	    m_currentPlanStatus.valid = solution.get_states_size() > 0 ? 1 : 0;
	    m_statusLock.unlock();	    

	    if (trivial)
		break;
	    while (m_currentRequestType == R_STATE && !m_collisionMonitorChange)
		m_collisionMonitorCondition.wait(m_continueReplanningLock);
	}
    }

    bool planToPosition(robot_srvs::KinematicPlanLinkPosition::request &req, robot_srvs::KinematicPlanLinkPosition::response &res)
    {	
	ROS_INFO("Request for planning to a position");
	bool trivial = false;
	if (req.value.start_state.get_vals_size() == 0)
	{
	    currentState(req.value.start_state);
	    ROS_INFO("Using current state as starting point");
	}
	bool result = m_requestLinkPosition.execute(m_models, req.value, res.value.path, res.value.distance, trivial);

	res.value.id = -1;
	res.value.done = trivial ? 1 : 0;
	res.value.valid = res.value.path.get_states_size() > 0;
	return result;
    }

    /** Wait for a change in the environment and recompute the motion plan */
    void replanToPositionThread(void)
    {		
	robot_msgs::KinematicPath solution;
	unsigned int step = 0;
	bool trivial = false;

	while (m_currentRequestType == R_POSITION && !trivial)
	{
	    step++;
	    ROS_INFO("Replanning step %d", step);
	    boost::mutex::scoped_lock lock(m_continueReplanningLock);
	    m_collisionMonitorChange = false;
	    double distance = 0.0;
	    
	    currentState(m_currentPlanToPositionRequest.start_state);
	    m_requestLinkPosition.execute(m_models, m_currentPlanToPositionRequest, solution, distance, trivial);

	    m_statusLock.lock();	    
	    m_currentPlanStatus.path = solution;
	    m_currentPlanStatus.distance = distance;
	    m_currentPlanStatus.done = trivial ? 1 : 0;
	    m_currentPlanStatus.valid = solution.get_states_size() > 0 ? 1 : 0;
	    m_statusLock.unlock();
	    
	    if (trivial)
		break;
	    while (m_currentRequestType == R_POSITION && !m_collisionMonitorChange)
		m_collisionMonitorCondition.wait(m_continueReplanningLock);
	}
    }
    
    /** Event executed after a change in the perceived world is observed */
    virtual void afterWorldUpdate(void)
    {
	CollisionSpaceMonitor::afterWorldUpdate();
	
	// notify the replanning thread of the change
	m_continueReplanningLock.lock();
	m_collisionMonitorChange = true;
	m_continueReplanningLock.unlock();
	m_collisionMonitorCondition.notify_all();
    }
    
    virtual void setRobotDescription(robot_desc::URDF *file)
    {
	CollisionSpaceMonitor::setRobotDescription(file);	
	defaultPosition();
	
	ROS_INFO("=======================================");	
	std::stringstream ss;
	m_kmodel->printModelInfo(ss);
	ROS_INFO(ss.str().c_str());	
	ROS_INFO("=======================================");

	/* set the data for the model */
	RKPModel *model = new RKPModel();
	model->collisionSpaceID = 0;
	model->collisionSpace = m_collisionSpace;
        model->kmodel = m_kmodel;
	model->groupName = m_kmodel->name;
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
    
    void knownModels(std::vector<std::string> &model_ids)
    {
	for (std::map<std::string, RKPModel*>::const_iterator i = m_models.begin() ; i != m_models.end() ; ++i)
	    model_ids.push_back(i->first);
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
    }
    
    ModelMap                                                        m_models;
    RKPBasicRequest<robot_msgs::KinematicPlanStateRequest>          m_requestState;
    RKPBasicRequest<robot_msgs::KinematicPlanLinkPositionRequest>   m_requestLinkPosition;
    



    /*********** DATA USED FOR REPLANNING ONLY ***********/
    
    // currently considered request
    robot_msgs::KinematicPlanStateRequest                           m_currentPlanToStateRequest;    
    robot_msgs::KinematicPlanLinkPositionRequest                    m_currentPlanToPositionRequest; 
    int                                                             m_currentRequestType;
    
    // current status of the motion planner
    robot_msgs::KinematicPlanStatus                                 m_currentPlanStatus; 
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
	ROS_ERROR(text.c_str());
    }	    
    
    /** Issue a ROS warning */
    virtual void warn(const std::string &text)
    {
	ROS_WARN(text.c_str());
    }
    
    /** Issue ROS info */
    virtual void inform(const std::string &text)
    {
	ROS_INFO(text.c_str());
    }	    
    
    /** Issue ROS info */
    virtual void message(const std::string &text)
    {
	ROS_INFO(text.c_str());
    }
    
};

void usage(const char *progname)
{
    printf("\nUsage: %s robot_model [standard ROS args]\n", progname);
    printf("       \"robot_model\" is the name (string) of a robot description to be used for planning.\n");
}

int main(int argc, char **argv)
{ 
    if (argc >= 2)
    { 
	ros::init(argc, argv);
	OutputHandlerROScon rosconOutputHandler;	
	ompl::msg::useOutputHandler(&rosconOutputHandler);
	
	KinematicPlanning *planner = new KinematicPlanning(argv[1]);
	planner->loadRobotDescription();
	
	std::vector<std::string> mlist;    
	planner->knownModels(mlist);
	ROS_INFO("Known models:");    
	for (unsigned int i = 0 ; i < mlist.size() ; ++i)
	    ROS_INFO("  * %s", mlist[i].c_str());    
	
	planner->waitForState();
	planner->startPublishingStatus();
	
	if (mlist.size() > 0)
	    planner->spin();
	else
	    ROS_ERROR("No models defined. Kinematic planning node cannot start.");
	
	planner->shutdown();

	delete planner;	
    }
    else
	usage(argv[0]);
    
    return 0;    
}
