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


#include <planning_environment/collision_space_monitor.h>
#include "kinematic_planning/RKPModel.h"
#include "kinematic_planning/RKPRequestHandler.h"

#include <motion_planning_msgs/KinematicPlanStatus.h>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include <std_srvs/Empty.h>

using namespace kinematic_planning;

class KinematicPlanning 
{
public:
    
    KinematicPlanning(void)
    {
	m_collisionModels = new planning_environment::CollisionModels("robot_description");
	m_collisionSpaceMonitor = new planning_environment::CollisionSpaceMonitor(m_collisionModels, false);
	m_collisionSpaceMonitor->setOnAfterMapUpdateCallback(boost::bind(&KinematicPlanning::afterWorldUpdate, this, _1));
	
	m_replanID = 0;
	m_replanningThread = NULL;
	m_collisionMonitorChange = false;
	m_publishStatus = false;
	
	m_currentPlanStatus.id = -1;
	m_currentPlanStatus.distance = -1.0;
	m_currentPlanStatus.done = 1;
	m_currentPlanStatus.approximate = 0;
	m_currentPlanStatus.valid = 1;
	m_currentPlanStatus.unsafe = 0;
	m_currentlyExecutedPath.set_states_size(0);
	
	m_replanKinematicPathService = m_nodeHandle.advertiseService("plan_kinematic_path", &KinematicPlanning::replanToGoal, this);
	m_replanForceService = m_nodeHandle.advertiseService("replan_force", &KinematicPlanning::forceReplanning, this);
	m_replanStopService = m_nodeHandle.advertiseService("replan_stop", &KinematicPlanning::stopReplanning, this);
	
	m_kinematicPlanningPublisher = m_nodeHandle.advertise<motion_planning_msgs::KinematicPlanStatus>("kinematic_planning_status", 1);

	// determine intervals; a value of 0 means forever
	m_nodeHandle.param("~refresh_interval_collision_map", m_intervalCollisionMap, 3.0);
	m_nodeHandle.param("~refresh_interval_kinematic_state", m_intervalKinematicState, 0.5);
	m_nodeHandle.param("~refresh_interval_base_pose", m_intervalBasePose, 0.5);
    }
    
    /** Free the memory */
    ~KinematicPlanning(void)
    {
	stopReplanning();
	stopPublishingStatus();
	for (std::map<std::string, RKPModel*>::iterator i = m_models.begin() ; i != m_models.end() ; i++)
	    delete i->second;
	delete m_collisionSpaceMonitor;
	delete m_collisionModels;
    }
    
    void run(void)
    {
	bool execute = false;
	std::vector<std::string> mlist;    
	
	if (m_collisionModels->loadedModels())
	{
	    setupPlanningModels();
	    
	    knownModels(mlist);
	    ROS_INFO("Known models:");    
	    for (unsigned int i = 0 ; i < mlist.size() ; ++i)
		ROS_INFO("  * %s", mlist[i].c_str());    

	    //	    m_collisionSpaceMonitor->waitForState();
	    execute = !mlist.empty() && m_collisionSpaceMonitor->haveState() || 1; // hack
	    
	    if (execute)
	    {
		ROS_INFO("Working in frame %s", m_collisionSpaceMonitor->getFrameId().c_str());
		startPublishingStatus();
	    }
	}
	
	if (execute)
	{
	    ROS_INFO("Motion planning is now available.");
	    ros::spin();
	}
	else
	    if (mlist.empty())
		ROS_ERROR("No robot model loaded. OMPL planning node cannot start.");
    }
    
    bool isSafeToPlan(bool report)
    {
	if (!m_collisionSpaceMonitor->isMapUpdated(m_intervalCollisionMap))
	{
	    if (report)
		ROS_WARN("Planning is not safe: map is not up to date");
	    return false;
	}
	
	if (!m_collisionSpaceMonitor->isStateUpdated(m_intervalKinematicState))
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
	if (m_requestHandler.isActive())
	{
	    m_requestHandler.release();
	    
	    /* make sure the working thread knows it is time to stop */
	    m_collisionMonitorCondition.notify_all();
	    stop = true;
	}
	m_continueReplanningLock.unlock();
	
	if (stop)
	{
	    /* wait for the thread to stop & clean up*/
	    m_replanningThread->join();
	    delete m_replanningThread;
	    m_replanningThread = NULL;
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
	if (m_collisionModels->loadedModels())
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

    bool replanToGoal(motion_planning_srvs::KinematicPlan::Request &req, motion_planning_srvs::KinematicPlan::Response &res)
    {
	ROS_INFO("Received request for replanning");
	bool st = false;
	res.id = -1;
	
	stopReplanning();
	
	if (m_collisionSpaceMonitor->haveState())
	{	    
	    motion_planning_msgs::KinematicState start;
	    m_collisionSpaceMonitor->getRobotState()->copyParams(start.vals);
	    st = m_requestHandler.configure(m_models, start, req);

	    if (st)
	    {
		// start planning thread
		m_replanningLock.lock();
		
		m_currentPlanStatus.id = ++m_replanID;
		m_currentPlanStatus.valid = 1;
		m_currentPlanStatus.path.set_states_size(0);
		m_currentPlanStatus.done = 0;
		m_currentPlanStatus.approximate = 1;
		m_currentPlanStatus.distance = -1.0;
		res.id = m_currentPlanStatus.id;
		m_statusLock.unlock();	    
		
		ROS_INFO("Start replanning with plan id %d", res.id);
		m_replanningThread = new boost::thread(boost::bind(&KinematicPlanning::replanToGoalThread, this));
		m_replanningLock.unlock();
	    }
	    else
		ROS_ERROR("Received invalid request");
	}
	else
	    ROS_ERROR("Current robot state is unknown. Cannot start replanning.");
	
	return st;	
    }
    
    void setupPlanningModels(void)
    {
	ROS_DEBUG("=======================================");	
	std::stringstream ss;
	m_collisionModels->getKinematicModel()->printModelInfo(ss);
	ROS_DEBUG("%s", ss.str().c_str());	
	ROS_DEBUG("=======================================");

	/* create a model for each group */
	std::map< std::string, std::vector<std::string> > groups = m_collisionModels->getPlanningGroups();
	
	for (std::map< std::string, std::vector<std::string> >::iterator it = groups.begin(); it != groups.end() ; ++it)
	{
	    RKPModel *model = new RKPModel();
	    model->collisionSpace = m_collisionSpaceMonitor->getEnvironmentModel();
	    model->kmodel = m_collisionSpaceMonitor->getKinematicModel();
	    model->groupID = model->kmodel->getGroupID(it->first);
	    model->groupName = it->first;

	    model->createMotionPlanningInstances(m_collisionModels->getGroupPlannersConfig(model->groupName));
	    m_models[model->groupName] = model;
	}
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
	m_nodeHandle.param("~kinematic_planning_status_interval", seconds, 0.02);
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
		if (m_requestHandler.isActive())
		{
		    m_collisionSpaceMonitor->getRobotState()->copyParams(m_requestHandler.activeStartState().vals);
		    m_currentPlanStatus.done = m_requestHandler.isTrivial(&m_currentPlanStatus.distance) ? 1  : 0;
		    issueStop = m_currentPlanStatus.done;
		}
	    }

	    // we check the safety of the plan, unless we are
	    // reporting a path, in which case we want to make sure
	    // the sent safety value is the one before the motion
	    // planning started
	    if (m_currentPlanStatus.path.states.empty())
		m_currentPlanStatus.unsafe = isSafeToPlan(false) ? 0 : 1;
	    
	    m_kinematicPlanningPublisher.publish(m_currentPlanStatus);
	    
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
    void replanToGoalThread(void)
    {	
	ros::Duration eps(0.001);
	motion_planning_msgs::KinematicPath solution;
	unsigned int step = 0;
	bool trivial = false;
	bool approximate = false;

	while (m_requestHandler.isActive() && !trivial)
	{
	    step++;
	    ROS_DEBUG("Replanning step %d", step);
	    boost::mutex::scoped_lock lock(m_continueReplanningLock);
	    m_collisionMonitorChange = false;
	    double distance = 0.0;
	    bool safe = isSafeToPlan(true);
	    
	    m_collisionSpaceMonitor->getRobotState()->copyParams(m_requestHandler.activeStartState().vals);
	    m_currentlyExecutedPathStart = m_requestHandler.activeStartState();
	    m_requestHandler.execute(solution, distance, trivial, approximate);
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
		while (m_requestHandler.isActive() && !m_collisionMonitorChange)
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
    void afterWorldUpdate(const robot_msgs::CollisionMapConstPtr &collisionMap)
    {
	bool update = false;
	
	// notify the replanning thread of the change
	m_continueReplanningLock.lock();
	m_statusLock.lock();
	if (m_requestHandler.isActive() && m_currentPlanStatus.valid)
	{
	    m_requestHandler.activeStartState() = m_currentlyExecutedPathStart;
	    update = !m_requestHandler.isStillValid(m_currentlyExecutedPath);
	    
	    if (update)
	    {
		// stop current plan, compute a new plan
		ROS_INFO("Currently executed path is no longer valid. Recomputing...");
		m_collisionMonitorChange = true;
		m_currentPlanStatus.valid = 0;
	    }
	    else
		ROS_DEBUG("Currently executed path is still valid");
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
    

    RKPRequestHandler                                               m_requestHandler;
    
    ros::NodeHandle                                                 m_nodeHandle;
    planning_environment::CollisionModels                          *m_collisionModels;
    planning_environment::CollisionSpaceMonitor                    *m_collisionSpaceMonitor;

    ModelMap                                                        m_models;

    // intervals for determining whether the monitored state & map are up to date
    double                                                          m_intervalCollisionMap;
    double                                                          m_intervalKinematicState;
    double                                                          m_intervalBasePose;


    ros::Publisher                                                  m_kinematicPlanningPublisher;
    ros::ServiceServer                                              m_replanKinematicPathService;
    ros::ServiceServer                                              m_replanForceService;
    ros::ServiceServer                                              m_replanStopService;

    /*********** DATA USED FOR REPLANNING ONLY ***********/
    
    // current status of the motion planner
    motion_planning_msgs::KinematicPlanStatus                       m_currentPlanStatus; 
    motion_planning_msgs::KinematicPath                             m_currentlyExecutedPath;
    motion_planning_msgs::KinematicState                            m_currentlyExecutedPathStart;
    
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
	ROS_DEBUG("%s", text.c_str());
    }
    
};

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "ompl_planning");

    ROSCONSOLE_AUTOINIT;
    log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    // Set the logger for this package to output all statements
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
    
    OutputHandlerROScon rosconOutputHandler;	
    ompl::msg::useOutputHandler(&rosconOutputHandler);
    
    KinematicPlanning planner;
    planner.run();
    
    return 0;
}
