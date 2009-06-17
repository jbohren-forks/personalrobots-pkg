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

#include "kinematic_planning/RKPModel.h"
#include "kinematic_planning/RKPRequestHandler.h"

using namespace kinematic_planning;

class KinematicPlanning 
{
public:
    
    KinematicPlanning(void)
    {	
	// register with ROS
	collisionModels_ = new planning_environment::CollisionModels("robot_description");
	if (nodeHandle_.hasParam("~planning_frame_id"))
	{
	    std::string frame; nodeHandle_.param("~planning_frame_id", frame, std::string(""));
	    planningMonitor_ = new planning_environment::PlanningMonitor(collisionModels_, frame);
	}
	else
	    planningMonitor_ = new planning_environment::PlanningMonitor(collisionModels_);
	
	planKinematicPathService_ = nodeHandle_.advertiseService("plan_kinematic_path", &KinematicPlanning::planToGoal, this);
    }
    
    /** Free the memory */
    ~KinematicPlanning(void)
    {
	for (std::map<std::string, RKPModel*>::iterator i = models_.begin() ; i != models_.end() ; i++)
	    delete i->second;
	delete planningMonitor_;
	delete collisionModels_;
    }
    
    void run(void)
    {
	bool execute = false;
	std::vector<std::string> mlist;    
	
	if (collisionModels_->loadedModels())
	{
	    setupPlanningModels();
	    
	    knownModels(mlist);
	    ROS_INFO("Known models:");    
	    for (unsigned int i = 0 ; i < mlist.size() ; ++i)
		ROS_INFO("  * %s", mlist[i].c_str());    

	    planningMonitor_->waitForState();
	    execute = !mlist.empty() && planningMonitor_->haveState();
	    
	    if (execute)
		ROS_INFO("Motion planning running in frame '%s'", planningMonitor_->getFrameId().c_str());
	}
	
	if (execute)
	    ros::spin();
	else
	    if (mlist.empty())
		ROS_ERROR("No robot model loaded. OMPL planning node cannot start.");
    }
    
    bool planToGoal(motion_planning_srvs::KinematicPlan::Request &req, motion_planning_srvs::KinematicPlan::Response &res)
    {
	ROS_INFO("Received request for planning");
	bool st = false;
	
	res.path.states.clear();
	res.path.names.clear();
	res.path.times.clear();
	res.path.start_state.vals.clear();
	res.path.model_id = req.params.model_id;
	res.path.header.frame_id = planningMonitor_->getFrameId();
	res.path.header.stamp = planningMonitor_->lastMapUpdate();
	res.unsafe = planningMonitor_->isEnvironmentSafe() ? 0 : 1;
	res.distance = -1.0;
	res.approximate = 0;
	
	if (planningMonitor_->haveState())
	{
	    // get current state
	    planningMonitor_->getRobotState()->copyParams(res.path.start_state.vals);

	    // apply changes as indicated in request
	    for (unsigned int i = 0 ; req.start_state.size() ; ++i)
	    {
		if (!planningMonitor_->getTransformListener()->frameExists(req.start_state[i].header.frame_id))
		{
		    ROS_ERROR("Frame '%s' for joint '%s' in starting state is unknown", req.start_state[i].header.frame_id.c_str(), req.start_state[i].joint_name.c_str());
		    continue;
		}
		
		planningMonitor_->transformJointToFrame(req.start_state[i], planningMonitor_->getFrameId());
		int idx = planningMonitor_->getKinematicModel()->getJointIndex(req.start_state[i].joint_name);
		if (idx >= 0)
		{
		    unsigned int u = planningMonitor_->getKinematicModel()->getJoint(req.start_state[i].joint_name)->usedParams;
		    if (u == req.start_state[i].value.size())
			for (unsigned int j = 0 ; j < u ; ++j)
			    res.path.start_state.vals[idx + j] = req.start_state[i].value[j];
		    else
			ROS_ERROR("Incorrect number of parameters for joint '%s': expected %u, got %u", req.start_state[i].joint_name.c_str(), u, (unsigned int)req.start_state[i].value.size());
		}
	    }
	    
	    st = requestHandler_.computePlan(models_, req, res);
	}
	else
	    ROS_ERROR("Starting robot state is unknown. Cannot start plan.");
	
	return st;	
    }
    
    void setupPlanningModels(void)
    {
	ROS_DEBUG("=======================================");	
	std::stringstream ss;
	collisionModels_->getKinematicModel()->printModelInfo(ss);
	ROS_DEBUG("%s", ss.str().c_str());	
	ROS_DEBUG("=======================================");

	/* create a model for each group */
	std::map< std::string, std::vector<std::string> > groups = collisionModels_->getPlanningGroups();
	
	for (std::map< std::string, std::vector<std::string> >::iterator it = groups.begin(); it != groups.end() ; ++it)
	{
	    RKPModel *model = new RKPModel();
	    model->planningMonitor = planningMonitor_;
	    model->collisionSpace = planningMonitor_->getEnvironmentModel();
	    model->kmodel = planningMonitor_->getKinematicModel();
	    model->groupID = model->kmodel->getGroupID(it->first);
	    model->groupName = it->first;

	    model->createMotionPlanningInstances(collisionModels_->getGroupPlannersConfig(model->groupName));
	    models_[model->groupName] = model;
	}
    }

    void knownModels(std::vector<std::string> &model_ids)
    {
	for (std::map<std::string, RKPModel*>::const_iterator i = models_.begin() ; i != models_.end() ; ++i)
	    model_ids.push_back(i->first);
    }

private:
    
    // ROS interface 
    ros::NodeHandle                                                 nodeHandle_;
    planning_environment::CollisionModels                          *collisionModels_;
    planning_environment::PlanningMonitor                          *planningMonitor_;
    ros::ServiceServer                                              planKinematicPathService_;


    // planning data
    ModelMap                                                        models_;
    RKPRequestHandler                                               requestHandler_;

    // intervals for determining whether the monitored state & map are up to date
    double                                                          intervalCollisionMap_;
    double                                                          intervalState_;
    
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

    OutputHandlerROScon rosconOutputHandler;	
    ompl::msg::useOutputHandler(&rosconOutputHandler);
    
    KinematicPlanning planner;
    planner.run();
    
    return 0;
}
