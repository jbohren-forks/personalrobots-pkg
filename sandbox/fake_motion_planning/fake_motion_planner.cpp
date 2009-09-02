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

#include <ros/ros.h>
#include <motion_planning_msgs/GetMotionPlan.h>
#include <planning_environment/models/robot_models.h>
#include <planning_models/kinematic_state.h>

class FakeMotionPlanner
{
public:
    
    FakeMotionPlanner(void) : rm_("robot_description")
    {
	planKinematicPathService_ = nodeHandle_.advertiseService("~plan_kinematic_path", &FakeMotionPlanner::planToGoal, this);
    }
    
    void run(void)
    {
	ROS_INFO("Fake motion planning is running");
	ros::spin();
    }
    
private:
    
    bool planToGoal(motion_planning_msgs::GetMotionPlan::Request &req, motion_planning_msgs::GetMotionPlan::Response &res)
    {
	ROS_INFO("Received motion planning request");
	if (req.goal_constraints.pose_constraint.empty())
	{
	    if (req.goal_constraints.joint_constraint.empty())
		return false;
	    
	    const planning_models::KinematicModel::JointGroup *jg = rm_.getKinematicModel()->getGroup(req.params.model_id);
	    if (!jg)
		return false;
	    std::vector<std::string> joints = jg->jointNames;
	    
	    res.approximate = 0;
	    res.distance = 0.0;
	    res.path.header = req.goal_constraints.joint_constraint[0].header;
	    res.path.start_state = req.start_state;
	    res.path.model_id = req.params.model_id;
	    res.path.names = joints;
	    res.path.states.resize(2);
	    
	    planning_models::KinematicState *sp = new planning_models::KinematicState(rm_.getKinematicModel().get());
	    
	    for (unsigned int i = 0 ; i < req.start_state.size() ; ++i)
		sp->setParamsJoint(req.start_state[i].value, req.start_state[i].joint_name);
	    sp->copyParamsJoints(res.path.states[0].vals, joints);
	    
	    for (unsigned int i = 0 ; i < req.goal_constraints.joint_constraint.size() ; ++i)
		sp->setParamsJoint(req.goal_constraints.joint_constraint[i].value, req.goal_constraints.joint_constraint[i].joint_name);
	    sp->copyParamsJoints(res.path.states[1].vals, joints);
	    
	    delete sp;
	    
	    res.path.times.resize(2);
	    res.path.times[0] = 0;
	    res.path.times[1] = 1;
	    return true;
	}
	else
	    return false;
    }
    
    ros::NodeHandle    nodeHandle_;
    ros::ServiceServer planKinematicPathService_;
    
    planning_environment::RobotModels rm_;
    
};
    
int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "fake_motion_planning");

    FakeMotionPlanner planner;
    planner.run();
    
    return 0;
}
