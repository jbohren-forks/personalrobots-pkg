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
 *   * Neither the name of Willow Garage nor the names of its
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
 *
 *
 *********************************************************************/

/* \author: Ioan Sucan */

#include "move_arm/move_arm_setup.h"
#include <geometry_msgs/Twist.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>
#include <manipulation_srvs/IKService.h>

namespace move_arm
{
    
    class TeleopArm
    {
    public:
	TeleopArm(MoveArmSetup &setup) : setup_(setup)
	{
	    ctrl_ = nh_.serviceClient<pr2_mechanism_controllers::TrajectoryStart>("/r_arm_joint_waypoint_controller/TrajectoryStart", true);
	    subSpaceNav_ = nh_.subscribe("/spacenav/twist", 1, &TeleopArm::twistCallback, this);
	    planningMonitor_ = setup_.planningMonitor_;
	}
	
	~TeleopArm(void)
	{
	}
	
	void run(void)
	{
	    ros::spin();
	}
	
    private:
	
	void twistCallback(const geometry_msgs::TwistConstPtr &twist)
	{
	    planningMonitor_->getKinematicModel()->lock();
	    boost::shared_ptr<planning_models::StateParams> start(new planning_models::StateParams(*planningMonitor_->getRobotState()));
	    start->enforceBounds();
	    planningMonitor_->getKinematicModel()->computeTransforms(start->getParams());

	    tf::Pose currentEff = planningMonitor_->getKinematicModel()->getLink(setup_.groupJointNames_.back())->globalTrans;
	    geometry_msgs::Pose currentEffMsg;
	    tf::poseTFToMsg(currentEff, currentEffMsg);
	    
	    geometry_msgs::Pose destEffMsg = tf::addDelta(currentEffMsg, *twist, 0.01);
	    
	    ros::ServiceClient ik_client = nh_.serviceClient<manipulation_srvs::IKService>("pr2_ik_right_arm/ik_service", true);
	    geometry_msgs::PoseStamped destEffMsgStmp;
	    destEffMsgStmp.pose = destEffMsg;
	    destEffMsgStmp.header.stamp = planningMonitor_->lastJointStateUpdate();
	    destEffMsgStmp.header.frame_id = planningMonitor_->getFrameId();
	    
	    std::vector<double> solution;
	    if (computeIK(ik_client, destEffMsgStmp, solution))
	    {
		boost::shared_ptr<planning_models::StateParams> goal(new planning_models::StateParams(*start));
		goal->setParamsJoints(solution, setup_.groupJointNames_);
		goal->enforceBounds();
		
		std::vector< boost::shared_ptr<planning_models::StateParams> > path;
		interpolatePath(start, goal, 10, path);
		
		unsigned int valid = findFirstInvalid(path);
		if (valid < path.size())
		    path.resize(valid);
		
		executePath(path);
	    }
	    
	    planningMonitor_->getKinematicModel()->unlock();
	}
	
	void interpolatePath(boost::shared_ptr<planning_models::StateParams> &start, boost::shared_ptr<planning_models::StateParams> &goal, unsigned int count,
			     std::vector< boost::shared_ptr<planning_models::StateParams> > &path)
	{
	    path.clear();
	    unsigned int dim = planningMonitor_->getKinematicModel()->getModelInfo().stateDimension;
	    std::vector<double> startv;
	    start->copyParams(startv);
	    std::vector<double> goalv;
	    goal->copyParams(goalv);
	    
	    std::vector<double> inc(dim);
	    for (unsigned int i = 0 ; i < dim ; ++i)
		inc[i] = (goalv[i] - startv[i]) / (double)count;

	    path.push_back(start);
	    for (unsigned int i = 1 ; i < count ; ++i)
	    {
		boost::shared_ptr<planning_models::StateParams> sp(planningMonitor_->getKinematicModel()->newStateParams());
		std::vector<double> v(dim);
		for (unsigned int j = 0 ; j < dim ; ++j)
		    v[j] = startv[j] + inc[j] * i;
		sp->setParams(v);
		path.push_back(sp);
	    }
	    path.push_back(goal);
	}
	
	unsigned int findFirstInvalid(std::vector< boost::shared_ptr<planning_models::StateParams> > &path)
	{
	    for (unsigned int i = 0 ; i < path.size() ; ++i)
		if (!planningMonitor_->isStateValid(path[i].get(), planning_environment::PlanningMonitor::COLLISION_TEST))
		    return i;
	    return path.size();
	}
	
	void executePath(std::vector< boost::shared_ptr<planning_models::StateParams> > &path)
	{
	    manipulation_msgs::JointTraj traj;
	    traj.names = setup_.groupJointNames_;
	    
	    for (unsigned int i = 0 ; i < path.size() ; ++i)
	    {
		traj.points[i].time = 0.1 * i;
		path[i]->copyParamsJoints(traj.points[i].positions, setup_.groupJointNames_);
	    }
	    
	    pr2_mechanism_controllers::TrajectoryStart::Request  send_traj_start_req;
	    pr2_mechanism_controllers::TrajectoryStart::Response send_traj_start_res;

	    send_traj_start_req.traj = traj;	    
	    send_traj_start_req.hastiming = 0;
	    send_traj_start_req.requesttiming = 0;
	    
	    if (ctrl_.call(send_traj_start_req, send_traj_start_res))
	    {
		int trajectoryId = send_traj_start_res.trajectoryid;
		if (trajectoryId < 0)
		    ROS_ERROR("Invalid trajectory id: %d", trajectoryId);
		else
		    ROS_INFO("Sent trajectory %d to controller", trajectoryId);
	    }
	    else
		ROS_ERROR("Unable to start trajectory controller");
	}
	
	bool computeIK(ros::ServiceClient &client, const geometry_msgs::PoseStamped &pose_stamped_msg, std::vector<double> &solution)
	{
	    // define the service messages
	    manipulation_srvs::IKService::Request request;
	    manipulation_srvs::IKService::Response response;
	    
	    request.data.pose_stamped = pose_stamped_msg;
	    request.data.joint_names = setup_.groupJointNames_;
	    
	    planning_models::StateParams *sp = planningMonitor_->getKinematicModel()->newStateParams();
	    sp->randomStateGroup(setup_.group_);
	    for(unsigned int i = 0; i < setup_.groupJointNames_.size() ; ++i)
	    {
		const double *params = sp->getParamsJoint(setup_.groupJointNames_[i]);
		const unsigned int u = planningMonitor_->getKinematicModel()->getJoint(setup_.groupJointNames_[i])->usedParams;
		for (unsigned int j = 0 ; j < u ; ++j)
		    request.data.positions.push_back(params[j]);
	    }
	    delete sp;
	    
	    if (client.call(request, response))
	    {
		ROS_DEBUG("Obtained IK solution");
		solution = response.solution;
		if (solution.size() != request.data.positions.size())
		{
		    ROS_ERROR("Incorrect number of elements in IK output");
		    return false;
		}
		for(unsigned int i = 0; i < solution.size() ; ++i)
		    ROS_DEBUG("IK[%d] = %f", (int)i, solution[i]);
	    }
	    else
	    {
		ROS_ERROR("IK service failed");
		return false;
	    }
	    
	    return true;
	}
	
	MoveArmSetup                          &setup_;
	ros::NodeHandle                        nh_;
	ros::Subscriber                        subSpaceNav_;
	ros::ServiceClient                     ctrl_;
	planning_environment::PlanningMonitor *planningMonitor_;
    };
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_arm");
    
    move_arm::MoveArmSetup setup;
    if (setup.configure())
    {
	move_arm::TeleopArm ta(setup);
	ta.run();
    }
    
    return 0;
}
