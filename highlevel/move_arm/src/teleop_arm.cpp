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
#include <visualization_msgs/Marker.h>
#include <joy/Joy.h>
#include <move_arm/MoveArmAction.h>
#include <actionlib/client/simple_action_client.h>

namespace move_arm
{
    
    class TeleopArm
    {
    public:
      TeleopArm(MoveArmSetup &setup) : setup_(setup), move_arm(nh_, "move_" + setup_.group_)
        {	    
	    nh_.param<bool>("~use_planning", use_planning_, false);
	    pubTwist_ = nh_.advertise<geometry_msgs::Twist>("/r_arm_cartesian_twist_controller/command", 1);
	    vizPub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1024);
	    ctrl_ = nh_.serviceClient<experimental_controllers::TrajectoryStart>("/r_arm_joint_waypoint_controller/TrajectoryStart", true);
	    subSpaceNav_ = nh_.subscribe("/spacenav/joy", 1, &TeleopArm::twistCallback, this);
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
	
	bool tryTwist(const geometry_msgs::Twist &tw, double frac)
	{
	    bool res = true;
	    planning_models::KinematicModel *kmodel = new planning_models::KinematicModel(*planningMonitor_->getKinematicModel());
	    
	    boost::shared_ptr<planning_models::KinematicState> start(new planning_models::KinematicState(*planningMonitor_->getRobotState()));
	    start->enforceBounds();
	    kmodel->computeTransforms(start->getParams());
	    
	    tf::Pose currentEff = kmodel->getJoint(setup_.groupJointNames_.back())->after->globalTrans;
	    geometry_msgs::Pose currentEffMsg;
	    tf::poseTFToMsg(currentEff, currentEffMsg);

	    geometry_msgs::Pose destEffMsg = tf::addDelta(currentEffMsg, tw, frac);
	    

	    ros::ServiceClient ik_client = nh_.serviceClient<manipulation_srvs::IKService>("pr2_ik_right_arm/ik_service", true);
	    geometry_msgs::PoseStamped destEffMsgStmp;
	    destEffMsgStmp.pose = destEffMsg;
	    destEffMsgStmp.header.stamp = planningMonitor_->lastJointStateUpdate();
	    destEffMsgStmp.header.frame_id = planningMonitor_->getFrameId();
	    showArrow(destEffMsgStmp);
	    
	    std::vector<double> solution;
	    if (computeIK(ik_client, destEffMsgStmp, solution))
	      {
		ROS_INFO("Starting at %f, %f, %f", currentEff.getOrigin().x(), currentEff.getOrigin().y(), currentEff.getOrigin().z());
		boost::shared_ptr<planning_models::KinematicState> goal(new planning_models::KinematicState(*start));
		goal->setParamsJoints(solution, setup_.groupJointNames_);
		goal->enforceBounds();
		
		kmodel->computeTransforms(goal->getParams());
		tf::Pose goalEff = kmodel->getJoint(setup_.groupJointNames_.back())->after->globalTrans;
		ROS_INFO("Going to %f, %f, %f", goalEff.getOrigin().x(), goalEff.getOrigin().y(), goalEff.getOrigin().z());
		
		std::vector< boost::shared_ptr<planning_models::KinematicState> > path;
		interpolatePath(start, goal, 20, path);
		
		ROS_INFO("Generated path with %d states", (int)path.size());
		
		unsigned int valid = findFirstInvalid(path);
		if (valid < path.size())
		  {
		    if (valid > 6)
		      path.resize(valid - 4);
		    else
		      path.clear();
		  }
		
		ROS_INFO("Valid part has %d states", (int)path.size());
		
		if (!path.empty())
		  executePath(path);
		else
		  {
		    geometry_msgs::PoseStamped destEffMsgStmp;
		    destEffMsgStmp.pose = destEffMsg;
		    destEffMsgStmp.header.stamp = planningMonitor_->lastJointStateUpdate();
		    destEffMsgStmp.header.frame_id = planningMonitor_->getFrameId();
		    showArrow(destEffMsgStmp);
		    
		    move_arm::MoveArmGoal goal;
		goal.goal_constraints.pose_constraint.resize(1);
		goal.goal_constraints.pose_constraint[0].type = motion_planning_msgs::PoseConstraint::POSITION_X + motion_planning_msgs::PoseConstraint::POSITION_Y + motion_planning_msgs::PoseConstraint::POSITION_Z +
		  + motion_planning_msgs::PoseConstraint::ORIENTATION_R + motion_planning_msgs::PoseConstraint::ORIENTATION_P + motion_planning_msgs::PoseConstraint::ORIENTATION_Y;
		goal.goal_constraints.pose_constraint[0].link_name = "r_wrist_roll_link";
		goal.goal_constraints.pose_constraint[0].pose = destEffMsgStmp;
		
		goal.goal_constraints.pose_constraint[0].position_tolerance_above.x = 0.005;
		goal.goal_constraints.pose_constraint[0].position_tolerance_above.y = 0.005;
		goal.goal_constraints.pose_constraint[0].position_tolerance_above.z = 0.005;
		goal.goal_constraints.pose_constraint[0].position_tolerance_below.x = 0.005;
		goal.goal_constraints.pose_constraint[0].position_tolerance_below.y = 0.005;
		goal.goal_constraints.pose_constraint[0].position_tolerance_below.z = 0.005;

		goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.x = 0.005;
		goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.y = 0.005;
		goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.z = 0.005;
		goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.x = 0.005;
		goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.y = 0.005;
		goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.z = 0.005;

		goal.goal_constraints.pose_constraint[0].orientation_importance = 0.2;

		move_arm.sendGoal(goal);

		  }
	      }
	    else
	      res = false;
	    
	    delete kmodel;	
	    return res;
      }
      
        void twistCallback(const joy::JoyConstPtr &joy)
	{
	    static int call = 0;
	    call = (call + 1) % 100;
	    if (call != 0)
	        return;
	    
	    if (!joy->buttons[0] && !joy->buttons[1])
	      return;

	    geometry_msgs::Twist tw;
	    tw.linear.x = joy->axes[0];
	    tw.linear.y = joy->axes[1];
	    tw.linear.z = joy->axes[2];
	    tw.angular.x = joy->axes[3];
	    tw.angular.y = joy->axes[4];
	    tw.angular.z = joy->axes[5];

	    
	    ROS_INFO("Got twist: %f, %f, %f : %f, %f, %f", tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.x, tw.angular.y, tw.angular.z);
	    
	    bool r = true;
	    for (int i = 0; i < 5 && r; i++)
	      r = !tryTwist(tw, 0.75 / (i + 1.0));
	}
	
	void showArrow(const geometry_msgs::PoseStamped &pose)
	{
	    visualization_msgs::Marker marker;
	    marker.header = pose.header;
	    marker.ns = "~";
	    marker.id = 1;
	    marker.type = visualization_msgs::Marker::ARROW;
	    marker.action = visualization_msgs::Marker::ADD;
	    marker.pose = pose.pose;
	    marker.scale.x = 0.25;
	    marker.scale.y = 0.4;
	    marker.scale.z = 0.4;
	    marker.color.a = 1.0;
	    marker.color.r = 1.0;
	    marker.color.g = 0.0;
	    marker.color.b = 0.0;
	    vizPub_.publish(marker);
	}
	
	void interpolatePath(boost::shared_ptr<planning_models::KinematicState> &start, boost::shared_ptr<planning_models::KinematicState> &goal, unsigned int count,
			     std::vector< boost::shared_ptr<planning_models::KinematicState> > &path)
	{
	    path.clear();
	    unsigned int dim = planningMonitor_->getKinematicModel()->getDimension();
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
		boost::shared_ptr<planning_models::KinematicState> sp(new planning_models::KinematicState(planningMonitor_->getKinematicModel()));
		std::vector<double> v(dim);
		for (unsigned int j = 0 ; j < dim ; ++j)
		    v[j] = startv[j] + inc[j] * i;
		sp->setParams(v);
		path.push_back(sp);
	    }
	    path.push_back(goal);
	}
	
	unsigned int findFirstInvalid(std::vector< boost::shared_ptr<planning_models::KinematicState> > &path)
	{
	    for (unsigned int i = 0 ; i < path.size() ; ++i)
		if (!planningMonitor_->isStateValid(path[i].get(), planning_environment::PlanningMonitor::COLLISION_TEST))
		    return i;
	    return path.size();
	}
	
	void executePath(std::vector< boost::shared_ptr<planning_models::KinematicState> > &path)
	{
	    manipulation_msgs::JointTraj traj;
	    traj.names = setup_.groupJointNames_;
	    traj.points.resize(path.size());
	    for (unsigned int i = 0 ; i < path.size() ; ++i)
	    {
		traj.points[i].time = 0.1 * i;
		path[i]->copyParamsJoints(traj.points[i].positions, setup_.groupJointNames_);
	    }
	    
	    experimental_controllers::TrajectoryStart::Request  send_traj_start_req;
	    experimental_controllers::TrajectoryStart::Response send_traj_start_res;

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
	    
	    planning_models::KinematicState *sp = new planning_models::KinematicState(planningMonitor_->getKinematicModel());
	    sp->defaultState();
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
        actionlib::SimpleActionClient<move_arm::MoveArmAction> move_arm;
	ros::Subscriber                        subSpaceNav_;
	ros::ServiceClient                     ctrl_;
        ros::Publisher                         pubTwist_;
	ros::Publisher                         vizPub_;
	planning_environment::PlanningMonitor *planningMonitor_;
        bool                                   use_planning_;
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
