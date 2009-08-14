/*********************************************************************
 *
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
 *
 *  \author Sachin Chitta, Ioan Sucan
 *********************************************************************/

#include "move_arm/move_arm_setup.h"

#include <actionlib/server/single_goal_action_server.h>
#include <move_arm/MoveArmAction.h>

#include <manipulation_msgs/JointTraj.h>
#include <manipulation_srvs/IKService.h>


#include <motion_planning_msgs/GetMotionPlan.h>
#include <motion_planning_msgs/ConvertToJointConstraint.h>

#include <visualization_msgs/Marker.h>

#include <planning_environment/util/construct_object.h>
#include <geometric_shapes/bodies.h>

#include <valarray>
#include <algorithm>
#include <cstdlib>

namespace move_arm
{
    
    class MoveArm
    {
    public:
	
	MoveArm(MoveArmSetup &setup) : setup_(setup), as_(nh_, "move_" + setup.group_, boost::bind(&MoveArm::execute, this, _1))
	{
	    if (setup_.show_collisions_)
		visMarkerPublisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 128);
	    
	    // advertise the topic for displaying kinematic plans
	    displayPathPublisher_ = nh_.advertise<motion_planning_msgs::KinematicPath>("executing_kinematic_path", 10);
	    
	    planningMonitor_ = setup_.planningMonitor_;
	    tf_              = &setup_.tf_;
	    
	    planningMonitor_->getEnvironmentModel()->setVerbose(false);
	}
	
	virtual ~MoveArm()
	{
	}
	
    private:
	
	/// The arm state maintained internally
	enum ArmActionState
	    {
		SUCCESS, PREEMPTED, ABORTED
	    };
	
	/// A state with a cost attached to it. Used internally to sort states by cost
	struct CostState
	{
	    boost::shared_ptr<planning_models::StateParams> state;
	    double                                          cost;
	    unsigned int                                    index;
	};
	
	/// Ordering function for states with cost attached
	struct CostStateOrder
	{
	    bool operator()(const CostState& a, const CostState& b) const
	    {
		return a.cost < b.cost;
	    }
	};
	
	/// Structure for maintaining cost of a state or a path
	struct CollisionCost
	{
	    CollisionCost(void)
	    {
		cost = 0.0;
		sum  = 0.0;
	    }
	    
	    std::string link;
	    double      cost;
	    double      sum;
	};
	
	/// Representation of a state and its corresponding joint constraints
	struct StateAndConstraint
	{
	    boost::shared_ptr<planning_models::StateParams>    state;
	    std::vector<motion_planning_msgs::JointConstraint> constraints;	    
	};
	
	
	/** \brief The ccost and display arguments should be bound by the caller. This is a callback function that gets called by the planning
	 * environment when a collision is found */
	void contactFound(CollisionCost *ccost, bool display, collision_space::EnvironmentModel::Contact &contact)
	{
	    double cdepth = fabs(contact.depth);
	    
	    if (ccost->cost < cdepth)
	    {
		ccost->cost = cdepth;
		if (contact.link1)
		    ccost->link = contact.link1->name;
		else
		    if (contact.link2)
			ccost->link = contact.link2->name;
	    }
	    ccost->sum += cdepth;
	    
	    if (display)
	    {
		static int count = 0;
		visualization_msgs::Marker mk;
		mk.header.stamp = planningMonitor_->lastMapUpdate();
		mk.header.frame_id = planningMonitor_->getFrameId();
		mk.ns = ros::this_node::getName();
		mk.id = count++;
		mk.type = visualization_msgs::Marker::SPHERE;
		mk.action = visualization_msgs::Marker::ADD;
		mk.pose.position.x = contact.pos.x();
		mk.pose.position.y = contact.pos.y();
		mk.pose.position.z = contact.pos.z();
		mk.pose.orientation.w = 1.0;
		
		mk.scale.x = mk.scale.y = mk.scale.z = 0.03;
		
		mk.color.a = 0.6;
		mk.color.r = 1.0;
		mk.color.g = 0.04;
		mk.color.b = 0.04;
		
		mk.lifetime = ros::Duration(30.0);
		
		visMarkerPublisher_.publish(mk);
	    }
	}
	
	/** \brief Evaluate the cost of a state, in terms of collisions */
	double computeStateCollisionCost(const planning_models::StateParams *sp)
	{
	    CollisionCost ccost;
	    
	    std::vector<collision_space::EnvironmentModel::AllowedContact> ac = planningMonitor_->getAllowedContacts();
	    planningMonitor_->clearAllowedContacts();
	    planningMonitor_->setOnCollisionContactCallback(boost::bind(&MoveArm::contactFound, this, &ccost, false, _1), 0);
	    
	    // check for collision, getting all contacts
	    planningMonitor_->isStateValid(sp, planning_environment::PlanningMonitor::COLLISION_TEST);
	    
	    planningMonitor_->setOnCollisionContactCallback(NULL);
	    planningMonitor_->setAllowedContacts(ac);
	    
	    return ccost.sum;
	}
	
	
	bool findValidNearJointConstraint(const std::vector<motion_planning_msgs::KinematicJoint> &start_state,
					  const motion_planning_msgs::KinematicSpaceParameters &params,
					  const motion_planning_msgs::KinematicConstraints &constraints,
					  const std::vector< boost::shared_ptr<planning_models::StateParams> > &hint_states,
					  StateAndConstraint &sac)
	{
	    bool result = false;
	    
	    // we make a request to a service that attempts to find a valid state close to the input state
	    motion_planning_msgs::ConvertToJointConstraint::Request c_req;
	    c_req.params = params;
	    c_req.start_state = start_state;
	    c_req.constraints = constraints;
	    c_req.names = setup_.groupJointNames_;
	    c_req.states.resize(hint_states.size());
	    c_req.allowed_time = 1.0;
	    
	    // if we have hints about where the valid input might be, we set them here
	    for (unsigned int i = 0 ; i < hint_states.size() ; ++i)
		hint_states[i]->copyParamsJoints(c_req.states[i].vals, setup_.groupJointNames_);
	    
	    motion_planning_msgs::ConvertToJointConstraint::Response c_res;
	    ros::ServiceClient s_client = nh_.serviceClient<motion_planning_msgs::ConvertToJointConstraint>(SEARCH_VALID_STATE_NAME);
	    if (s_client.call(c_req, c_res))
	    {	
		// if we found a valid state
		if (!c_res.joint_constraint.empty())
		{
		    // construct a state representation from our found joint constraints
		    sac.state.reset(new planning_models::StateParams(*planningMonitor_->getRobotState()));
		    
		    for (unsigned int i = 0 ; i < c_res.joint_constraint.size() ; ++i)
		    {
			const motion_planning_msgs::JointConstraint &kj = c_res.joint_constraint[i];
			sac.state->setParamsJoint(kj.value, kj.joint_name);
		    }
		    sac.state->enforceBounds();
		    sac.constraints = c_res.joint_constraint;
		    result = true;
		}
	    }
	    else
		ROS_ERROR("Service for searching for valid states failed");
	    
	    return result;
	}
	
	/** \brief If we have a complex goal for which we have not yet found a valid goal state, we use this function*/
	ArmActionState solveGoalComplex(std::vector< boost::shared_ptr<planning_models::StateParams> > &states,
					motion_planning_msgs::GetMotionPlan::Request &req)
	{
	    ROS_DEBUG("Acting on goal with unknown valid goal state ...");
	    
	    StateAndConstraint sac;
	    bool found = findValidNearJointConstraint(req.start_state, req.params, req.goal_constraints, states, sac);
	    
	    if (found)
	    {
		
		// if the state is in fact in the goal region, simply run the LR planner
		if (planningMonitor_->isStateValidAtGoal(sac.state.get()))
		{
		    ROS_DEBUG("Found valid goal state ...");
		    
		    req.goal_constraints.joint_constraint = sac.constraints;
		    req.goal_constraints.pose_constraint.clear();
		    
		    // update the goal constraints for the planning monitor as well
		    planningMonitor_->setGoalConstraints(req.goal_constraints);
		    
		    return runLRplanner(req);
		}
		else
		{
		    // if the state is valid but not in the goal region,
		    // we plan in two steps: first to this intermediate state
		    // that we hope is close to the goal (using LR planner) and 
		    // second to the final goal position using the SR planner
		    ROS_DEBUG("Found intermediate state ...");
		    
		    // backup for constraints
		    motion_planning_msgs::KinematicConstraints kc = req.goal_constraints;
		    
		    req.goal_constraints.joint_constraint = sac.constraints;
		    req.goal_constraints.pose_constraint.clear();
		    
		    // update the goal constraints for the planning monitor as well
		    planningMonitor_->setGoalConstraints(req.goal_constraints);
		    
		    ArmActionState result = runLRplanner(req);
		    
		    // restore constraints
		    req.goal_constraints = kc;
		    planningMonitor_->setGoalConstraints(req.goal_constraints);
		    
		    // if reaching the intermediate state was succesful
		    if (result == SUCCESS)
			// run the short range planner to the original goal
			return runSRplanner(states, req);
		    else
			return result;
		}
	    }
	    else
		return runLRplanner(states, req);
	}
	
	/** \brief Extract the state specified by the goal and run a planner towards it, if it is valid */
	ArmActionState solveGoalJoints(motion_planning_msgs::GetMotionPlan::Request &req)
	{
	    ROS_DEBUG("Acting on goal to set of joints ...");
	    
	    // construct a state representation from our goal joint
	    boost::shared_ptr<planning_models::StateParams> sp(new planning_models::StateParams(*planningMonitor_->getRobotState()));
	    
	    for (unsigned int i = 0 ; i < req.goal_constraints.joint_constraint.size() ; ++i)
	    {
		const motion_planning_msgs::JointConstraint &kj = req.goal_constraints.joint_constraint[i];
		sp->setParamsJoint(kj.value, kj.joint_name);
	    }
	    sp->enforceBounds();
	    
	    // try to skip straight to planning
	    if (planningMonitor_->isStateValidAtGoal(sp.get()))
		return runLRplanner(req);
	    else
	    {
		// if we can't, go to the more generic joint solver
		std::vector< boost::shared_ptr<planning_models::StateParams> > states;
		states.push_back(sp);
		return solveGoalJoints(states, req);
	    }
	}
	
	void updateRequest(motion_planning_msgs::GetMotionPlan::Request &req, const planning_models::StateParams *sp)
	{
	    // update request
	    for (unsigned int i = 0 ; i < setup_.groupJointNames_.size() ; ++i)
	    {
		motion_planning_msgs::JointConstraint jc;
		jc.joint_name = setup_.groupJointNames_[i];
		jc.header.frame_id = planningMonitor_->getFrameId();
		jc.header.stamp = planningMonitor_->lastJointStateUpdate();
		sp->copyParamsJoint(jc.value, setup_.groupJointNames_[i]);
		jc.tolerance_above.resize(jc.value.size(), 0.0);
		jc.tolerance_below.resize(jc.value.size(), 0.0);
		req.goal_constraints.joint_constraint.push_back(jc);
	    }
	    req.goal_constraints.pose_constraint.clear();
	    
	    // update the goal constraints for the planning monitor as well
	    planningMonitor_->setGoalConstraints(req.goal_constraints);
	}
	
	/** \brief Find a plan to given request, given a set of hint states in the goal region */
	ArmActionState solveGoalJoints(std::vector< boost::shared_ptr<planning_models::StateParams> > &states,
				       motion_planning_msgs::GetMotionPlan::Request &req)
	{
	    ROS_DEBUG("Acting on goal to set of joints pointing to potentially invalid state ...");
	    
	    // just in case we received no states (this should not happen)
	    if (states.empty())
		return solveGoalComplex(states, req);
	    
	    std::vector<CostState> cstates(states.size());
	    
	    for (unsigned int i = 0 ; i < states.size() ; ++i)
	    {
		cstates[i].state = states[i];
		cstates[i].cost = computeStateCollisionCost(states[i].get());
		cstates[i].index = i;
	    }
	    
	    // find the state with minimal cost
	    std::sort(cstates.begin(), cstates.end(), CostStateOrder());
	    
	    for (unsigned int i = 0 ; i < cstates.size() ; ++i)
		ROS_DEBUG("Cost of hint state %d is %f", i, cstates[i].cost);
	    
	    if (planningMonitor_->isStateValidAtGoal(cstates[0].state.get()))
	    {
		updateRequest(req, cstates[0].state.get());
		return runLRplanner(req);
	    }
	    else
	    {
		// order the states by cost before passing them forward
		std::vector< boost::shared_ptr<planning_models::StateParams> > backup = states;
		for (unsigned int i = 0 ; i < cstates.size() ; ++i)
		    states[i] = backup[cstates[i].index];
		return solveGoalComplex(states, req);
	    }
	}
	
	double uniformDouble(double lower_bound, double upper_bound)
	{
	    return (upper_bound - lower_bound) * drand48() + lower_bound;
	}
	
	/** \brief We generate IK solutions in the goal region. We stop
	    generating possible solutions when we find a valid one or we
	    reach a maximal number of steps. If we have candidate
	    solutions, we forward this to the joint-goal solver. If not,
	    the complex goal solver is to be used. */
	ArmActionState solveGoalPose(motion_planning_msgs::GetMotionPlan::Request &req)
	{
	    ROS_DEBUG("Acting on goal to pose ...");
	    
	    // we do IK to find corresponding states
	    ros::ServiceClient ik_client = nh_.serviceClient<manipulation_srvs::IKService>(ARM_IK_NAME, true);
	    std::vector< boost::shared_ptr<planning_models::StateParams> > states;
	    
	    // find an IK solution
	    for (int step = 0 ; step < 10 ; ++step)
	    {
		std::vector<double> solution;
		
		geometry_msgs::PoseStamped tpose = req.goal_constraints.pose_constraint[0].pose;
		
		if (step > 0)
		{
		    tpose.pose.position.x = uniformDouble(tpose.pose.position.x - req.goal_constraints.pose_constraint[0].position_tolerance_below.x,
							  tpose.pose.position.x + req.goal_constraints.pose_constraint[0].position_tolerance_above.x);
		    tpose.pose.position.y = uniformDouble(tpose.pose.position.y - req.goal_constraints.pose_constraint[0].position_tolerance_below.y,
							  tpose.pose.position.y + req.goal_constraints.pose_constraint[0].position_tolerance_above.y);
		    tpose.pose.position.z = uniformDouble(tpose.pose.position.z - req.goal_constraints.pose_constraint[0].position_tolerance_below.z,
							  tpose.pose.position.z + req.goal_constraints.pose_constraint[0].position_tolerance_above.z);
		}
		
		if (computeIK(ik_client, tpose, solution))
		{
		    // check if it is a valid state
		    boost::shared_ptr<planning_models::StateParams> spTest(new planning_models::StateParams(*planningMonitor_->getRobotState()));
		    spTest->setParamsJoints(solution, setup_.groupJointNames_);
		    spTest->enforceBounds();
		    
		    states.push_back(spTest);
		    if (planningMonitor_->isStateValidAtGoal(spTest.get()))
			break;
		}
		else
		    break;
	    }
	    
	    if (states.empty())
		return solveGoalComplex(states, req);
	    else
		return solveGoalJoints(states, req);
	}
	
	/** \brief Depending on the type of constraint, decide whether or not to use IK, decide which planners to use */
	ArmActionState solveGoal(motion_planning_msgs::GetMotionPlan::Request &req)
	{
	    ROS_DEBUG("Acting on goal...");
	    
	    // change pose constraints to joint constraints, if possible and so desired
	    if (setup_.perform_ik_ && req.goal_constraints.joint_constraint.empty() &&         // we have no joint constraints on the goal,
		req.goal_constraints.pose_constraint.size() == 1 &&      // we have a single pose constraint on the goal
		req.goal_constraints.pose_constraint[0].type ==
		motion_planning_msgs::PoseConstraint::POSITION_X + motion_planning_msgs::PoseConstraint::POSITION_Y + motion_planning_msgs::PoseConstraint::POSITION_Z +
		motion_planning_msgs::PoseConstraint::ORIENTATION_R + motion_planning_msgs::PoseConstraint::ORIENTATION_P + motion_planning_msgs::PoseConstraint::ORIENTATION_Y)  // that is active on all 6 DOFs
		return solveGoalPose(req);
	    
	    // if we have only joint constraints, we call the method that gets us to a goal state
	    if (req.goal_constraints.pose_constraint.empty())
		return solveGoalJoints(req);
	    
	    // otherwise, more complex constraints, run a generic method; we have no hint states
	    std::vector< boost::shared_ptr<planning_models::StateParams> > states;
	    return solveGoalComplex(states, req);
	}
	
	ArmActionState runLRplanner(motion_planning_msgs::GetMotionPlan::Request &req)
	{
	    std::vector< boost::shared_ptr<planning_models::StateParams> > states;
	    return runLRplanner(states, req);
	}
	
	ArmActionState runLRplanner(std::vector< boost::shared_ptr<planning_models::StateParams> > &states,
				    motion_planning_msgs::GetMotionPlan::Request &req)
	{
	    ROS_DEBUG("Running long range planner...");
	    ros::ServiceClient clientPlan = nh_.serviceClient<motion_planning_msgs::GetMotionPlan>(LR_MOTION_PLAN_NAME, true);
	    
	    ArmActionState result = runPlanner(clientPlan, req);
	    
	    // if the planner aborted and we have an idea about an invalid state that
	    // may be in the goal region, we make one last try using the short range planner
	    if (result == ABORTED && !states.empty() && !req.goal_constraints.pose_constraint.empty())
	    {
		// set the goal to be a state
		ROS_INFO("Trying again with a state in the goal region (although the state is invalid)...");
		updateRequest(req, states[0].get());
		result = runPlanner(clientPlan, req);
	    }
	    
	    return result;
	}
	
	ArmActionState runSRplanner(motion_planning_msgs::GetMotionPlan::Request &req)
	{
	    std::vector< boost::shared_ptr<planning_models::StateParams> > states;
	    return runSRplanner(states, req);
	}
	
	ArmActionState runSRplanner(std::vector< boost::shared_ptr<planning_models::StateParams> > &states,
				    motion_planning_msgs::GetMotionPlan::Request &req)
	{
	    ROS_DEBUG("Running short range planner...");
	    ros::ServiceClient clientPlan = nh_.serviceClient<motion_planning_msgs::GetMotionPlan>(SR_MOTION_PLAN_NAME, true);
	    
	    ArmActionState result = runPlanner(clientPlan, req);
	    
	    // if the planner aborted and we have an idea about an invalid state that
	    // may be in the goal region, we make one last try using the short range planner
	    if (result == ABORTED && !states.empty() && !req.goal_constraints.pose_constraint.empty())
	    {
		// set the goal to be a state
		ROS_INFO("Trying again with a state in the goal region (although the state is invalid)...");
		updateRequest(req, states[0].get());
		result = runPlanner(clientPlan, req);
	    }
	    
	    return result;
	}
	
	
	ArmActionState runPlanner(ros::ServiceClient &clientPlan, motion_planning_msgs::GetMotionPlan::Request &req)
	{
	    move_arm::MoveArmFeedbackPtr feedback(new move_arm::MoveArmFeedback());
	    
	    planningMonitor_->setAllowedContacts(req.params.contacts);
	    std::vector<collision_space::EnvironmentModel::AllowedContact> allowed_contacts = planningMonitor_->getAllowedContacts();
	    planningMonitor_->clearAllowedContacts();
	    
	    ArmActionState state = SUCCESS;
	    
	    feedback->mode = move_arm::MoveArmFeedback::PLANNING;
	    feedback->time_to_completion = ros::Duration(0);
	    as_.publishFeedback(feedback);
	    
	    motion_planning_msgs::GetMotionPlan::Response res;
	    
	    ros::ServiceClient clientStart  = nh_.serviceClient<pr2_mechanism_controllers::TrajectoryStart>(CONTROL_START_NAME, true);
	    ros::ServiceClient clientQuery  = nh_.serviceClient<pr2_mechanism_controllers::TrajectoryQuery>(CONTROL_QUERY_NAME, true);
	    ros::ServiceClient clientCancel = nh_.serviceClient<pr2_mechanism_controllers::TrajectoryCancel>(CONTROL_CANCEL_NAME, true);
	    
	    motion_planning_msgs::KinematicPath currentPath;
	    int                                 currentPos   = 0;
	    bool                                approx       = false;
	    int                                 trajectoryId = -1;
	    ros::Duration                       eps(0.01);
	    ros::Duration                       epsLong(0.1);
	    
	    std::valarray<double>               velocityHistory(3);
	    unsigned int                        velocityHistoryIndex = 0;
	    
	    while (true)
	    {
		// if we have to stop, do so
		if (as_.isPreemptRequested() || as_.isNewGoalAvailable() || !nh_.ok())
		    state = PREEMPTED;
		
		// if we have to plan, do so
		if (state == SUCCESS && feedback->mode == move_arm::MoveArmFeedback::PLANNING)
		{
		    if (!planningMonitor_->isEnvironmentSafe())
		    {
			ROS_WARN("Environment is not safe. Will not issue request for planning");
			epsLong.sleep();
			continue;
		    }
		    
		    fillStartState(req.start_state);
		    
		    ROS_DEBUG("Issued request for motion planning");
		    
		    // call the planner and decide whether to use the path
		    if (clientPlan.call(req, res))
		    {
			if (res.path.states.empty())
			{
			    ROS_WARN("Unable to plan path to desired goal");
			    epsLong.sleep();
			}
			else
			{
			    if (res.path.model_id != req.params.model_id)
				ROS_ERROR("Received path for incorrect model: expected '%s', received '%s'", req.params.model_id.c_str(), res.path.model_id.c_str());
			    else
			    {
				if (!planningMonitor_->getTransformListener()->frameExists(res.path.header.frame_id))
				    ROS_ERROR("Received path in unknown frame: '%s'", res.path.header.frame_id.c_str());
				else
				{
				    approx = res.approximate;
				    if (res.approximate)
					ROS_INFO("Approximate path was found. Distance to goal is: %f", res.distance);
				    ROS_INFO("Received path with %u states from motion planner", (unsigned int)res.path.states.size());
				    if (res.path.states.size() > 1)
				    {
					currentPath = res.path;
					currentPos = 0;
					if (planningMonitor_->transformPathToFrame(currentPath, planningMonitor_->getFrameId()))
					{
					    displayPathPublisher_.publish(currentPath);
					    //				    printPath(currentPath);
					    
					    feedback->mode = move_arm::MoveArmFeedback::MOVING;
					    as_.publishFeedback(feedback);
					}
				    }
				    else
					ROS_ERROR("Received path is too short");
				}
			    }
			}
		    }
		    else
		    {
			ROS_ERROR("Motion planning service failed");
			state = ABORTED;
			break;
		    }
		}
		
		// if we have to stop, do so
		if (as_.isPreemptRequested() || as_.isNewGoalAvailable() || !nh_.ok())
		    state = PREEMPTED;
		
		// if preeemt was requested while we are planning, terminate
		if (state == PREEMPTED && feedback->mode == move_arm::MoveArmFeedback::PLANNING)
		{
		    ROS_INFO("Preempt requested. Stopped planning.");
		    break;
		}
		
		// stop the robot if we need to
		if (feedback->mode == move_arm::MoveArmFeedback::MOVING)
		{
		    bool safe = planningMonitor_->isEnvironmentSafe();
		    
		    // we don't want to check the part of the path that was already executed
		    currentPos = planningMonitor_->closestStateOnPath(currentPath, currentPos, currentPath.states.size() - 1, planningMonitor_->getRobotState());
		    if (currentPos < 0)
		    {
			ROS_WARN("Unable to identify current state in path");
			currentPos = 0;
		    }
		    
		    CollisionCost ccost;
		    planningMonitor_->setOnCollisionContactCallback(boost::bind(&MoveArm::contactFound, this, &ccost, true, _1), 0);
		    planningMonitor_->setAllowedContacts(allowed_contacts);
		    bool valid = planningMonitor_->isPathValid(currentPath, currentPos, currentPath.states.size() - 1, planning_environment::PlanningMonitor::COLLISION_TEST +
							       planning_environment::PlanningMonitor::PATH_CONSTRAINTS_TEST, false);
		    planningMonitor_->clearAllowedContacts();
		    planningMonitor_->setOnCollisionContactCallback(NULL);
		    
		    if (!valid)
			ROS_INFO("Maximum path contact penetration depth is %f at link %s, sum of all contact depths is %f", ccost.cost, ccost.link.c_str(), ccost.sum);
		    
		    if (velocityHistoryIndex >= velocityHistory.size())
		    {
			double sum = velocityHistory.sum();
			if (sum < 1e-3)
			{
			    ROS_INFO("The total velocity of the robot over the last %d samples is %f. Self-preempting...", (int)velocityHistory.size(), sum);
			    state = PREEMPTED;
			}
		    }
		    
		    if (state == PREEMPTED || !safe || !valid)
		    {
			if (state == PREEMPTED)
			    ROS_INFO("Preempt requested. Stopping arm.");
			else
			    if (!safe)
				ROS_WARN("Environment is no longer safe. Cannot decide if path is valid. Stopping & replanning...");
			    else
				ROS_INFO("Current path is no longer valid. Stopping & replanning...");
			
			if (trajectoryId != -1)
			{
			    // we are already executing the path; we need to stop it
			    pr2_mechanism_controllers::TrajectoryCancel::Request  send_traj_cancel_req;
			    pr2_mechanism_controllers::TrajectoryCancel::Response send_traj_cancel_res;
			    send_traj_cancel_req.trajectoryid = trajectoryId;
			    if (clientCancel.call(send_traj_cancel_req, send_traj_cancel_res))
				ROS_INFO("Stopped trajectory %d", trajectoryId);
			    else
				ROS_ERROR("Unable to cancel trajectory %d. Continuing...", trajectoryId);
			    trajectoryId = -1;
			}
			
			if (state != PREEMPTED)
			{
			    // if we were not preempted
			    feedback->mode = move_arm::MoveArmFeedback::PLANNING;
			    as_.publishFeedback(feedback);
			    continue;
			}
			else
			    break;
		    }
		}
		
		// execute & monitor a path if we need to
		if (feedback->mode == move_arm::MoveArmFeedback::MOVING)
		{
		    // start the controller if we have to, using trajectory start
		    if (trajectoryId == -1)
		    {
			pr2_mechanism_controllers::TrajectoryStart::Request  send_traj_start_req;
			pr2_mechanism_controllers::TrajectoryStart::Response send_traj_start_res;
			
			fillTrajectoryPath(currentPath, send_traj_start_req.traj);
			send_traj_start_req.hastiming = 0;
			send_traj_start_req.requesttiming = 0;
			
			if (clientStart.call(send_traj_start_req, send_traj_start_res))
			{
			    trajectoryId = send_traj_start_res.trajectoryid;
			    if (trajectoryId < 0)
			    {
				ROS_ERROR("Invalid trajectory id: %d", trajectoryId);
				state = ABORTED;
				break;
			    }
			    ROS_INFO("Sent trajectory %d to controller", trajectoryId);
			    velocityHistoryIndex = 0;
			}
			else
			{
			    ROS_ERROR("Unable to start trajectory controller");
			    state = ABORTED;
			    break;
			}
		    }
		    else
		    {
			velocityHistory[velocityHistoryIndex % velocityHistory.size()] = planningMonitor_->getTotalVelocity();
			velocityHistoryIndex++;
		    }
		    
		    // monitor controller execution by calling trajectory query
		    
		    pr2_mechanism_controllers::TrajectoryQuery::Request  send_traj_query_req;
		    pr2_mechanism_controllers::TrajectoryQuery::Response send_traj_query_res;
		    send_traj_query_req.trajectoryid = trajectoryId;
		    if (clientQuery.call(send_traj_query_req, send_traj_query_res))
		    {
			// we are done; exit with success
			if (send_traj_query_res.done == pr2_mechanism_controllers::TrajectoryQuery::Response::State_Done)
			{
			    if (approx && !planningMonitor_->isStateValidAtGoal(planningMonitor_->getRobotState()))
			    {
				ROS_INFO("Completed approximate path (trajectory %d). Trying again to reach goal...", trajectoryId);
				feedback->mode = move_arm::MoveArmFeedback::PLANNING;
				as_.publishFeedback(feedback);
				trajectoryId = -1;
				continue;
			    }
			    ROS_INFO("Completed trajectory %d", trajectoryId);
			    break;
			}
			// something bad happened in the execution
			if (send_traj_query_res.done != pr2_mechanism_controllers::TrajectoryQuery::Response::State_Active &&
			    send_traj_query_res.done != pr2_mechanism_controllers::TrajectoryQuery::Response::State_Queued)
			{
			    ROS_ERROR("Unable to execute trajectory %d: query returned status %d", trajectoryId, (int)send_traj_query_res.done);
			    state = ABORTED;
			    break;
			}
		    }
		    else
		    {
			ROS_ERROR("Unable to query trajectory %d", trajectoryId);
			state = ABORTED;
			break;
		    }
		}
		eps.sleep();
	    }
	    
	    return state;
	}
	
	ArmActionState moveToValidState(motion_planning_msgs::GetMotionPlan::Request &req)
	{
	    ROS_DEBUG("Trying to use short range planner to move to a valid state...");

	    // construct a state representation + kinematic constraints from our start state
	    boost::shared_ptr<planning_models::StateParams> sp(new planning_models::StateParams(*planningMonitor_->getRobotState()));
	    motion_planning_msgs::KinematicConstraints constraints;
	    
	    for (unsigned int i = 0 ; i < req.start_state.size() ; ++i)
	    {
		const motion_planning_msgs::KinematicJoint &kj = req.start_state[i];
		sp->setParamsJoint(kj.value, kj.joint_name);
		motion_planning_msgs::JointConstraint jc;
		jc.header.frame_id = planningMonitor_->getFrameId();
		jc.header.stamp = ros::Time::now();
		jc.joint_name = kj.joint_name;
		jc.value = kj.value;
		jc.tolerance_above.resize(jc.value.size(), 0.0);
		jc.tolerance_below.resize(jc.value.size(), 0.0);
		constraints.joint_constraint.push_back(jc);
	    }
	    std::vector< boost::shared_ptr<planning_models::StateParams> > states;
	    states.push_back(sp);
	    
	    // find valid state near by
	    StateAndConstraint sac;
	    bool found = findValidNearJointConstraint(req.start_state, req.params, constraints, states, sac);
	    
	    ArmActionState result = ABORTED;
	    if (found)
	    {
		motion_planning_msgs::KinematicConstraints kc = req.goal_constraints;
		
		// move to that state (we update goal constraints temporarily)
		req.goal_constraints.pose_constraint.clear();
		req.goal_constraints.joint_constraint = sac.constraints;
		
		// try to move to the state
		result = runSRplanner(req);
		
		// restore previous goal constraints
		req.goal_constraints = kc;
		
		// refill with new start state
		if (result == SUCCESS)
		    fillStartState(req.start_state);
	    }
	    
	    return result;
	}
	
	void execute(const move_arm::MoveArmGoalConstPtr& goal)
	{
	    motion_planning_msgs::GetMotionPlan::Request req;
	    
	    req.params.model_id = setup_.group_;      // the model to plan for (should be defined in planning.yaml)
	    req.params.distance_metric = "L2Square"; // the metric to be used in the robot's state space
	    req.params.contacts = goal->contacts;
	    
	    // forward the goal & path constraints
	    req.goal_constraints = goal->goal_constraints;
	    req.path_constraints = goal->path_constraints;
	    
	    // transform them to the local coordinate frame since we may be updating this request later on
	    planningMonitor_->transformConstraintsToFrame(req.goal_constraints, planningMonitor_->getFrameId());
	    planningMonitor_->transformConstraintsToFrame(req.path_constraints, planningMonitor_->getFrameId());
	    
	    // compute the path once
	    req.times = 1;
	    
	    // do not spend more than this amount of time
	    req.allowed_time = 1.0;
	    
	    // tell the planning monitor about the constraints we will be following
	    planningMonitor_->setPathConstraints(req.path_constraints);
	    planningMonitor_->setGoalConstraints(req.goal_constraints);
	    
	    ROS_INFO("Received planning request");
	    
	    std::stringstream ss;
	    planningMonitor_->printConstraints(ss);
	    planningMonitor_->setAllowedContacts(req.params.contacts);
	    planningMonitor_->printAllowedContacts(ss);
	    planningMonitor_->clearAllowedContacts();
	    ROS_DEBUG("%s", ss.str().c_str());

	    ArmActionState result = SUCCESS;
	    
	    // fill the starting state; if state is not valid, use short range planner to move out of it
	    if (!fillStartState(req.start_state))
		result = moveToValidState(req);
	    
	    if (as_.isPreemptRequested() || as_.isNewGoalAvailable())
		result = PREEMPTED;
	    
	    // find & execute path to goal
	    if (result != PREEMPTED)
		result = solveGoal(req);
	    
	    if (result == SUCCESS)
	    {
		ROS_INFO("Goal was reached");
		as_.setSucceeded();
	    }
	    else
	    {
		ROS_INFO("Goal was not reached");
		if (result == PREEMPTED)
		    as_.setPreempted();
		else
		    as_.setAborted();
	    }
	}
	
	void fillTrajectoryPath(const motion_planning_msgs::KinematicPath &path, manipulation_msgs::JointTraj &traj)
	{
	    traj.names = setup_.groupJointNames_;

	    // get the current state
	    double d = 0.0;
	    std::vector<double> current;
	    planningMonitor_->getRobotState()->copyParamsGroup(current, setup_.group_);
	    for (unsigned int i = 0 ; i < current.size() ; ++i)
	    {
		double dif = current[i] - path.states[0].vals[i];
		d += dif * dif;
	    }
	    d = sqrt(d);
	    
	    // decide whether we place the current state in front of the path
	    int includeFirst = (d > 0.1) ? 1 : 0;
	    double offset = 0.0;
	    traj.points.resize(path.states.size() + includeFirst);

	    if (includeFirst)
	    {
		// add the current state at the start of the path, with an offset
		planningMonitor_->getRobotState()->copyParamsJoints(traj.points[0].positions, setup_.groupJointNames_);
		traj.points[0].time = 0.0;
		offset = 0.3 + d;
	    }
	    
	    // add the actual path
	    planning_models::StateParams *sp = planningMonitor_->getKinematicModel()->newStateParams();
	    for (unsigned int i = 0 ; i < path.states.size() ; ++i)
	    {
		traj.points[i + includeFirst].time = offset + path.times[i];
		sp->setParamsGroup(path.states[i].vals, setup_.group_);
		sp->copyParamsJoints(traj.points[i + includeFirst].positions, setup_.groupJointNames_);
	    }
	    delete sp;
	}
	
	void printPath(const motion_planning_msgs::KinematicPath &path)
	{
	    for (unsigned int i = 0 ; i < path.states.size() ; ++i)
	    {
		std::stringstream ss;
		for (unsigned int j = 0 ; j < path.states[i].vals.size() ; ++j)
		    ss << path.states[i].vals[j] << " ";
		ROS_DEBUG(ss.str().c_str());
	    }
	}
	
	bool fixStartState(planning_models::StateParams &st)
	{
	    bool result = true;
	    
	    // just in case the system is a bit outside bounds, we enforce the bounds
	    st.enforceBounds();
	    
	    // if the state is not valid, we try to fix it
	    if (!planningMonitor_->isStateValidOnPath(&st))
	    {
		// try 2% change in each component
		planning_models::StateParams temp(st);
		int count = 0;
		do
		{
		    temp = st;
		    temp.perturbStateGroup(0.02, setup_.group_);
		    count++;
		} while (!planningMonitor_->isStateValidOnPath(&temp) && count < 50);
		
		if (!planningMonitor_->isStateValidOnPath(&temp))
		    st = temp;
		else
		    result = false;
	    }
	    return result;
	}
	
	bool fillStartState(std::vector<motion_planning_msgs::KinematicJoint> &start_state)
	{
	    // get the current state
	    planning_models::StateParams st(*planningMonitor_->getRobotState());
	    bool result = fixStartState(st);
	    
	    if (!result)
		ROS_DEBUG("Starting state for the robot is in collision and attempting to fix it failed.");
	    
	    // fill in start state with current one
	    std::vector<planning_models::KinematicModel::Joint*> joints;
	    planningMonitor_->getKinematicModel()->getJoints(joints);
	    
	    start_state.resize(joints.size());
	    for (unsigned int i = 0 ; i < joints.size() ; ++i)
	    {
		start_state[i].header.frame_id = planningMonitor_->getFrameId();
		start_state[i].header.stamp = planningMonitor_->lastJointStateUpdate();
		start_state[i].joint_name = joints[i]->name;
		st.copyParamsJoint(start_state[i].value, joints[i]->name);
	    }
	    
	    return result;
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
	
	
    private:
	
	ros::NodeHandle                                  nh_;
	MoveArmSetup                                     &setup_;
	actionlib::SingleGoalActionServer<MoveArmAction> as_;
	
	planning_environment::PlanningMonitor           *planningMonitor_;
	tf::TransformListener                           *tf_;
	
	ros::Publisher                                   displayPathPublisher_;
	ros::Publisher                                   visMarkerPublisher_;
	
    };
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_arm");
    
    move_arm::MoveArmSetup setup;
    ROS_INFO("Starting action...");
    
    if (setup.configure())
    {
	move_arm::MoveArm move_arm(setup);
	ROS_INFO("Action started");
	ros::spin();
    }
    
    return 0;
}
