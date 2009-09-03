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

#include <pr2_msgs/SetLaserTrajCmd.h>
#include <pr2_msgs/LaserTrajCmd.h>

#include <pr2_msgs/SetLaserTrajCmd.h>
#include <pr2_msgs/SetPeriodicCmd.h>


namespace move_arm
{


  class TeleopArm
  {
    public:
  enum TeleopState { NONE,
    DRIVING,
    PLANNING,
    ARM_TUCKED,
    TUCKING_ARM
  };
      TeleopArm(MoveArmSetup &setup) : setup_(setup), move_arm(nh_, "move_" + setup_.group_)
      {	    
        nh_.param<bool>("~use_planning", use_planning_, false);
        pubTwist_ = nh_.advertise<geometry_msgs::Twist>("/r_arm_cartesian_twist_controller/command", 1);
        vizPub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1024);
        ctrl_ = nh_.serviceClient<experimental_controllers::TrajectoryStart>("/r_arm_joint_waypoint_controller/TrajectoryStart", true);

        pose_teleop_sub_ = nh_.subscribe("~command",1, &TeleopArm::poseCallback, this);

        subSpaceNav_ = nh_.subscribe("/spacenav/joy", 1, &TeleopArm::twistCallback, this);
        planningMonitor_ = setup_.planningMonitor_;

        double tmp;
        nh_.param<double>("~drive_wait_time",tmp,5.0);
        drive_wait_time_ = ros::Duration(tmp);

        nh_.param<double>("~arm_control_interval",tmp,0.5);
        arm_control_interval_ = ros::Duration(tmp);


        base_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/joy_cmd_vel",1);
        joy_sub_ = nh_.subscribe("/teleop_cmd_vel", 1, &TeleopArm::baseControlCallback, this);
        drive_request_ = false;
        arm_tucked_ = false;
        tucking_arm_ = false;

        tuck_arm_joint_values_.push_back(0.0);
        tuck_arm_joint_values_.push_back(1.3);
        tuck_arm_joint_values_.push_back(-1.57);

        tuck_arm_joint_values_.push_back(-1.57);
        tuck_arm_joint_values_.push_back(0.0);
        tuck_arm_joint_values_.push_back(0.0);
        tuck_arm_joint_values_.push_back(0.0);

        last_base_ctrl_ = ros::Time();
        last_arm_ctrl_ = ros::Time();

        state_ = NONE;
        arm_state_ = NONE;
        laser_state_ = NONE;
      }
	
      ~TeleopArm(void)
      {
      }
	
      void run(void)
      {
        ros::spin();
      }
    private:

      bool setLaserParams(const pr2_msgs::LaserTrajCmd &nav_cmd)
      {
        pr2_msgs::SetLaserTrajCmd srv;
        ros::ServiceClient laser_client = nh_.serviceClient<pr2_msgs::SetLaserTrajCmd>("/laser_tilt_controller/set_traj_cmd",true);
        srv.request.command = nav_cmd;
        if(laser_client.call(srv))
          return true;
        else
          return false;
      }

      bool setLaserParams(const std::string &profile, const double &period, const double &amplitude, const double &offset)
      {
        ros::ServiceClient laser_client = nh_.serviceClient<pr2_msgs::SetPeriodicCmd>("/laser_tilt_controller/set_periodic_cmd",true);
        pr2_msgs::SetPeriodicCmd srv;
        srv.request.command.profile = profile;
        srv.request.command.period = period;
        srv.request.command.amplitude = amplitude;
        srv.request.command.offset = offset;
        if(laser_client.call(srv))
          return true;
        else
          return false;
      }

	
      geometry_msgs::Pose getCurrentPose(void)
      {
        planning_models::KinematicModel *kmodel = planningMonitor_->getKinematicModel()->clone();
        boost::shared_ptr<planning_models::StateParams> start(new planning_models::StateParams(*planningMonitor_->getRobotState()));
        start->enforceBounds();
        kmodel->computeTransforms(start->getParams());

        tf::Pose currentEff = kmodel->getJoint(setup_.groupJointNames_.back())->after->globalTrans;
        geometry_msgs::Pose currentEffMsg;
        tf::poseTFToMsg(currentEff, currentEffMsg);
        delete kmodel;
        return currentEffMsg;
      }
	
	
      bool tryPose(const geometry_msgs::Pose &destEffMsg)
      {
        bool res = true;
	    
        planning_models::KinematicModel *kmodel = planningMonitor_->getKinematicModel()->clone();
        boost::shared_ptr<planning_models::StateParams> start(new planning_models::StateParams(*planningMonitor_->getRobotState()));
        start->enforceBounds();
	    
        geometry_msgs::Pose currentEffMsg = getCurrentPose();
	    
        ros::ServiceClient ik_client = nh_.serviceClient<manipulation_srvs::IKService>("pr2_ik_right_arm/ik_service", true);
        geometry_msgs::PoseStamped destEffMsgStmp;
        destEffMsgStmp.pose = destEffMsg;
        destEffMsgStmp.header.stamp = planningMonitor_->lastJointStateUpdate();
        destEffMsgStmp.header.frame_id = planningMonitor_->getFrameId();
	    
        showArrow(destEffMsgStmp);
	    
        std::vector<double> solution;
        if (computeIK(ik_client, destEffMsgStmp, solution))
        {
          ROS_INFO("Starting at %f, %f, %f", currentEffMsg.position.x, currentEffMsg.position.y, currentEffMsg.position.z);
          boost::shared_ptr<planning_models::StateParams> goal(new planning_models::StateParams(*start));
          goal->setParamsJoints(solution, setup_.groupJointNames_);
          goal->enforceBounds();
		
          kmodel->computeTransforms(goal->getParams());
          tf::Pose goalEff = kmodel->getJoint(setup_.groupJointNames_.back())->after->globalTrans;
          ROS_INFO("Going to %f, %f, %f", goalEff.getOrigin().x(), goalEff.getOrigin().y(), goalEff.getOrigin().z());
		
          std::vector< boost::shared_ptr<planning_models::StateParams> > path;
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
	
      bool tryTwist(const geometry_msgs::Twist &tw, double frac)
      { 
        geometry_msgs::Pose currentEffMsg = getCurrentPose();
        geometry_msgs::Pose destEffMsg = tf::addDelta(currentEffMsg, tw, frac);
        return tryPose(destEffMsg);
      }

      void baseControlCallback(const geometry_msgs::Twist::ConstPtr& msg)
      {
        geometry_msgs::Twist base_cmd = *msg;
        if(state_ != DRIVING)
          state_ = DRIVING;

        if(arm_state_ == TUCKING_ARM)
        {
          ROS_INFO("Tucking arm");
          return;
        }
        if(arm_state_ != ARM_TUCKED)
        {
          tuckArmSafely();
        }

        if(laser_state_ != DRIVING)
        {
          pr2_msgs::LaserTrajCmd nav_cmd;
          nav_cmd.profile = "blended_linear";
          nav_cmd.pos.resize(3);
          nav_cmd.time.resize(3);
          nav_cmd.max_rate = 5.0;
          nav_cmd.max_accel = 5.0;
          nav_cmd.pos[0] = 1.2;
          nav_cmd.pos[1] = -0.7;
          nav_cmd.pos[2] = 1.2;
          nav_cmd.time[0] = 0.0;
          nav_cmd.time[1] = 1.8;
          nav_cmd.time[2] = 2.025;
          if(!setLaserParams(nav_cmd))
          {
            ROS_ERROR("Could not set laser params");
            return;
          }
          laser_state_ = DRIVING;
        }
       
        if(arm_state_ == ARM_TUCKED)
        {
          base_cmd_pub_.publish(base_cmd);
          last_base_ctrl_ = ros::Time::now();
        }
      }

      void tuckArmSafely()
      {
        arm_state_ = TUCKING_ARM;
        if(laser_state_ != PLANNING)
        {
          if(!setLaserParams("linear",20.0,0.75,0.25))
          {
            ROS_ERROR("Could not set laser periodic command for move arm");
            return;
          }
          laser_state_ = PLANNING;
        }
        ros::Duration wait_for_laser(2.0);
        wait_for_laser.sleep();

        ros::Duration timeout_long(20.0);
        ros::Duration timeout_small(1.0);
        move_arm::MoveArmGoal goal;
        goal.goal_constraints.joint_constraint.resize(setup_.groupJointNames_.size());
        for(unsigned int i=0; i<setup_.groupJointNames_.size(); i++)
        {
          goal.goal_constraints.joint_constraint[i].joint_name = setup_.groupJointNames_[i];
          goal.goal_constraints.joint_constraint[i].value.push_back(tuck_arm_joint_values_[i]);
          goal.goal_constraints.joint_constraint[i].tolerance_below.push_back(0.1);
          goal.goal_constraints.joint_constraint[i].tolerance_above.push_back(0.1);
        }
        bool done = false;
        move_arm.sendGoal(goal);
        while(!done)
        {
          bool finished_before_timeout = move_arm.waitForGoalToFinish(timeout_small);
          if (finished_before_timeout)
          {
            std::cout << "Final state is " << move_arm.getTerminalState().toString() << std::endl;
            done = true;
            arm_state_ = ARM_TUCKED;
          }
          else
          {
            std::cerr << "Not yet achieved goal" << std::endl;
          }
          last_base_ctrl_ = ros::Time::now();
          ros::spinOnce();
        }
      }


      void poseCallback(const geometry_msgs::PoseStampedConstPtr &cmd)
      {

        if(ros::Time::now() - last_arm_ctrl_ < arm_control_interval_)
        {
          return;
        }

        if(arm_state_ == TUCKING_ARM)
        {
          return;
        }

        geometry_msgs::PoseStamped my_cmd = *cmd;
        geometry_msgs::PoseStamped goalEffMsgStamped;

        geometry_msgs::Pose goalEffMsg;

        my_cmd.header.stamp = ros::Time();
        tf_.transformPose(planningMonitor_->getFrameId(),my_cmd,goalEffMsgStamped);
        goalEffMsg = goalEffMsgStamped.pose;

        if(ros::Time::now() - last_base_ctrl_ > drive_wait_time_ && state_ == DRIVING)
        {
          if(laser_state_ != PLANNING)
          {
            if(!setLaserParams("linear",20.0,0.75,0.25))
            {
              ROS_ERROR("Could not set laser periodic command for move arm");
              return;
            }
            laser_state_ = PLANNING;
          }
          state_ = PLANNING;
          arm_state_ = PLANNING;
        }
        if(state_ == DRIVING)
          return;
//	    ROS_INFO("Got twist: %f, %f, %f : %f, %f, %f", tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.x, tw.angular.y, tw.angular.z);
	    
        geometry_msgs::Pose currentEffMsg = getCurrentPose();

        tf::Pose currentEffTF;
        tf::poseMsgToTF(currentEffMsg,currentEffTF);
        tf::Quaternion current_quat = currentEffTF.getRotation();
        tf::Vector3 current_vec = currentEffTF.getOrigin();

        tf::Pose goalEffTF;
        tf::poseMsgToTF(goalEffMsg,goalEffTF);
        tf::Quaternion goal_quat = goalEffTF.getRotation();
        tf::Vector3 goal_vec = goalEffTF.getOrigin();

        bool r = true;
        for (int i = 0; i < 5 && r; i++)
        {
          double frac = 1.0/(i+1.0);              
          geometry_msgs::Pose interp_pose_msg;
          tf::Vector3 interp_vec = current_vec.lerp(goal_vec,frac);
          tf::Quaternion interp_quat = current_quat.slerp(goal_quat,frac);
          tf::Pose interp_pose(interp_quat,interp_vec);
          tf::poseTFToMsg(interp_pose,interp_pose_msg);
          r = !tryPose(interp_pose_msg);
        }
        last_arm_ctrl_ = ros::Time::now();
      }
	
      void twistCallback(const joy::JoyConstPtr &joy)
      {
/*
        if(ros::Time::now() - last_base_ctrl_ > drive_wait_time_)
        {
          drive_request_ = false;
        }

        if(drive_request_)
          return;


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
*/
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
	    
        planning_models::StateParams *sp = planningMonitor_->getKinematicModel()->newStateParams();
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
      ros::Subscriber                        pose_teleop_sub_;
      ros::ServiceClient                     ctrl_;
      ros::Publisher                         pubTwist_;
      ros::Publisher                         vizPub_;
      planning_environment::PlanningMonitor *planningMonitor_;
      bool                                   use_planning_;

      bool arm_tucked_;
      bool drive_request_;
      bool tucking_arm_;

      ros::Publisher base_cmd_pub_;
      ros::Subscriber joy_sub_;

      ros::Time last_base_ctrl_;
      ros::Time last_arm_ctrl_;

      std::vector<double> tuck_arm_joint_values_;

      ros::Duration drive_wait_time_;
      ros::Duration arm_control_interval_;

      tf::TransformListener tf_;

      TeleopState state_;
      TeleopState arm_state_;
      TeleopState laser_state_;
  };
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_arm");
  move_arm::MoveArmSetup setup;
  if (!setup.configure())
  {
    exit(1);
  }
  move_arm::TeleopArm ta(setup);    
  ros::spin();
  return 0;
}
