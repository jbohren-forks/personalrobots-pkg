/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sbpl_door_planner_action/sbpl_door_planner_action.h>
#include <ros/node.h>

using namespace std;
using namespace ros;
using namespace robot_actions;
using namespace door_handle_detector;

SBPLDoorPlanner::SBPLDoorPlanner(ros::Node& ros_node, tf::TransformListener& tf) : 
  Action<robot_msgs::Door, robot_msgs::Door>(ros_node.getName()), ros_node_(ros_node), tf_(tf),
  cost_map_ros_(NULL),cm_getter_(this)
{
  ros_node_.param("~allocated_time", allocated_time_, 1.0);
  ros_node_.param("~forward_search", forward_search_, true);

  ros_node_.param("~planner_type", planner_type_, string("ARAPlanner"));
  ros_node_.param("~plan_stats_file", plan_stats_file_, string("/tmp/move_base_sbpl.log"));

  double inscribed_radius, circumscribed_radius;
  //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
  ros_node_.param("~costmap/inscribed_radius", inscribed_radius, 0.325);
  ros_node_.param("~costmap/circumscribed_radius", circumscribed_radius, 0.46);
  ros_node_.param("~global_frame", global_frame_, std::string("odom_combined"));
  ros_node_.param("~robot_base_frame", robot_base_frame_, std::string("base_link"));

  cost_map_ros_ = new costmap_2d::Costmap2DROS(ros_node_,tf_,std::string(""));
  cost_map_ros_->getCostmapCopy(cost_map_);

  sleep(10.0);
  ros_node_.param<double>("~door_thickness", door_env_.door_thickness, 0.05);
  ros_node_.param<double>("~arm/min_workspace_angle",   door_env_.arm_min_workspace_angle, -M_PI);
  ros_node_.param<double>("~arm/max_workspace_angle",   door_env_.arm_max_workspace_angle, M_PI);
  ros_node_.param<double>("~arm/min_workspace_radius",  door_env_.arm_min_workspace_radius, 0.0);
  ros_node_.param<double>("~arm/max_workspace_radius",  door_env_.arm_max_workspace_radius, 1.1);
  ros_node_.param<double>("~arm/discretization_angle",  door_env_.door_angle_discretization_interval, 0.01);

  double shoulder_x, shoulder_y;
  ros_node_.param<double>("~arm/shoulder/x",shoulder_x, 0.0);
  ros_node_.param<double>("~arm/shoulder/y",shoulder_y, -0.188);
  door_env_.shoulder.x = shoulder_x;
  door_env_.shoulder.y = shoulder_y;

//  ros_node_.advertise<visualization_msgs::Polyline>("~robot_footprint", 1);
  ros_node_.advertise<visualization_msgs::Polyline>("~global_plan", 1);

  robot_msgs::Point pt;
  //create a square footprint
  pt.x = inscribed_radius;
  pt.y = -1 * (inscribed_radius);
  footprint_.push_back(pt);
  pt.x = -1 * (inscribed_radius);
  pt.y = -1 * (inscribed_radius);
  footprint_.push_back(pt);
  pt.x = -1 * (inscribed_radius);
  pt.y = inscribed_radius;
  footprint_.push_back(pt);
  pt.x = inscribed_radius;
  pt.y = inscribed_radius;
  footprint_.push_back(pt);
};

SBPLDoorPlanner::~SBPLDoorPlanner()
{
//  ros_node_.unadvertise("~robot_footprint");
  ros_node_.unadvertise("~global_plan");

  if(cost_map_ros_ != NULL)
    delete cost_map_ros_;
}

void PrintUsage(char *argv[])
{
  printf("USAGE: %s <cfg file>\n", argv[0]);
}

bool SBPLDoorPlanner::initializePlannerAndEnvironment(const robot_msgs::Door &door)
{
  door_env_.door = door;
  // First set up the environment
  try 
  {
    // We're throwing int exit values if something goes wrong, and
    // clean up any new instances in the catch clause. The sentry
    // gets destructed when we go out of scope, so unlock() gets
    // called no matter what.
    // sentry<SBPLPlannerNode> guard(this);  
    cm_access_.reset(mpglue::createCostmapAccessor(&cm_getter_));
    cm_index_.reset(mpglue::createIndexTransform(&cm_getter_));
    
    FILE *action_fp;
    std::string filename = "sbpl_action_tmp.txt";
    std::string sbpl_action_string;
    if(!ros::Node::instance()->getParam("~sbpl_action_file", sbpl_action_string))
      return false;
    action_fp = fopen(filename.c_str(),"wt");
    fprintf(action_fp,"%s",sbpl_action_string.c_str());
    fclose(action_fp);

    double nominalvel_mpersecs, timetoturn45degsinplace_secs;
    ros::Node::instance()->param("~nominalvel_mpersecs", nominalvel_mpersecs, 0.4);
    ros::Node::instance()->param("~timetoturn45degsinplace_secs", timetoturn45degsinplace_secs, 0.6);
    // Could also sanity check the other parameters...
    env_.reset(mpglue::SBPLEnvironment::createXYThetaDoor(cm_access_, cm_index_, footprint_, nominalvel_mpersecs,timetoturn45degsinplace_secs, filename, 0, door));
	
    boost::shared_ptr<SBPLPlanner> sbplPlanner;
    if ("ARAPlanner" == planner_type_)
    {
      sbplPlanner.reset(new ARAPlanner(env_->getDSI(), forward_search_));
    }
    else if ("ADPlanner" == planner_type_)
    {
      sbplPlanner.reset(new ADPlanner(env_->getDSI(), forward_search_));
    }
    else 
    {
      ROS_ERROR("in MoveBaseSBPL ctor: invalid planner_type_ \"%s\",use ARAPlanner or ADPlanner",planner_type_.c_str());
      throw int(5);
    }
    pWrap_.reset(new mpglue::SBPLPlannerWrap(env_, sbplPlanner));
  }
  catch (int ii) 
  {
    exit(ii);
  }
  return true;
}

bool SBPLDoorPlanner::removeDoor()
{
  const std::vector<robot_msgs::Point> door_polygon = door_handle_detector::getPolygon(door_env_.door,door_env_.door_thickness); 
  if(cost_map_.setConvexPolygonCost(door_polygon,costmap_2d::FREE_SPACE))
    return true;
  return false;
}

bool SBPLDoorPlanner::makePlan(const pr2_robot_actions::Pose2D &start, const pr2_robot_actions::Pose2D &goal, robot_msgs::JointTraj &path)
{
  ROS_INFO("[replan] getting fresh copy of costmap");
  lock_.lock();
  cost_map_ros_->getCostmapCopy(cost_map_);
  lock_.unlock();
  
  // XXXX this is where we would cut out the doors in our local copy
  removeDoor();
  
  ROS_INFO("[replan] replanning...");
  try {
    // Update costs
    for (mpglue::index_t ix(cm_access_->getXBegin()); ix < cm_access_->getXEnd(); ++ix) {
      for (mpglue::index_t iy(cm_access_->getYBegin()); iy < cm_access_->getYEnd(); ++iy) {
	mpglue::cost_t cost;
	if (cm_access_->getCost(ix, iy, &cost)) { // always succeeds though
	  // Note that ompl::EnvironmentWrapper::UpdateCost() will
	  // check if the cost has actually changed, and do nothing
	  // if it hasn't.  It internally maintains a list of the
	  // cells that have actually changed, and this list is what
	  // gets "flushed" to the planner (a couple of lines
	  // further down).
	  env_->UpdateCost(ix, iy, cost);
	}
      }
    }
    
    // Tell the planner about the changed costs. Again, the called
    // code checks whether anything has really changed before
    // embarking on expensive computations.
    pWrap_->flushCostChanges(true);
		
    // Assume the robot is constantly moving, so always set start.
    // Maybe a bit inefficient, but not as bad as "changing" the
    // goal when it hasn't actually changed.
    pWrap_->setStart(start.x, start.y, start.th);	
    pWrap_->setGoal(goal.x, goal.y, goal.th);
	
    // BTW if desired, we could call pWrap_->forcePlanningFromScratch(true)...
	
    // Invoke the planner, updating the statistics in the process.
    // The returned plan might be empty, but it will not contain a
    // null pointer.  On planner errors, the createPlan() method
    // throws a std::exception.
    boost::shared_ptr<mpglue::waypoint_plan_t> plan(pWrap_->createPlan());
	
    if (plan->empty()) 
    {
      ROS_ERROR("No plan found\n");
      return false;
    }
    
    path.points.clear();	// just paranoid
    mpglue::PlanConverter::convertToJointTraj(plan.get(), &path);
    return true;
  }
  catch (std::runtime_error const & ee) {
    ROS_ERROR("runtime_error in makePlan(): %s\n", ee.what());
  }
  return false;
}

robot_actions::ResultStatus SBPLDoorPlanner::execute(const robot_msgs::Door& door_msg_in, robot_msgs::Door& feedback)
{
  robot_msgs::JointTraj path;
  robot_msgs::Door door;
  if(!updateGlobalPose())
  {
    return robot_actions::ABORTED;
  }

  ROS_INFO("Current position: %f %f %f",global_pose_2D_.x,global_pose_2D_.y,global_pose_2D_.th);

  if (!door_handle_detector::transformTo(tf_,global_frame_,door_msg_in,door))
  {
    return robot_actions::ABORTED;
  }
  cout  << door;

  ROS_INFO("Initializing planner and environment");
   
  if(!initializePlannerAndEnvironment(door))
  {
    ROS_ERROR("Door planner and environment not initialized");
    return robot_actions::ABORTED;
  }

  ROS_INFO("Door planner and environment initialized");
  goal_.x = (door.frame_p1.x+door.frame_p2.x)/2.0;
  goal_.y = (door.frame_p1.x+door.frame_p2.x)/2.0;
  goal_.th = 0.0;

  ROS_INFO("Goal: %f %f %f",goal_.x,goal_.y,goal_.th);
  if(!isPreemptRequested())
  {
    if(!makePlan(global_pose_2D_, goal_, path))
    {
      return robot_actions::ABORTED;      
    }
  }

  if(!isPreemptRequested())
  {
    publishPath(path,"global_plan",0,1,0,0);
  }
  return robot_actions::SUCCESS;
}

void SBPLDoorPlanner::publishPath(const robot_msgs::JointTraj &path, std::string topic, double r, double g, double b, double a)
{
    visualization_msgs::Polyline gui_path_msg;
    gui_path_msg.header.frame_id = global_frame_;

    //given an empty path we won't do anything
    if(path.get_points_size() > 0){
      // Extract the plan in world co-ordinates, we assume the path is all in the same frame
      gui_path_msg.header.stamp = door_env_.door.header.stamp;
      gui_path_msg.set_points_size(path.get_points_size());
      for(unsigned int i=0; i < path.get_points_size(); i++){
        gui_path_msg.points[i].x = path.points[i].positions[0];
        gui_path_msg.points[i].y = path.points[i].positions[1];
        gui_path_msg.points[i].z = path.points[i].positions[2];
      }
    }

    gui_path_msg.color.r = r;
    gui_path_msg.color.g = g;
    gui_path_msg.color.b = b;
    gui_path_msg.color.a = a;

    ros_node_.publish("~" + topic, gui_path_msg);
}


bool SBPLDoorPlanner::updateGlobalPose()
{
  tf::Stamped<tf::Pose> robot_pose;
  robot_pose.setIdentity();
  robot_pose.frame_id_ = robot_base_frame_;
  robot_pose.stamp_ = ros::Time();

  try
  {
    tf_.transformPose(global_frame_, robot_pose, global_pose_);
  }
  catch(tf::LookupException& ex) {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ConnectivityException& ex) {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ExtrapolationException& ex) {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    return false;
  }
  global_pose_2D_ = getPose2D(global_pose_);
  return true;
}

pr2_robot_actions::Pose2D SBPLDoorPlanner::getPose2D(const tf::Stamped<tf::Pose> &pose)
{
  pr2_robot_actions::Pose2D tmp_pose;
  double useless_pitch, useless_roll, yaw;
  pose.getBasis().getEulerZYX(yaw, useless_pitch, useless_roll);
  tmp_pose.x = pose.getOrigin().x();
  tmp_pose.y = pose.getOrigin().y();
  tmp_pose.th = yaw;
  return tmp_pose;
}

