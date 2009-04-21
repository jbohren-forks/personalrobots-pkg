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
*********************************************************************/
#include <people_aware_nav/move_base_constrained.h>
#include <robot_msgs/Polygon3D.h>

using namespace base_local_planner;
using namespace costmap_2d;
using namespace navfn;
using namespace robot_actions;
using std::vector;
using robot_msgs::Point;
using robot_msgs::Point32;

namespace people_aware_nav {

  MoveBaseConstrained::MoveBaseConstrained(ros::Node& ros_node, tf::TransformListener& tf) : 
    Action<ConstrainedGoal, Pose2D>(ros_node.getName()), ros_node_(ros_node), tf_(tf),
    run_planner_(true), tc_(NULL), planner_cost_map_ros_(NULL), controller_cost_map_ros_(NULL), 
    planner_(NULL), valid_plan_(false) {

    //get some parameters that will be global to the move base node
    ros_node_.param("~global_frame", global_frame_, std::string("map"));
    ros_node_.param("~robot_base_frame", robot_base_frame_, std::string("base_link"));
    ros_node_.param("~controller_frequency", controller_frequency_, 20.0);

    //for display purposes
    ros_node_.advertise<robot_msgs::Polyline2D>("gui_path", 1);
    ros_node_.advertise<robot_msgs::Polyline2D>("local_path", 1);
    ros_node_.advertise<robot_msgs::Polyline2D>("robot_footprint", 1);

    //for comanding the base
    ros_node_.advertise<robot_msgs::PoseDot>("cmd_vel", 1);

    //pass on some parameters to the components of the move base node if they are not explicitly overridden 
    //(perhaps the controller and the planner could operate in different frames)
    if(!ros_node_.hasParam("~base_local_planner/global_frame")) ros_node_.setParam("~base_local_planner/global_frame", global_frame_);
    if(!ros_node_.hasParam("~base_local_planner/robot_base_frame")) ros_node_.setParam("~base_local_planner/robot_base_frame", robot_base_frame_);
    if(!ros_node_.hasParam("~navfn/global_frame")) ros_node_.setParam("~navfn/global_frame", global_frame_);
    if(!ros_node_.hasParam("~navfn/robot_base_frame")) ros_node_.setParam("~navfn/robot_base_frame", robot_base_frame_);

    ros_node_.param("~inscribed_radius", inscribed_radius_, 0.325);
    ros_node_.param("~circumscribed_radius", circumscribed_radius_, 0.46);
    ros_node_.param("~inflation_radius", inflation_radius_, 0.55);

    //pass on inlfation parameters to the planner's costmap if they're not set explicitly
    if(!ros_node_.hasParam("~navfn/costmap/inscribed_radius")) ros_node_.setParam("~navfn/costmap/inscribed_radius", inscribed_radius_);
    if(!ros_node_.hasParam("~navfn/costmap/circumscribed_radius")) ros_node_.setParam("~navfn/costmap/circumscribed_radius", circumscribed_radius_);
    if(!ros_node_.hasParam("~navfn/costmap/inflation_radius")) ros_node_.setParam("~navfn/costmap/inflation_radius", inflation_radius_);

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_cost_map_ros_ = new Costmap2DROS(ros_node_, tf_, std::string("navfn"));
    planner_cost_map_ros_->getCostMapCopy(planner_cost_map_);

    //initialize the NavFn planner
    planner_ = new NavfnROS(ros_node_, tf_, planner_cost_map_);
    ROS_INFO("MAP SIZE: %d, %d", planner_cost_map_.cellSizeX(), planner_cost_map_.cellSizeY());

    //pass on inlfation parameters to the controller's costmap if they're not set explicitly
    if(!ros_node_.hasParam("~base_local_planner/costmap/inscribed_radius")) ros_node_.setParam("~base_local_planner/costmap/inscribed_radius", inscribed_radius_);
    if(!ros_node_.hasParam("~base_local_planner/costmap/circumscribed_radius")) ros_node_.setParam("~base_local_planner/costmap/circumscribed_radius", circumscribed_radius_);
    if(!ros_node_.hasParam("~base_local_planner/costmap/inflation_radius")) ros_node_.setParam("~base_local_planner/costmap/inflation_radius", inflation_radius_);

    //create the ros wrapper for the controller's cost_map... and initializer a pointer we'll use with the underlying map
    controller_cost_map_ros_ = new Costmap2DROS(ros_node_, tf_, std::string("base_local_planner"));
    controller_cost_map_ros_->getCostMapCopy(controller_cost_map_);

    robot_msgs::Point pt;
    //create a square footprint
    pt.x = inscribed_radius_ + .01;
    pt.y = -1 * (inscribed_radius_ + .01);
    footprint_.push_back(pt);
    pt.x = -1 * (inscribed_radius_ + .01);
    pt.y = -1 * (inscribed_radius_ + .01);
    footprint_.push_back(pt);
    pt.x = -1 * (inscribed_radius_ + .01);
    pt.y = inscribed_radius_ + .01;
    footprint_.push_back(pt);
    pt.x = inscribed_radius_ + .01;
    pt.y = inscribed_radius_ + .01;
    footprint_.push_back(pt);

    //give the robot a nose
    pt.x = circumscribed_radius_;
    pt.y = 0;
    footprint_.push_back(pt);

    //create a trajectory controller
    tc_ = new TrajectoryPlannerROS(ros_node_, tf_, controller_cost_map_, footprint_, &planner_cost_map_);

    //TODO:spawn planning thread here?
  }

  MoveBaseConstrained::~MoveBaseConstrained(){
    if(planner_ != NULL)
      delete planner_;

    if(tc_ != NULL)
      delete tc_;

    if(planner_cost_map_ros_ != NULL)
      delete planner_cost_map_ros_;

    if(controller_cost_map_ros_ != NULL)
      delete controller_cost_map_ros_;
  }

  void MoveBaseConstrained::clearRobotFootprint(Costmap2D& cost_map){
    double useless_pitch, useless_roll, yaw;
    global_pose_.getBasis().getEulerZYX(yaw, useless_pitch, useless_roll);

    //get the oriented footprint of the robot
    std::vector<robot_msgs::Point> oriented_footprint = tc_->drawFootprint(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), yaw);

    //set the associated costs in the cost map to be free
    if(!cost_map.setConvexPolygonCost(oriented_footprint, costmap_2d::FREE_SPACE))
      return;

    double max_inflation_dist = inflation_radius_ + inscribed_radius_;

    //make sure to re-inflate obstacles in the affected region
    cost_map.reinflateWindow(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), max_inflation_dist, max_inflation_dist);

  }

  void MoveBaseConstrained::makePlan(const ConstrainedGoal& goal){

    ROS_DEBUG_NAMED("move", "Entering makePlan with goal %f, %f and size %u", 
                    goal.x, goal.y, goal.forbidden.get_points_size());

    //since this gets called on handle activate
    if(planner_cost_map_ros_ == NULL)
      return;

    //make a plan for controller
    planner_cost_map_ros_->getCostMapCopy(planner_cost_map_);

    //make sure we clear the robot's footprint from the cost map
    clearRobotFootprint(planner_cost_map_);

    // set cost of forbidden region
    vector<Point> polygon;
    for (vector<Point32>::const_iterator iter = goal.forbidden.points.begin(); iter!=goal.forbidden.points.end(); ++iter) {
      Point p;
      p.x = iter->x;
      p.y = iter->y;
      p.z = iter->z;
      polygon.push_back(p);
    }
    planner_cost_map_.setConvexPolygonCost(polygon, costmap_2d::LETHAL_OBSTACLE);

    ROS_DEBUG_NAMED("move", "Modified costmap");

    
    /*
    std::vector<robot_actions::Pose2D> global_plan;

    Pose2D goal_pose;
    goal_pose.header = goal.header;
    goal_pose.x = goal.x;
    goal_pose.y = goal.y;
    goal_pose.z = goal.z;
    goal_pose.th = goal.th;
    */
    
    std::vector<robot_msgs::PoseStamped> global_plan;

    robot_msgs::PoseStamped goal_pose;
    goal_pose.header = goal.header;
    tf::PoseTFToMsg(tf::Pose(btQuaternion(goal.th, 0, 0), 
                             btVector3(goal.x, goal.y, goal.z)),
                    goal_pose.pose);

    bool valid_plan = planner_->makePlan(goal_pose, global_plan);

    ROS_DEBUG_NAMED("move", "Planner found plan");

    //we'll also push the goal point onto the end of the plan to make sure orientation is taken into account
    global_plan.push_back(goal_pose);

    lock_.lock();
    //copy over the new global plan
    valid_plan_ = valid_plan;
    global_plan_ = global_plan;
    lock_.unlock();

    publishPath(global_plan, "gui_path", 0.0, 1.0, 0.0, 0.0);
    
    ROS_DEBUG_NAMED ("move", "Exiting makePlan");
  }

  void MoveBaseConstrained::updateGlobalPose(){
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = robot_base_frame_;
    robot_pose.stamp_ = ros::Time();

    try{
      tf_.transformPose(global_frame_, robot_pose, global_pose_);
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    }
  }


  void MoveBaseConstrained::prunePlan(){
    lock_.lock();
    std::vector<robot_msgs::PoseStamped>::iterator it = global_plan_.begin();
    while(it != global_plan_.end()){
      const robot_msgs::PoseStamped& w = *it;
      // Fixed error bound of 2 meters for now. Can reduce to a portion of the map size or based on the resolution
      double x_diff = global_pose_.getOrigin().x() - w.pose.position.x;
      double y_diff = global_pose_.getOrigin().y() - w.pose.position.y;
      double distance = sqrt(x_diff * x_diff + y_diff * y_diff);
      if(distance < 1){
        ROS_DEBUG("Nearest waypoint to <%f, %f> is <%f, %f>\n", global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), w.pose.position.x, w.pose.position.y);
        break;
      }
      it = global_plan_.erase(it);
    }
    lock_.unlock();
  }

  robot_actions::ResultStatus MoveBaseConstrained::execute(const ConstrainedGoal& goal, robot_actions::Pose2D& feedback){
    //update the goal
    goal_ = goal;

    //update the global pose
    updateGlobalPose();

    //first... make a plan to the goal
    makePlan(goal_);

    ros::Duration cycle_time = ros::Duration(1.0 / controller_frequency_);
    while(!isPreemptRequested()){
      //get the start time of the loop
      ros::Time start_time = ros::Time::now();

      //update the global pose
      updateGlobalPose();

      //update feedback to correspond to our current position
      double useless_pitch, useless_roll, yaw;
      global_pose_.getBasis().getEulerZYX(yaw, useless_pitch, useless_roll);
      feedback.header.frame_id = global_frame_;
      feedback.header.stamp = ros::Time::now();
      feedback.x = global_pose_.getOrigin().x();
      feedback.y = global_pose_.getOrigin().y();
      feedback.z = 0.0;
      feedback.th = yaw;

      //push the feedback out
      update(feedback);


      //make sure to update the cost_map we'll use for this cycle
      controller_cost_map_ros_->getCostMapCopy(controller_cost_map_);

      //make sure that we clear the robot footprint in the cost map
      clearRobotFootprint(controller_cost_map_);

      //prune the plan before we pass it to the controller
      prunePlan();

      struct timeval start, end;
      double start_t, end_t, t_diff;
      gettimeofday(&start, NULL);

      //check that the observation buffers for the costmap are current
      if(!controller_cost_map_ros_->isCurrent()){
        ROS_WARN("Sensor data is out of date, we're not going to allow commanding of the base for safety");
        continue;
      }

      bool valid_control = false;
      robot_msgs::PoseDot cmd_vel;
      std::vector<robot_msgs::PoseStamped> local_plan;
      //pass plan to controller
      lock_.lock();
      if(valid_plan_){
        //get observations for the non-costmap controllers
        std::vector<Observation> observations;
        controller_cost_map_ros_->getMarkingObservations(observations);
        valid_control = tc_->computeVelocityCommands(global_plan_, cmd_vel, local_plan, observations);
      }
      else{
        //we don't have a valid plan... so we want to stop
        cmd_vel.vel.vx = 0.0;
        cmd_vel.vel.vy = 0.0;
        cmd_vel.ang_vel.vz = 0.0;
      }

      //give the base the velocity command
      ros_node_.publish("cmd_vel", cmd_vel);

      lock_.unlock();

      //check for success
      if(tc_->goalReached())
        return robot_actions::SUCCESS;


      //if we don't have a valid control... we need to re-plan explicitly
      if(!valid_control){
        makePlan(goal_);
      }

      //for visualization purposes
      publishPath(global_plan_, "gui_path", 0.0, 1.0, 0.0, 0.0);
      publishPath(local_plan, "local_path", 0.0, 0.0, 1.0, 0.0);
      publishFootprint();

      gettimeofday(&end, NULL);
      start_t = start.tv_sec + double(start.tv_usec) / 1e6;
      end_t = end.tv_sec + double(end.tv_usec) / 1e6;
      t_diff = end_t - start_t;
      ROS_DEBUG("Full control cycle: %.9f Valid control: %d, Vel Cmd (%.2f, %.2f, %.2f)", t_diff, valid_control, cmd_vel.vel.vx, cmd_vel.vel.vy, cmd_vel.ang_vel.vz);

      //sleep the remainder of the cycle
      if(!sleepLeftover(start_time, cycle_time))
        ROS_WARN("Controll loop missed its desired cycle time of %.4f", cycle_time.toSec());
    }
    return robot_actions::PREEMPTED;
  }

  bool MoveBaseConstrained::sleepLeftover(ros::Time start, ros::Duration cycle_time){
    ros::Time expected_end = start + cycle_time;
    ///@todo: because durations don't handle subtraction properly right now
    ros::Duration sleep_time = ros::Duration((expected_end - ros::Time::now()).toSec()); 

    if(sleep_time < ros::Duration(0.0)){
      return false;
    }

    sleep_time.sleep();
    return true;
  }

  void MoveBaseConstrained::resetCostMaps(){
    planner_cost_map_ros_->resetMapOutsideWindow(5.0, 5.0);
    controller_cost_map_ros_->resetMapOutsideWindow(5.0, 5.0);
  }

  void MoveBaseConstrained::publishFootprint(){
    double useless_pitch, useless_roll, yaw;
    global_pose_.getBasis().getEulerZYX(yaw, useless_pitch, useless_roll);
    std::vector<robot_msgs::Point> footprint = tc_->drawFootprint(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), yaw);
    robot_msgs::Polyline2D footprint_msg;
    footprint_msg.header.frame_id = global_frame_;
    footprint_msg.set_points_size(footprint.size());
    footprint_msg.color.r = 1.0;
    footprint_msg.color.g = 0;
    footprint_msg.color.b = 0;
    footprint_msg.color.a = 0;
    for(unsigned int i = 0; i < footprint.size(); ++i){
      footprint_msg.points[i].x = footprint[i].x;
      footprint_msg.points[i].y = footprint[i].y;
    }
    ros_node_.publish("robot_footprint", footprint_msg);
  }

  void MoveBaseConstrained::publishPath(const std::vector<robot_msgs::PoseStamped>& path, std::string topic, double r, double g, double b, double a){
    // Extract the plan in world co-ordinates
    robot_msgs::Polyline2D gui_path_msg;
    gui_path_msg.header.frame_id = global_frame_;
    gui_path_msg.set_points_size(path.size());
    for(unsigned int i=0; i < path.size(); i++){
      gui_path_msg.points[i].x = path[i].pose.position.x;
      gui_path_msg.points[i].y = path[i].pose.position.y;
    }

    gui_path_msg.color.r = r;
    gui_path_msg.color.g = g;
    gui_path_msg.color.b = b;
    gui_path_msg.color.a = a;

    ros_node_.publish(topic, gui_path_msg);
  }

};

namespace pan=people_aware_nav;
namespace ra=robot_actions;

int main(int argc, char** argv){
  ros::init(argc, argv);
  ros::Node ros_node("move_base_node");
  tf::TransformListener tf(ros_node, true, ros::Duration(10));
  
  pan::MoveBaseConstrained move_base(ros_node, tf);
  ra::ActionRunner runner(20.0);
  runner.connect<pan::ConstrainedGoal, pan::ConstrainedMoveBaseState, ra::Pose2D>(move_base);
  runner.run();

  ros_node.spin();

  return(0);

}
