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
 *********************************************************************/

#include <doors_core/door_reactive_planner.h>

using namespace tf;
using namespace ros;
using namespace std;
using namespace door_reactive_planner;


DoorReactivePlanner::DoorReactivePlanner(ros::Node &ros_node, TransformListener &tf, costmap_2d::Costmap2D* cost_map, std::string path_frame_id, std::string costmap_frame_id):node_(ros_node),tf_(tf)
{
   cost_map_ = cost_map;
   cost_map_model_ = new base_local_planner::CostmapModel(*cost_map_);
   path_frame_id_ = path_frame_id;
   costmap_frame_id_ = costmap_frame_id;
}

void DoorReactivePlanner::getParams()
{
   node_.param<double>("~door_reactive_planner/min_distance_from_obstacles_",min_distance_from_obstacles_,0.03);
   node_.param<double>("~door_reactive_planner/max_waypoint_distance",dist_waypoints_max_,0.025);
   node_.param<double>("~door_reactive_planner/max_waypoint_rot_distance",dist_rot_waypoints_max_,0.1);

   node_.param<double>("~door_reactive_planner/max_explore_distance",max_explore_distance_,2.0);
   node_.param<double>("~door_reactive_planner/horizontal_explore_distance",horizontal_explore_distance_,0.15);
   node_.param<double>("~door_reactive_planner/max_explore_distance",max_explore_distance_,2.0);
   node_.param<double>("~door_reactive_planner/max_explore_delta_angle",max_explore_delta_angle_,M_PI/4.0+0.1);
   node_.param<double>("~door_reactive_planner/door_goal_distance",door_goal_distance_,0.7);

   node_.param<int>("~door_reactive_planner/num_explore_paths",num_explore_paths_,32);
   node_.param<bool>("~door_reactive_planner/choose_straight_line_trajectory",choose_straight_line_trajectory_,false);
   node_.param<double>("~door_reactive_planner/circumscribed_radius",circumscribed_radius_,0.46);
   node_.param<double>("~door_reactive_planner/inscribed_radius",inscribed_radius_,0.325);

   min_inflated_cost_ = cost_map_->computeCost(min_distance_from_obstacles_);
   cell_distance_from_obstacles_ = std::max<int>((int) (min_distance_from_obstacles_/dist_waypoints_max_),1);

    robot_msgs::Point pt;
    //create a square footprint
    pt.x = inscribed_radius_;
    pt.y = -1 * (inscribed_radius_);
    footprint_.push_back(pt);
    pt.x = -1 * (inscribed_radius_);
    pt.y = -1 * (inscribed_radius_);
    footprint_.push_back(pt);
    pt.x = -1 * (inscribed_radius_);
    pt.y = inscribed_radius_;
    footprint_.push_back(pt);
    pt.x = inscribed_radius_;
    pt.y = inscribed_radius_;
    footprint_.push_back(pt);
}

void DoorReactivePlanner::setDoor(robot_msgs::Door door_msg_in)
{
   //Assumption is that the normal points in the direction we want to travel through the door
   robot_msgs::Door door;
   door_handle_detector::transformTo(tf_,path_frame_id_,door_msg_in,door);

   vector_along_door_.x = door.normal.y;
   vector_along_door_.y = -door.normal.x;
   vector_along_door_.z = 0.0;    

   centerline_angle_ = atan2(door.normal.y,door.normal.x);

   double door_midpoint_x = 0.5*(door.frame_p1.x + door.frame_p2.x);
   double door_midpoint_y = 0.5*(door.frame_p1.y + door.frame_p2.y);

   goal_.x = door_midpoint_x + door_goal_distance_*door.normal.x;
   goal_.y = door_midpoint_y + door_goal_distance_*door.normal.y;
   goal_.th = centerline_angle_;

   door_information_set_ = true;
}

bool DoorReactivePlanner::computeOrientedFootprint(const robot_actions::Pose2D &position, const std::vector<robot_msgs::Point>& footprint_spec, std::vector<robot_msgs::Point>& oriented_footprint)
{
   if(footprint_spec.size() < 3)//if we have no footprint... do nothing
   {
      ROS_ERROR("No footprint available");
      return -1.0;
   }
   double cos_th = cos(position.th);
   double sin_th = sin(position.th);
   for(unsigned int i = 0; i < footprint_spec.size(); ++i) //build the oriented footprint
   {
      robot_msgs::Point new_pt;
      new_pt.x = position.x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
      new_pt.y = position.y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
      oriented_footprint.push_back(new_pt);
   }
   return true;
}

double DoorReactivePlanner::distance(const robot_actions::Pose2D &p, const robot_actions::Pose2D &q)
{
   return sqrt(pow(p.x-q.x,2)+pow(p.y-q.y,2));
}

bool DoorReactivePlanner::createLinearPath(const robot_actions::Pose2D &cp,const robot_actions::Pose2D &fp, std::vector<robot_actions::Pose2D> &return_path)
{
   ROS_DEBUG("Creating trajectory from: (%f,%f) to (%f,%f)",cp.x,cp.y,fp.x,fp.y);
   robot_actions::Pose2D temp;
   double dist_trans = distance(cp,fp);        
   double dist_rot = fabs(angles::normalize_angle(cp.th-fp.th));

   int num_intervals = std::max<int>(1,(int) (dist_trans/dist_waypoints_max_));
   num_intervals = std::max<int> (num_intervals, (int) (dist_rot/dist_rot_waypoints_max_));

   double delta_x = (fp.x-cp.x)/num_intervals;
   double delta_y = (fp.y-cp.y)/num_intervals;
   double delta_theta = angles::normalize_angle((fp.th-cp.th)/num_intervals);


   for(int i=0; i< num_intervals; i++)
   {
      temp.x = cp.x + i * delta_x;
      temp.y = cp.y + i * delta_y;
      temp.th =  angles::normalize_angle(cp.th + i*delta_theta);
      return_path.push_back(temp);
   }

   temp.x = fp.x;
   temp.y = fp.y;
   temp.th = angles::normalize_angle(fp.th);
   return_path.push_back(temp);
   return true;
}

void DoorReactivePlanner::getFinalPosition(const robot_actions::Pose2D &current_position, const double &delta_angle, const double &distance_to_centerline, robot_actions::Pose2D &end_position)
{
   double new_explore_distance;
   double global_explore_angle;
   double explore_distance;
   int multiplier = 1;
   if(delta_angle < 0)
      multiplier = -1;

   new_explore_distance = fabs((horizontal_explore_distance_+multiplier*distance_to_centerline)/sin(delta_angle));

   if(isnan(new_explore_distance) || isinf(new_explore_distance))//when delta_angle is zero, i.e. straight line trajectory
   {
      new_explore_distance = FLT_MAX;
   }
   explore_distance = std::min<double>(max_explore_distance_,new_explore_distance);

   global_explore_angle = centerline_angle_ + delta_angle;
   end_position.x = current_position.x + cos(global_explore_angle) * explore_distance;
   end_position.y = current_position.y + sin(global_explore_angle) * explore_distance;
   end_position.th = centerline_angle_;
}


bool DoorReactivePlanner::makePlan(const robot_actions::Pose2D &start, std::vector<robot_actions::Pose2D> &best_path)
{
   if(!door_information_set_)
   {
      ROS_ERROR("Door information not set");
      return false;
   }
   robot_actions::Pose2D end_position;
   std::vector<robot_actions::Pose2D> checked_path;
   std::vector<robot_actions::Pose2D> linear_path;

   double max_distance_to_goal(FLT_MAX);

   double delta_theta = max_explore_delta_angle_/num_explore_paths_;
   double distance_to_centerline;

   best_path.clear();
   distance_to_centerline = (start.x-goal_.x)*vector_along_door_.x + (start.y-goal_.y)*vector_along_door_.y;

   for(int i=0; i < num_explore_paths_; i++)
   {
      linear_path.clear();
      checked_path.clear();
      getFinalPosition(start,i*delta_theta, distance_to_centerline,end_position);
      createLinearPath(start,end_position,linear_path);
      checkPath(linear_path,path_frame_id_,checked_path,costmap_frame_id_);
      if(checked_path.size() > 0)
      {
         double new_distance = distance(checked_path.back(),goal_);
         if( new_distance < max_distance_to_goal)
         {
            best_path = checked_path;
            max_distance_to_goal = new_distance;
         }
         if(i == 0 && choose_straight_line_trajectory_)
            return true;
      }
   } 
   for(int i=1; i < num_explore_paths_; i++)
   {
      linear_path.clear();
      checked_path.clear();
      getFinalPosition(start,-i*delta_theta,distance_to_centerline,end_position);
      createLinearPath(start,end_position,linear_path);
      checkPath(linear_path,path_frame_id_,checked_path,costmap_frame_id_);
      if(checked_path.size() > 0)
      {
         double new_distance = distance(checked_path.back(),goal_);
         if(new_distance < max_distance_to_goal)
         {
            best_path = checked_path;
            max_distance_to_goal = new_distance;
         }
      }
   } 
   return true;
}

void DoorReactivePlanner::checkPath(const std::vector<robot_actions::Pose2D> &path, const std::string &path_frame_id, std::vector<robot_actions::Pose2D> &return_path, std::string &costmap_frame_id)
{
   double theta;
   double cost;
   int last_valid_point;
   robot_msgs::Point position;

   return_path = path;

   for(int i=0; i < (int) path.size(); i++)
   {
      std::vector<robot_msgs::Point> oriented_footprint;
      robot_actions::Pose2D out_pose;

      transform2DPose(path[i],path_frame_id, out_pose, costmap_frame_id);
      computeOrientedFootprint(out_pose, footprint_, oriented_footprint);
      position.x = out_pose.x;
      position.y = out_pose.y;
      theta = out_pose.th;
      if(cost_map_model_->footprintActualCost(position, oriented_footprint, inscribed_radius_, circumscribed_radius_,cost))
      {
         if(cost < min_inflated_cost_) //0.03
         {
            ROS_DEBUG("Point %d: position: %f, %f, %f is not in collision",i,out_pose.x,out_pose.y,out_pose.th);
            continue;
         }
      }
      ROS_DEBUG("Point %d: position: %f, %f, %f is in collision",i,out_pose.x,out_pose.y,out_pose.th);
      ROS_DEBUG("Radius inscribed: %f, circumscribed: %f",inscribed_radius_, circumscribed_radius_);
      for(int j=0; j < (int) oriented_footprint.size(); j++)
         ROS_DEBUG("Footprint point: %d is : %f,%f",j,oriented_footprint[j].x,oriented_footprint[j].y);
  
      last_valid_point = std::max<int>(i-cell_distance_from_obstacles_,0);
      if(last_valid_point > 0)
      {
         return_path.resize(last_valid_point+1);
      }
      else
      {
         return_path.resize(0);
      }
      break;
   }
   ROS_DEBUG("Return path has %d points",return_path.size());
}

void DoorReactivePlanner::transformPath(const std::vector<robot_actions::Pose2D> &path_in, const std::string &frame_in, std::vector<robot_actions::Pose2D> &path_out, const std::string &frame_out)
{
   path_out.resize((int) path_in.size());      
   for(int i=0; i < (int) path_out.size(); i++)
   {
      transform2DPose(path_in[i],frame_in,path_out[i],frame_out);
   }
}

void DoorReactivePlanner::transform2DPose(const robot_actions::Pose2D &path_in, const std::string original_frame_id, robot_actions::Pose2D &path_out, const std::string &transform_frame_id)
{
   btQuaternion qt;
   tf::Stamped<tf::Pose> pose;
   tf::Stamped<tf::Pose> transformed_pose;
   double useless_pitch, useless_roll, yaw;

   qt.setEulerZYX(path_in.th, 0, 0);
   pose.setData(btTransform(qt, btVector3(path_in.x, path_in.y, 0)));
   pose.frame_id_ = original_frame_id;
   pose.stamp_ = ros::Time();

   try
   {
      tf_.transformPose(transform_frame_id, pose, transformed_pose);
   }
   catch(tf::LookupException& ex) 
   {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
   }
   catch(tf::ConnectivityException& ex) 
   {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
   }
   catch(tf::ExtrapolationException& ex) 
   {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
   }

   path_out.x = transformed_pose.getOrigin().x();
   path_out.y = transformed_pose.getOrigin().y();
   transformed_pose.getBasis().getEulerZYX(yaw, useless_pitch, useless_roll);
   path_out.th = (double)yaw;      
};
