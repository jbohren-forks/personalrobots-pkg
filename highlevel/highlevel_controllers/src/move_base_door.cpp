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

/**
 * @mainpage
 *
 * @htmlinclude manifest.html
 *
 * @b move_base is...
 *
 * <hr>
 *
 *  @section usage Usage
 *  @verbatim
 *  $ move_base
 *  @endverbatim
 *
 * <hr>
 *
 * @section topic ROS topics
 *
 * Subscribes to (name/type):
 * - @b 
 *
 * Publishes to (name / type):
 * - @b 
 *
 *  <hr>
 *
 * @section parameters ROS parameters
 *
 * - None
 **/

#include <highlevel_controllers/move_base.hh>
#include <navfn.h>

#include <robot_msgs/JointTraj.h>
#include <angles/angles.h>

using namespace costmap_2d;
using namespace deprecated_msgs;

namespace ros 
{
  namespace highlevel_controllers 
  {

    /**
     * @brief Specialization for the SBPL planner
     */
    class MoveBaseDoor: public MoveBase {
   
      public:

        MoveBaseDoor();

        virtual ~MoveBaseDoor();

      private:

        /**
         * @brief Builds a plan from current state to goal state
         */
        virtual bool dispatchCommands();

        virtual bool makePlan();

        void checkTrajectory(const robot_msgs::JointTraj& trajectory, robot_msgs::JointTraj &return_trajectory);

        bool computeOrientedFootprint(double x_i, double y_i, double th_i, const std::vector<deprecated_msgs::Point2DFloat32>& footprint_spec, std::vector<deprecated_msgs::Point2DFloat32>& oriented_footprint);

        trajectory_rollout::CostmapModel *cost_map_model_;

        double goal_x_, goal_y_, goal_theta_;

        double current_x_, current_y_, current_theta_;

        double dist_waypoints_max_;

        robot_msgs::JointTraj path_;

        robot_msgs::JointTraj valid_path_;

        std::string base_trajectory_controller_topic_;

        double circumscribed_radius_;

        double inscribed_radius_;
    };
    
    
    MoveBaseDoor::MoveBaseDoor()
      : MoveBase(), goal_x_(0.0), goal_y_(0.0), goal_theta_(0.0)
    {
      initialize();
      cost_map_model_ = new trajectory_rollout::CostmapModel(*costMap_);
      ros::Node::instance()->param<std::string>("~move_base_door/trajectory_control_topic",base_trajectory_controller_topic_,"base/trajectory_controller/trajectory_command");
      ros::Node::instance()->advertise<robot_msgs::JointTraj>(base_trajectory_controller_topic_,1);
      ros::Node::instance()->param("~costmap_2d/circumscribed_radius", circumscribed_radius_, 0.46);
      ros::Node::instance()->param("~costmap_2d/inscribed_radius", inscribed_radius_, 0.325);
   }

    bool MoveBaseDoor::makePlan()
    {

    }

    bool MoveBaseDoor::computeOrientedFootprint(double x_i, double y_i, double theta_i, const std::vector<deprecated_msgs::Point2DFloat32>& footprint_spec, std::vector<deprecated_msgs::Point2DFloat32>& oriented_footprint)
    {
      //if we have no footprint... do nothing
      if(footprint_spec.size() < 3)
        return -1.0;

      //build the oriented footprint
      double cos_th = cos(theta_i);
      double sin_th = sin(theta_i);
      for(unsigned int i = 0; i < footprint_spec.size(); ++i){
        Point2DFloat32 new_pt;
        new_pt.x = x_i + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
        new_pt.y = y_i + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
        oriented_footprint.push_back(new_pt);
      }
      return true;
    }

    void MoveBaseDoor::checkTrajectory(const robot_msgs::JointTraj& trajectory, robot_msgs::JointTraj &return_trajectory)
    {
      double theta;
      return_trajectory = trajectory;
      deprecated_msgs::Point2DFloat32 position;
      std::vector<deprecated_msgs::Point2DFloat32> oriented_footprint;

      for(int i=0; i < (int) trajectory.points.size(); i++)
      {
        position.x  = trajectory.points[i].positions[0];
        position.y  = trajectory.points[i].positions[1];
        theta = trajectory.points[i].positions[2];

        computeOrientedFootprint(position.x,position.y,theta, footprint_, oriented_footprint);

        if(cost_map_model_->footprintCost(position, oriented_footprint, inscribed_radius_, circumscribed_radius_) > 0)
        {
          continue;
        }
        else
        {
          int last_valid_point = std::max<int>(i-2,0);
          if(last_valid_point == 0)
          {
            return_trajectory.points.resize(last_valid_point+1);
          }
          else
          {
            return_trajectory.points.resize(0);
          }
          break;
        }       
      }
    };

    MoveBaseDoor::~MoveBaseDoor(){
    }

    bool MoveBaseDoor::dispatchCommands()
    {
      ROS_DEBUG("Planning for new goal...\n");
      robot_msgs::JointTraj valid_path;
      stateMsg.lock();
      current_x_ = stateMsg.pos.x;
      current_y_ = stateMsg.pos.y;
      current_theta_ = stateMsg.pos.th;

      goal_x_ = stateMsg.goal.x;
      goal_y_ = stateMsg.goal.y;
      goal_theta_ = stateMsg.goal.th;
      stateMsg.unlock();

      // For now plan is a straight line with waypoints about 2.5 cm apart
      double dist = sqrt( pow(goal_x_-current_x_,2) + pow(goal_y_-current_y_,2));        
      int num_intervals = std::min<int>(1,(int) dist/dist_waypoints_max_);
      int num_path_points = num_intervals + 1;

      path_.set_points_size(num_path_points);

      for(int i=0; i< num_intervals; i++)
      {
        path_.points[i].set_positions_size(3);
        path_.points[i].positions[0] = current_x_ + (double) i * (goal_x_-current_x_)/num_intervals ;
        path_.points[i].positions[1] = current_y_ + (double) i * (goal_y_-current_y_)/num_intervals ;
        path_.points[i].positions[2] = angles::normalize_angle(current_theta_ + (double) i * angles::normalize_angle(goal_theta_-current_theta_)/num_intervals) ;
        path_.points[i].time = 0.0;
      }

      path_.points[num_intervals].set_positions_size(3);
      path_.points[num_intervals].positions[0] = goal_x_;
      path_.points[num_intervals].positions[1] = goal_y_;
      path_.points[num_intervals].positions[2] = goal_theta_;

      checkTrajectory(path_,valid_path_);
      // First criteria is that we have had a sufficiently recent sensor update to trust perception and that we have a valid plan. This latter
      // case is important since we can end up with an active controller that becomes invalid through the planner looking ahead. 
      // We want to be able to stop the robot in that case
      bool planOk = checkWatchDog();

      if(!planOk)
      {
        ROS_ERROR("MoveBaseDoor: Plan not ok");
        return false;
      }

      ros::Node::instance()->publish(base_trajectory_controller_topic_,valid_path_);         
      return true;
    }
  }
}


int main(int argc, char** argv)
{
  ros::init(argc,argv); 
  ros::Node rosnode("move_base_door");

  ros::highlevel_controllers::MoveBaseDoor node;

  try 
  {
    node.run();
  }
  catch(char const* e)
  {
    std::cout << e << std::endl;
  }
  return(0);
}
