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
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef TRAJECTORY_ROLLOUT_TRAJECTORY_CONTROLLER_ROS_H_
#define TRAJECTORY_ROLLOUT_TRAJECTORY_CONTROLLER_ROS_H_

#include <ros/node.h>
#include <old_costmap_2d/obstacle_map_accessor.h>
#include <old_costmap_2d/costmap_2d.h>
#include <trajectory_rollout/world_model.h>
#include <trajectory_rollout/point_grid.h>
#include <trajectory_rollout/costmap_model.h>
#include <trajectory_rollout/voxel_grid_model.h>
#include <trajectory_rollout/trajectory_controller.h>

#include <trajectory_rollout/planar_laser_scan.h>

#include <tf/transform_datatypes.h>

#include <deprecated_msgs/Point2DFloat32.h>
#include <robot_msgs/PoseDot.h>

#include <laser_scan/LaserScan.h>
#include <tf/message_notifier.h>
#include <tf/transform_listener.h>
#include <laser_scan/laser_scan.h>

#include <boost/thread.hpp>

#include <string>

namespace trajectory_rollout {
  /**
   * @class TrajectoryControllerROS
   * @brief A ROS wrapper for the trajectory controller that queries the param server to construct a controller
   */
  class TrajectoryControllerROS{
    public:
      /**
       * @brief  Constructs the ros wrapper
       * @param ros_node The node that is running the controller, used to get parameters from the parameter server
       * @param tf A reference to a transform listener
       * @param global_frame The frame in which the controller will operate
       * @param ma An CostMapAccessor that allows the trajectory controller to query a costmap
       * @param footprint_spec A polygon representing the footprint of the robot. (Must be convex)
       * @param inscribed_radius The inscribed radius of the robot
       * @param circumscribed_radius The circumscribed radius of the robot
       */
      TrajectoryControllerROS(ros::Node& ros_node, tf::TransformListener& tf, std::string global_frame,
          const old_costmap_2d::CostMapAccessor& ma, std::vector<deprecated_msgs::Point2DFloat32> footprint_spec,
          double inscribed_radius, double cirumscribed_radius);

      /**
       * @brief  Destructor for the wrapper
       */
      ~TrajectoryControllerROS();
      
      /**
       * @brief  Used for display purposes, allows the footprint of the robot to be drawn in visualization tools
       * @param x_i The x position of the robot
       * @param y_i The y position of the robot
       * @param theta_i The orientation of the robot
       * @return A vector of points in world coordinates that correspond to the verticies of the robot's footprint 
       */
      std::vector<deprecated_msgs::Point2DFloat32> drawFootprint(double x_i, double y_i, double theta_i);

      /**
       * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
       * @param global_plan The plan to pass to the controller
       * @param global_pose The current pose of the robot in world space 
       * @param global_vel The current velocity of the robot in world space
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @param local_plan Will be set from the points of the selected trajectory for display purposes
       * @param observations A vector of updates from the robot's sensors in world space, is sometimes unused depending on the model
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(const std::list<deprecated_msgs::Pose2DFloat32>& global_plan, 
          const tf::Stamped<tf::Pose>& global_pose, 
          const robot_msgs::PoseDot& global_vel, 
          robot_msgs::PoseDot& cmd_vel,
          std::list<deprecated_msgs::Pose2DFloat32>& localPlan,
          const std::vector<old_costmap_2d::Observation>& observations = std::vector<old_costmap_2d::Observation>(0));

      /**
       * @brief  Returns the local goal the robot is pursuing
       * @param x Will be set to the x position of the goal in world coordinates 
       * @param y Will be set to the y position of the goal in world coordinates 
       * @return 
       */
      void getLocalGoal(double& x, double& y);

      void baseScanCallback(const tf::MessageNotifier<laser_scan::LaserScan>::MessagePtr& message);
      void tiltScanCallback(const tf::MessageNotifier<laser_scan::LaserScan>::MessagePtr& message);

    private:
      WorldModel* world_model_; ///< @brief The world model that the controller will use
      TrajectoryController* tc_; ///< @brief The trajectory controller
      tf::MessageNotifier<laser_scan::LaserScan>* base_scan_notifier_; ///< @brief Used to guarantee that a transform is available for base scans
      tf::MessageNotifier<laser_scan::LaserScan>* tilt_scan_notifier_; ///< @brief Used to guarantee that a transform is available for tilt scans
      tf::TransformListener& tf_; ///< @brief Used for transforming point clouds
      std::string global_frame_; ///< @brief The frame in which the controller will run
      laser_scan::LaserProjection projector_; ///< @brief Used to project laser scans into point clouds
      boost::recursive_mutex lock_; ///< @brief A lock for accessing data in callbacks safely
      std::vector<PlanarLaserScan> laser_scans_; ///< @breif Storage for the last scan the lasers took... used for clearing free-space in front of the robot
      PointGrid* point_grid_; ///< @brief If using a freespace grid... we want to access it
      VoxelGridModel* voxel_grid_; ///< @brief If using a voxel grid... we want to access it
      bool freespace_model_;  ///< @brief Keep track of whether we are using a freespace controller or not
      double max_sensor_range_; ///< @brief Keep track of the effective maximum range of our sensors
  };

};

#endif
