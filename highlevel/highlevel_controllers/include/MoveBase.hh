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

#ifndef HIGHLEVEL_CONTROLLERS_MOVE_BASE_H
#define HIGHLEVEL_CONTROLLERS_MOVE_BASE_H

#include <HighlevelController.hh>
#include <VelocityControllers.hh>

// Costmap used for the map representation
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/basic_observation_buffer.h>

//Ransac ground filter used to see small obstacles
#include <ransac_ground_plane_extraction/ransac_ground_plane_extraction.h>
#include <pr2_msgs/PlaneStamped.h>
#include <std_msgs/Point.h>
#include <std_msgs/Vector3.h>

// Generic OMPL plan representation (will move into <sbpl_util/...> or <ompl_tools/...> later)
#include <plan_wrap.h>

// Message structures used
#include <robot_msgs/Planner2DState.h>
#include <robot_msgs/Planner2DGoal.h>
#include <std_msgs/LaserScan.h>
#include <std_msgs/BaseVel.h>
#include <std_msgs/RobotBase2DOdom.h>
#include <std_msgs/PointCloud.h>

// For transform support
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_notifier.h>

// Laser projection
#include "laser_scan/laser_scan.h"

// Thread suppport
#include <boost/thread.hpp>

#include <list>

namespace robot_filter {
  class RobotFilter;
}

namespace ros {
  namespace highlevel_controllers {

    class MoveBase : public HighlevelController<robot_msgs::Planner2DState, robot_msgs::Planner2DGoal> {

    public:
      
      typedef std::vector<std_msgs::Point2DFloat32> footprint_t;
      
      /**
       * @brief Constructor
       */
      MoveBase();

      virtual ~MoveBase();
      
      footprint_t const & getFootprint() const;
      
    protected:

      /**
       * @brief Accessor for the cost map. Use mainly for initialization
       * of specialized map strunture for planning
       */
      const CostMap2D& getCostMap() const {return *costMap_;}

      /**
       * @brief A handler to be over-ridden in the derived class to handle a diff stream from the
       * cost map. This is called on a map update, which means it will be on a separate thread to the main
       * node control loop. The client will obtain the lock, so derived class implementations
       * can assume excusinve access
       */
      virtual void handleMapUpdates(const std::vector<unsigned int>& updates){}

      /**
       * @brief When planning has failed should reset the cost map to clear persistent
       * dynamic obstacles. This is important to provide some hysterisis when interleaving planning
       */
      virtual void handlePlanningFailure();

      /**
       * @brief Overwrites the current plan with a new one. Will handle suitable publication
       * @see publishPlan
       */
      void updatePlan(const std::list<std_msgs::Pose2DFloat32>& newPlan);
      
      /**
       * @brief Overwrites the current plan with a new one. Will handle suitable publication
       * @see publishPlan
       */
      void updatePlan(ompl::waypoint_plan_t const & newPlan);

      void updateCostMap(bool static_map_reset);
      
      /**
       * @brief test the current plan for collisions with obstacles
       */
      bool inCollision() const;
      
      /**
       * @brief projection of the robot outline onto the ground plane.
       */
      footprint_t footprint_;
      
      /**
       * @brief Will process a goal update message.
       */
      virtual void updateGoalMsg();

    private:
      /**
       * @brief Use global pose to publish currrent state data at the start of each cycle
       */
      virtual void updateStateMsg();

      /**
       * @brief Evaluate if final goal x, y, th has been reached.
       */
      virtual bool goalReached();

      /**
       * @brief Send velocity commands based on local plan. Should check for consistency of
       * local plan.
       */
      virtual bool dispatchCommands();      

      /**
       * @brief On deactivation we should stop the robot
       */
      virtual void handleDeactivation();

      /**
       * @brief Callbacks for perceiving obstalces
       */
      void baseScanCallback();
      void tiltScanCallback();
      void tiltCloudCallback();
      void tiltCloudCallbackTransform(const tf::MessageNotifier<std_msgs::PointCloud>::MessagePtr& message);
      void groundPlaneCloudCallback();
      void stereoCloudCallback();
      void groundPlaneCallback();

      /**
       * @brief Robot odometry call back
       */
      void odomCallback();

      void updateGlobalPose();

      /**
       * @brief Issue zero velocity commands
       */
      void stopRobot();

      void publishPath(bool isGlobal, const std::list<std_msgs::Pose2DFloat32>& path);

      void publishFootprint(double x, double y, double th);

      /**
       * Utility to publish the local cost map around the robot
       */
      void publishLocalCostMap();

      /**
       * @brief Utility to publish updates in terms of cells with cost (red) and new free space (blue)
       */
      void publishFreeSpaceAndObstacles();

      /**
       * @brief Utility for comparing 2 points to be within a required distance, which is specified as a
       * configuration parameter of the object.
       */
      bool withinDistance(double x1, double y1, double th1, double x2, double y2, double th2) const ;

      /**
       * @brief Tests if all the buffers are appropriately up to date
       */
      bool checkWatchDog() const;

      // Callback messages
      std_msgs::LaserScan baseScanMsg_; /**< Filled by subscriber with new base laser scans */
      std_msgs::LaserScan tiltScanMsg_; /**< Filled by subscriber with new tilte laser scans */
      std_msgs::PointCloud tiltCloudMsg_; /**< Filled by subscriber with new tilte laser scans */
      std_msgs::PointCloud groundPlaneCloudMsg_; /**< Filled by subscriber with point clouds */
      std_msgs::PointCloud stereoCloudMsg_; /**< Filled by subscriber with point clouds */
      std_msgs::RobotBase2DOdom odomMsg_; /**< Odometry in the odom frame picked up by subscription */
      laser_scan::LaserProjection projector_; /**< Used to project laser scans */

      tf::TransformListener tf_; /**< Used to do transforms */

      // Observation Buffers are dynamically allocated since their constructors take
      // arguments bound by lookup to the param server. This could be chnaged with some reworking of how paramaters
      // are looked up. If we wanted to generalize this node further, we could use a factory pattern to dynamically
      // load specific derived classes. For that we would need a hand-shaking pattern to register subscribers for them
      // with this node
      costmap_2d::BasicObservationBuffer* baseScanBuffer_;
      costmap_2d::BasicObservationBuffer* tiltScanBuffer_;
      costmap_2d::BasicObservationBuffer* lowObstacleBuffer_;
      costmap_2d::BasicObservationBuffer* stereoCloudBuffer_;

      /** Should encapsulate as a controller wrapper that is not resident in the trajectory rollout package */
      VelocityController* controller_;

      CostMap2D* costMap_; /**< The cost map mainatined incrementally from laser scans */

      CostMapAccessor* ma_; /**< Sliding read-only window on the cost map */

      tf::Stamped<tf::Pose> global_pose_; /**< The global pose in the map frame */

      std_msgs::RobotBase2DOdom base_odom_; /**< Odometry in the base frame */

      /** Parameters that will be passed on initialization soon */
      double baseLaserMaxRange_; /**< Used in laser scan projection */
      double tiltLaserMaxRange_; /**< Used in laser scan projection */

      std::list<std_msgs::Pose2DFloat32>  plan_; /**< The 2D plan in grid co-ordinates of the cost map */

      // Filter parameters
      double minZ_;
      double maxZ_;
      double robotWidth_;

      // Thread control
      void mapUpdateLoop();
      boost::thread *map_update_thread_; /*<! Thread to process laser data and apply to the map */
      bool active_; /*<! Thread control parameter */
      double map_update_frequency_;

      // Tolerances for determining if goal has been reached
      double yaw_goal_tolerance_;
      double xy_goal_tolerance_;
      
      //Robot filter
      robot_filter::RobotFilter* filter_;
      tf::MessageNotifier<std_msgs::PointCloud>* tiltLaserNotifier_;


      //ground plane extraction
      ransac_ground_plane_extraction::RansacGroundPlaneExtraction ground_plane_extractor_;
      pr2_msgs::PlaneStamped groundPlaneMsg_;
      pr2_msgs::PlaneStamped ground_plane_;
      std_msgs::PointCloud *filtered_cloud_;
      double ransac_distance_threshold_;
    };
  }
}
#endif
