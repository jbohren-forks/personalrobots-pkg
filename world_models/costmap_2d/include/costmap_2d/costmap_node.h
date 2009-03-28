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

// Costmap used for the map representation
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/basic_observation_buffer.h>

//Ransac ground filter used to see small obstacles
#include <pr2_msgs/PlaneStamped.h>
#include <robot_msgs/Point.h>
#include <robot_msgs/Vector3.h>

// Message structures used
#include <robot_msgs/PoseDot.h>
#include <laser_scan/LaserScan.h>
#include <deprecated_msgs/RobotBase2DOdom.h>
#include <robot_msgs/PointCloud.h>

// For transform support
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_notifier.h>

// Laser projection
#include "laser_scan/laser_scan.h"

// Thread suppport
#include <boost/thread.hpp>

// Static map service type (for sending costmap)
#include "robot_srvs/StaticMap.h"

//we'll take in a path as a vector of points
#include <deprecated_msgs/Point2DFloat32.h>

#include <list>

namespace robot_filter {
  class RobotFilter;
}

namespace ros {

  class CostMapNode{

    public:
      
    typedef std::vector<deprecated_msgs::Point2DFloat32> footprint_t;
      
    /**
     * @brief Constructor
     */
    CostMapNode();

    virtual ~CostMapNode();
      
    footprint_t const & getFootprint() const;

    double local_access_mapsize_;
      
    protected:

    /**
     * @brief Accessor for the cost map. Use mainly for initialization
     * of specialized map strunture for planning
     */
    const costmap_2d::CostMapAccessor& getCostMap() const {return *global_map_accessor_;}

    /**
     * @brief A handler to be over-ridden in the derived class to handle a diff stream from the
     * cost map. This is called on a map update, which means it will be on a separate thread to the main
     * node control loop. The client will obtain the lock, so derived class implementations
     * can assume excusinve access
     */
    virtual void handleMapUpdates(const std::vector<unsigned int>& updates){}


    void updateCostMap();
      
    /**
     * @brief projection of the robot outline onto the ground plane.
     */
    footprint_t footprint_;

    /**
     * @brief Aquire node level lock
     */
    void lock(){lock_.lock();}
  
    /**
     * @brief Release node level lock
     */
    void unlock(){lock_.unlock();}
      
    bool use_base_scan_;

    bool use_tilt_scan_;

    bool use_stereo_;

    bool use_low_obstacles_;

    protected:


    /**
     * @brief Callbacks for perceiving obstalces
     */
    void baseScanCallback(const tf::MessageNotifier<laser_scan::LaserScan>::MessagePtr& message);
    void tiltScanCallback();
    void tiltCloudCallback(const tf::MessageNotifier<robot_msgs::PointCloud>::MessagePtr& message);
    void baseCloudCallback(const tf::MessageNotifier<robot_msgs::PointCloud>::MessagePtr& message);
    void groundPlaneCloudCallback();
    void stereoCloudCallback();
    void groundPlaneCallback();

    /**
     * @brief costmap service callback
     */
    bool costmapCallback(robot_srvs::StaticMap::Request &req, robot_srvs::StaticMap::Response &res);

    void updateGlobalPose();

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
     * @brief Tests if all the buffers are appropriately up to date
     */
    bool checkWatchDog() const;

    // Callback messages
    laser_scan::LaserScan baseScanMsg_; /**< Filled by subscriber with new base laser scans */
    laser_scan::LaserScan tiltScanMsg_; /**< Filled by subscriber with new tilt laser scans */
    robot_msgs::PointCloud tiltCloudMsg_; /**< Filled by subscriber with new tilte laser scans */
    robot_msgs::PointCloud groundPlaneCloudMsg_; /**< Filled by subscriber with point clouds */
    robot_msgs::PointCloud stereoCloudMsg_; /**< Filled by subscriber with point clouds */
    deprecated_msgs::RobotBase2DOdom odomMsg_; /**< Odometry in the odom frame picked up by subscription */
    laser_scan::LaserProjection projector_; /**< Used to project laser scans */

    tf::TransformListener tf_; /**< Used to do transforms */

    std::string global_frame_; /**< Which is our global frame? Usually "map" */

    // Observation Buffers are dynamically allocated since their constructors take
    // arguments bound by lookup to the param server. This could be chnaged with some reworking of how paramaters
    // are looked up. If we wanted to generalize this node further, we could use a factory pattern to dynamically
    // load specific derived classes. For that we would need a hand-shaking pattern to register subscribers for them
    // with this node
    costmap_2d::BasicObservationBuffer* baseScanBuffer_;
    costmap_2d::BasicObservationBuffer* baseCloudBuffer_;
    costmap_2d::BasicObservationBuffer* tiltScanBuffer_;
    costmap_2d::BasicObservationBuffer* lowObstacleBuffer_;
    costmap_2d::BasicObservationBuffer* stereoCloudBuffer_;

    costmap_2d::CostMap2D* costMap_; /**< The cost map mainatined incrementally from laser scans */
    costmap_2d::CostMapAccessor* global_map_accessor_; /**< Read-only access to global cost map */
    costmap_2d::CostMapAccessor* local_map_accessor_; /**< Read-only access to a window on the cost map */

    robot_srvs::StaticMap::Response costmap_response_;

    tf::Stamped<tf::Pose> global_pose_; /**< The global pose in the map frame */

    deprecated_msgs::RobotBase2DOdom base_odom_; /**< Odometry in the base frame */

    /** Parameters that will be passed on initialization soon */
    double baseLaserMaxRange_; /**< Used in laser scan projection */
    double tiltLaserMaxRange_; /**< Used in laser scan projection */

    std::list<deprecated_msgs::Pose2DFloat32>  plan_; /**< The 2D plan in grid co-ordinates of the cost map */

    // Filter parameters
    double minZ_;
    double maxZ_;
    double robotWidth_;

    // Thread control
    void mapUpdateLoop();
    boost::thread *map_update_thread_; /*<! Thread to process laser data and apply to the map */
    bool active_; /*<! Thread control parameter */
    double map_update_frequency_, trans_stopped_velocity_, rot_stopped_velocity_, min_abs_theta_vel_;

    // Tolerances for determining if goal has been reached
    double yaw_goal_tolerance_;
    double xy_goal_tolerance_;
      
    //Robot filter
    robot_filter::RobotFilter* filter_;
    tf::MessageNotifier<laser_scan::LaserScan>* baseScanNotifier_; 
    tf::MessageNotifier<robot_msgs::PointCloud>* baseCloudNotifier_; 
    tf::MessageNotifier<robot_msgs::PointCloud>* tiltLaserNotifier_;

    //flag for reseting the costmap.
    bool reset_cost_map_;

    //ground plane extraction
    pr2_msgs::PlaneStamped groundPlaneMsg_;
    pr2_msgs::PlaneStamped ground_plane_;
    robot_msgs::PointCloud *filtered_cloud_;
    boost::recursive_mutex lock_; /*!< Lock for access to class members in callbacks */

  };
}
#endif
