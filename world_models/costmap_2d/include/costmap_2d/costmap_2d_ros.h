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
#ifndef COSTMAP_COSTMAP_2D_ROS_H_
#define COSTMAP_COSTMAP_2D_ROS_H_

#include <ros/node.h>
#include <ros/console.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/observation_buffer.h>
#include <robot_msgs/Polyline.h>
#include <map>
#include <vector>
#include <string>
#include <sstream>

#include <tf/transform_datatypes.h>
#include <tf/message_notifier_base.h>
#include <tf/message_notifier.h>
#include <tf/transform_listener.h>

#include <laser_scan/LaserScan.h>
#include <laser_scan/laser_scan.h>

#include <robot_msgs/PointCloud.h>

// Thread suppport
#include <boost/thread.hpp>


namespace costmap_2d {

  /**
   * @class Costmap2DROS
   * @brief A ROS wrapper for a 2D Costmap. Handles subscribing to topics that
   * provide observations about obstacles in either the form of PointCloud or LaserScan
   * messages.
   */
  class Costmap2DROS {
    public:
      /**
       * @brief  Constructor for the wrapper
       * @param ros_node A reference to the ros node to run on
       * @param tf A reference to a TransformListener
       * @param prefix An optional prefix to prepend to the parameter list for the costmap
       */
      Costmap2DROS(ros::Node& ros_node, tf::TransformListener& tf, std::string prefix = std::string(""));

      /**
       * @brief  Destructor for the wrapper. Cleans up pointers.
       */
      ~Costmap2DROS();

      /**
       * @brief  If you want to manage your own observation buffer you can add it to the costmap 
       * <B>(NOTE: The buffer will be deleted on destruction of the costmap... 
       * perhaps a boost shared pointer should go here eventually)</B>
       * @param  buffer A pointer to your observation buffer
       */
      void addObservationBuffer(ObservationBuffer* buffer);

      /**
       * @brief  Get the observations used to mark space
       * @param marking_observations A reference to a vector that will be populated with the observations 
       * @return True if all the observation buffers are current, false otherwise
       */
      bool getMarkingObservations(std::vector<Observation>& marking_observations);

      /**
       * @brief  Get the observations used to clear space
       * @param marking_observations A reference to a vector that will be populated with the observations 
       * @return True if all the observation buffers are current, false otherwise
       */
      bool getClearingObservations(std::vector<Observation>& clearing_observations);

      /**
       * @brief  A callback to handle buffering LaserScan messages
       * @param message The message returned from a message notifier 
       * @param buffer A pointer to the observation buffer to update
       */
      void laserScanCallback(const tf::MessageNotifier<laser_scan::LaserScan>::MessagePtr& message, ObservationBuffer* buffer);

      /**
       * @brief  A callback to handle buffering PointCloud messages
       * @param message The message returned from a message notifier 
       * @param buffer A pointer to the observation buffer to update
       */
      void pointCloudCallback(const tf::MessageNotifier<robot_msgs::PointCloud>::MessagePtr& message, ObservationBuffer* buffer);

      /**
       * @brief  Update the underlying costmap with new sensor data. 
       * If you want to update the map outside of the update loop that runs, you can call this.
       */
      void updateMap();

      /**
       * @brief  Reset to the static map outside of a window around the robot specified by the user
       * @param size_x The x size of the window to keep unchanged 
       * @param size_y The y size of the window to keep unchanged 
       */
      void resetMapOutsideWindow(double size_x, double size_y);

      /**
       * @brief  Publish the underlying costmap to the visualizer
       * If you want to publish the map outside of the publish loop that runs, you can call this.
       * @param map The map to be published
       */
      void publishCostMap(Costmap2D& map);

      /**
       * @brief  Returns a copy of the underlying costmap
       * @param cost_map A reference to the map to populate
       */
      void getCostMapCopy(Costmap2D& cost_map);

      /**
       * @brief  Returns a copy of the underlying unsigned character array <B>(NOTE: THE USER IS RESPONSIBLE FOR DELETION)</B>
       * @return A copy of the underlying unsigned character character array used for the costmap
       */
      unsigned char* getCharMapCopy();

      /**
       * @brief  Returns the x size of the costmap in cells
       * @return The x size of the costmap in cells
       */
      unsigned int cellSizeX();

      /**
       * @brief  Returns the y size of the costmap in cells
       * @return The y size of the costmap in cells
       */
      unsigned int cellSizeY();

      /**
       * @brief  Check if the observation buffers for the cost map are current
       * @return True if the buffers are current, false otherwise
       */
      bool isCurrent() {return current_;}

    private:
      /**
       * @brief  Sleeps for the remaining cycle time of a update loop
       * @param  start The time the loop started
       * @param  cycle_time How long the loop is supposed to take
       * @return True if the cycle was completed in time false if it took too long
       */
      bool sleepLeftover(ros::Time start, ros::Duration cycle_time);

      /**
       * @brief  The loop that handles updating the costmap
       * @param  frequency The rate at which to run the loop
       */
      void mapUpdateLoop(double frequency);

      /**
       * @brief  The loop that handles displaying the costmap
       * @param  frequency The rate at which to run the loop
       */
      void mapPublishLoop(double frequency);

      ros::Node& ros_node_; ///< @brief The ros node to use
      tf::TransformListener& tf_; ///< @brief Used for transforming point clouds
      laser_scan::LaserProjection projector_; ///< @brief Used to project laser scans into point clouds
      boost::recursive_mutex map_lock_; ///< @brief A lock for accessing data in callbacks safely
      Costmap2D* costmap_; ///< @brief The underlying costmap to update
      std::string global_frame_; ///< @brief The global frame for the costmap
      std::string robot_base_frame_; ///< @brief The frame_id of the robot base
      boost::thread* visualizer_thread_; ///< @brief A thread for publising to the visualizer
      boost::thread* map_update_thread_; ///< @brief A thread for updating the map

      std::vector<tf::MessageNotifierBase*> observation_notifiers_; ///< @brief Used to make sure that transforms are available for each sensor
      std::vector<ObservationBuffer*> observation_buffers_; ///< @brief Used to store observations from various sensors
      std::vector<ObservationBuffer*> marking_buffers_; ///< @brief Used to store observation buffers used for marking obstacles
      std::vector<ObservationBuffer*> clearing_buffers_; ///< @brief Used to store observation buffers used for clearing obstacles
      bool rolling_window_; ///< @brief Whether or not the costmap should roll with the robot
      bool current_; ///< @brief Whether or not all the observation buffers are updating at the desired rate
      double transform_tolerance_; // timeout before transform errors

  };
};

#endif

