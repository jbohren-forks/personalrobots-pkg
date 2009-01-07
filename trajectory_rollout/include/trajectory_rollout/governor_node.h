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
*********************************************************************/
#ifndef GOVERNOR_NODE_H_
#define GOVERNOR_NODE_H_
#include <ros/node.h>
#include <rosthread/mutex.h>

#include <vector>

//The messages that we'll use
#include <std_msgs/BaseVel.h>
#include <trajectory_rollout/ScoreMap2D.h>
#include <trajectory_rollout/WavefrontPlan.h>
#include <std_msgs/RobotBase2DOdom.h>

//for GUI debugging
#include <std_msgs/Polyline2D.h>
#include <std_msgs/Point2DFloat32.h>


//for transform support
#include "tf/transform_listener.h"

//the trajectory controller
#include <trajectory_rollout/trajectory_controller.h>
#include <trajectory_rollout/map_grid.h>
#include <trajectory_rollout/trajectory.h>

#include <costmap_2d/obstacle_map_accessor.h>

#define MAP_SIZE_X 10
#define MAP_SIZE_Y 10
#define MAX_ACC_X 0.15
#define MAX_ACC_Y 1.0
#define MAX_ACC_THETA 1.0
#define SIM_TIME 3.0
#define SIM_STEPS 30
#define VEL_SAMPLES 25
#define ROBOT_FRONT_RADIUS .325
#define ROBOT_SIDE_RADIUS .325
#define OUTER_RADIUS .465
#define PDIST_SCALE .4
#define GDIST_SCALE .6
#define OCCDIST_SCALE 0
#define DFAST_SCALE 0
#define SAFE_DIST .005

//So we can still use wavefront player we'll fake an obs accessor
class WavefrontMapAccessor : public costmap_2d::ObstacleMapAccessor {
 public:
 WavefrontMapAccessor(MapGrid &map, double outer_radius) 
   : costmap_2d::ObstacleMapAccessor(map.origin_x, map.origin_y, map.size_x_, map.size_y_, map.scale),
    map_(map), outer_radius_(outer_radius) {
    synchronize();
  }

    virtual ~WavefrontMapAccessor(){};

    void synchronize(){
      // Write Cost Data from the map
      for(unsigned int x = 0; x < width_; x++){
	for (unsigned int y = 0; y < height_; y++){
	  unsigned int ind = x + (y * width_);
	  if(map_(x, y).occ_state == 1)
	    costData_[ind] = costmap_2d::ObstacleMapAccessor::LETHAL_OBSTACLE;
	  else if(map_(x, y).occ_dist < outer_radius_)
	    costData_[ind] = costmap_2d::ObstacleMapAccessor::INSCRIBED_INFLATED_OBSTACLE/2;
	  else 
	    costData_[ind] = 0;
	}
      }
    }

    void updateOrigin(double o_x, double o_y){
      origin_x_ = o_x;
      origin_y_ = o_y;
    }
    /*
    void updateResolution(double res){
      resolution_ = res;
    }
    void updateSize(unsigned int width, unsigned int height){
      width_ = width;
      height_ = height;
    }
    */

 private:
    MapGrid& map_;
    double outer_radius_;
};

class GovernorNode: public ros::node
{
  public:
    GovernorNode(std::vector<std_msgs::Point2DFloat32> footprint_spec);

    //callback for when the planned passes a new map
    void planReceived();

    //callback for odometry information
    void odomReceived();

    //processing loop
    void processPlan();

    //sleep for remaining time of cycle
    void sleep(ros::Time start);

    //a map
    MapGrid map_;
    
    //transform client
    tf::TransformListener tf_;

    //map accessor
    WavefrontMapAccessor ma_;

    //local controller
    TrajectoryController tc_;

    //incoming messages
    trajectory_rollout::WavefrontPlan plan_msg_;
    std_msgs::RobotBase2DOdom odom_msg_;

    //outgoing messages
    std_msgs::Polyline2D poly_line_msg_;
    std_msgs::BaseVel cmd_vel_msg_;

    //since both odomReceived and processPlan access robot_vel we need to lock
    ros::thread::mutex vel_lock;

    //since both planReceived and processPlan access map_ we need to lock
    ros::thread::mutex map_lock;

    //keep track of the robot's velocity
    tf::Stamped<tf::Pose> robot_vel_;

    //how long for each cycle
    double cycle_time_;

    //for debugging output
    std_msgs::Polyline2D path_msg;
    std_msgs::Polyline2D footprint_msg;
    

};
#endif
