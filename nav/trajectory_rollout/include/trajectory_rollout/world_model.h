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
#ifndef TRAJECTORY_ROLLOUT_WORLD_MODEL_H_
#define TRAJECTORY_ROLLOUT_WORLD_MODEL_H_

#include <vector>
#include <costmap_2d/observation.h>
#include <deprecated_msgs/Point2DFloat32.h>
#include <trajectory_rollout/planar_laser_scan.h>

namespace trajectory_rollout {
  /**
   * @class WorldModel
   * @brief An interface the trajectory controller uses to interact with the
   * world regardless of the underlying world model.
   */
  class WorldModel{
    public:
      /**
       * @brief  Subclass will implement this method to insert observations from sensors into its world model
       * @param footprint The footprint of the robot in its current location
       * @param observations The observations from various sensors 
       * @param laser_scan The planar scan used to clear freespace
       */
      virtual void updateWorld(const std::vector<deprecated_msgs::Point2DFloat32>& footprint,
          const std::vector<costmap_2d::Observation>& observations, const PlanarLaserScan& laser_scan) = 0;

      /**
       * @brief  Subclass will implement this method to check a footprint at a given position and orientation for legality in the world
       * @param  position The position of the robot in world coordinates
       * @param  footprint The specification of the footprint of the robot in world coordinates
       * @param  inscribed_radius The radius of the inscribed circle of the robot
       * @param  circumscribed_radius The radius of the circumscribed circle of the robot
       * @return True if the footprint is legal based on the world model, false otherwise
       */
      virtual bool legalFootprint(const deprecated_msgs::Point2DFloat32& position, const std::vector<deprecated_msgs::Point2DFloat32>& footprint,
          double inscribed_radius, double circumscribed_radius) = 0;

      /**
       * @brief  Subclass will implement a destructor
       */
      virtual ~WorldModel(){}

    protected:
      WorldModel(){}
  };

};
#endif
