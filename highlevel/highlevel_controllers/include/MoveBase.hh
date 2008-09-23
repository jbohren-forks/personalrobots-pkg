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

// Costmap used for the map representation
#include <costmap_2d/costmap_2d.h>

// Message structures used
#include <std_msgs/Planner2DState.h>
#include <std_msgs/Planner2DGoal.h>
#include <std_msgs/LaserScan.h>

// For transform support
#include <rosTF/rosTF.h>

// Laser projection
#include "laser_scan_utils/laser_scan.h"

class MoveBase : public HighlevelController<std_msgs::Planner2DState, std_msgs::Planner2DGoal> {

public:

  /**
   * @brief Constructor
   */
  MoveBase(double windowLength, unsigned char lethalObstacleThreshold, unsigned char noInformation, double maxZ, double inflationRadius);

  virtual ~MoveBase();

protected:

  /**
   * @brief Accessor for the cost map. Use mainly for initialization
   * of specialized map strunture for planning
   */
  const CostMap2D& getCostMap() const {return *costMap_;}

  /**
   * @brief A handler to be over-ridden in the derived class to handle a diff stream from the
   * cost map. This is called on a map update, which means it will be on a separate thread to the main
   * node control loop
   */
  virtual void handleMapUpdates(const std::vector<unsigned int>& insertions, std::vector<unsigned int>& deletions){}

  void publishPlan();

  std::vector< std::pair<unsigned int, unsigned int> > plan_; /**< The 2D plan in grid co-ordinates of the cost map */

private:
  /**
   * @brief Will process a goal update message.
   */
  virtual void updateGoalMsg();


  /**
   * @brief Use global pose to publish currrent state data at the start of each cycle
   */
  virtual void updateStateMsg();

  /**
   * @brief Evaluate if final goal x, y, th has been reached.
   */
  virtual bool goalReached(){return true;}

  /**
   * @brief Send velocity commands based on local plan. Should check for consistency of
   * local plan.
   */
  virtual bool dispatchCommands(){return true;}

  /**
   * @brief Call back for handling new laser scans
   */
  void laserScanCallback();

  void updateGlobalPose();

  std_msgs::LaserScan laserScanMsg_; /**< Filled by subscriber with new laser scans */

  laser_scan::LaserProjection projector_; /**< Used to project laser scans */

  rosTFClient tf_; /**< Used to do transforms */

  CostMap2D* costMap_; /**< The cost map mainatined incrementally from laser scans */

  libTF::TFPose2D global_pose_; /**< The global pose in the map frame */

  /** Parameters that will be passed on initialization soon */
  const double laserMaxRange_; /**< Used in laser scan projection */
};
#endif
