/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: E. Gil Jones, Conor McGann
 */

#ifndef COSTMAP_2D_COSTMAP_2D_H
#define COSTMAP_2D_COSTMAP_2D_H

/**

   @mainpage 

   @htmlinclude manifest.html

   This library provides functionality for maintaining a 2D cost map.  The primary inputs are:
   - a priori 2-D occupancy grid map
   - 3D obstacle data
   - control parameters

   This cost map is initialized with the a priori map, and thereafter updated 
   with obstacle data.  A sliding window model of persistence is used, in which
   obstacle data lives in the map for a fixed (but configurable) amount of time
   before being discarded.  The static map lives forever. Dynamic obstacle data is filtered
   to account for the robot body, and based on a z-value threshold to permit projection of 3D
   data to the 2D plane. Control parameters dictate the sliding window length, z threshold and
   inflation radius.

   For examples of usage see the test harness.
*/

// Base class
#include "obstacle_map_accessor.h"

//c++
#include <list>
#include <vector>
#include <map>
#include <set>
#include <string>
#include <map>
#include <queue>

// For point clouds <Could Make it a template>
#include "std_msgs/PointCloud.h"

namespace costmap_2d {

  typedef unsigned char TICK;

  class CostMap2D;

  class QueueElement {
  public:
  QueueElement(double d, unsigned int s, unsigned int i): distance(d), source(s), ind(i) {}

  QueueElement(const QueueElement& org): distance(org.distance), source(org.source), ind(org.ind){}

    double distance;
    unsigned int source;
    unsigned int ind;
  };

  class QueueElementComparator {
  public:
    /**
     * Weak ordering property respected
     */
    inline bool operator()(const QueueElement* lhs, const QueueElement* rhs){
      return lhs->distance > rhs->distance;
    }
  };

  typedef std::priority_queue< QueueElement*, std::vector<QueueElement*>, QueueElementComparator > QUEUE;

  class CostMap2D: public ObstacleMapAccessor 
  {
  public:

    /**
     * @brief Constructor.
     *
     * @param width width of map [cells]
     * @param height height of map [cells]
     * @param data row-major obstacle data, each value indicates a cost
     * @param resolution resolution of map [m/cell]
     * @param window_length how long to hold onto obstacle data [sec]
     * @param threshold The cost threshold where a cell is considered an obstacle
     * @param maxZ gives the cut-off for points in 3D space
     * @param freeSpaceProjectionHeight gives the upper bound for evaluating points in z for projecting free space
     * @param inflationRadius the radius used to bound inflation - limit of cost propagation
     * @param circumscribedRadius the radius used to indicate objects in the circumscribed circle around the robot
     * @param inscribedRadius the radius used to indicate objects in the inscribed circle around the robot
     */
    CostMap2D(unsigned int width, unsigned int height, const std::vector<unsigned char>& data, 
	      double resolution, double window_length,
	      unsigned char threshold, 
	      double maxZ = 0, double freeSpaceProjectionHeight = 0,
	      double inflationRadius = 0, double circumscribedRadius = 0, double inscribedRadius = 0);
  
    /**
     * @brief Destructor.
     */
    virtual ~CostMap2D();
  
    /**
     * @brief Updates the cost map accounting for the new value of time and a new set of obstacles. 
     * @param current time stamp
     * @param cloud holds projected scan data
     * @param updates holds the updated cell ids and values
     *
     * @see removeStaleObstacles
     */
    void updateDynamicObstacles(double ts,
				const std_msgs::PointCloud& cloud,
				std::vector<unsigned int>& updates);

    /**
     * @brief Updates the cost map accounting for the new value of time and a new set of obstacles. 
     * @param current time stamp
     * @param wx The current x position
     * @param wy The current y position
     * @param cloud holds projected scan data
     * @param updates holds the updated cell ids and values
     *
     * @see removeStaleObstacles
     */
    void updateDynamicObstacles(double ts,
				double wx, double wy,
				const std_msgs::PointCloud& cloud,
				std::vector<unsigned int>& updates);
    /**
     * @brief A convenience method which will skip calculating the diffs
     * @param current time stamp
     * @param cloud holds projected scan data
     *
     * @see updateDynamicObstacles
     */
    void updateDynamicObstacles(double ts, const std_msgs::PointCloud& cloud);

    /**
     * @brief Updates the cost map, removing stale obstacles based on the new time stamp.
     * @param current time stamp
     * @param deletedObstacleCells holds vector for returning newly unoccupied ids
     */
    void removeStaleObstacles(double ts, std::vector<unsigned int>& updates);

    /**
     * @brief Get pointer into the obstacle map (which contains both static
     * and dynamic obstacles)
     */
    const unsigned char* getMap() const;

    /**
     * @brief Obtain the collection of all occupied cells: the union of static and dynamic obstacles
     */
    void getOccupiedCellDataIndexList(std::vector<unsigned int>& results) const;

    /**
     * @brief Accessor for contents of full map cell by cell index
     */
    unsigned char operator [](unsigned int ind) const;

    /**
     * @brief Accessor by map coordinates
     */
    unsigned char getCost(unsigned int mx, unsigned int my) const;

    /**
     * @brief Utility for debugging
     */
    std::string toString() const;

  private:
    /**
     * @brief Internal method that does not clear the updates first
     */
    void removeStaleObstaclesInternal(double ts, std::vector<unsigned int>& updates);

    /**
     * @brief Compute the number of ticks that have elapsed between the given timestamp ts and the last time stamp.
     * Will update the last time stamp value
     *
     * @param ts The current time stamp. Must be >= lastTimeStamp_
     */
    TICK getElapsedTime(double ts);

    /**
     * @brief Helper method to compute the inflated cells around an obstacle based on resolution and inflation radius
     */
    void computeInflation(unsigned int ind, std::vector< std::pair<unsigned int, unsigned char> >& inflation) const;

    /**
     * @brief Utility to encapsulate dynamic cell updates
     * @return true if the cell value is updated, otherwise false
     */
    void updateCellCost(unsigned int cell, unsigned char cellState, std::vector<unsigned int>& updates);

    /**
     * @brief Utility to encapsulate marking cells free.
     */
    void markFreeSpace(unsigned int cell, std::vector<unsigned int>& updates);

    /**
     * @brief Utility to propagate costs
     * @param A priority queue to seed propagation
     * @param A collection to retrieve all updated cells
     */
    void propagateCosts(std::vector<unsigned int>& updates);

    /**
     * @brief Utility to push free space inferred from a laser point hit via ray-tracing
     * @param ind the cell index of the obstacle detected
     * @param updates the buffer for updated cells as a result
     */ 
    void updateFreeSpace(unsigned int ind, std::vector<unsigned int>& updates);

    /**
     * @brief Simple test for a cell having been marked during current propaagtion
     */
    bool marked(unsigned int ind) const;

    /**
     * @brief Encapsulate calls to peth the watchdog, which impacts the set of dynamic obstacles
     */
    void petWatchDog(unsigned int cell, unsigned char value);

    /**
     * Added to provide a single update location. Only need to add the update if we have not visitied the cell
     * with an update already. This case is checked because of the watchdog value. This prevents multiply updating cells
     * for free space and cost propagation.
     */
    inline void addUpdate(unsigned int cell, std::vector<unsigned int>& updates){
      updates.push_back(cell);
    }
    
    /**
     * Utilities for cost propagation
     */
    void enqueueNeighbors(unsigned int source, unsigned int ind);

    void enqueue(unsigned int source, unsigned int mx, unsigned int my);

    double computeDistance(unsigned int a, unsigned int b) const;

    unsigned char computeCost(double distance) const;
			   
    static const TICK MARKED_FOR_COST = 255; /**< The value used to denote a cell has been marked in the current iteration of cost propagation */
    static const TICK WATCHDOG_LIMIT = 254; /**< The value for a reset watchdog time for observing dynamic obstacles */
    const double tickLength_; /**< The duration in seconds of a tick, used to manage the watchdog timeout on obstacles. Computed from window length */
    const double maxZ_; /**< Points above this will be excluded from consideration */
    const double freeSpaceProjectionHeight_; /**< Filters points for free space projection */
    const unsigned int inflationRadius_; /**< The radius in meters to propagate cost and obstacle information */
    const unsigned int circumscribedRadius_;
    const unsigned int inscribedRadius_;
    unsigned char* staticData_; /**< data loaded from the static map */
    unsigned char* fullData_; /**< the full map data that has both static and obstacle data */
    TICK* obsWatchDog_; /**< Records time remaining in ticks before expiration of the observation */
    double lastTimeStamp_; /** < The last recorded time value for when obstacles were added */
    unsigned int mx_; /** < The x position of the robot in the grid */
    unsigned int my_; /** < The y position of the robot in the grid */
			   
    std::list<unsigned int> dynamicObstacles_; /**< Dynamic Obstacle Collection */
    std::vector<unsigned int> staticObstacles_; /**< Vector of statically occupied cells */
    QUEUE queue_; /**< Used for cost propagation */

    double** cachedDistances; /**< Cached distances indexed by dx, dy */
    
  };

  /**
   * Wrapper class that provides a local window onto a global cost map
   */
  class CostMapAccessor: public ObstacleMapAccessor {
  public:
    /**
     * @brief Constructor
     * @param costMap The underlying global cost map
     * @param maxSize The maximum width and height of the window in meters
     * @param the current x position in global coords
     * @param the current y position in global coords
     */
    CostMapAccessor(const CostMap2D& costMap, double maxSize, double pose_x, double pose_y);

    /** Implementation for base class interface **/
    virtual unsigned char operator[](unsigned int ind) const;
    virtual unsigned char getCost(unsigned int mx, unsigned int my) const;

    /**
     * @brief Set the pose for the robot. Will adjust other parameters accordingly.
     */
    void updateForRobotPosition(double wx, double wy);

  private:

    static double computeWX(const CostMap2D& costMap, double maxSize, double wx, double wy);
    static double computeWY(const CostMap2D& costMap, double maxSize, double wx, double wy);
    static unsigned int computeSize(double maxSize, double resolution);

    const CostMap2D& costMap_;
    const double maxSize_;
    unsigned int mx_0_;
    unsigned int my_0_;
  };
}
#endif
