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

// Need observations
#include "observation.h"

//c++
#include <vector>
#include <string>
#include <queue>

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

    static std::vector<std_msgs::PointCloud*> toVector(std_msgs::PointCloud& cloud){
      std::vector<std_msgs::PointCloud*> v;
      v.push_back(&cloud);
      return v;
    }

    /**
     * @brief Constructor.
     *
     * @param width width of map [cells]
     * @param height height of map [cells]
     * @param data row-major obstacle data, each value indicates a cost
     * @param resolution resolution of map [m/cell]
     * @param threshold The cost threshold where a cell is considered an obstacle
     * @param maxZ gives the cut-off for points in 3D space
     * @param zLB lower bound for evaluating points in z for projecting free space
     * @param zUB upper bound for evaluating points in z for projecting free space
     * @param inflationRadius the radius used to bound inflation - limit of cost propagation
     * @param circumscribedRadius the radius used to indicate objects in the circumscribed circle around the robot
     * @param inscribedRadius the radius used to indicate objects in the inscribed circle around the robot
     * @param weight the scaling factor in the cost function. Should be <=1. Lower values reduce the effective cost
     */
    CostMap2D(unsigned int width, unsigned int height, const std::vector<unsigned char>& data, 
	      double resolution, unsigned char threshold, 
	      double maxZ = 0.5,  double zLB = 0.15, double zUB = 0.20,
	      double inflationRadius = 0, double circumscribedRadius = 0, double inscribedRadius = 0, double weight = 1, double obstacleRange = 10.0, double raytraceRange = 10.0);
  
    /**
     * @brief Destructor.
     */
    virtual ~CostMap2D();

    // Hack for backward compatability
    void updateDynamicObstacles(std_msgs::PointCloud& cloud,
				std::vector<unsigned int>& updates){
      updateDynamicObstacles(0, 0, toVector(cloud), updates);
    }

    /**
     * @brief Updates the cost map accounting for the new value of time and a new set of obstacles. 
     * @param wx The current x position
     * @param wy The current y position
     * @param cloud holds projected scan data
     * @param updates holds the updated cell ids and values
     */
    void updateDynamicObstacles(double wx, double wy,
				const std::vector<std_msgs::PointCloud*>& clouds,
				std::vector<unsigned int>& updates);
    /**
     * @brief Updates dynamic obstacles
     * @param wx The current x position
     * @param wy The current y position
     * @param clouds holds projected scan data
     */
    void updateDynamicObstacles(double wx, double wy,
				const std::vector<std_msgs::PointCloud*>& clouds);

    /**
     * @brief Updates dynamic obstacles based on a buffer of observations
     * @param wx The current x position
     * @param wy The current y position
     * @param observations The collection of observations from all data sources
     */
    void updateDynamicObstacles(double wx, double wy, const std::vector<Observation>& observations);

    /**
     * @brief Obtain the collection of all occupied cells: the union of static and dynamic obstacles
     */
    void getOccupiedCellDataIndexList(std::vector<unsigned int>& results) const;

    /**
     * @brief The weight for scaling in the cost function
     */
    double getWeight() const {return weight_;}

    /**
     * @brief Will reset the cost data
     * @param wx the x position in world coordinates
     * @param wy the y position in world coordinates
     */
    void revertToStaticMap(double wx = 0.0, double wy = 0.0);

  private:

    /**
     * @brief Helper method to compute the inflated cells around an obstacle based on resolution and inflation radius
     */
    void computeInflation(unsigned int ind, std::vector< std::pair<unsigned int, unsigned char> >& inflation) const;

    /**
     * @brief Utility to propagate costs
     * @param A priority queue to seed propagation
     * @param A collection to retrieve all updated cells
     */
    void propagateCosts();

    /**
     * @brief Utility to push free space inferred from a laser point hit via ray-tracing
     * @param The origin from which to trace out
     * @param The target in map co-ordinates to trace to
     */ 
    void updateFreeSpace(const std_msgs::Point& origin, double wx, double wy);

    /**
     * @brief Simple test for a cell having been marked during current propaagtion
     */
    bool marked(unsigned int ind) const {return xy_markers_[ind];}

    /**
     * @mark a cell
     */
    void mark(unsigned int ind){xy_markers_[ind] = true;}
    
    /**
     * Utilities for cost propagation
     */
    void enqueueNeighbors(unsigned int source, unsigned int ind);

    void enqueue(unsigned int source, unsigned int mx, unsigned int my);


    /**
     * Euclidean distance
     */
    inline double computeDistance(unsigned int a, unsigned int b) const{
      unsigned int mx_a, my_a, mx_b, my_b;
      IND_MC(a, mx_a, my_a);
      IND_MC(b, mx_b, my_b);
      unsigned int dx = abs((int)(mx_a) - (int) mx_b);
      unsigned int dy = abs((int)(my_a) - (int) my_b);

      ROS_ASSERT((dx <= inflationRadius_) && (dy <= inflationRadius_));
      double distance = cachedDistances[dx][dy];

      return distance;
    }

    inline unsigned char computeCost(double distance) const{
      unsigned char cost = 0;
      if(distance == 0)
	cost = LETHAL_OBSTACLE;
      else if(distance <= inscribedRadius_)
	cost = INSCRIBED_INFLATED_OBSTACLE;
      else {
	// The cost for a non-lethal obstacle should be in the range of [0, inscribed_inflated_obstacle - 1].
	double factor = weight_ / (1 + pow(distance - inscribedRadius_, 2));
	cost = (unsigned char) ( (INSCRIBED_INFLATED_OBSTACLE -1) * factor);
      }
      return cost;
    }

    void updateCellCost(unsigned int ind, unsigned char cost);

    bool in_projection_range(double z){return z >= zLB_ && z <= zUB_;}

    const double maxZ_; /**< Points above this will be excluded from consideration */
    const double zLB_; /**< Filters points for free space projection */
    const double zUB_; /**< Filters points for free space projection */
    const unsigned int inflationRadius_; /**< The radius in cells to propagate cost and obstacle information */
    const unsigned int circumscribedRadius_; /**< The radius for the circumscribed radius, in cells */
    const unsigned int inscribedRadius_; /**< The radius for the inscribed radius, in cells */
    const double weight_;  /**< The weighting to apply to a normalized cost value */

    //used squared distance because square root computations are expensive
    double sq_obstacle_range_; /** The range out to which we will consider laser hitpoints **/
    double sq_raytrace_range_; /** The range out to which we will raytrace **/

    unsigned char* staticData_; /**< initial static map */
    bool* xy_markers_; /**< Records time remaining in ticks before expiration of the observation */
    QUEUE queue_; /**< Used for cost propagation */

    double** cachedDistances; /**< Cached distances indexed by dx, dy */  
    const unsigned int kernelWidth_; /**< The width of the kernel matrix, which will be square */
    unsigned char* kernelData_; /**< kernel data structure for cost map updates around the robot */
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
