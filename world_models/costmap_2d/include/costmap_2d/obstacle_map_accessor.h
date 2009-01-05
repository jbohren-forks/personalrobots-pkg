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
#ifndef OBSTACLE_MAP_ACCESSOR_H
#define OBSTACLE_MAP_ACCESSOR_H

#include "ros/common.h"

namespace costmap_2d {

  /**
   * Abstract base class for read-only access to obstacles in a 2 D grid
   */
  class ObstacleMapAccessor {
  public:

    /**
     * @brief Defines the cell value to indicate that no information is available
     */
    static const unsigned char NO_INFORMATION;

    /**
     * @brief Defines the cell value to indicate a lethal obstacles. Cell values less than this value are
     * traversable at a given cost
     */
    static const unsigned char LETHAL_OBSTACLE;

    /**
     * @brief Defines the cell value to indicate an inflated obstacle within the inscribed circle of the robot
     */
    static const unsigned char INSCRIBED_INFLATED_OBSTACLE;

    /**
     * @brief Get pointer into the obstacle map (which contains both static
     * and dynamic obstacles)
     */
    const unsigned char* getMap() const {
      return costData_;
    }

    /**
     * @brief Accessor for contents of cost map cell by cell index
     */
    unsigned char operator [](unsigned int ind) const{
      return costData_[ind];
    }

    /**
     * @brief Accessor by map coordinates stored cost value integrating dynamic and static data
     * @param mx the x map index. mx must be in [0, width-1]
     * @param my the y map index. my must be in [0, height-1]
     * @return A cost value in the range [0 255]
     * @note The inputs should always be in bounds. We could add a check for these values to avoid out of range errors
     * but that should only arise as an error condition and should not burden the code for a common accessor
     */
    unsigned char getCost(unsigned int mx, unsigned int my) const{
      ROS_ASSERT(mx < width_);
      ROS_ASSERT(my < height_);
      unsigned int ind = mx + my * width_;
      return costData_[ind];
    }

    /**
     * @brief Test if the given cell is necessarily in the footprint of the robot. Note that a negative result
     * does not mean it is not in the footprint of the robot, it just means that we are not certain.
     * @param mx the x map index. mx must bi in [0, width-1]
     * @param my the y map index. my must be in [0, height-1]
     * @return true if the cell is in the inscried circle of the robot, including the raw obstacle point itself.
     */
    bool isDefinitelyBlocked(unsigned int mx, unsigned int my) const{
      const unsigned char cost = getCost(mx, my);
      return cost == INSCRIBED_INFLATED_OBSTACLE || cost == LETHAL_OBSTACLE;
    }

    /**
     * @brief Function to test if a cell is in difference between the circusmcribed radius
     * and the inscribed radius. This check is used to efficiently test if a client must lay 
     * down the robot footprint to test for being in collision.
     * @param mx the x map index. mx must bi in [0, width-1]
     * @param my the y map index. my must be in [0, height-1]
     */
    bool isCircumscribedCell(unsigned int mx, unsigned int my) const;

    /**
     * @brief Get the origin
     */
    void getOriginInWorldCoordinates(double& wx, double& wy) const;

    /**
     * @brief The width in cells
     */	
    unsigned int getWidth() const {return width_;}

    /**
     * @brief The Height ion cells
     */
    unsigned int getHeight() const {return height_;}

    /**
     * @brief the resolution in meters per cell, where cells are square
     */
    double getResolution() const {return resolution_;}

    /**
     * @brief Sets the bound for testing if in circumscribed circle overflow
     */
    void setCircumscribedCostLowerBound(unsigned char c){
      costLB_ = c;
    }

    /**
     * @brief Accessor for lower bound for a cost being in the circumscribed circle
     */
    unsigned char getCircumscribedCostLowerBound() const {return costLB_;}

    /**
     * @brief Obtain world co-ordinates for the given index
     * @param ind index
     * @param wx world x location of the cell
     * @param wy world y location of the cell
     */
    inline void IND_WC(unsigned int ind, double& wx, double& wy) const {
      unsigned int mx, my;
      IND_MC(ind, mx, my);
      MC_WC(mx, my, wx, wy);
    }

    /**
     * @brief Converts from 1D map index into x y map coords
     * 
     * @param ind 1d map index
     * @param x 2d map return value
     * @param y 2d map return value
     */
    inline void IND_MC(unsigned int ind, unsigned int& mx, unsigned int& my) const{
      my = ind / width_;
      mx = ind - (my*width_);
    }

    /**
     * @brief Get index of given (x,y) point into data
     * 
     * @param x x-index of the cell
     * @param y y-index of the cell
     */
    inline unsigned int MC_IND(unsigned int mx, unsigned int my) const {
      return(mx+my*width_);
    }

    /**
     * @brief Get world (x,y) point given map indexes. Returns center of a cell
     * 
     * @param mx map x index location
     * @param my map y index location
     * @param wx world x return value
     * @param wy world y return value
     */
    inline void MC_WC(unsigned int mx, unsigned int my, double& wx, double& wy) const{
      wx = origin_x_ + (mx + 0.5) * resolution_;
      wy = origin_y_ + (my + 0.5) * resolution_;
    }

    /**
     * @brief Get index of given (x,y) point into data
     * 
     * @param wx world x location of the cell
     * @param wy world y location of the cell
     */
    inline unsigned int WC_IND(double wx, double wy) const{
      unsigned int mx, my;
      WC_MC(wx, wy, mx, my);
      return MC_IND(mx, my);
    }

    /**
     * @brief Get index of given world (x,y) point in map indexes
     * 
     * @param wx world x location of the cell
     * @param wy world y location of the cell
     * @param mx map x index return value
     * @param my map y index return value
     */
    inline bool WC_MC(double wx, double wy, unsigned int& mx, unsigned int& my) const{
      if(wx < 0 || wy < 0) {
	mx = 0;
	my = 0;
	return false;
      }

      //not much for now
      mx = (int) ((wx - origin_x_)/resolution_);
      my = (int) ((wy - origin_y_)/resolution_);

      //printf("x: %.2f y: %.2f or_x: %.2f, or_y: %.2f, resolution: %.2f\n   ", wx, wy, origin_x_, origin_y_, resolution_);
      if(mx >= width_) {
	mx = 0;
	return false;
      } 

      if(my >= height_) {
	//printf("WC_MC converted  %d greater than height %d\n", my, height_);
	my = 0;
	return false;
      }

      return true;
    }

    virtual ~ObstacleMapAccessor();

    /**
     * @brief Utility for debugging
     */
    std::string toString() const;


    static double computeWX(const ObstacleMapAccessor& costMap, double maxSize, double wx, double wy);
    static double computeWY(const ObstacleMapAccessor& costMap, double maxSize, double wx, double wy);

  protected:

    /**
     * @brief Constructor
     * @param origin_x Origin of the map in world coords
     * @param origin_y Origin of the map in world coords
     * @param width Number of cells accross (x direction)
     * @param height Number of cells down (y direction)
     * @param resolution Width and hight of a cell in meters
     */
    ObstacleMapAccessor(double origin_x, double origin_y, unsigned int width, unsigned int height, double resolution);

    double origin_x_;
    double origin_y_;
    unsigned int width_;
    unsigned int height_;
    double resolution_;
    unsigned char* costData_; /**< the full cost map data */

  private:
    unsigned char costLB_; /**< The cost value for the lowest cost cell in the circumscribed radius.*/
  };
}
#endif
