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
     * @brief Defines the cell value to indicate an inflated obstacle within the circumscribed circle of the robot
     */
    static const unsigned char CIRCUMSCRIBED_INFLATED_OBSTACLE;

    /**
     * @brief Access the cost value of the cell at the given index
     * @see MC_IND
     */
    virtual unsigned char operator[](unsigned int ind) const = 0;

    /**
     * @brief Get the cost for the cell given in map coordinates
     */
    virtual unsigned char getCost(unsigned int mx, unsigned int my) const = 0;

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
    bool WC_MC(double wx, double wy, unsigned int& mx, unsigned int& my) const;

    virtual ~ObstacleMapAccessor(){}

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
  };
}
#endif
