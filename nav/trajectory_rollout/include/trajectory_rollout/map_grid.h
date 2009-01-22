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
#ifndef MAP_GRID_H_
#define MAP_GRID_H_

#include <vector>
#include <iostream>
#include "assert.h"
#include <trajectory_rollout/trajectory_inc.h>
#include <trajectory_rollout/ScoreMap2D.h>

#include <trajectory_rollout/map_cell.h>

//A grid of MapCells
class MapGrid{
  public:
    MapGrid(unsigned int size_x, unsigned int size_y);

    MapGrid(unsigned int size_x, unsigned int size_y, double scale, double x, double y);

    //cells will be accessed by (col, row)
    inline MapCell& operator() (unsigned int x, unsigned int y){
      return map_[size_x_ * y + x];
    }

    inline MapCell operator() (unsigned int x, unsigned int y) const {
      return map_[size_x_ * y + x];
    }

    ~MapGrid(){}

    MapGrid(const MapGrid& mg);
    MapGrid& operator= (const MapGrid& mg);
    
    //allow easy updating from message representations
    void update(trajectory_rollout::ScoreMap2D& new_map);

    //reset path distance fields for all cells
    void resetPathDist();

    //check if we need to resize
    void sizeCheck(unsigned int size_x, unsigned int size_y, double o_x, double o_y);

    // Utility to share initialization code across constructors
    void commonInit();

    size_t getIndex(int x, int y);

    //allow easy creation of messages
    trajectory_rollout::ScoreMap2D genMsg();

    unsigned int size_x_, size_y_;
    std::vector<MapCell> map_;

    //grid scale in meters/cell
    double scale;

    //lower left corner of grid in world space
    double origin_x, origin_y;
};

#endif
