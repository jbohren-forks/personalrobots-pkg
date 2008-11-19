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
#include <trajectory_rollout/map_grid.h>

using namespace std;
using namespace trajectory_rollout;

MapGrid::MapGrid(unsigned int size_x, unsigned int size_y) 
  : size_x_(size_x), size_y_(size_y)
{
  commonInit();
}

MapGrid::MapGrid(unsigned int size_x, unsigned int size_y, double s, double x, double y)
  : size_x_(size_x), size_y_(size_y), scale(s), origin_x(x), origin_y(y)
{
  commonInit();
}

MapGrid::MapGrid(const MapGrid& mg){
  size_y_ = mg.size_y_;
  size_x_ = mg.size_x_;
  map_ = mg.map_;
}

void MapGrid::commonInit(){
  //don't allow construction of zero size grid
  assert(size_y_ != 0 && size_x_ != 0);

  map_.resize(size_y_ * size_x_);

  //make each cell aware of its location in the grid
  for(unsigned int i = 0; i < size_y_; ++i){
    for(unsigned int j = 0; j < size_x_; ++j){
      unsigned int id = size_x_ * i + j;
      map_[id].cx = j;
      map_[id].cy = i;
    }
  }
}

size_t MapGrid::getIndex(int x, int y){
  return size_x_ * y + x;
}

MapGrid& MapGrid::operator= (const MapGrid& mg){
  size_y_ = mg.size_y_;
  size_x_ = mg.size_x_;
  map_ = mg.map_;
  return *this;
}

void MapGrid::sizeCheck(unsigned int size_x, unsigned int size_y, double o_x, double o_y){
  if(map_.size() != size_x * size_y)
    map_.resize(size_x * size_y);

  if(size_x_ != size_x || size_y_ != size_y){
    size_x_ = size_x;
    size_y_ = size_y;

    for(unsigned int i = 0; i < size_y_; ++i){
      for(unsigned int j = 0; j < size_x_; ++j){
        int index = size_x_ * i + j;
        map_[index].cx = j;
        map_[index].cy = i;
      }
    }
  }
  origin_x = o_x;
  origin_y = o_y;
}

//allow easy updating from message representations
void MapGrid::update(ScoreMap2D& new_map){
  if(map_.size() != new_map.size_y * new_map.size_x){
    map_.resize(new_map.size_y * new_map.size_x);
  }

  size_y_ = new_map.size_y;
  size_x_ = new_map.size_x;
  scale = new_map.scale;
  origin_x = new_map.origin.x;
  origin_y = new_map.origin.y;

  for(unsigned int i = 0; i < size_y_; ++i){
    for(unsigned int j = 0; j < size_x_; ++j){
      int index = size_x_ * i + j;
      map_[index].cx = j;
      map_[index].cy = i;
      map_[index].path_dist = new_map.data[index].path_dist;
      map_[index].goal_dist = new_map.data[index].goal_dist;
      map_[index].occ_dist = new_map.data[index].occ_dist;
      map_[index].occ_state = new_map.data[index].occ_state;
    }
  }
}

//reset the path_dist and goal_dist fields for all cells
void MapGrid::resetPathDist(){
  for(unsigned int i = 0; i < map_.size(); ++i){
    map_[i].path_dist = DBL_MAX;
    map_[i].goal_dist = DBL_MAX;
    map_[i].path_mark = false;
    map_[i].goal_mark = false;
    map_[i].within_robot = false;
  }
}

//allow easy creation of messages
ScoreMap2D MapGrid::genMsg(){
  ScoreMap2D map_msg;
  map_msg.size_y = size_y_;
  map_msg.size_x = size_x_;
  map_msg.scale =scale;
  map_msg.origin.x = origin_x;
  map_msg.origin.y = origin_y;

  map_msg.set_data_size(size_y_ * size_x_);

  for(unsigned int i = 0; i < size_y_; ++i){
    for(unsigned int j = 0; j < size_x_; ++j){
      int index = size_x_ * i + j;
      map_msg.data[i].path_dist = map_[index].path_dist;
      map_msg.data[i].goal_dist = map_[index].goal_dist;
      map_msg.data[i].occ_state = map_[index].occ_state;
      map_msg.data[i].occ_dist = map_[index].occ_dist;
    }
  }
  return map_msg;
}
