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
#include "map_grid.h"

using namespace std;

MapGrid::MapGrid(unsigned rows, unsigned cols) 
  : rows_(rows), cols_(cols)
{
  //don't allow construction of zero size grid
  assert(rows != 0 && cols != 0);

  map_.resize(rows * cols);

  //make each cell aware of its location in the grid
  for(unsigned int i = 0; i < rows; ++i){
    for(unsigned int j = 0; j < cols; ++ j){
      map_[cols * i + j].ci = i;
      map_[cols * i + j].cj = j;
    }
  }
}

MapCell& MapGrid::operator() (unsigned row, unsigned col){
  assert(row < rows_ && col < cols_);
  //check for legal index
  return map_[cols_ * row + col];
}

MapCell MapGrid::operator() (unsigned row, unsigned col) const {
  //check for legal index
  assert(row < rows_ && col < cols_);
  return map_[cols_ * row + col];
}


MapGrid::MapGrid(const MapGrid& mg){
  rows_ = mg.rows_;
  cols_ = mg.cols_;
  map_ = mg.map_;
}

MapGrid& MapGrid::operator= (const MapGrid& mg){
  rows_ = mg.rows_;
  cols_ = mg.cols_;
  map_ = mg.map_;
  return *this;
}
