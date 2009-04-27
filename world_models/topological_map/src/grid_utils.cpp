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
 */


#include <topological_map/grid_utils.h>
#include <fstream>
#include <string>
#include <algorithm>
#include <ros/console.h>
#include <topological_map/exception.h>

namespace topological_map
{

using std::string;
using std::max;
using std::min;

OccupancyGrid loadOccupancyGrid (const string& filename)
{
  std::ifstream str(filename.c_str());

  string type;
  str >> type;
  if (type!="P5") {
    throw GridFileTypeException(type);
  }

  uint width, height, maxgrey;
  str >> width >> height >> maxgrey;
  char line[100];
  str.getline(line, 100);
  
  ROS_INFO_STREAM_NAMED ("grid", "Loading " << height << "x" << width << " grid");
  
  char* data = new char[width*height];
  str.read(data, width*height);

  OccupancyGrid grid(boost::extents[height][width]);
  uint ind=0;
  for (uint r=height; r>0; --r) {
    for (uint c=0; c<width; ++c) {
      unsigned char val = maxgrey - data[ind++];
      grid[r-1][c] = val>(float)maxgrey*.1;
    }
  }

  ROS_INFO_NAMED ("grid", "Done loading grid");

  delete[] data;
  return grid;
}

OccupancyGrid inflateObstacles (const OccupancyGrid& grid, uint radius)
{
  uint nr=numRows(grid);
  uint nc=numCols(grid);
  
  OccupancyGrid new_grid = grid;
  
  for (uint r=0; r<nr; ++r) {
    for (uint c=0; c<nc; ++c) {
      uint r0 = r>=radius ? r-radius : 0;
      for (uint r1=r0; (r1<=min(r+radius, nr-1)) && !new_grid[r][c]; ++r1) {
        uint c0 = c>=radius ? c-radius : 0;
        for (uint c1=c0; (c1<=min(c+radius, nc-1)) && !new_grid[r][c]; ++c1) {
          if (grid[r1][c1]) {
            new_grid[r][c] = true;
          }
        }
      }
    }
  }

  return new_grid;
}





}
