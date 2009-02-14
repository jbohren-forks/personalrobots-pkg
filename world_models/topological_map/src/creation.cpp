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
 */

/**
 * \file
 *
 * Creating a topological map from an occupancy grid
 *
 * \author Bhaskara Marthi
 *
 */





#include <topological_map/topological_map.h>
#include <string>
#include <fstream>
#include <map>
#include <boost/multi_array.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <ros/console.h>
#include <ros/assert.h>

using boost::get;
using boost::tie;
using boost::multi_array;
using boost::listS;
using boost::undirectedS;
using boost::adjacency_list;
using boost::graph_traits;
using boost::extents;
using std::string;
using std::ofstream;
using std::endl;
using std::map;

namespace topological_map
{

/************************************************************
 * Type defs
 ************************************************************/

typedef adjacency_list<listS, listS, undirectedS, Cell2D> GridGraph;
typedef graph_traits<GridGraph>::vertex_descriptor GridGraphVertex;

namespace cell_stat { enum CellStatus { FREE, OBSTACLE, BOTTLENECK }; }
typedef cell_stat::CellStatus CellStatus;
typedef multi_array<CellStatus, 2> CellStatusArray;
typedef map<Cell2D, GridGraphVertex> CellVertexMap;
typedef CellStatusArray::size_type grid_size;
typedef OccupancyGrid::size_type occ_grid_size;

namespace mask_entries { enum MaskEntry { MASK, NONMASK, CONNECTOR1, CONNECTOR2 }; }
typedef mask_entries::MaskEntry MaskEntry;
typedef multi_array<MaskEntry, 2> Mask;
typedef const Mask::size_type* MaskDims;


/// Class that contains the state of the bottleneck finding algorithm
class BottleneckFinder
{
public:
  BottleneckFinder(const OccupancyGrid& g, const uint l, const uint w, const uint r, const string& dir);

  void initializeFromGrid();
  void findBottlenecks();
  TopologicalMapPtr getTopologicalMap() { return topological_map_; }


private:
  BottleneckFinder& operator= (const BottleneckFinder&);
  BottleneckFinder (const BottleneckFinder&);

  bool validCoords (uint r, uint c) { return (r<num_rows_) && (c<num_cols_); }

  void findBottlenecksUsingMask (const Mask& mask);
  void possiblyConnectCells (const uint r, const uint c, const uint r2, const uint c2);
  void possiblyDisconnectCells (const uint r, const uint c, const uint r2, const uint c2);
  void disconnectMask (const uint top, const uint left, const Mask& mask);
  void connectMask (const uint top, const uint left, const Mask& mask);


  /// Write the current cell status array to pgm file
  void writePgm (const string& filename);

  // Input params
  const OccupancyGrid& grid_;
  const uint bottleneck_length_, bottleneck_width_, inflation_radius_;
  const string pgm_output_dir_;
  const uint num_rows_, num_cols_;
  
  // Each cell is obstacle, free, or bottleneck
  CellStatusArray cell_status_array_;

  // Graph over non-obstacle grid cells
  GridGraph grid_graph_;

  // Map from cells to vertices
  CellVertexMap cell_vertex_map_;

  // The computed map
  TopologicalMapPtr topological_map_;
};



// Used only by constructor
uint numRows(const OccupancyGrid& grid)
{
  const occ_grid_size* dims=grid.shape();
  return dims[0];
}

uint numCols(const OccupancyGrid& grid)
{
  const occ_grid_size* dims=grid.shape();
  return dims[1];
}


BottleneckFinder::BottleneckFinder(const OccupancyGrid& g, const uint l, const uint w, const uint r, const string& dir) :
  grid_(g), bottleneck_length_(l), bottleneck_width_(w), inflation_radius_(r), pgm_output_dir_(dir), 
  num_rows_(numRows(g)), num_cols_(numCols(g))
{}
  





void BottleneckFinder::writePgm (const string& filename)
{
  if (!pgm_output_dir_.empty()) {
    const string pathname=pgm_output_dir_+"/"+filename;
    ofstream stream;
    ROS_INFO_NAMED ("writePgm", "Beginning to write pgm file %s", pathname.c_str());
    stream.open(pathname.c_str());
  
    stream << "P5" << endl << num_cols_ << " " << num_rows_ << endl << "255" << endl;
    for (uint r=0; r<num_rows_; ++r) {
      ROS_DEBUG_COND_NAMED (!(r%100), "writePgm", "Row %u", r);
      for (uint c=0; c<num_cols_; ++c) {
        switch (cell_status_array_[r][c]) {
        case cell_stat::FREE: stream << char(255); break;
        case cell_stat::OBSTACLE: stream << char(0); break;
        case cell_stat::BOTTLENECK: stream << char(100); break;
        }
      }
    }
    ROS_INFO_NAMED ("writePgm", "Done writing pgm");
  }
  else {
    ROS_INFO_NAMED ("writePgm", "Not writing pgm as directory not supplied");
  }
}



void BottleneckFinder::initializeFromGrid ()
{
  ROS_INFO_NAMED ("initializeFromGrid", "Starting to initialize graph from grid");
  cell_status_array_.resize(extents[num_rows_][num_cols_]);
  uint threshold = inflation_radius_*inflation_radius_;

  for (uint r=0; r<num_rows_; ++r) {
    ROS_DEBUG_COND_NAMED (!(r%100), "initializeFromGrid", "Row %u", r);
    for (uint c=0; c<num_cols_; ++c) {
      bool found_obstacle=false;
      for (uint r2 = inflation_radius_<=r ? r-inflation_radius_ : 0; r2<=r+inflation_radius_ && !found_obstacle; r2++) {
        for (uint c2 = inflation_radius_<=c ? c-inflation_radius_ : 0; c2<=c+inflation_radius_ && !found_obstacle; c2++) {
          if (validCoords(r2, c2) && grid_[r2][c2] && (r2-r)*(r2-r)+(c2-c)*(c2-c)<= threshold) {
            found_obstacle=true;
          }
        }
      }
      if (found_obstacle) {
        cell_status_array_[r][c] = cell_stat::OBSTACLE;
      }
      else {
        cell_status_array_[r][c] = cell_stat::FREE;
        Cell2D cell(r,c);
        cell_vertex_map_[cell] = add_vertex (cell, grid_graph_);
      }
    }
  }
  ROS_INFO_NAMED ("initializeFromGrid", "Done initializing graph from grid");
  writePgm("inflated.pgm");
}



void BottleneckFinder::possiblyConnectCells (const uint r, const uint c, const uint r2, const uint c2) 
{
  if (cell_status_array_[r][c]!=cell_stat::OBSTACLE && cell_status_array_[r2][c2]!=cell_stat::OBSTACLE) {
    CellVertexMap::iterator v1 = cell_vertex_map_.find(Cell2D(r,c));
    CellVertexMap::iterator v2 = cell_vertex_map_.find(Cell2D(r2,c2));
    ROS_ASSERT (v1!=cell_vertex_map_.end() && v2!=cell_vertex_map_.end());
    add_edge (v1->second, v2->second, grid_graph_);
  }
}

void BottleneckFinder::possiblyDisconnectCells (const uint r, const uint c, const uint r2, const uint c2) 
{
  if (cell_status_array_[r][c]!=cell_stat::OBSTACLE && cell_status_array_[r2][c2]!=cell_stat::OBSTACLE) {
    CellVertexMap::iterator v1 = cell_vertex_map_.find(Cell2D(r,c));
    CellVertexMap::iterator v2 = cell_vertex_map_.find(Cell2D(r2,c2));
    ROS_ASSERT (v1!=cell_vertex_map_.end() && v2!=cell_vertex_map_.end());
    remove_edge (v1->second, v2->second, grid_graph_);
  }
}


void BottleneckFinder::connectMask (const uint top, const uint left, const Mask& mask)
{
  MaskDims dims = mask.shape();
  for (uint r=0; r<dims[0]-1; ++r) {
    for (uint c=0; c<dims[1]-1; ++c) {
      if (mask[r][c]!=mask[r+1][c]) {
        possiblyConnectCells(top+r,left+c,top+r+1,left+c);
      }

      if (mask[r][c]!=mask[r][c+1]) {
        possiblyConnectCells(top+r,left+c,top+r,left+c+1);
      }
    }
  }
}

  

void BottleneckFinder::disconnectMask (const uint top, const uint left, const Mask& mask)
{
  MaskDims dims = mask.shape();
  for (uint r=0; r<dims[0]-1; ++r) {
    for (uint c=0; c<dims[1]-1; ++c) {
      if (mask[r][c]!=mask[r+1][c]) {
        possiblyDisconnectCells(top+r,left+c,top+r+1,left+c);
      }

      if (mask[r][c]!=mask[r][c+1]) {
        possiblyDisconnectCells(top+r,left+c,top+r,left+c+1);
      }
    }
  }
}


void BottleneckFinder::findBottlenecksUsingMask (const Mask& mask)
{
  MaskDims dims = mask.shape();

  // Todo precompute the possible disconnections

  for (uint top=0; top<num_rows_-dims[0]; ++top) {
    ROS_DEBUG_COND_NAMED (!(top%10), "findBottlenecks", "Row %u", top);

    for (uint left=0; left<num_cols_-dims[1]; ++left) {
      disconnectMask (top, left, mask);

      connectMask (top, left, mask);
    }
  }
}

void BottleneckFinder::findBottlenecks ()
{
  ROS_INFO_NAMED ("findBottlenecks", "Commencing scan for bottlenecks");

  // Create a horizontal mask
  Mask horizontal(extents[2*bottleneck_length_][bottleneck_length_+2]);
  for (uint r=0; r<2*bottleneck_length_; ++r) {
    for (uint c=0; c<bottleneck_length_+2; ++c) {
      horizontal[r][c] = mask_entries::NONMASK;
    }
  }
  for (uint r=bottleneck_length_-bottleneck_width_/2; r<=bottleneck_length_+bottleneck_width_/2; ++r) {
    for (uint c=1; c<=bottleneck_length_; ++c) {
      horizontal[r][c] = mask_entries::MASK;
    }
  }
  horizontal[0][bottleneck_length_/2] = mask_entries::CONNECTOR1;
  horizontal[2*bottleneck_length_-1][bottleneck_length_/2] = mask_entries::CONNECTOR2;

  findBottlenecksUsingMask (horizontal);
}


  



/************************************************************
 * Main
 ************************************************************/


TopologicalMapPtr topologicalMapFromGrid (const OccupancyGrid& grid, const uint bottleneck_length, const uint bottleneck_width, const uint inflation_radius, const string& pgm_output_dir)
{
  BottleneckFinder b(grid, bottleneck_length, bottleneck_width, inflation_radius, pgm_output_dir);
  b.initializeFromGrid();
  b.findBottlenecks();

  return b.getTopologicalMap();
}


} // namespace topological_map


