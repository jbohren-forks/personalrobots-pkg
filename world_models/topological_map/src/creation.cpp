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
#include <boost/graph/breadth_first_search.hpp>     
#include <boost/graph/connected_components.hpp>     
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
typedef map<GridGraphVertex, int> VertexIndexMap;

namespace cell_stat { enum CellStatus { FREE, OBSTACLE, BOTTLENECK }; }
typedef cell_stat::CellStatus CellStatus;
typedef multi_array<CellStatus, 2> CellStatusArray;
typedef map<Cell2D, GridGraphVertex> CellVertexMap;
typedef CellStatusArray::size_type grid_size;
typedef OccupancyGrid::size_type occ_grid_size;

typedef multi_array<bool, 2> Mask;

typedef const unsigned int cuint;

/// Class that contains the state of the bottleneck finding algorithm
class BottleneckFinder
{
public:
  BottleneckFinder(const OccupancyGrid& g, const uint size, const uint skip, const uint r, const string& dir);

  void initializeFromGrid();
  void findBottlenecks();
  TopologicalMapPtr getTopologicalMap() const { return topological_map_; }


private:
  BottleneckFinder& operator= (const BottleneckFinder&);
  BottleneckFinder (const BottleneckFinder&);

  const GridGraphVertex& cellVertex (const Cell2D& cell);
  const GridGraphVertex& cellVertex (const int r, const int c);

  bool validCoords (const int r, const int c) const;
  bool validFreeCoords (const int r, const int c) const;
  void getConnectors (cuint vertical, cuint top, cuint left, int* c1r, int* c1c, int* c2r, int* c2c) const;


  bool distanceWithoutBlockExceeds (cuint top, cuint left, cuint bottom, cuint right, cuint c1r, cuint c1c, cuint c2r, cuint c2c, cuint threshold);

  void possiblyConnectCells (const uint r, const uint c, const uint r2, const uint c2);
  void possiblyDisconnectCells (const uint r, const uint c, const uint r2, const uint c2);
  void disconnectLine (const int r0, const int c0, const int dr, const int dc, const int last);
  void connectLine (const int r0, const int c0, const int dr, const int dc, const int last);
  void disconnectBlock (cuint top, cuint left, cuint bottom, cuint right);
  void connectBlock (cuint top, cuint left, cuint bottom, cuint right);

  /// Write the current cell status array to pgm file
  void writePgm (const string& filename) const;

  // Input params
  const OccupancyGrid& grid_;
  const uint bottleneck_size_, bottleneck_skip_, inflation_radius_;
  const string pgm_output_dir_;
  const uint num_rows_, num_cols_;
  
  // Each cell is obstacle, free, or bottleneck
  CellStatusArray cell_status_array_;

  // Graph over non-obstacle grid cells
  GridGraph grid_graph_;
  VertexIndexMap vertex_index_map_;
  uint num_vertices_;

  // Map from cells to vertices
  CellVertexMap cell_vertex_map_;

  // The computed map
  TopologicalMapPtr topological_map_;
};


/************************************************************
 * Accessors
 ************************************************************/

const GridGraphVertex& BottleneckFinder::cellVertex (const Cell2D& cell) 
{
  CellVertexMap::iterator iter = cell_vertex_map_.find(cell);
  ROS_ASSERT (iter != cell_vertex_map_.end());
  return iter->second;
}

const GridGraphVertex& BottleneckFinder::cellVertex (const int r, const int c)
{
  return cellVertex(Cell2D(r,c));
}


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


BottleneckFinder::BottleneckFinder(const OccupancyGrid& g, const uint size, const uint skip, const uint r, const string& dir) :
  grid_(g), bottleneck_size_(size), bottleneck_skip_(skip), inflation_radius_(r), pgm_output_dir_(dir), 
  num_rows_(numRows(g)), num_cols_(numCols(g)), num_vertices_(0)
{}
  


/************************************************************
 * Distance computations
 * This is ultimately done using the bgl breadth_first_search.
 ************************************************************/

// Thrown as an exception to terminate bfs 
struct TerminateBfs
{
  bool found;
  TerminateBfs (bool f) : found(f) {}
};

class BfsVisitor : public boost::default_bfs_visitor {
public:
  void discover_vertex (const GridGraphVertex& u, const GridGraph& g) const
  {
    cuint r = g[u].r;
    cuint c = g[u].c;
    ROS_DEBUG_NAMED ("distance_comp", "Discovering %d, %d", r, c);
    
    if ((r == rg_) && (c == cg_)) {
      ROS_DEBUG_NAMED ("distance_comp", "Terminating due to goal reached");
      throw TerminateBfs (true);
    }
    else if (abs(r-r0_) + abs(c-c0_) > threshold_) {
      ROS_DEBUG_NAMED ("distance_comp", "Terminating due to distance threshold");
      throw TerminateBfs (false);
    }
  }

  BfsVisitor (cuint r0, cuint c0, cuint threshold, cuint rg, cuint cg) :
    r0_(r0), c0_(c0), rg_(rg), cg_(cg), threshold_(threshold) {}

private:
  cuint r0_, c0_, rg_, cg_;
  const int threshold_;
};



inline bool BottleneckFinder::distanceWithoutBlockExceeds (cuint top, cuint left, cuint bottom, cuint right, cuint c1r, cuint c1c, cuint c2r, cuint c2c, cuint threshold)
{
  disconnectBlock(top, left, bottom, right);
  BfsVisitor vis(c1r, c1c, threshold, c2r, c2c);
  boost::associative_property_map<VertexIndexMap> index_map(vertex_index_map_);
  ROS_DEBUG_NAMED ("find_bottlenecks", "Checking if distance from (%u, %u) to (%u, %u) exceeds %u", c1r, c1c, c2r, c2c, threshold);
  try {
    boost::breadth_first_search (grid_graph_, cellVertex(c1r,c1c), boost::visitor(vis).vertex_index_map(index_map));
  }
  catch (TerminateBfs& e) {
    if (e.found) {
      ROS_DEBUG_NAMED ("find_bottlenecks", "Distance does not exceed threshold");
      connectBlock(top, left, bottom, right);
      return false;
    }
  }
  ROS_DEBUG_NAMED ("find_bottlenecks", "Distance exceeds threshold");
  connectBlock(top, left, bottom, right);
  return true;
}


void BottleneckFinder::writePgm (const string& filename) const
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
  ROS_INFO_NAMED ("initialize_from_grid", "Starting to initialize graph from grid");
  cell_status_array_.resize(extents[num_rows_][num_cols_]);
  uint threshold = inflation_radius_*inflation_radius_;

  for (uint r=0; r<num_rows_; ++r) {
    ROS_INFO_COND_NAMED (!(r%100), "initialize_from_grid", "Row %u", r);
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
        GridGraphVertex v = add_vertex (cell, grid_graph_);
        ROS_DEBUG_NAMED ("initialize_from_grid", "Adding vertex (%d, %d)", r, c);
        cell_vertex_map_[cell] = v;
        vertex_index_map_[v] = num_vertices_++;

        // Possibly add edges
        for (uint upward=0; upward<2; ++upward) {
          int r2 = upward ? r-1 : r;
          int c2 = upward ? c : c-1;
          if (validCoords(r2,c2) && cell_status_array_[r2][c2] == cell_stat::FREE) {
            ROS_DEBUG_NAMED ("initialize_from_grid", "Adding edge from (%d, %d) to (%d, %d)", r2, c2, r, c);
            add_edge (v, cellVertex(r2,c2), grid_graph_);
          }
        }
      }
    }
  }

  ROS_INFO_NAMED ("initialize_from_grid", "Done initializing graph from grid");
  writePgm("inflated.pgm");
}



// Connect r,c and r2,c2 if neither is an obstacle
void BottleneckFinder::possiblyConnectCells (cuint r, cuint c, cuint r2, cuint c2) 
{
  if (validCoords(r,c) && validCoords(r2,c2) && cell_status_array_[r][c]!=cell_stat::OBSTACLE && cell_status_array_[r2][c2]!=cell_stat::OBSTACLE) {
    // Should also assert there's no existing edge
    ROS_DEBUG_NAMED("connect", "Connecting (%u, %u) and (%d, %d)", r, c, r2, c2);
    add_edge (cellVertex(r,c), cellVertex(r2,c2), grid_graph_);
  }
}

// Disconnect r,c and r2,c2 if neither is an obstacle
void BottleneckFinder::possiblyDisconnectCells (cuint r, cuint c, cuint r2, cuint c2) 
{
  if (validCoords(r,c) && validCoords(r2,c2) && cell_status_array_[r][c]!=cell_stat::OBSTACLE && cell_status_array_[r2][c2]!=cell_stat::OBSTACLE) {
    ROS_DEBUG_NAMED("connect", "Disconnecting (%u, %u) and (%d, %d)", r, c, r2, c2);
    remove_edge (cellVertex(r,c), cellVertex(r2,c2), grid_graph_);
  }
}



inline bool BottleneckFinder::validCoords (const int r, const int c) const
{ 
  return (r>=0) && (c>=0) && (r<(int)num_rows_) && (c<(int)num_cols_);
}

inline bool BottleneckFinder::validFreeCoords (const int r, const int c) const
{ 
  return validCoords(r,c) && cell_status_array_[r][c]==cell_stat::FREE; 
}


// Get vertical/horizontal connects
inline void BottleneckFinder::getConnectors (cuint vertical, cuint top, cuint left, int* c1r, int* c1c, int* c2r, int* c2c) const
{
  uint half_size = bottleneck_size_/2;
  *c1r = vertical ? top - bottleneck_size_ : top + half_size;
  *c1c = vertical ? left + half_size : left - bottleneck_size_;
  *c2r = vertical ? top + 2*bottleneck_size_ : top + half_size;
  *c2c = vertical ? left + half_size : left + 2*bottleneck_size_;
}


inline void getLoopParams (const int r0, const int c0, const int dr, const int dc, const int last, uint* incr, uint* incc, uint* lastr, uint* lastc)
{
  if (dr) {
    *incr=0;
    *incc=1;
    *lastr=r0;
    *lastc=last;
  }
  else {
    *incr=1;
    *incc=0;
    *lastr=last;
    *lastc=c0;
  }
}

void BottleneckFinder::disconnectLine (const int r0, const int c0, const int dr, const int dc, const int last)
{
  uint incr, incc, lastr, lastc;
  getLoopParams(r0, c0, dr, dc, last, &incr, &incc, &lastr, &lastc);
  uint r(r0), c(c0);
  for (; r<=lastr && c<=lastc; r+=incr, c+=incc) {
    possiblyDisconnectCells (r, c, r+dr, c+dc);
  }
}

void BottleneckFinder::connectLine (const int r0, const int c0, const int dr, const int dc, const int last)
{
  uint incr, incc, lastr, lastc;
  getLoopParams(r0, c0, dr, dc, last, &incr, &incc, &lastr, &lastc);
  uint r(r0), c(c0);
  for (; r<=lastr && c<=lastc; r+=incr, c+=incc) {
    possiblyConnectCells (r, c, r+dr, c+dc);
  }
}

void BottleneckFinder::disconnectBlock (cuint top, cuint left, cuint bottom, cuint right)
{
  ROS_DEBUG_NAMED ("connect", "Disconnecting block (%u, %u) to (%u, %u)", top, left, bottom, right);
  disconnectLine (top, left, 0, -1, bottom);
  disconnectLine (top, left, -1, 0, right);
  disconnectLine (top, right, 0, 1, bottom);
  disconnectLine (bottom, left, 1, 0, right);
}

void BottleneckFinder::connectBlock (cuint top, cuint left, cuint bottom, cuint right)
{
  ROS_DEBUG_NAMED ("connect", "Connecting block (%u, %u) to (%u, %u)", top, left, bottom, right);
  connectLine (top, left, 0, -1, bottom);
  connectLine (top, left, -1, 0, right);
  connectLine (top, right, 0, 1, bottom);
  connectLine (bottom, left, 1, 0, right);
}


void BottleneckFinder::findBottlenecks ()
{
  ROS_INFO_NAMED ("findBottlenecks", "Commencing scan for bottlenecks");

  for (uint top=0; top<num_rows_-bottleneck_size_; ++top) {
    for (uint left=0; left<num_cols_-bottleneck_size_; ++left) {
      for (uint vertical=0; vertical<2; vertical++) {
        int c1r, c1c, c2r, c2c;
        getConnectors (vertical, top, left, &c1r, &c1c, &c2r, &c2c);
        // At this point we've chosen a block and pair of connectors on either side

        // If not valid skip
        bool valid = validFreeCoords(c1r,c1c) && validFreeCoords(c2r,c2c) && validCoords(top, left) && validCoords(top+bottleneck_size_, left+bottleneck_size_);
        for (uint r=top; r<top+bottleneck_size_; r++) 
          for (uint c=left; c<left+bottleneck_size_; c++) 
            if (cell_status_array_[r][c]==cell_stat::BOTTLENECK) { valid=false; break; }
        if (!valid) break;

        if (distanceWithoutBlockExceeds(top, left, top+bottleneck_size_, left+bottleneck_size_, c1r, c1c, c2r, c2c, 6*bottleneck_size_)) {
          ROS_DEBUG_NAMED ("find_bottlenecks", "Block from (%u, %u) to (%u, %u) is a bottleneck", top, left, top+bottleneck_size_, left+bottleneck_size_);
        }
        else {
          ROS_DEBUG_NAMED ("find_bottlenecks", "Block from (%u, %u) to (%u, %u) is not a bottleneck", top, left, top+bottleneck_size_, left+bottleneck_size_);
        }
      }
    }
  }
}

  



/************************************************************
 * Main
 ************************************************************/


TopologicalMapPtr topologicalMapFromGrid (const OccupancyGrid& grid, const uint bottleneck_size, const uint bottleneck_skip, const uint inflation_radius, const string& pgm_output_dir)
{
  BottleneckFinder b(grid, bottleneck_size, bottleneck_skip, inflation_radius, pgm_output_dir);
  b.initializeFromGrid();
  b.findBottlenecks();

  return b.getTopologicalMap();
}


} // namespace topological_map


