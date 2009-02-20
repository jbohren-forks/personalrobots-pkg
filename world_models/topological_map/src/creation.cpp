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
#include <queue>
#include <set>
#include <algorithm>
#include <boost/multi_array.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/breadth_first_search.hpp>     
#include <boost/graph/connected_components.hpp>     
#include <limits.h>
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
using boost::associative_property_map;
using boost::connected_components;
using std::string;
using std::ofstream;
using std::endl;
using std::map;
using std::queue;
using std::set;
using std::max;
using std::pair;
using std::min;

namespace topological_map
{

/************************************************************
 * Type defs
 ************************************************************/

typedef adjacency_list<listS, listS, undirectedS, Cell2D> GridGraph;
typedef graph_traits<GridGraph>::vertex_descriptor GridGraphVertex;
typedef graph_traits<GridGraph>::adjacency_iterator GridGraphAdjacencyIterator;
typedef pair<GridGraphAdjacencyIterator, GridGraphAdjacencyIterator> AdjIterPair;
typedef map<GridGraphVertex, int> VertexIndexMap;
typedef map<GridGraphVertex, int> VertexCompMap;

namespace cell_stat { enum CellStatus { FREE, OBSTACLE, BOTTLENECK }; }
typedef cell_stat::CellStatus CellStatus;
typedef multi_array<CellStatus, 2> CellStatusArray;
typedef map<Cell2D, GridGraphVertex> CellVertexMap;
typedef CellStatusArray::size_type grid_size;

typedef multi_array<bool, 2> Mask;

typedef const unsigned int cuint;


struct Block
{
  Block(int top, int left, int bottom, int right) : top(top), left(left), bottom(bottom), right(right) {}
  int top, left, bottom, right;
};
typedef vector<Block> Blocks;


/// Class that contains the state of the bottleneck finding algorithm
class BottleneckFinder
{
public:
  BottleneckFinder(const OccupancyGrid& g, const double resolution, const uint size, const uint width, const uint skip, const uint r, const string& dir);

  void initializeFromGrid();
  void findBottlenecks();
  void findOpenRegions();
  TopologicalMapPtr getTopologicalMap() const { return topological_map_; }

  bool distanceWithoutBlockExceeds (cuint top, cuint left, cuint bottom, cuint right, cuint c1r, cuint c1c, cuint c2r, cuint c2c, cuint threshold);



private:
  BottleneckFinder& operator= (const BottleneckFinder&);
  BottleneckFinder (const BottleneckFinder&);

  const GridGraphVertex& cellVertex (const Cell2D& cell);
  const GridGraphVertex& cellVertex (const int r, const int c);

  bool validCoords (const int r, const int c) const;
  bool validNonObstacleCoords (const int r, const int c) const;
  void getConnectors (cuint vertical, cuint top, cuint left, int* c1r, int* c1c, int* c2r, int* c2c) const;


  void possiblyConnectCells (const int r, const int c, const int r2, const int c2);
  void possiblyDisconnectCells (const int r, const int c, const int r2, const int c2);
  void disconnectLine (const int r0, const int c0, const int dr, const int dc, const int last);
  void connectLine (const int r0, const int c0, const int dr, const int dc, const int last);
  void disconnectBlock (cuint top, cuint left, cuint bottom, cuint right);
  void disconnectBlock (const Block& block);
  void connectBlock (cuint top, cuint left, cuint bottom, cuint right);
  void connectBlock (const Block& block);
  void markSmallestBottleneck (int top, int left, int bottom, int right, const int c1r, const int c1c, const int c2r, const int c2c, cuint threshold);

  /// Write the current cell status array to ppm file
  void writePpm (const string& filename) const;

  // Input params
  const OccupancyGrid& grid_;
  const uint bottleneck_size_, bottleneck_width_, bottleneck_skip_, inflation_radius_;
  const string ppm_output_dir_;
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

  // Vector of found bottlenecks
  Blocks bottlenecks_;

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

BottleneckFinder::BottleneckFinder(const OccupancyGrid& g, const double resolution, const uint size, const uint width, const uint skip, const uint r, const string& dir) :
  grid_(g), bottleneck_size_(size), bottleneck_width_(width), bottleneck_skip_(skip), inflation_radius_(r), ppm_output_dir_(dir), 
  num_rows_(numRows(g)), num_cols_(numCols(g)), num_vertices_(0), topological_map_(new TopologicalMap(num_rows_, num_cols_, resolution))
{
  ROS_ASSERT_MSG (max(num_rows_, num_cols_)<INT_MAX/2, "Grid size is %ux%u, which is too large to work given INT_MAX=%d", num_rows_, num_cols_, INT_MAX);
}
  


/************************************************************
 * Distance computations
 ************************************************************/

// Thrown as an exception to terminate bfs 
struct TerminateBfs
{
  bool found;
  TerminateBfs (bool f) : found(f) {}
};

// \todo remove the bfs visitor stuff since hand-implemented bfs below seems faster
class BfsVisitor : public boost::default_bfs_visitor {
public:
  void discover_vertex (const GridGraphVertex& u, const GridGraph& g) const
  {
    cuint r = g[u].r;
    cuint c = g[u].c;
    ROS_DEBUG_NAMED ("distance_internal", "Discovering %d, %d", r, c);
    
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
  ROS_DEBUG_NAMED ("distance_comp", "Checking if distance from (%u, %u) to (%u, %u) exceeds %u when block between (%u,%u) and (%u,%u) is removed.", 
                   c1r, c1c, c2r, c2c, threshold, top, left, bottom, right);
  typedef map<GridGraphVertex,uint> VertexIntMap;
  VertexIntMap costs;
  queue<GridGraphVertex> q;

  disconnectBlock(top, left, bottom, right);

  q.push(cellVertex(c1r,c1c));
  costs[cellVertex(c1r,c1c)] = 0;
  
  // Breadth-first search
  while (true) {

    // Termination condition 1
    if (q.empty()) {
      ROS_DEBUG_NAMED ("distance_comp", "Queue became empty in distance computation, so goal not reachable");
      connectBlock (top, left, bottom, right);
      return true;
    }
    const GridGraphVertex v=q.front();
    q.pop();
    cuint cost = costs[v];

    // Termination condition 2
    if (cost >= threshold) {
      ROS_DEBUG_NAMED ("distance_comp", "Reached cell (%u, %u) at threshold distance %u, so goal not reachable", grid_graph_[v].r, grid_graph_[v].c, threshold);
      connectBlock (top, left, bottom, right);
      return true;
    }

    ROS_DEBUG_NAMED ("distance_internal", "Adding neighbors of (%u, %u)", grid_graph_[v].r, grid_graph_[v].c);


    GridGraphAdjacencyIterator iter, end;
    for (tie(iter, end) = adjacent_vertices(v, grid_graph_); iter!=end; ++iter) {
      const Cell2D c=grid_graph_[*iter];

      // Termination condition 3
      if ((c.r == (int)c2r) && (c.c == (int)c2c)) {
        ROS_DEBUG_NAMED ("distance_comp", "Found goal");
        connectBlock (top, left, bottom, right);
        return false;
      }
      
      VertexIntMap::iterator i = costs.find(*iter);
      if (i==costs.end()) {
        costs[*iter] = 1+costs[v];
        ROS_DEBUG_NAMED ("distance_internal", "Adding (%u, %u) at distance %u", grid_graph_[*iter].r, grid_graph_[*iter].c, 1+costs[v]);
        q.push(cellVertex(c));
      }
    }
 }

//   ROS_DEBUG_NAMED ("find_bottlenecks", "Checking if distance from (%u, %u) to (%u, %u) exceeds %u", c1r, c1c, c2r, c2c, threshold);
//   disconnectBlock(top, left, bottom, right);
//   BfsVisitor vis(c1r, c1c, threshold, c2r, c2c);
//   boost::associative_property_map<VertexIndexMap> index_map(vertex_index_map_);
//   try {
//     boost::breadth_first_search (grid_graph_, cellVertex(c1r,c1c), boost::visitor(vis).vertex_index_map(index_map));
//   }
//   catch (TerminateBfs& e) {
//     if (e.found) {
//       ROS_DEBUG_NAMED ("find_bottlenecks", "Distance does not exceed threshold");
//       connectBlock(top, left, bottom, right);
//       return false;
//     }
//   }
//   ROS_DEBUG_NAMED ("find_bottlenecks", "Distance exceeds threshold");
//   connectBlock(top, left, bottom, right);
//   return true;
}


void BottleneckFinder::writePpm (const string& filename) const
{
  if (!ppm_output_dir_.empty()) {
    const string pathname=ppm_output_dir_+"/"+filename;
    ofstream stream;
    ROS_INFO_NAMED ("write_ppm", "Beginning to write ppm file %s", pathname.c_str());
    stream.open(pathname.c_str());
  
    stream << "P6" << endl << num_cols_ << " " << num_rows_ << endl << "255" << endl;
    for (uint r=0; r<num_rows_; ++r) {
      ROS_DEBUG_COND_NAMED (!(r%100), "write_ppm", "Row %u", r);
      const char on = 255;
      const char off = 0;
      for (uint c=0; c<num_cols_; ++c) {
        switch (cell_status_array_[r][c]) {
        case cell_stat::FREE: stream << on << on << on; break;
        case cell_stat::OBSTACLE: stream << off << off << off; break;
        case cell_stat::BOTTLENECK: stream << on << off << off; break;
        }
      }
    }
    ROS_INFO_NAMED ("write_ppm", "Done writing ppm");
  }
  else {
    ROS_INFO_NAMED ("write_ppm", "Not writing ppm as directory not supplied");
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
  writePpm("inflated.ppm");
}



bool areNeighbors (const GridGraphVertex& v1, const GridGraphVertex& v2, const GridGraph& g)
{
  AdjIterPair iter_pair = adjacent_vertices(v1,g);
  return (iter_pair.second != find(iter_pair.first, iter_pair.second, v2));
}
  

// Connect r,c and r2,c2 if neither is an obstacle
void BottleneckFinder::possiblyConnectCells (const int r, const int c, const int r2, const int c2) 
{
  if (validCoords(r,c) && validCoords(r2,c2) && cell_status_array_[r][c]!=cell_stat::OBSTACLE && cell_status_array_[r2][c2]!=cell_stat::OBSTACLE) {
    GridGraphVertex v1=cellVertex(r,c);
    GridGraphVertex v2=cellVertex(r2,c2);

    if (!areNeighbors(v1,v2,grid_graph_)) {
      ROS_DEBUG_NAMED("connect", "Connecting (%u, %u) and (%d, %d)", r, c, r2, c2);
      add_edge (cellVertex(r,c), cellVertex(r2,c2), grid_graph_);
    }
  }
}

// Disconnect r,c and r2,c2 if neither is an obstacle
void BottleneckFinder::possiblyDisconnectCells (const int r, const int c, const int r2, const int c2) 
{
  if (validCoords(r,c) && validCoords(r2,c2) && cell_status_array_[r][c]!=cell_stat::OBSTACLE && cell_status_array_[r2][c2]!=cell_stat::OBSTACLE) {
    GridGraphVertex v1=cellVertex(r,c);
    GridGraphVertex v2=cellVertex(r2,c2);
    
    if (areNeighbors(v1,v2,grid_graph_)) {
      ROS_DEBUG_NAMED("connect", "Disconnecting (%u, %u) and (%d, %d)", r, c, r2, c2);
      remove_edge (v1, v2, grid_graph_);
    }
  }
}



inline bool BottleneckFinder::validCoords (const int r, const int c) const
{ 
  return (r>=0) && (c>=0) && (r<(int)num_rows_) && (c<(int)num_cols_);
}

inline bool BottleneckFinder::validNonObstacleCoords (const int r, const int c) const
{ 
  return validCoords(r,c) && cell_status_array_[r][c]!=cell_stat::OBSTACLE; 
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


inline void getLoopParams (const int r0, const int c0, const int dr, const int dc, const int last, int* incr, int* incc, int* lastr, int* lastc)
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
  int incr, incc, lastr, lastc;
  getLoopParams(r0, c0, dr, dc, last, &incr, &incc, &lastr, &lastc);
  int r(r0), c(c0);
  for (; r<=lastr && c<=lastc; r+=incr, c+=incc) {
    possiblyDisconnectCells (r, c, r+dr, c+dc);
  }
}

void BottleneckFinder::connectLine (const int r0, const int c0, const int dr, const int dc, const int last)
{
  int incr, incc, lastr, lastc;
  getLoopParams(r0, c0, dr, dc, last, &incr, &incc, &lastr, &lastc);
  int r(r0), c(c0);
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

void BottleneckFinder::disconnectBlock (const Block& block) 
{
  disconnectBlock (block.top, block.left, block.bottom, block.right);
}


void BottleneckFinder::connectBlock (cuint top, cuint left, cuint bottom, cuint right)
{
  ROS_DEBUG_NAMED ("connect", "Connecting block (%u, %u) to (%u, %u)", top, left, bottom, right);
  connectLine (top, left, 0, -1, bottom);
  connectLine (top, left, -1, 0, right);
  connectLine (top, right, 0, 1, bottom);
  connectLine (bottom, left, 1, 0, right);
}


void BottleneckFinder::connectBlock (const Block& block)
{
  connectBlock (block.top, block.left, block.bottom, block.right);
}



// Helper class for markSmallestBottleneck to avoid having to write binary search four times
struct BinarySearcher {
  BinarySearcher (int* topIn, int* leftIn, int* bottomIn, int* rightIn, const int c1rIn, const int c1cIn, const int c2rIn, const int c2cIn, cuint thresholdIn, BottleneckFinder* bf) :
    top(topIn), left(leftIn), bottom(bottomIn), right(rightIn), c1r(c1rIn), c1c(c1cIn), c2r(c2rIn), c2c(c2cIn), threshold(thresholdIn), bottleneck_finder(bf) {}
  
  void search (cuint ind, const int bound, bool upper_val)
  {
    int* var_of_interest;
    switch (ind) {
    case 0: var_of_interest=top; break;
    case 1: var_of_interest=left; break;
    case 2: var_of_interest=bottom; break;
    default: var_of_interest=right; break;
    }
    int lower = min(*var_of_interest, bound);
    int upper = max(*var_of_interest, bound);
    while (upper>lower+1) {
      ROS_DEBUG_NAMED ("find_bottlenecks", "Params are %d, %d, %d, %d", *top, *left, *bottom, *right);
      *var_of_interest = (upper+lower)/2;
      if (bottleneck_finder->distanceWithoutBlockExceeds(*top, *left, *bottom, *right, c1r, c1c, c2r, c2c, threshold) == upper_val)
        upper=*var_of_interest;
      else 
        lower=*var_of_interest;
    }
    *var_of_interest = upper_val ? upper : lower;
    ROS_DEBUG_NAMED ("find_bottlenecks", "Bsearch result is %d", *var_of_interest);
    
  }

  int *top, *left, *bottom, *right;
  const int c1r, c1c, c2r, c2c;
  cuint threshold;
  BottleneckFinder* bottleneck_finder;

};


// We know that the given block is a bottleneck w.r.t c1 and c2, and threshold
// This function tries to shrink the block as much as possible while still retaining this property, and then marks the cells as bottlenecks and adds the region to the graph
inline void BottleneckFinder::markSmallestBottleneck (int top_orig, int left_orig, int bottom, int right, const int c1r, const int c1c, const int c2r, const int c2c, cuint threshold) 
{
  int top(top_orig), left(left_orig);
  ROS_ASSERT (distanceWithoutBlockExceeds(top, left, bottom, right, c1r, c1c, c2r, c2c, threshold));

  BinarySearcher b(&top, &left, &bottom, &right, c1r, c1c, c2r, c2c, threshold, this);
  b.search (2, top-1, true);
  b.search (3, left-1, true);
  b.search (0, bottom+1, false);
  b.search (1, right+1, false);

  ROS_ASSERT ((top<=bottom)&&(left<=right));
  uint dy=bottom-top;
  uint dx=right-left;
  if (dy<bottleneck_width_) {
    top -= (bottleneck_width_-dy);
    if (top<top_orig) {
      top=top_orig;
      bottom=top+bottleneck_width_;
    }
  }
  if (dx<bottleneck_width_) {
    left -= (bottleneck_width_-dx);
    if (left<left_orig) {
      left=left_orig;
      right=left+bottleneck_width_;
    }
  }

  
  ROS_DEBUG_NAMED ("find_bottlenecks", "Marking bottleneck between (%d, %d) and (%d, %d)", top, left, bottom, right);
  for (int r=top; r<=bottom; ++r) {
    for (int c=left; c<=right; ++c) {
      if (cell_status_array_[r][c] == cell_stat::FREE) {
        cell_status_array_[r][c] = cell_stat::BOTTLENECK;
      }
    }
  }
  bottlenecks_.push_back(Block(top, left, bottom, right));
}

void BottleneckFinder::findBottlenecks ()
{
  ROS_INFO_NAMED ("bottleneck_finder", "Commencing scan for bottlenecks");

  for (uint top=0; top<num_rows_-bottleneck_size_; top+=bottleneck_skip_) {
    uint bottom = top+bottleneck_size_;
    ROS_DEBUG_COND_NAMED (!(top%1), "bottleneck_finder", "Row %u", top);
    for (uint left=0; left<num_cols_-bottleneck_size_; left+=bottleneck_skip_) {
      uint right = left+bottleneck_size_;
      ROS_DEBUG_NAMED ("find_bottlenecks", "Considering block from (%u, %u) to (%u, %u)", top, left, bottom, right);
        for (uint vertical=0; vertical<2; vertical++) {
        int c1r, c1c, c2r, c2c;
        getConnectors (vertical, top, left, &c1r, &c1c, &c2r, &c2c);
        // At this point we've chosen a block and pair of connectors on either side
        

        // If not valid coords, or if block contains bottleneck cells, skip
        bool valid = validNonObstacleCoords(c1r,c1c) && validNonObstacleCoords(c2r,c2c) && validCoords(top, left) && validCoords(bottom, right);
        for (uint r=top; r<=bottom; r++) 
          for (uint c=left; c<=right; c++) 
            if (cell_status_array_[r][c]==cell_stat::BOTTLENECK) { valid=false; break; }
        if (!valid) break;

        if (distanceWithoutBlockExceeds(top, left, bottom, right, c1r, c1c, c2r, c2c, 6*bottleneck_size_)) {
          ROS_DEBUG_NAMED ("find_bottlenecks", "Block from (%u, %u) to (%u, %u) is a bottleneck", top, left, bottom, right);
          markSmallestBottleneck (top, left, bottom, right, c1r, c1c, c2r, c2c, 6*bottleneck_size_);
        }
      }
    }
  }
  ROS_INFO_NAMED ("bottleneck_finder", "Done scanning for bottlenecks.");
  writePpm ("bottlenecks.ppm");
}

  
void BottleneckFinder::findOpenRegions ()
{
  ROS_INFO_NAMED ("bottleneck_finder", "Constructing topological graph");
  // First disconnect all the existing bottlenecks
  for (Blocks::iterator iter=bottlenecks_.begin(); iter!=bottlenecks_.end(); ++iter) {
    disconnectBlock (*iter);
  }

  // Compute connected components
  VertexCompMap components;
  associative_property_map<VertexCompMap> adapted_comps(components);
  associative_property_map<VertexIndexMap> adapted_index_map(vertex_index_map_);
  int numComps = connected_components(grid_graph_, adapted_comps, vertex_index_map(adapted_index_map));

  // Add them to graph (should actually do this in a single loop)
  for (int i=0; i<numComps; i++) {
    MutableRegionPtr r(new Region);
    for (VertexCompMap::iterator iter=components.begin(); iter!=components.end(); ++iter) {
      if (iter->second==i) {
        r->insert(grid_graph_[iter->first]);
      }
    }

    // Add to graph
    if (r->size()>0) {
      Cell2D cell=*r->begin();
      if (cell_status_array_[cell.r][cell.c]==cell_stat::FREE) {
        topological_map_->addRegion(r, OPEN);
      }
      else {
        topological_map_->addRegion(r, DOORWAY);
      }
    }
  }
  ROS_INFO_NAMED ("bottleneck_finder", "Done constructing topological graph");
}  




  
  


/************************************************************
 * Main
 ************************************************************/


TopologicalMapPtr topologicalMapFromGrid (const OccupancyGrid& grid, const double resolution, const uint bottleneck_size, const uint bottleneck_width, const uint bottleneck_skip, const uint inflation_radius, const string& ppm_output_dir)
{
  BottleneckFinder b(grid, resolution, bottleneck_size, bottleneck_width, bottleneck_skip, inflation_radius, ppm_output_dir);
  b.initializeFromGrid();
  b.findBottlenecks();
  b.findOpenRegions();
  
  return b.getTopologicalMap();
}


} // namespace topological_map


