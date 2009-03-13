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


#ifndef TOPOLOGICAL_MAP_GRID_GRAPH_H
#define TOPOLOGICAL_MAP_GRID_GRAPH_H

#include <topological_map/topological_map.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/multi_array.hpp>
#include <map>
#include <vector>



namespace topological_map 
{

using boost::listS;
using boost::undirectedS;
using boost::adjacency_list;
using boost::graph_traits;
using boost::multi_array;
using boost::tuple;
using std::map;
using std::vector;

struct CellInfo
{
  CellInfo(const Cell2D& cell) : cell(cell) {}
  CellInfo() : cell(4242, 4242) {} // Shouldn't ever be used, but bundled properties are required to be default constructible
  Cell2D cell;
  uint index;
  uint component;
};

struct EdgeCost
{
  EdgeCost(double cost=0.0) : cost(cost) {}
  double cost;
};

typedef adjacency_list<listS, listS, undirectedS, CellInfo, EdgeCost> Grid;
typedef graph_traits<Grid>::vertex_descriptor GridGraphVertex;
typedef graph_traits<Grid>::edge_descriptor GridGraphEdge;
typedef graph_traits<Grid>::adjacency_iterator GridGraphAdjacencyIterator;
typedef graph_traits<Grid>::vertex_iterator GridGraphIterator;
typedef pair<GridGraphAdjacencyIterator, GridGraphAdjacencyIterator> AdjIterPair;
typedef map<Cell2D, GridGraphVertex> CellVertexMap;
typedef vector<Cell2D> GridPath;

/// Internally used class for representing a graph over 2d cells
class GridGraph
{
public:
  // Create an empty graph.  
  GridGraph() {}

  // Create a graph from an occupancy grid.  Edges between non-obstacle vertices have cost 1.0, and others have cost obstacle_cost;
  GridGraph(boost::shared_ptr<OccupancyGrid> grid, double obstacle_cost);

  // Does the graph contain this cell
  bool containsCell (const Cell2D& cell) const;

  // Does the graph contain edge between these two cells
  bool containsEdge (const Cell2D& cell, const Cell2D& cell2) const;
  
  // Return cost of edge between two cells (assert if there isn't one)
  double costBetween (const Cell2D& cell1, const Cell2D& cell2) const;

  // Neighbors of a given cell
  vector<Cell2D> neighbors (const Cell2D& cell) const;

  // Remove edge
  void removeEdge (const Cell2D& cell1, const Cell2D& cell2);

  // Add edge
  void addEdge (const Cell2D& cell1, const Cell2D& cell2, double cost);

  // set costmap.  Array element i,j corresponds to cell with row i, column j
  void setCostmap (multi_array<double, 2> costmap);

  // Shortest path query
  // Return values: 1) The path 2) The cost 3) Was a path found?  1) and 2) aren't valid if 3) is false.
  tuple<bool, double, GridPath> shortestPath (const Cell2D& cell1, const Cell2D& cell2);

  // Costs from a given source to a vector of destinations
  vector<pair<bool, double> > singleSourceCosts (const Cell2D& source, const vector<Cell2D>& dests);

  // Connected components query
  vector<MutableRegionPtr> connectedComponents ();

  GridGraphVertex cellVertex(const Cell2D& cell) const;

private:

  void resetIndices();
  
  Grid graph_;
  CellVertexMap cell_vertex_map_;

};


// Internally used class
// Declaring an instance of this will temporarily sever the given region from the rest of the graph
// until the instance goes out of scope
class RegionIsolator
{
public:
  RegionIsolator (GridGraph* graph, const Region& region);
  ~RegionIsolator();
private:
  typedef tuple<Cell2D, Cell2D, double> RemovedEdge;
  typedef vector<RemovedEdge> RemovedEdges;
  RemovedEdges removed_edges_;
  GridGraph* graph;
};
  





} // namespace 


#endif
