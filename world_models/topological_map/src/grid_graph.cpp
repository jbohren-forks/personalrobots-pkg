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


#include <topological_map/grid_graph.h>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <ros/console.h>
#include <ros/assert.h>

namespace topological_map
{

using boost::associative_property_map;
using boost::make_tuple;
using boost::tie;
using std::pair;


/************************************************************
 * basic
 ************************************************************/

GridGraph::GridGraph(const OccupancyGrid& grid)
{
  for (uint r=0; r<numRows(grid); ++r) {
    for (uint c=0; c<numCols(grid); ++c) {
      if (!grid[r][c]) {
        addNode(Cell2D(r,c));
      }
    }
  }
}

void GridGraph::addNode (const Cell2D& cell)
{
  ROS_ASSERT_MSG (cell_vertex_map_.find(cell)==cell_vertex_map_.end(), "Cell %d, %d already exists", cell.r, cell.c);
  
  GridGraphVertex v = add_vertex(CellInfo(cell), graph_);
  cell_vertex_map_[cell]=v;
  for (char vertical=0; vertical<2; ++vertical) {
    for (char multiplier=-1; multiplier<=1; multiplier+=2) {
      char dr = multiplier * (vertical ? 1 : 0);
      char dc = multiplier * (vertical ? 0 : 1);
      Cell2D neighbor(cell.r+dr,cell.c+dc);
      CellVertexMap::iterator iter = cell_vertex_map_.find(neighbor);
      if (iter!=cell_vertex_map_.end()) {
        addEdge(cell,neighbor);
      }
    }
  }
}

void GridGraph::addEdge (const Cell2D& cell1, const Cell2D& cell2)
{
  GridGraphVertex v=cellVertex(cell1);
  GridGraphVertex w=cellVertex(cell2);
  ROS_ASSERT_MSG (!edge(v,w,graph_).second, "Edge already exists between cells");
  add_edge(v,w,EdgeCost(1.0),graph_);
  ROS_DEBUG_STREAM_NAMED ("grid_graph", "Added grid graph edge between " << cell1 << " and " << cell2);
}

void GridGraph::removeEdge (const Cell2D& cell1, const Cell2D& cell2)
{
  GridGraphVertex v=cellVertex(cell1);
  GridGraphVertex w=cellVertex(cell2);
  ROS_ASSERT_MSG (edge(v,w,graph_).second, "Attempted to remove nonexistent edge");
  remove_edge(v,w,graph_);
  ROS_DEBUG_STREAM_NAMED ("grid_graph", "Removed grid graph edge between " << cell1 << " and " << cell2);
}

bool GridGraph::containsCell (const Cell2D& cell) const
{
  CellVertexMap::const_iterator pos = cell_vertex_map_.find(cell);
  return (pos!=cell_vertex_map_.end());
}



struct TransformVertexToCell
{
  TransformVertexToCell (const Grid& grid) : grid(grid) {}
  Cell2D operator() (const GridGraphVertex& v)
  {
    return grid[v].cell;
  }
  const Grid& grid;
};



vector<Cell2D> GridGraph::neighbors (const Cell2D& cell) const
{
  GridGraphVertex v=cellVertex(cell);
  GridGraphAdjacencyIterator begin, end;
  tie(begin,end) =adjacent_vertices(v, graph_);
  vector<Cell2D> neighbors(distance(begin, end));
  transform (begin, end, neighbors.begin(), TransformVertexToCell(graph_));
  return neighbors;
}
  



/************************************************************
 * Shortest paths
 ************************************************************/

/// \todo make the dijkstra visitor exit early when goal is discovered
struct GridGraphDijkstra : public boost::default_dijkstra_visitor
{
  GridGraphDijkstra (const GridGraphVertex& goal) : goal(goal) {}
  void discover_vertex(const GridGraphVertex& v, const Grid& graph) const
  {
    ROS_DEBUG_STREAM_NAMED("dijkstra", "Discovering vertex " << graph[v].cell);
  }
  GridGraphVertex goal;
};


tuple<bool, double, GridPath> GridGraph::shortestPath (const Cell2D& cell1, const Cell2D& cell2)
{
  ROS_DEBUG_STREAM_NAMED("grid_graph_shortest_path", "Looking for shortest path from " << cell1 << " to " << cell2);
  typedef map<GridGraphVertex, double> GridDistances;
  typedef map<GridGraphVertex, GridGraphVertex> GridPreds;
  GridDistances distances;
  GridPreds predecessors;
  
  GridGraphVertex start = cellVertex(cell1);
  GridGraphVertex finish = cellVertex(cell2);

  resetIndices();

  dijkstra_shortest_paths (graph_, start, weight_map(get(&EdgeCost::cost, graph_)).
                           vertex_index_map(get(&CellInfo::index, graph_)).
                           predecessor_map(associative_property_map<GridPreds>(predecessors)).
                           distance_map(associative_property_map<GridDistances>(distances)).
                           visitor(GridGraphDijkstra(finish)));
                           


  // Extract path from predecessor map
  vector<GridGraphVertex> reverse_path(1,finish);
  GridGraphVertex current = finish;

  while (current!=predecessors[current]) {
    reverse_path.push_back(current=predecessors[current]);
  }

  GridPath path(reverse_path.size());

  if (current!=start){
    ROS_DEBUG_STREAM_NAMED("grid_graph_shortest_path", "Did not find a path");
    return make_tuple(false, -1, path);
  }

  // Put it in source-to-goal order, replace vertex descriptors with cells, and return
  transform(reverse_path.rbegin(), reverse_path.rend(), path.begin(), TransformVertexToCell(graph_));
  ROS_DEBUG_STREAM_NAMED("grid_graph_shortest_path", "Found path with cost " << distances[finish]);

  return make_tuple(true, distances[finish], path);
}



/************************************************************
 * Internal
 ************************************************************/

GridGraphVertex GridGraph::cellVertex (const Cell2D& cell) const
{
  CellVertexMap::const_iterator pos = cell_vertex_map_.find(cell);
  ROS_ASSERT_MSG (pos!=cell_vertex_map_.end(), "Could not find vertex for %d, %d", cell.r, cell.c);
  return pos->second;
}

void GridGraph::resetIndices ()
{
  uint ind=0;
  GridGraphIterator iter, end;
  for (tie(iter, end) = vertices(graph_); iter!=end; ++iter) {
    graph_[*iter].index=ind++;
  }
}




/************************************************************
 * Isolate region
 ************************************************************/


RegionIsolator::RegionIsolator (GridGraph* graph, const Region& region) : graph(graph)
{  
  for (Region::iterator iter=region.begin(); iter!=region.end(); ++iter) {
    for (char vertical=0; vertical<2; ++vertical) {
      for (char multiplier=-1; multiplier<=1; multiplier+=2) {
        int dr = multiplier * ( vertical ? 1 : 0 );
        int dc = multiplier * ( vertical ? 0 : 1 );
        Cell2D neighbor(iter->r+dr, iter->c+dc);
        if (region.find(neighbor)==region.end() && graph->containsCell(neighbor)) {
          graph->removeEdge(*iter, neighbor);
          removed_edges_.push_back(pair<Cell2D, Cell2D>(*iter, neighbor));
        }
      }
    }
  }
}



RegionIsolator::~RegionIsolator ()
{
  for (RemovedEdges::iterator iter=removed_edges_.begin(); iter!=removed_edges_.end(); ++iter) {
    graph->addEdge(iter->first, iter->second);
  }
}


} // namespace
