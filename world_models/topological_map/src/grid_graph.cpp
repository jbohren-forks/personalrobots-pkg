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
#include <queue>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/bind.hpp>
#include <ros/console.h>
#include <ros/assert.h>

namespace topological_map
{

using boost::associative_property_map;
using boost::make_tuple;
using boost::tie;
using boost::bind;
using boost::shared_ptr;
using boost::edge;
using std::pair;
using std::queue;


/************************************************************
 * basic
 ************************************************************/

GridGraph::GridGraph(shared_ptr<OccupancyGrid> grid, const double obstacle_cost)
{
  for (uint r=0; r<numRows(*grid); ++r) {
    for (uint c=0; c<numCols(*grid); ++c) {
      
      // Add node
      Cell2D cell(r,c);
      ROS_ASSERT_MSG (cell_vertex_map_.find(cell)==cell_vertex_map_.end(), "Cell %d, %d already exists", cell.r, cell.c);
      GridGraphVertex v = add_vertex(CellInfo(cell), graph_);
      cell_vertex_map_[cell]=v;

      // Add edges
      for (char vertical=0; vertical<2; ++vertical) {
        for (char multiplier=-1; multiplier<=1; multiplier+=2) {
          char dr = multiplier * (vertical ? 1 : 0);
          char dc = multiplier * (vertical ? 0 : 1);
          int r2=cell.r+dr, c2=cell.c+dc;
          Cell2D neighbor(r2,c2);
          CellVertexMap::iterator iter = cell_vertex_map_.find(neighbor);
          if (iter!=cell_vertex_map_.end()) {
            double cost = (*grid)[r][c] || (*grid)[r2][c2] ? obstacle_cost : 1.0;
            addEdge(cell,neighbor,cost);
          }
        }
      }
    }
  }
}

void GridGraph::addEdge (const Cell2D& cell1, const Cell2D& cell2, const double cost)
{
  const GridGraphVertex v=cellVertex(cell1);
  const GridGraphVertex w=cellVertex(cell2);
  ROS_ASSERT_MSG (!edge(v,w,graph_).second, "Edge already exists between cells");
  add_edge(v,w,EdgeCost(cost),graph_);
}

void GridGraph::removeEdge (const Cell2D& cell1, const Cell2D& cell2)
{
  ROS_ASSERT_MSG (containsEdge(cell1, cell2), "Attempted to remove nonexistent edge");
  GridGraphVertex v=cellVertex(cell1);
  GridGraphVertex w=cellVertex(cell2);
  remove_edge(v,w,graph_);
}

bool GridGraph::containsCell (const Cell2D& cell) const
{
  CellVertexMap::const_iterator pos = cell_vertex_map_.find(cell);
  return (pos!=cell_vertex_map_.end());
}

bool GridGraph::containsEdge (const Cell2D& cell1, const Cell2D& cell2) const
{
  if (!containsCell(cell1) || !containsCell(cell2)) {
    return false;
  }
  else {
    return edge(cellVertex(cell1), cellVertex(cell2), graph_).second;
  }
}

double GridGraph::costBetween (const Cell2D& cell1, const Cell2D& cell2) const
{
  ROS_ASSERT(containsEdge(cell1, cell2));
  return graph_[edge(cellVertex(cell1), cellVertex(cell2), graph_).first].cost;
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

struct GraphSearchVisitor : public boost::default_dijkstra_visitor
{
  void discover_vertex(const GridGraphVertex& v, const Grid& graph) const
  {
    ROS_DEBUG_STREAM_NAMED("dijkstra", "Discovering vertex " << graph[v].cell);
  }
};

typedef map<GridGraphVertex, double> GridDistances;
typedef pair<bool, double> ReachableCost;
typedef vector<ReachableCost> ReachableCostVector;

ReachableCost extractCost (const GridGraph* g, const GridDistances& distances, const Cell2D& cell)
{
  GridDistances::const_iterator iter=distances.find(g->cellVertex(cell));
  if (iter==distances.end()) {
    return ReachableCost(false,-1);
  }
  else {
    return ReachableCost(true, iter->second);
  }
}

ReachableCostVector GridGraph::singleSourceCosts (const Cell2D& source, const vector<Cell2D>& dests)
{
  GridDistances distances;
  GridGraphVertex start = cellVertex(source);

  resetIndices();
  ROS_DEBUG_NAMED ("grid_graph_shortest_path", "Computing single source costs");


  boost::dijkstra_shortest_paths (graph_, start,
                                  weight_map(get(&EdgeCost::cost, graph_)).
                                  vertex_index_map(get(&CellInfo::index, graph_)).
                                  distance_map(associative_property_map<GridDistances>(distances)).
                                  visitor(GraphSearchVisitor()));

   
  ReachableCostVector v(dests.size());
  transform (dests.begin(), dests.end(), v.begin(), bind(extractCost, this, distances, _1));
  for (uint i=0; i<dests.size(); ++i) {
    ROS_DEBUG_STREAM_NAMED ("grid_graph_shortest_path", " connector " << dests[i] << " ; cost " << v[i].second);
  }
  ROS_DEBUG_NAMED ("grid_graph_shortest_path", "Done computing single source costs");
  return v;
}

   


//   catch (GridGraphVertex& e) {
//     // Extract path from predecessor map
//     vector<GridGraphVertex> reverse_path(1,finish);
//     GridGraphVertex current = finish;

//     while (current!=predecessors[current]) {
//       reverse_path.push_back(current=predecessors[current]);
//     }
//     GridPath path(reverse_path.size());
//     ROS_ASSERT_MSG (current==start, "Unexpectedly could not find shortest path");

//     // Put it in source-to-goal order, replace vertex descriptors with cells, and return
//     transform(reverse_path.rbegin(), reverse_path.rend(), path.begin(), TransformVertexToCell(graph_));
//     ROS_DEBUG_STREAM_NAMED("grid_graph_shortest_path", "Found path with cost " << distances[finish]);
//     return make_tuple(true, distances[finish], path);
//   }

//   // Else, we must not have found a path
//   ROS_DEBUG_STREAM_NAMED("grid_graph_shortest_path", "Did not find a path");
//   return make_tuple(false, -1, GridPath());
  



// Find shortest path between cells in grid graph
tuple<bool, double, GridPath> GridGraph::shortestPath (const Cell2D& cell1, const Cell2D& cell2)
{
  ROS_DEBUG_STREAM_NAMED("grid_graph_shortest_path", "Looking for shortest path from " << cell1 << " to " << cell2);

  typedef pair<GridGraphVertex,int> QueueItem;
  queue<QueueItem> q;
  set<GridGraphVertex> visited;
  map<GridGraphVertex,GridGraphVertex> predecessor_map;
  GridGraphVertex goal=cellVertex(cell2), start=cellVertex(cell1);

  q.push(QueueItem(start,0));
  visited.insert(start);
  while (!q.empty()) {
    GridGraphVertex v;
    int d;
    tie(v,d) = q.front();
    q.pop();

    // If goal is found, extract the path and return
    if (v==goal) {
      ROS_DEBUG_STREAM_NAMED("grid_graph_shortest_path", "Found path with length " << d);
      vector<GridGraphVertex> reverse_path(1,goal);
      GridGraphVertex current=goal;
      while (current!=start) {
        ROS_ASSERT(predecessor_map.find(current)!=predecessor_map.end());
        current = predecessor_map[current];
        reverse_path.push_back(current);
      }
      GridPath path(reverse_path.size());
      transform(reverse_path.rbegin(), reverse_path.rend(), path.begin(), TransformVertexToCell(graph_));
      return make_tuple(true, d, path);
    }

    // Otherwise, add successors to the queue
    else {
      GridGraphAdjacencyIterator iter, end;
      for (tie(iter,end)=adjacent_vertices(v, graph_); iter!=end; iter++) {
        if (visited.find(*iter)==visited.end()) {
          q.push(QueueItem(*iter, 1+d));
          visited.insert(*iter);
          predecessor_map[*iter]=v;
        }
      }
    }
  }



  return make_tuple(false, -1, GridPath());
        
      
  
  
//   typedef map<GridGraphVertex, double> GridDistances;
//   typedef map<GridGraphVertex, GridGraphVertex> GridPreds;
//   GridDistances distances;
//   GridPreds predecessors;
  
//   GridGraphVertex start = cellVertex(cell1);
//   GridGraphVertex finish = cellVertex(cell2);

//   resetIndices();

//   try {
//     dijkstra_shortest_paths (graph_, start,
//                              weight_map(get(&EdgeCost::cost, graph_)).
//                              vertex_index_map(get(&CellInfo::index, graph_)).
//                              predecessor_map(associative_property_map<GridPreds>(predecessors)).
//                              distance_map(associative_property_map<GridDistances>(distances)).
//                              visitor(GraphSearchVisitor(finish)));
//   }
//   catch (GridGraphVertex& e) {
//     // Extract path from predecessor map
//     vector<GridGraphVertex> reverse_path(1,finish);
//     GridGraphVertex current = finish;

//     while (current!=predecessors[current]) {
//       reverse_path.push_back(current=predecessors[current]);
//     }
//     GridPath path(reverse_path.size());
//     ROS_ASSERT_MSG (current==start, "Unexpectedly could not find shortest path");

//     // Put it in source-to-goal order, replace vertex descriptors with cells, and return
//     transform(reverse_path.rbegin(), reverse_path.rend(), path.begin(), TransformVertexToCell(graph_));
//     ROS_DEBUG_STREAM_NAMED("grid_graph_shortest_path", "Found path with cost " << distances[finish]);
//     return make_tuple(true, distances[finish], path);
//   }

//   // Else, we must not have found a path
//   ROS_DEBUG_STREAM_NAMED("grid_graph_shortest_path", "Did not find a path");
//   return make_tuple(false, -1, GridPath());
}


/************************************************************
 * Connected comps
 ************************************************************/

MutableRegionPtr makeEmptyRegion()
{
  return MutableRegionPtr(new Region);
}

vector<MutableRegionPtr> GridGraph::connectedComponents ()
{
  resetIndices();
  int num_comps=connected_components(graph_, get(&CellInfo::component, graph_), 
                                    vertex_index_map(get(&CellInfo::index, graph_)));
  vector<MutableRegionPtr> regions(num_comps);
  generate(regions.begin(), regions.end(), makeEmptyRegion);

  GridGraphIterator iter, end;
  for (tie(iter, end) = vertices(graph_); iter!=end; ++iter) {
    regions[graph_[*iter].component]->insert(graph_[*iter].cell);
  }
  return regions;
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
  ROS_DEBUG_STREAM_NAMED ("grid_graph", "Temporarily disconnecting region (lexicographically) bounded by " << *(region.begin()) << " and " << *(region.rbegin()));
  for (Region::iterator iter=region.begin(); iter!=region.end(); ++iter) {
    if (graph->containsCell(*iter)) {
      for (char vertical=0; vertical<2; ++vertical) {
        for (char multiplier=-1; multiplier<=1; multiplier+=2) {
          int dr = multiplier * ( vertical ? 1 : 0 );
          int dc = multiplier * ( vertical ? 0 : 1 );
          Cell2D neighbor(iter->r+dr, iter->c+dc);
          if (region.find(neighbor)==region.end() && graph->containsEdge(*iter, neighbor)) {
            double cost = graph->costBetween(*iter, neighbor);
            graph->removeEdge(*iter, neighbor);
            removed_edges_.push_back(RemovedEdge(*iter, neighbor, cost));
          }
        }
      }
    }
  }
}


RegionIsolator::~RegionIsolator ()
{
  ROS_DEBUG_STREAM_NAMED ("grid_graph", "Adding back disconnected region");
  for (RemovedEdges::iterator iter=removed_edges_.begin(); iter!=removed_edges_.end(); ++iter) {
    graph->addEdge(iter->get<0>(), iter->get<1>(), iter->get<2>());
  }
}


} // namespace
