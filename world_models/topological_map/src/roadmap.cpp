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

#include <iostream>
#include <set>
#include <queue>
#include <ros/console.h>
#include <topological_map/roadmap_bottleneck_graph.h>


using namespace std; 
using boost::adjacent_vertices;

namespace topological_map
{

const int g_dx[] = {1, -1, 0, 0};
const int g_dy[] = {0, 0, -1, 1};

const double MAX_PATH_COST=1.0e10;

enum CellType { BOTTLENECKCELL, OBSTACLE, FREECELL, ROADMAPCELL };


/// Functor class used for iterating over a set of connectors, evaluating each one, and keeping track of the best one
struct EvaluateConnector
{
  EvaluateConnector (RoadmapBottleneckGraph* g, const GridCell& start_cell, const GridCell& goal_cell) : best_cost (MAX_PATH_COST), graph(g), start(start_cell), goal(goal_cell) {}
  ConnectorCostPair operator() (const RegionConnectorPair& pair) 
  { 
    const GridCell& connector=pair.second;
    double cost = graph->computeConnectorCost(start, goal, connector, &solution);
    if (cost < best_cost) {
      best_cost=cost;
      best_solution=solution;
      ROS_DEBUG_STREAM ("  New best connector found; solution length is " << best_solution.size());
    }
    return ConnectorCostPair(connector, cost);
  }

  GridCellVector solution, best_solution;
  double best_cost;
  RoadmapBottleneckGraph* graph;
  const GridCell& start;
  const GridCell& goal;
};



RoadmapBottleneckGraph::RoadmapBottleneckGraph(int num_rows, int num_cols, double costmap_multiplier) : 
  IndexedBottleneckGraph(num_rows, num_cols), nav_fn_planner_(0, 0), costmap_multiplier_(costmap_multiplier), distance_map_initialized_(false)
{}



GridCell RoadmapBottleneckGraph::pointOnBorder (const Region& r1, const Region& r2)
{
  GridCell neighbor;
  Region::iterator end = r2.end();

  if (distance_map_initialized_) {
    GridCell best_cell;
    int best_distance = -1;

    for (Region::iterator i=r1.begin(); i!=r1.end(); i++) {
      for (unsigned int j=0; j<4; j++)  {
        neighbor.first = i->first+g_dy[j];
        neighbor.second = i->second+g_dx[j];
        if (r2.find(neighbor)!=end) {
          if (distance_map_[i->first][i->second]>best_distance) {
            best_distance = distance_map_[i->first][i->second];
            best_cell = *i; 
            break; // Out of inner loop, since we're not going to learn anything new
          }
        }
      }
    }

    if (best_distance > -1) {
      return best_cell;
    }
  }

  else {
    for (Region::iterator i=r1.begin(); i!=r1.end(); i++) {
      for (unsigned int j=0; j<4; j++) {
        neighbor.first = i->first+g_dy[j];
        neighbor.second = i->second+g_dx[j];
        if (r2.find (neighbor)!=end) {
          return *i;
        }
      }
    }
  }
  throw TopologicalMapException("Couldn't find point on border between regions");
}




void RoadmapBottleneckGraph::initializeDistanceMap ()
{
  distance_map_.resize(boost::extents[num_rows_][num_cols_]);
  queue<GridCell> propagation_queue;

  // Initialize with a guaranteed upper bound on distance
  for (int r=0; r<num_rows_; r++) {
    for (int c=0; c<num_cols_; c++) {
      if (is_free_[r][c]) {
        distance_map_[r][c]=num_rows_+num_cols_;
      }
      else {
        distance_map_[r][c]=0;
        GridCell cell(r,c);
        propagation_queue.push(cell);
      }
    }
  }
  ROS_DEBUG ("Initializing %dx%d distance map", num_cols_, num_rows_);


  // Loop over queue of propagations to do
  int num_propagations=0;
  while (!propagation_queue.empty()) {
    GridCell cell=propagation_queue.front();
    propagation_queue.pop();
    int r=cell.first;
    int c=cell.second;
    int new_distance_bound=distance_map_[r][c]+1;

    ROS_DEBUG_COND (!(++num_propagations%5000000), "Propagation %d of %d, %d with bound %d",
                    num_propagations, r, c, new_distance_bound);
    
    for (int i=0; i<4; i++) {
      int r2=r+g_dy[i];
      int c2=c+g_dx[i];
      if (withinBounds(r2,c2) && distance_map_[r2][c2]>new_distance_bound) {
        distance_map_[r2][c2]=new_distance_bound;
        propagation_queue.push(GridCell(r2,c2));
      }
    }
  }
  ROS_INFO ("Distance map computed");
  distance_map_initialized_=true;
}




void RoadmapBottleneckGraph::initializeRoadmap ()
{
  initializeDistanceMap();
  roadmap_ = new AdjacencyListSBPLEnv<GridCell>();


  /* Map from vertex v to another map from vertex w to added border point between regions v and w
   * where v is an open region and w is a bottleneck.
   * Note that having a map like this implicitly assumes that BottleneckVertex objects have operator<
   * defined.  That is currently true because they come from a boost graph with vertex set of type vecS. */
    
  BottleneckVertexIterator vertex_iter, end;

  // Loop over open regions of the graph
  for (tie(vertex_iter,end) = vertices(graph_); vertex_iter!=end; ++vertex_iter) {
    VertexDescription desc = vertexDescription (*vertex_iter);
    if (desc.type == OPEN) {
      BottleneckAdjacencyIterator adjacency_iter, adjacency_end;

      // Add a roadmap point corresponding to each neighboring region
      for (tie(adjacency_iter, adjacency_end) = adjacent_vertices(*vertex_iter, graph_); adjacency_iter!=adjacency_end; adjacency_iter++) {
        VertexDescription neighborDesc = vertexDescription (*adjacency_iter);
        GridCell borderPoint = pointOnBorder (desc.region, neighborDesc.region);
        if (!roadmap_->hasPoint(borderPoint)) {
          roadmap_->addPoint (borderPoint);
        }
        roadmap_points_[*vertex_iter][*adjacency_iter] = borderPoint;
      }
    }
  }

    
  VertexPairCellMap::iterator vertex_pair_iter;

  // Loop over open regions
  int region=0;
  for (vertex_pair_iter=roadmap_points_.begin(); vertex_pair_iter!=roadmap_points_.end(); vertex_pair_iter++) {
    ROS_DEBUG_COND (!(++region%100), "Generating roadmap points in region %d", vertexDescription(vertex_pair_iter->first).id);

    // Loop over pairs consisting of a neighboring bottleneck region and the corresponding roadmap point
    for (VertexCellMap::iterator target_vertex_iter = vertex_pair_iter->second.begin(); target_vertex_iter != vertex_pair_iter->second.end(); target_vertex_iter++) {
      GridCell roadmap_cell = target_vertex_iter->second;

      // Loop again over such pairs so we can add edges between roadmap points within this open region
      for (VertexCellMap::iterator target_vertex_iter2 = vertex_pair_iter->second.begin(); target_vertex_iter2 != vertex_pair_iter->second.end(); target_vertex_iter2++) {
        GridCell neighbor_cell = target_vertex_iter2->second;

        // Add edges only between different cells, and only once
        if (neighbor_cell < roadmap_cell) {
          roadmap_->setCost (target_vertex_iter->second, target_vertex_iter2->second);
        }
      }

      // Loop over other neighbors of this bottleneck region
      BottleneckVertex neighboring_vertex = target_vertex_iter->first;
      BottleneckAdjacencyIterator adjacency_iter, adjacency_end;
      for (tie(adjacency_iter, adjacency_end) = adjacent_vertices(neighboring_vertex, graph_); adjacency_iter!=adjacency_end; adjacency_iter++) {
        if (*adjacency_iter != vertex_pair_iter->first) {
          roadmap_->setCost (roadmap_points_[*adjacency_iter][neighboring_vertex], roadmap_cell);
        }
      }
    }
  }
  ROS_INFO ("Roadmap generated");
}
    



void RoadmapBottleneckGraph::setCostmap (const unsigned char* costmap) {
  costmap_=costmap;
  nav_fn_planner_.setNavArr(num_cols_, num_rows_);
  nav_fn_planner_.setCostMap(costmap,true);
}



bool RoadmapBottleneckGraph::planUsingNavFn (const GridCell& start, const GridCell& goal, GridCellVector* solution, float* cost)
{
  int pos[2];
  int goal_pos[2];

  solution->clear();

  pos[0]=start.second;
  pos[1]=start.first;
  goal_pos[0]=goal.second;
  goal_pos[1]=goal.first;
  nav_fn_planner_.setStart(pos);
  nav_fn_planner_.setGoal(goal_pos);

  bool success = nav_fn_planner_.calcNavFnAstar();
  
  if (success) {
    float *x = nav_fn_planner_.getPathX();
    float *y = nav_fn_planner_.getPathY();
    int len = nav_fn_planner_.getPathLen();
    solution->reserve(len);

    for (int i=0; i<len; i++) {
      GridCell c;
      c.first=y[i];
      c.second=x[i];
      solution->push_back(c);
    }
    
    *cost = nav_fn_planner_.getLastPathCost();
    return true;
  }
  else {
    return false;
  }
}




ConnectorCostVector RoadmapBottleneckGraph::evaluateConnectors (const GridCell& start, const GridCell& goal)
{
  ensureCellExists(goal);
  ConnectorCostVector connector_costs;
  BottleneckVertex startVertex;
  lookupVertex (start, &startVertex);
  vector<RegionConnectorPair> adjacent_connectors = adjacentConnectors (startVertex);

  connector_costs.reserve(adjacent_connectors.size()+1);
  transform (adjacent_connectors.begin(), adjacent_connectors.end(), back_inserter(connector_costs), EvaluateConnector(this, start, goal));
  if (regionId(start)==regionId(goal)) {
    GridCellVector solution;
    float cost;
    planUsingNavFn(start, goal, &solution, &cost);
    connector_costs.push_back(ConnectorCostPair(start,cost));
  }
  ROS_DEBUG_STREAM ("Connector costs has length " << connector_costs.size());
  
  return connector_costs;
}



GridCellVector RoadmapBottleneckGraph::findOptimalPath (const GridCell& start, const GridCell& goal, float* cost)
{
  ensureCellExists(goal);
  BottleneckVertex start_vertex, goal_vertex;
  lookupVertex(start, &start_vertex);
  lookupVertex(goal, &goal_vertex);
  VertexDescription& start_desc = vertexDescription (start_vertex);
  VertexDescription& goal_desc = vertexDescription (goal_vertex);
  ROS_DEBUG ("Start and goal are in regions %d and %d", start_desc.id, goal_desc.id);
  
  // See if start and goal are in same or adjacent regions
  BottleneckAdjacencyIterator adjacency_iter, adjacency_end;
  tie(adjacency_iter, adjacency_end) = adjacent_vertices(start_vertex, graph_);
  bool useNavFn = ( (start_vertex == goal_vertex) || (find(adjacency_iter, adjacency_end, goal_vertex) != adjacency_end) );

  if (useNavFn) {
    ROS_DEBUG ("Using navfn");
    GridCellVector nav_fn_solution;
    if (planUsingNavFn (start, goal, &nav_fn_solution, cost)) {
      return nav_fn_solution;
    }
    else {
      throw TopologicalMapException("Navfn failed to find solution");
    }
  }

  else {
    ROS_DEBUG ("Using top planner."); 
    vector<RegionConnectorPair> adjacent_connectors = adjacentConnectors(start_vertex);
    EvaluateConnector connectorEvaluator = for_each(adjacent_connectors.begin(), adjacent_connectors.end(), EvaluateConnector(this, start, goal));
    *cost = connectorEvaluator.best_cost;
    ROS_DEBUG_STREAM ("Returning solution with length " << connectorEvaluator.best_solution.size());
    return connectorEvaluator.best_solution;
  }
}



GridCellVector RoadmapBottleneckGraph::findOptimalPath (const GridCell& start, const GridCell& goal)
{
  float cost;
  return findOptimalPath (start, goal, &cost);
}




/// \todo uses a magic constant to combine the navfn and sbpl costs
double RoadmapBottleneckGraph::computeConnectorCost(const GridCell& start, const GridCell& goal, const GridCell& connector, GridCellVector* solution)
{
  float candidate_cost;
  int topological_cost;
  GridCellVector candidate_path;

  if (planUsingNavFn (start, connector, &candidate_path, &candidate_cost)) {
    if (solution) {
      *solution=candidate_path;
    }

    candidate_cost /= 50;  // MAGIC CONSTANT - and also, figure out what the right way is to combine the navfn and sbpl costs
    roadmap_->setStartState(connector);
    roadmap_->setGoalState(goal);
    roadmap_->findOptimalPath(&topological_cost);
    candidate_cost += topological_cost;

    ROS_DEBUG_STREAM ("Connector " << connector << " had cost-to-goal " << topological_cost << "  and total cost " << candidate_cost);
    return candidate_cost;
  }
  else {
    ROS_DEBUG ("Connector unreachable, so returning %f as cost", MAX_PATH_COST);
    
    return MAX_PATH_COST;
  }
}




int RoadmapBottleneckGraph::ensureCellExists (const GridCell& cell)
{
  int r=cell.first;
  int c=cell.second;
  BottleneckVertex v;
  if (!lookupVertex(r, c, &v)) {
    ROS_FATAL ("Tried to look up cell %d, %d which is an obstacle", r, c);
    throw;
  }
  VertexDescription& desc = vertexDescription(v);

  if (roadmap_->hasPoint(cell)) {
    return 0;
  }
  else {
    roadmap_->addPoint (cell);
    if (desc.type == OPEN) {
      for (VertexCellMap::iterator vertex_iter = roadmap_points_[v].begin(); vertex_iter != roadmap_points_[v].end(); vertex_iter++) {
        GridCell other_cell = vertex_iter->second;
        if ((other_cell<cell) || (cell<other_cell)) {
          roadmap_->setCost(cell,other_cell);
        }
      }
    }
    else {
      BottleneckAdjacencyIterator adj_iter, adj_end;
      for (tie(adj_iter,adj_end) = adjacent_vertices(v, graph_); adj_iter!=adj_end; adj_iter++) {
        roadmap_->setCost(cell, roadmap_points_[*adj_iter][v]);
      }
    }
    return 1;
  }

}

vector<RegionConnectorPair> RoadmapBottleneckGraph::adjacentConnectors (const BottleneckVertex& v) 
{
  vector<RegionConnectorPair> adjacent_connectors;
  VertexDescription& desc = vertexDescription(v);
  ROS_DEBUG ("Looking for adjacent connectors of region %d", desc.id);

  if (desc.type == OPEN) {
    for (VertexCellMap::const_iterator vertex_iter = roadmap_points_[v].begin(); vertex_iter != roadmap_points_[v].end(); vertex_iter++) {
      RegionConnectorPair pair(vertex_iter->first, vertex_iter->second);
      adjacent_connectors.push_back(pair);
      ROS_DEBUG_STREAM (" Adding connector " << pair.second);
    }
  }
  else {
    BottleneckAdjacencyIterator adj_iter, adj_end;
    for (tie(adj_iter, adj_end) = adjacent_vertices(v, graph_); adj_iter!=adj_end; adj_iter++) {
      RegionConnectorPair pair(*adj_iter, roadmap_points_[*adj_iter][v]);
      adjacent_connectors.push_back(pair);
      ROS_DEBUG_STREAM (" Adding connector " << pair.second);
    }
  }
  ROS_DEBUG ("Finished searching for connectors");
  return adjacent_connectors;
}







void RoadmapBottleneckGraph::printRoadmap (void)
{
  roadmap_->writeToStream();
}



void RoadmapBottleneckGraph::outputPpm (ostream& stream, int roadmap_vertex_radius)
{
  boost::multi_array<CellType, 2> cells(boost::extents[num_rows_][num_cols_]);
  char* buffer = new char[num_cols_*num_rows_*3];
  int pos=0;
  stream << "P6" << endl << num_cols_ << " " << num_rows_ << endl << "255" << endl;
  for (int r=0; r<num_rows_; r++) {
    for (int c=0; c<num_cols_; c++) {
      GridCell cell(r,c);
      if (is_free_[r][c]) {
        if (roadmap_->hasPoint(cell)) {
          for (int i=r-roadmap_vertex_radius; i<=r+roadmap_vertex_radius; i++) {
            for (int j=c-roadmap_vertex_radius; j<=c+roadmap_vertex_radius; j++) {
              if ((i>=0) && (i<num_rows_) && (j>=0) && (j<num_cols_)) {
                cells[i][j] = ROADMAPCELL;
              }
            }
          }
        }
        else if (cells[r][c] != ROADMAPCELL) {
          BottleneckVertex v = grid_cell_vertex_[r][c];
          VertexDescription& d = vertexDescription (v);
          if (d.type == BOTTLENECK) {
            cells[r][c] = BOTTLENECKCELL;
          }
          else {
            cells[r][c] = FREECELL;
          }
        }
      }
      else {
        cells[r][c] = OBSTACLE;
      }
    }
  }
  

  for (int r=0; r<num_rows_; r++) {
    for (int c=0; c<num_cols_; c++) {

      switch (cells[r][c]) {
      case ROADMAPCELL: buffer[pos++]=0; buffer[pos++]=0; buffer[pos++]=255; break;
      case BOTTLENECKCELL: buffer[pos++]=255; buffer[pos++]=0; buffer[pos++]=0; break;
      case FREECELL: buffer[pos++]=255; buffer[pos++]=255; buffer[pos++]=255; break;
      case OBSTACLE: buffer[pos++]=0; buffer[pos++]=0; buffer[pos++]=0; break;
      }

    }

  }
  stream.write(buffer, pos);
  delete[] buffer;
}
        





} // namespace topological_map
