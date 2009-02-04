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
            best_cell = *i; // Copy
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

    ROS_DEBUG_COND (!(++num_propagations%100000), "Propagation %d of %d, %d with bound %d",
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
      map<BottleneckVertex,GridCell> new_map;
      roadmap_points_[*vertex_iter] = new_map;

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
    

  

GridCellVector RoadmapBottleneckGraph::findOptimalPath (const GridCell& start, const GridCell& goal, float* cost)
{

  // Get vertex info
  ensureCellExists(goal);
  BottleneckVertex start_vertex, goal_vertex;
  if(!lookupVertex(start.first, start.second, &start_vertex)) {
    ROS_FATAL_STREAM ("Invalid grid cell " << start);
    throw InvalidGridCellException();
  }
  if(!lookupVertex(goal.first, goal.second, &goal_vertex)) {
    ROS_FATAL_STREAM ("Invalid grid cell " << goal);
    throw InvalidGridCellException();
  }
  VertexDescription& start_desc = vertexDescription (start_vertex);
  VertexDescription& goal_desc = vertexDescription (goal_vertex);
  ROS_DEBUG ("Start and goal are in regions %d and %d", start_desc.id, goal_desc.id);
  GridCellVector solution;


  // See if start and goal are in same or adjacent regions
  BottleneckAdjacencyIterator adjacency_iter, adjacency_end;
  tie(adjacency_iter, adjacency_end) = adjacent_vertices(start_vertex, graph_);
  bool useNavFn = ( (start_vertex == goal_vertex) || (find(adjacency_iter, adjacency_end, goal_vertex) != adjacency_end) );



  if (useNavFn) {
    ROS_DEBUG ("Using navfn");
    if (planUsingNavFn (start, goal, &solution, cost)) {
      return solution;
    }
    else {
      throw TopologicalMapException("Navfn failed to find solution");
    }
  }
  else {
    ROS_DEBUG ("Using top planner."); 
      
    if (start_desc.type == OPEN) {
      ROS_DEBUG ("Start is in an open region.  Looking for connectors within it.");
      BottleneckVertex next_region_in_best_path;
      float best_neighbor_cost=MAX_PATH_COST;
      findBestConnector(roadmap_points_[start_vertex], start, goal, &best_neighbor_cost, &next_region_in_best_path);
      if (best_neighbor_cost==MAX_PATH_COST) {
        ROS_ERROR ("Could not find a path in topological planner");
      }
      else {
        ROS_DEBUG_STREAM ("Best neighboring region is " << vertexDescription(next_region_in_best_path).id << "; looking for best connector of that region");
        // We've found the neighboring region to go to next.  Now we actually plan which connector of that region to aim for.
        // This is so that we always have a low-level plan that extends a little distance out from where the robot currently is.
        *cost = MAX_PATH_COST;
        findBestNeighborConnector(next_region_in_best_path, start, goal, cost, &solution, start_vertex);
        if (*cost==MAX_PATH_COST) {
          ROS_ERROR ("Could not find a path through neighbor in topological planner");
        }
      }
    }
    else {
      ROS_DEBUG ("Start is in a bottleneck region.  Looking for neighboring connectors.");
      *cost=MAX_PATH_COST;
      findBestNeighborConnector(start_vertex, start, goal, cost, &solution);
      if (*cost==MAX_PATH_COST) {
        ROS_ERROR ("Could not find a path in topological planner");
      }
        
    }
  }


  return solution;
}


GridCellVector RoadmapBottleneckGraph::findOptimalPath (const GridCell& start, const GridCell& goal)
{
  float cost;
  return findOptimalPath (start, goal, &cost);
}

  



void RoadmapBottleneckGraph::findBestConnector(const VertexCellMap& neighbor_connectors, const GridCell& start, const GridCell& goal, float* best_cost, BottleneckVertex* next_region)
{
  for (VertexCellMap::const_iterator adjacent_pair_iter=neighbor_connectors.begin(); adjacent_pair_iter!=neighbor_connectors.end(); adjacent_pair_iter++) {
    double candidate_cost=computeConnectorCost (start, goal, adjacent_pair_iter->second);
    if (candidate_cost<*best_cost) {
      ROS_DEBUG ("Found new best connector");
      *best_cost=candidate_cost;
      *next_region=adjacent_pair_iter->first;
    }
  }
}

void RoadmapBottleneckGraph::findBestNeighborConnector(const BottleneckVertex& region, const GridCell& start, const GridCell& goal, 
                                                       float* best_cost, GridCellVector* solution, const BottleneckVertex& current, bool use_current)
{
  ROS_DEBUG ("Looking for best connector in region %d", vertexDescription(region).id);
  BottleneckAdjacencyIterator adjacency_iter, adjacency_end;
  for (tie(adjacency_iter, adjacency_end) = adjacent_vertices(region, graph_); adjacency_iter!=adjacency_end; adjacency_iter++) {
    if (!use_current || *adjacency_iter!=current) {
      GridCell& connector=roadmap_points_[*adjacency_iter][region];
      GridCellVector candidate_path;
      double candidate_cost=computeConnectorCost(start, goal, connector, &candidate_path);
      if (candidate_cost<*best_cost) {
        ROS_DEBUG ("Found new best connector in neighbor region");
        *best_cost=candidate_cost;
        *solution=candidate_path; // possibly inefficient but shouldn't happen too often
      }
    }
  }
}

void RoadmapBottleneckGraph::findBestNeighborConnector(const BottleneckVertex& region, const GridCell& start, const GridCell& goal, float* best_cost, GridCellVector* solution)
{
  BottleneckVertex dummy;
  findBestNeighborConnector (region, start, goal, best_cost, solution, dummy, false);
}
    


double RoadmapBottleneckGraph::computeConnectorCost(const GridCell& start, const GridCell& goal, const GridCell& connector, GridCellVector* solution)
{
  float candidate_cost;
  int topological_cost;
  GridCellVector candidate_path;

  if (planUsingNavFn (start, connector, &candidate_path, &candidate_cost)) {
    if (solution) {
      *solution=candidate_path;
      cout << "Assigning solution of length " << solution->size() << endl;
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
