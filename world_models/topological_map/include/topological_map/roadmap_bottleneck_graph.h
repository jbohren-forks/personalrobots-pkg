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

#ifndef TOPOLOGICAL_MAP_ROADMAP_BOTTLENECK_GRAPH_H
#define TOPOLOGICAL_MAP_ROADMAP_BOTTLENECK_GRAPH_H

#include "bottleneck_graph.h"
#include <discrete_space_information/precomputed_adjacency_list/environment_precomputed_adjacency_list.h>
#include <navfn.h>

namespace topological_map
{


typedef AdjacencyListSBPLEnv<GridCell> Roadmap;
typedef map<BottleneckVertex,GridCell> VertexCellMap;
typedef map<BottleneckVertex,VertexCellMap> VertexPairCellMap;



/// \brief A bottleneck graph together with a roadmap that can be passed to SBPL planners
class RoadmapBottleneckGraph : public IndexedBottleneckGraph
{
public:
  RoadmapBottleneckGraph(int num_rows=-1, int num_cols=-1, double costmap_multiplier=1.0);
  ~RoadmapBottleneckGraph();

  /// Initialize the roadmap associated to this topological map
  void initializeRoadmap ();

  /// Set costmap
  /// The roadmap is not the owner of this memory.  So it expects that costmap won't be deallocated
  /// while roadmap holds a reference to it, and after setCostmap is called again in future, caller is
  /// expected to deallocate old one.
  void setCostmap (const unsigned char* costmap);

  /// Return vector of grid cells going from start to goal.
  /// Grid cells will be contiguous early on but eventually become noncontiguous.
  vector<GridCell> findOptimalPath (const GridCell& start, const GridCell& goal);

  /// Output the roadmap and region graph as a plain ppm file
  void outputPpm (std::ostream& = std::cout, int bottleneck_vertex_radius = 1);

  /// Print the roadmap in human readable form
  void printRoadmap ();
  
  
private:

  /// \post A cell equal to c exists in the roadmap, and is connected to other cells currently in the same region
  /// \return Number of cells added (either 0 or 1)
  int ensureCellExists (const GridCell& cell);

  GridCell pointOnBorder (const Region& r1, const Region& r2);

  /// Use NavFn planner to plan from start to goal, and store the resulting path and cost
  /// solution is overwritten if it already holds something.  Return true iff path was found.
  bool planUsingNavFn (const GridCell& start, const GridCell& goal, vector<GridCell>* solution, float* cost);

  const unsigned char* costmap_;
  Roadmap* roadmap_;
  NavFn nav_fn_planner_;
  GridCell start_;
  GridCell goal_;
  int current_region_;
  double costmap_multiplier_;
  VertexPairCellMap roadmap_points_;

};




} // namespace topological_map

#endif // TOPOLOGICAL_MAP_ROADMAP_BOTTLENECK_GRAPH_H
