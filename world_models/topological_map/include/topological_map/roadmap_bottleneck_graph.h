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


namespace topological_map
{


typedef AdjacencyListSBPLEnv<GridCell> Roadmap;
typedef map<BottleneckVertex,GridCell> VertexCellMap;
typedef map<BottleneckVertex,VertexCellMap> VertexPairCellMap;



/// \brief A bottleneck graph together with a roadmap that can be passed to SBPL planners
class RoadmapBottleneckGraph : public IndexedBottleneckGraph
{
public:
  RoadmapBottleneckGraph();
  ~RoadmapBottleneckGraph();

  /// Initialize the roadmap associated to this topological map
  void initializeRoadmap ();

  void printRoadmap ();


  /// Make it so that roadmap is low-level in given region and high-level elsewhere
  void switchToRegion (int region_id);

  /// Find optimal path between configurations
  vector<GridCell> findOptimalPath (const GridCell& start, const GridCell& goal);


  
  
private:
  /// \post A cell equal to c exists in the roadmap, and is connected to other cells currently in the same region
  /// \return Number of cells added (either 0 or 1)
  int ensureCellExists (const GridCell& cell);

  /// Add to the roadmap the grid cells in the given region (except the ones that are already in the roadmap).
  void addRegionGridCells (int region_id);

  /// Remove from the roadmap the grid cells from the last added region
  void removeLastAddedRegionCells ();

  GridCell pointOnBorder (const Region& r1, const Region& r2);
  Roadmap* roadmap_;
  int num_temporary_added_cells_;
  VertexPairCellMap roadmap_points_;
};




} // namespace topological_map

#endif // TOPOLOGICAL_MAP_ROADMAP_BOTTLENECK_GRAPH_H
