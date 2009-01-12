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
#include <topological_map/bottleneck_graph.h>

using namespace std; 
using boost::adjacent_vertices;

namespace topological_map
{

const int g_dx[] = {1, -1, 0, 0};
const int g_dy[] = {0, 0, -1, 1};

GridCell pointOnBorder (const Region& r1, const Region& r2)
{
  GridCell neighbor;

  Region::iterator end = r2.end();
  for (Region::iterator i=r1.begin(); i!=r1.end(); i++) {
    for (unsigned int j=0; j<4; j++) {
      neighbor.first = i->first+g_dx[j];
      neighbor.second = i->second+g_dy[j];
      if (r2.find (neighbor)!=end) {
	return *i;
      }
    }
  }
  return 0;
}



  Roadmap* IndexedBottleneckGraph::makeRoadmap ()
  {
    Roadmap* roadmap = new AdjacencyListSBPLEnv<GridCell>();

    typedef map<BottleneckVertex,GridCell> VertexCellMap;
    typedef map<BottleneckVertex,VertexCellMap> VertexPairCellMap;

    /* Map from vertex v to another map from vertex w to added border point between regions v and w
     * where v is an open region and w is a bottleneck.
     * Note that having a map like this implicitly assumes that BottleneckVertex objects have operator<
     * defined.  That is currently true because they come from a boost graph with vertex set of type vecS. */
    VertexPairCellMap roadmap_points;
    
    BottleneckVertexIterator vertex_iter, end;
    for (tie(vertex_iter,end) = vertices(graph_); vertex_iter!=end; ++vertex_iter) {
      VertexDescription desc = vertexDescription (*vertex_iter);
      if (desc.type == OPEN) {
	BottleneckAdjacencyIterator adjacency_iter, adjacency_end;
	map<BottleneckVertex,GridCell> new_map;
	roadmap_points[*vertex_iter] = new_map;
	for (tie(adjacency_iter, adjacency_end) = adjacent_vertices(*vertex_iter, graph_); adjacency_iter!=adjacency_end; adjacency_iter++) {
	  VertexDescription neighborDesc = vertexDescription (*adjacency_iter);
	  GridCell borderPoint = pointOnBorder (desc.region, neighborDesc.region);
	  roadmap->addPoint (borderPoint);
	  roadmap_points[*vertex_iter][*adjacency_iter] = borderPoint;
	}
      }
    }

    
    VertexPairCellMap::iterator vertex_pair_iter;

    // Loop open regions
    for (vertex_pair_iter=roadmap_points.begin(); vertex_pair_iter!=roadmap_points.end(); vertex_pair_iter++) {


      // Loop over pairs consisting of a neighboring bottleneck region and the corresponding roadmap point
      for (VertexCellMap::iterator target_vertex_iter = vertex_pair_iter->second.begin(); target_vertex_iter != vertex_pair_iter->second.end(); target_vertex_iter++) {
        GridCell roadmap_cell = target_vertex_iter->second;

        // Loop again over such pairs so we can add edges between roadmap points within this open region
        for (VertexCellMap::iterator target_vertex_iter2 = vertex_pair_iter->second.begin(); target_vertex_iter2 != vertex_pair_iter->second.end(); target_vertex_iter2++) {
          GridCell neighbor_cell = target_vertex_iter2->second;

          // Add edges only between different cells, and only once
          if (neighbor_cell < roadmap_cell) {
            roadmap->setCost (target_vertex_iter->second, target_vertex_iter2->second);
          }
        }

        // Loop over other neighbors of this bottleneck region
        BottleneckVertex neighboring_vertex = target_vertex_iter->first;
        BottleneckAdjacencyIterator adjacency_iter, adjacency_end;
        for (tie(adjacency_iter, adjacency_end) = adjacent_vertices(neighboring_vertex, graph_); adjacency_iter!=adjacency_end; adjacency_iter++) {
          if (*adjacency_iter != vertex_pair_iter->first) {
            roadmap->setCost (roadmap_points[*adjacency_iter][neighboring_vertex], roadmap_cell);
          }
        }
      }

    }

    

    /* TODO
       1. Look up online documentation
       2. Iterate over (v,m) in VertexPairCellMap
       a) Iterate over (w,n) pairs in map and add edges between them
       b) Also, for each (w,n) pair: iterate over neighbors x of w in the bottleneck graph
       c) Add an edge between n and roadmapPoints[x][w]
    */

    return roadmap;
  }    

  

  int IndexedBottleneckGraph::addRegionGridCells (Roadmap* roadmap, int region_id)
  {
    BottleneckVertex v = id_vertex_map_[region_id];
    VertexDescription desc = vertexDescription (v);
    int num_added=0;
    
    for (Region::iterator grid_cell_iter = desc.region.begin(); grid_cell_iter != desc.region.end(); ++grid_cell_iter) {
      if (!(roadmap->hasPoint(*grid_cell_iter))) {
	roadmap->addPoint (*grid_cell_iter);
	++num_added;
	for (int i=0; i<4; i++) {
	  GridCell neighbor(grid_cell_iter->first+g_dx[i], grid_cell_iter->second+g_dy[i]);
	  if (roadmap->hasPoint(neighbor)) {
	    roadmap->setCost (*grid_cell_iter, neighbor);
	  }
	}
      }
    }

    return num_added;
  }
    
    
    
    






}
