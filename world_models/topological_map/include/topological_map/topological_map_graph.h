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
 * Contains implementation definitions for topological map
 * (not meant to be externally included)
 */

#ifndef TOPOLOGICAL_MAP_TOPOLOGICAL_MAP_GRAPH_H
#define TOPOLOGICAL_MAP_TOPOLOGICAL_MAP_GRAPH_H

#include "topological_map.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

using boost::listS;
using boost::undirectedS;
using boost::adjacency_list;
using boost::graph_traits;


namespace topological_map
{

struct RegionInfo
{
  int type;
  RegionPtr region;
  RegionId id;
  
  RegionInfo ();
  RegionInfo (const int region_type, const RegionPtr region_cells, const RegionId region_id) :
    type(region_type), region(region_cells), id(region_id) {}
};

typedef adjacency_list<listS, listS, undirectedS, RegionInfo> TopologicalGraph;
typedef graph_traits<TopologicalGraph>::vertex_descriptor TopologicalGraphVertex;
typedef graph_traits<TopologicalGraph>::adjacency_iterator AdjacencyIterator;
typedef std::map<Cell2D, RegionId> RegionMap;
typedef std::map<RegionId, TopologicalGraphVertex> IdVertexMap;

// Implementation details for top map
class TopologicalMap::GraphImpl
{
public:

  /// Default constructor creates empty graph
  GraphImpl() : next_id_(0) {}

  /// \return Id of region containing \a p
  /// \throws UnknownCell2DException
  RegionId containingRegion(const Cell2D& p) const;

  /// \return Integer representing type of region
  /// \throws UnknownRegionException
  int regionType(const RegionId id) const;

  /// \return Vector of id's of neighboring regions to region \a r
  RegionIdVector neighbors(const RegionId r) const;

  /// \return Vector of all region ids
  const RegionIdVector& allRegions() const;

  /// \post New region has been added
  /// \return Id of new region
  /// \throws OverlappingRegionException
  RegionId addRegion (const RegionPtr region, const int region_type);

  /// \post Region no longer exists
  /// \throws UnknownRegionException
  void removeRegion (const RegionId id);

private:

  GraphImpl(const GraphImpl&);
  GraphImpl& operator= (const GraphImpl&);

  TopologicalGraphVertex idVertex(const RegionId id) const;

  IdVertexMap id_vertex_map_;
  RegionMap region_map_;
  TopologicalGraph graph_;
  RegionIdVector regions_;
  
  RegionId next_id_;
};


  
} // namespace topological_map

#endif // TOPOLOGICAL_MAP_TOPOLOGICAL_MAP_GRAPH_H
