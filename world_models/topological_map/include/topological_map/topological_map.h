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

#ifndef TOPOLOGICAL_MAP_TOPOLOGICAL_MAP_H
#define TOPOLOGICAL_MAP_TOPOLOGICAL_MAP_H

#include "region.h"
#include <map>

namespace topological_map
{

typedef unsigned int RegionId;
typedef std::map<GridCell, RegionId> RegionMap;
typedef std::map<RegionId, TopologicalGraphVertex> IdVertexMap;
typedef std::vector<RegionId> RegionIdVector;

/// Represents a topological map of a 2-dimensional discrete grid decomposed into regions of various types
class TopologicalMap
{
public:

  /// \return Id of region containing \a p
  /// \throws UnknownGridCellException
  RegionId containingRegion(const GridCell& p) const;

  /// \return Integer representing type of region
  /// \throws UnknownRegionException
  int RegionType(const RegionId id) const;

  /// \return Vector of id's of neighboring regions to region \a r
  RegionIdVector neighbors(const RegionId r) const;

  /// \post New region has been added
  /// \return Id of new region
  /// \throws OverlappingRegionException
  RegionId addRegion (const RegionPtr region, const int region_type);

  /// \post Region no longer exists
  /// \throws UnknownRegionException
  void removeRegion (const RegionId id);

private:

  /// \return Is the point known
  bool knownGridCell(const GridCell& p) const;

  /// \return vertex descriptor for this id
  /// \throws UnknownRegionException
  TopologicalGraphVertex idVertex (const RegionId id) const;

  /// Map from grid cell to region id.  Guaranteed to be stable (unless if the region itself is removed).
  RegionMap region_map_;
  
  /// Graph object
  TopologicalGraph graph_;

  /// Map from region id to vertex descriptor
  IdVertexMap id_vertex_map_;

};
  
} // namespace topological_map

#endif // TOPOLOGICAL_MAP_TOPOLOGICAL_MAP_H
