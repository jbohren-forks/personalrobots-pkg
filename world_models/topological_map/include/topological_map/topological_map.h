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
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/multi_array.hpp>

namespace topological_map
{

typedef unsigned int RegionId;
typedef std::vector<RegionId> RegionIdVector;

/// Represents a topological map of a 2-dimensional discrete grid decomposed into regions of various types
class TopologicalMap
{
public:

  /// Default constructor makes an empty map (see also topologicalMapFromGrid)
  TopologicalMap();

  /// \return Id of region containing \a p
  /// \throws UnknownCell2DException
  RegionId containingRegion(const Cell2D& p) const;

  /// \return Integer representing type of region
  /// \throws UnknownRegionException
  int regionType(const RegionId id) const;

  /// \return Vector of id's of neighboring regions to region \a r
  RegionIdVector neighbors(const RegionId r) const;

  /// \return Vector of all region ids.  This is a reference and may change.
  const RegionIdVector& allRegions() const;

  /// \post New region has been added
  /// \return Id of new region, which will be 1+the highest previously seen region id (or 0)
  /// \throws OverlappingRegionException
  RegionId addRegion (const RegionPtr region, const int region_type);

  /// \post Region no longer exists
  /// \throws UnknownRegionException
  void removeRegion (const RegionId id);

private:

  /// Avoid client compilation dependency on implementation details
  class GraphImpl;
  boost::shared_ptr<GraphImpl> graph_impl_;

  /// Forbid copy and assign
  TopologicalMap(const TopologicalMap&);
  TopologicalMap& operator= (const TopologicalMap&);
};


/************************************************************
 * Creation
 ************************************************************/

typedef boost::multi_array<bool, 2> OccupancyGrid;
typedef boost::shared_ptr<TopologicalMap> TopologicalMapPtr;

/// \return shared_ptr to a new topological map generated using a bottleneck analysis of \a grid
TopologicalMapPtr topologicalMapFromGrid (const OccupancyGrid& grid, const uint bottleneck_size, const uint bottleneck_skip, const uint inflation_radius, const std::string& pgm_output_dir);






  
} // namespace topological_map

#endif // TOPOLOGICAL_MAP_TOPOLOGICAL_MAP_H
